#!/usr/bin/python3
# Import original files and messages
from tools import img_parameters
from robot_control.msg import ImgShowMsg
from robot_control.msg import ROI
from sas_datalogger.msg import AddValueMsg

# Import files for image processing
import torch
import cv2
import segmentation_models_pytorch as smp
import albumentations as albu
from cv_bridge import CvBridge
from collections import deque
from threading import Thread

# Import other dependencies
import numpy as np
import rospy
import time
import os

# Import files from SAS library
from sas_datalogger import DataloggerInterface


def main():
    # Init "predict_4K" node
    rospy.init_node('predict_keypoint')
    predict_node = PredictKeyPointsNode()
    rospy.spin()


class PredictKeyPointsNode:
    """
    PredictKeyPointsNode subscribes microscopic images from "microscope" node, predicts the positions of the instruments' tips
    and the shaft line of the surgical instrument, and publishes the distances used for autonomous positioning to
    "distance" node.
    For more details, see Koyama, Y., Marinho, M. M., Mitsuishi, M., and Harada, K. (2022).
    Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal Surgery.
    Transactions on Medical Robotics and Bionics (TMRB)
    """
    def __init__(self):
        # load parameters from "img_parameters"
        self.parameters = img_parameters.ImageParameters
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load the tip detection model and move to GPU
        self.best_model = torch.load(self.parameters.best_model)
        self.best_model.to(self.device)
        self.best_model.eval()  # Set the model to evaluation mode
        print("[" + rospy.get_name() + "]::Loaded!")

        # Set up processing
        self.encoder = 'resnet18'  
        self.encoder_weights = 'imagenet'
        self.preprocessing_fn = smp.encoders.get_preprocessing_fn(self.encoder, self.encoder_weights)
        self.preprocessing = self.get_preprocessing(self.preprocessing_fn)

        # Initialize data logger
        self.data_logger = DataloggerInterface(10)

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.bridge_to_cv2 = self.bridge.imgmsg_to_cv2
        self.bridge_to_imgmsg = self.bridge.cv2_to_imgmsg

        # Initialize frame buffer and queue for asynchronous processing
        self.frame_queue = deque(maxlen=1)  # Queue of size 1 to hold the latest frame

        # Start the processing thread
        self.processing_thread = Thread(target=self.process_frames)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        # Parameters for image processing
        self.predict_size = self.parameters.predict_size
        self.output_size = self.parameters.output_size

        # Keypoint detection parameters
        self.previous_point_instrument = deque(maxlen=1)
        self.previous_point_shadow = deque(maxlen=self.parameters.EMA_shadow_N)
        self.previous_point_point_instrument = deque(maxlen=self.parameters.EMA_point_N)
        self.shaft_vector = None
        self.stuck_counter = 0
        self.print_outlier = self.parameters.print_outlier

        # Logging and debugging parameters
        self.print_time = self.parameters.print_predict_node_time
        self.print_time_from_get_to_predict = self.parameters.print_time_from_get_to_predict
        self.print_debugging_information = self.parameters.print_debugging_information

        self.roi_pub_count = 0

        # Launch ROS publishers and ROS Subscribers
        self.publisher_distance_ = rospy.Publisher("predict/distances", ImgShowMsg, queue_size=1)
        self.publisher_tip_positions_ = rospy.Publisher("predict/tip_positions", AddValueMsg, queue_size=1)
        self.subscriber_microscopic_image_ = rospy.Subscriber("microscope/capture", ROI, self._get_image)

        self.i = 0

        print("[" + rospy.get_name() + "]::Ready!")


    def _get_image(self, msg):
        """
        Function is called when an image is published to the node "microscope/capture".
        """
        start = time.time()
        self.roi_pub_count = self.roi_pub_count + 1
        header_time = msg.roi_image.header.stamp

        # For debug
        if self.print_time_from_get_to_predict:
            print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-header_time)+" nsec")

        # Get ROI image
        frame = self.bridge_to_cv2(msg.roi_image, "bgr8")
        ROI_center = np.array([msg.roi_values[0], msg.roi_values[1]])
        ROI_half_dis = msg.roi_values[2]
        point_instr = np.array([msg.roi_values[3], msg.roi_values[4]])
        if self.print_debugging_information:
            print("frame:", np.shape(frame))
            print("ROI_center: ", ROI_center)
            print("ROI_half_dis: ", ROI_half_dis)
            print("Received ROI image")
        
        # Add frame to queue
        if len(self.frame_queue) < self.frame_queue.maxlen:
            self.frame_queue.append((frame, ROI_center, ROI_half_dis, point_instr, header_time, start))
        else:
            self.frame_queue[0] = (frame, ROI_center, ROI_half_dis, point_instr, header_time, start)        
        

    def process_frames(self):
        """
        Process the frames in the queue asynchronously.
        """    
        while True:
            if len(self.frame_queue) > 0:
                frame_data = self.frame_queue.popleft()
                frame, ROI_center, ROI_half_dis, point_instr, header_time, start = frame_data

                # Predict the positions of the instruments' tip, instruments' shadow, and the shaft line of the surgical instrument
                (ratio, tip_dis, shaft_dis, tip_instrument, tip_shadow, point_instrument
                ) = self.predict_and_mark(frame, ROI_half_dis, point_instr)

                # Publish the calculated distances
                msg = ImgShowMsg(value=[tip_dis, shaft_dis])
                msg.header.stamp = header_time
                self.publisher_distance_.publish(msg)

                # Calculate the relative instrument tip position in the ROI image
                left_top = (ROI_center - np.array([ROI_half_dis, ROI_half_dis])).astype('uint64')
                tip_instrument_overall = left_top + np.array([int(tip_instrument[1]*ratio), int(tip_instrument[0]*ratio)])

                predicted_points = np.vstack([tip_instrument.reshape([2, 1]),
                                            tip_shadow.reshape([2, 1]),
                                            point_instrument.reshape([2, 1]),
                                            tip_instrument_overall.reshape([2, 1])])

                # Publish the predicted points
                msg_tip = AddValueMsg(name="predicted_points", value=predicted_points)
                self.publisher_tip_positions_.publish(msg_tip)

                # Save calculation time for debug
                self.data_logger.log("time_capture2predict", float(str(rospy.Time.now()-header_time)))
                self.data_logger.log("hz_predict", float(1/(time.time()-start)))

                if self.print_time_from_get_to_predict:
                    print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-header_time)+" nsec")

                if self.print_time:
                    print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")
        

    def predict_and_mark(self, resized_img, ROI_half_dis, point_instrument):
        """
        predict_and_mark(self, resized_img, ROI_half_dis) predicts the positions of the instruments' tips
        and the shaft line of the surgical instrument, marks these values on the received image for showing,
        calculates the shaft distance and tip distance for autonomous positioning, and publishes the calculated distances.
        """
        # Preprocess the frame
        frame = self.preprocess_image(resized_img)
        frame = self.preprocessing(image=frame)['image']

        # Convert the frame to a tensor and move to GPU
        frame_tensor = torch.from_numpy(frame).float().to(self.device).unsqueeze(0)
        
        # Forward pass the frame through the ROI model
        if self.parameters.mixed_precision:
             with torch.no_grad():
                with torch.cuda.amp.autocast():
                    pr_mask = self.best_model(frame_tensor)  # torch.tensor
        else:    
            with torch.no_grad():
                pr_mask = self.best_model.predict(frame_tensor)  # torch.tensor

        # Convert the tensor to a numpy array and transpose the dimensions
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

        if self.parameters.save_predicted_images:
            RESULT_DIR = '/home/yuki/Videos/'
            SAVE_DIR = os.path.join(RESULT_DIR, 'prediction_check/')

            if self.i % 50 == 0:
                cv2.imwrite(SAVE_DIR + str(self.i + 1) + '-test' + '.png', resized_img)
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask1' + '.png', mask_vis_test[:, :, 0] * 255)
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask2' + '.png', mask_vis_test[:, :, 1] * 255)
                # cv2.imwrite(SAVE_DIR + str(i + 1) + '-test_mask3' + '.png', mask_vis_test[:, :, 2] * 255)

                cv2.imwrite(SAVE_DIR + str(self.i + 1) + '-test_predict1' + '.png', pr_mask[:, :, 0])
                cv2.imwrite(SAVE_DIR + str(self.i + 1) + '-test_predict2' + '.png', pr_mask[:, :, 1])
                # cv2.imwrite(SAVE_DIR + str(self.i + 1) + '-test_predict3' + '.png', pr_mask[:, :, 2])

        # Get the predicted positions
        tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
        tip_shadow= np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))

        # Filter out the outliers
        tip_instrument = self.outlier_detection(self.previous_point_instrument, tip_instrument, self.parameters.outlier_threshold_instrument, "instrument")
        point_instrument = self.outlier_detection(self.previous_point_point_instrument, point_instrument, self.parameters.outlier_threshold_another_instrument, "point")
        # If the shadow tip is stuck at the same point, ignore outlier detection
        if len(self.previous_point_shadow) == 0 or np.array_equal(tip_shadow, self.previous_point_shadow[-1]) or not self.stuck_counter >= self.parameters.stuck_threshold:
            tip_shadow = self.outlier_detection(self.previous_point_shadow, tip_shadow, self.parameters.outlier_threshold_shadow, "shadow")
            if self.previous_point_shadow and np.array_equal(tip_shadow, self.previous_point_shadow[-1]):
                self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        # Add the new points to the deque
        self.previous_point_shadow.append(tip_shadow)
        self.previous_point_instrument.append(tip_instrument)
        self.previous_point_point_instrument.append(point_instrument)
        
        # Ignore outlier shaft point if the angle between the last and current iteration is too large
        shaft_vector_angle = int(self.vector_ignore(point_instrument - tip_instrument))
        if abs(shaft_vector_angle) > int(self.parameters.shaft_vector_angle_threshold):
            if self.print_outlier:
                print('Point and shaft exceeding the angle threshold')
            point_instrument = self.previous_point_point_instrument[-1]
        
        # Calculate the EMA of the points (smoothing)
        tip_shadow = self.EMA(self.previous_point_shadow, self.parameters.alpha_shadow)
        point_instrument = self.EMA(self.previous_point_point_instrument, self.parameters.alpha_point)

        # Calculate the shaft distance and the tip distance---Section VI and VII of Koyama et al. (2022)
        vec_1 = point_instrument - tip_instrument
        vec_2 = tip_shadow - tip_instrument
        ratio = ROI_half_dis / self.predict_size * 2
        shaft_dis = np.linalg.norm(vec_1.dot(vec_2)/(vec_1.dot(vec_1))*vec_1-vec_2)*ratio
        tip_dis = int(np.linalg.norm(tip_instrument-tip_shadow)*ratio)

        return ratio, tip_dis, shaft_dis, tip_instrument, tip_shadow, point_instrument


    def outlier_detection(self, previous_point, point, threshold, name):
        if previous_point and np.linalg.norm(point - previous_point[-1]) > threshold:
            if self.print_outlier:
                print(name, 'outlier detected:', np.linalg.norm(point - previous_point[-1]))
            return previous_point[-1]
        else:
            return point
        
    def vector_ignore(self, current_vector):
        if self.shaft_vector is None:
            self.shaft_vector = current_vector
        # Calculate angle between the current vector and the previous vector
        dot_product = np.dot(current_vector, self.shaft_vector)
        norm_current = np.linalg.norm(current_vector)
        norm_previous = np.linalg.norm(self.shaft_vector)
        cos_angle = dot_product / (norm_current * norm_previous)

        # Clamp cos_angle to the range [-1, 1] to avoid invalid values for arccos
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        angle = np.degrees(np.arccos(cos_angle))
        self.shaft_vector = current_vector
        if np.isnan(angle):
            angle = 0

        return angle
    
    @staticmethod
    def EMA(previous_point, alpha):
        """
        Get the Exponential Moving Average (EMA) of the points
        """
        if len(previous_point) == 1:   
            point = previous_point[-1]  # First data point is set as initial EMA
            return point
        else:                                          
            point = alpha  * previous_point[-1] + (1 - alpha) * np.mean(list(previous_point)[:-1])  # EMA calculation
            return point
        
    @staticmethod
    def preprocess_image(image):
        # # Convert the image to grayscale if it's not already
        image = cv2.bilateralFilter(image, 6, 100, 50)
     
        # # Apply CLAHE (Contrast Limited Adaptive Histogram Equation) to the image
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)                # Convert the image to the LAB color space
        l, a, b = cv2.split(lab)                                    # Split the LAB image into L, A and B channels
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(6,6))  # Apply histogram equalization to the L channel
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))                                  # Merge the equalized L channel back with the A and B channels
        image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)                # Convert the image back to the BGR color space
        
        # # Min-Max normalization to [0, 1]
        image = (image - np.min(image)) / (np.max(image) - np.min(image))
        image = (image * 255).astype(np.uint8)  # Convert the image back to 8-bit

        # # Apply Sobel edge detection
        sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=9)
        sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=9)
        edges = np.hypot(sobelx, sobely)
        edges *= 255.0 / np.max(edges)
        edges = edges.astype(np.uint8)  # Convert the image back to 8-bit
        image = cv2.addWeighted(image, 0.7, edges, 0.3, 0)

        return image
    
    def get_preprocessing(self, preprocessing_fn):
        """
        get_preprocessing(preprocessing_fn) defines preprocessing for input images.
        """
        _transform = [
            albu.Lambda(image=preprocessing_fn),
            albu.Lambda(image=self.to_tensor, mask=self.to_tensor),
        ]
        return albu.Compose(_transform)

    @staticmethod
    def to_tensor( x, **kwargs):
        """
        to_tensor(x, **kwargs) transposes input images for prediction.
        """
        return x.transpose(2, 0, 1).astype('float32')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
