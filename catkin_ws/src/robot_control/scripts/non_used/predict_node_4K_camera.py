#!/usr/bin/python3
# Import original files and messages
from tools import img_parameters
from robot_control.msg import ImgShowMsg
from robot_control.msg import CImage
from robot_control.msg import ROI
from sas_datalogger.msg import AddValueMsg

# Import files for image processing
import torch
import cv2
import segmentation_models_pytorch as smp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import albumentations as albu
from collections import deque

# Import other dependencies
import numpy as np
import rospy
import time
import os

# Import files from SAS library
from sas_datalogger import DataloggerInterface


def main():
    # Init "predict_4K" node
    rospy.init_node('predict_4K')
    predict_node = PredictKeyPointsNode()
    rospy.spin()


class PredictKeyPointsNode:
    """
    PredictNode subscribes microscopic images from "microscope" node, predicts the positions of the instruments' tips
    and the shaft line of the surgical instrument, and publishes the distances used for autonomous positioning to
    "distance" node.
    For more details, see Koyama, Y., Marinho, M. M., Mitsuishi, M., and Harada, K. (2022).
    Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal Surgery.
    Transactions on Medical Robotics and Bionics (TMRB)
    """
    def __init__(self):
        # load parameters from "img_parameters"
        self.parameters = img_parameters.ImageParameters
        self.best_model = torch.load(self.parameters.best_model)
        self.roi_model = torch.load(self.parameters.ROI_model)
        print("loaded!")
        self.ENCODER = 'resnet18'  # 'resnet18'
        self.ENCODER_WEIGHTS = 'imagenet'
        self.DEVICE = 'cuda'
        self.preprocessing_fn = smp.encoders.get_preprocessing_fn(self.ENCODER, self.ENCODER_WEIGHTS)
        self.preprocessing = self.get_preprocessing(self.preprocessing_fn)

        self.data_logger = DataloggerInterface(10)

        self.bridge = CvBridge()
        self.bridge_to_cv2 = self.bridge.imgmsg_to_cv2
        self.bridge_to_imgmsg = self.bridge.cv2_to_imgmsg

        self.counter_window = np.zeros([5])        

        self.original_w = self.parameters.original_w
        self.original_h = self.parameters.original_h
        self.predict_size = self.parameters.predict_size
        self.output_size = self.parameters.output_size

        self.threshold1 = self.parameters.threshold1
        self.threshold2 = self.parameters.threshold2
        self.threshold3 = self.parameters.threshold3

        # ROI parameters
        self.previous_centers = deque(maxlen=self.parameters.SMA_smoothing_N)
        self.freeze_counter = 0

        # Keypoint detection parameters
        self.previous_point_instrument = deque(maxlen=1)
        self.previous_point_shadow = deque(maxlen=self.parameters.EMA_shadow_N)
        self.previous_point_point_instrument = deque(maxlen=self.parameters.EMA_point_N)
        self.shaft_vector = None
        self.stuck_counter = 0

        self.put_text_x = int(self.predict_size*250/512)
        self.put_text_y_1 = int(self.predict_size*420/512)
        self.put_text_y_2 = int(self.predict_size*460/512)
        self.put_text_y_3 = int(self.predict_size*500/512)
        self.fontsize = self.parameters.fontsize

        #self.max_dis = self.threshold1/2
        self.margin = self.parameters.margin

        self.print_time = self.parameters.print_predict_node_time
        self.print_time_from_get_to_predict = self.parameters.print_time_from_get_to_predict
        self.print_debugging_information = self.parameters.print_debugging_information
        self.print_outlier = self.parameters.print_outlier
        self.print_center = self.parameters.print_center

        self.cframe_pub_count = 0
        self.roi_pub_count = 0

        # Launch ROS publishers and ROS Subscribers
        self.publisher_distance_ = rospy.Publisher("predict/distances", ImgShowMsg, queue_size=1)
        self.publisher_ROI_parameter_ = rospy.Publisher("predict/ROI_parameter", ImgShowMsg, queue_size=1)
        self.publisher_tip_positions_ = rospy.Publisher("predict/tip_positions", AddValueMsg, queue_size=1)
        self.publisher_output_image_ = rospy.Publisher("img_show/ROI", Image, queue_size=1)

        self.subscriber_compressed_image_ = rospy.Subscriber("microscope/compressed_frame", CImage, self._get_compressed_image)
        self.subscriber_microscopic_image_ = rospy.Subscriber("microscope/capture", ROI, self._get_image)

        self.i = 0

        print("[" + rospy.get_name() + "]::Ready!")


    def _get_compressed_image(self, msg):
        """
        Function is called when an image is published to the node "microscope/compressed_frame". Predicts the ROI center and max distance.
        """
        start = time.time()
        self.cframe_pub_count += 1
        header_time = msg.cframe_image.header.stamp

        # For debug
        if self.print_time_from_get_to_predict:
            print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-header_time)+" nsec")

        # Get compressed image
        frame = self.bridge_to_cv2(msg.cframe_image, "bgr8")
        if self.print_debugging_information:
            print(np.shape(frame))

        print("Received compressed image")

        # Store the compressed image information
        resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height = msg.cframe_values

        # Predict the region of interest (ROI) in the frame and calculate max distance
        ROI_center, self.max_dis = self.predict_ROI(frame, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height)

        # Check if the keypoints are are within the ROI and adjust the size of the ROI if necessary
        if self.max_dis + self.margin <= self.threshold1 / 2:
            ROI_dis = self.threshold1 / 2
        elif self.max_dis + self.margin <= self.threshold2 / 2:
            ROI_dis = self.threshold2 / 2
        else:
            ROI_dis = self.threshold3 / 2

        if ROI_center[0] - ROI_dis < 0:
            ROI_center[0] = ROI_dis
        if ROI_center[1] - ROI_dis < 0:
            ROI_center[1] = ROI_dis
        if ROI_center[0] + ROI_dis >= self.original_w:
            ROI_center[0] = self.original_w - ROI_dis
        if ROI_center[1] + ROI_dis >= self.original_h:
            ROI_center[1] = self.original_h - ROI_dis

        # Publish the center and the size of the ROI image
        msg_ROI_parameter = ImgShowMsg(value=[ROI_center[0], ROI_center[1], ROI_dis])
        msg_ROI_parameter.header.stamp = header_time
        print(f"Publishing ROI parameters: {msg_ROI_parameter}")
        self.publisher_ROI_parameter_.publish(msg_ROI_parameter)
        print("Published ROI parameters")

        # Save calculation time for debug
        self.data_logger.log("time_capture2predict", float(str(rospy.Time.now()-header_time)))
        self.data_logger.log("hz_predict", float(1/(time.time()-start)))

        if self.print_time_from_get_to_predict:
            print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-header_time)+" nsec")

        if self.print_time:
            print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")    


    def _get_image(self, msg):
        """
        _get_image(self, msg) is called when an image is published to the node "microscope/capture".
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
        ROI_dis = msg.roi_values[2]
        # if self.print_debugging_information:
        #     # print("frame:", np.shape(frame))
            # print("ROI_center: ", ROI_center)
            # print("ROI_dis: ", ROI_dis)
        
        print("Received image")

        # Preprocess the frame image
        preprocessed_frame = self.preprocess_image(frame)

        # Predict the positions of the instruments' tip, instruments' shadow, and the shaft line of the surgical instrument
        (ratio, tip_dis, shaft_dis, tip_instrument, tip_shadow, point_instrument
        ) = self.predict_and_mark(preprocessed_frame, ROI_dis)

        # Publish the calculated distances
        msg = ImgShowMsg(value=[tip_dis, shaft_dis])
        msg.header.stamp = header_time
        self.publisher_distance_.publish(msg)

        # Calculate the relative instrument tip position in the ROI image
        left_top = (ROI_center - np.array([ROI_dis, ROI_dis])).astype('uint64')
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

    
    def predict_ROI(self, frame, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height):
        """
        Predict the region of interest (ROI) in the frame and smooth the results using a simple moving average (SMA) filter
        """
        x_img = self.preprocessing(image=frame)['image']
        x_tensor = torch.from_numpy(x_img).to(self.DEVICE).unsqueeze(0)
        pr_mask = self.roi_model.predict(x_tensor) # torch.tensor
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

        # Unravel indecies of the maximum value in each channel and calculate center, max distance
        tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
        tip_shadow = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
        center = self.get_ROI_center(tip_instrument, tip_shadow)
        max_dis = self.max_distance(tip_instrument, tip_shadow, center)

        # Adjust point to original image size considering cropped area
        original_x, original_y = self.scale_keypoints(center[1], center[0], new_width, new_height, self.original_w, self.original_h)
        adjusted_x = original_x + crop_width_left * self.original_w / resize_width 
        adjusted_y = original_y + crop_height_top * self.original_h / resize_height
        ROI_center = np.array([adjusted_x, adjusted_y])

        # If there's a previous center and the change is too large, ignore this center
        if self.previous_centers and np.linalg.norm(ROI_center - self.previous_centers[-1]) > self.parameters.outlier_threshold_roi:
            self.freeze_counter += 1
            if self.freeze_counter >= self.parameters.freeze_threshold:
                self.freeze_counter = 0
                self.previous_centers.clear()
                return ROI_center, max_dis
            return self.previous_centers[-1], max_dis
        
        self.freeze_counter = 0 

        # Add the new center to the deque and compute the average
        self.previous_centers.append(ROI_center)
        smooth_ROI_center = np.mean(self.previous_centers, axis=0)

        return smooth_ROI_center, max_dis
    
    @staticmethod
    def preprocess_image(image):
        # # Convert the image to grayscale if it's not already
        # if len(image.shape) == 3:
        #     image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # # Apply bilateral filter for noise reduction
        # image = cv2.GaussianBlur(image, (5, 5), 0)
        image = cv2.bilateralFilter(image, 6, 100, 50)
        # # image = cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)

        # # Apply histogram equalization for contrast enhancement
        # image = cv2.equalizeHist(image)

        # # Apply CLAHE (Contrast Limited Adaptive Histogram Equation) to the image
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)                # Convert the image to the LAB color space
        l, a, b = cv2.split(lab)                                    # Split the LAB image into L, A and B channels
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(6,6))  # Apply histogram equalization to the L channel
        l = clahe.apply(l)
        lab = cv2.merge((l, a, b))                                  # Merge the equalized L channel back with the A and B channels
        image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)                # Convert the image back to the BGR color space
        
        # # Min-Max normalization to [0, 1]
        image = (image - np.min(image)) / (np.max(image) - np.min(image))
        # # image = (image - np.mean(image)) / np.std(image)
        # # image = (image - np.mean(image)) / (np.max(image) - np.min(image))
        image = (image * 255).astype(np.uint8)  # Convert the image back to 8-bit

        # # Apply Sobel edge detection
        sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=9)
        sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=9)
        edges = np.hypot(sobelx, sobely)
        edges *= 255.0 / np.max(edges)
        edges = edges.astype(np.uint8)  # Convert the image back to 8-bit
        image = cv2.addWeighted(image, 0.7, edges, 0.3, 0)

        return image
    

    def predict_and_mark(self, resized_img, ROI_dis):
        """
        predict_and_mark(self, resized_img, ROI_dis) predicts the positions of the instruments' tips
        and the shaft line of the surgical instrument, marks these values on the received image for showing,
        calculates the shaft distance and tip distance for autonomous positioning, and publishes the calculated distances.
        """
        # Predict the tips' positions and the shaft line
        ratio = ROI_dis / self.predict_size * 2
        x_img = self.preprocessing(image=resized_img)['image']
        x_tensor = torch.from_numpy(x_img).to(self.DEVICE).unsqueeze(0)
        pr_mask = self.best_model.predict(x_tensor)  # torch.tensor
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy())*255, (1, 2, 0))

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
                cv2.imwrite(SAVE_DIR + str(self.i + 1) + '-test_predict3' + '.png', pr_mask[:, :, 2])

        # Get the predicted positions
        tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
        tip_shadow = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
        point_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 2]), pr_mask[:, :, 2].shape))
        center = self.get_ROI_center(tip_instrument, tip_shadow)

        #self.max_dis = self.max_distance(tip_instrument, tip_shadow, center) * ratio

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
            # print('Exceeding the threshold@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
            point_instrument = self.previous_point_point_instrument[-1]
        
        # Calculate the EMA of the points (smoothing)
        tip_shadow = self.EMA(self.previous_point_shadow, self.parameters.alpha_shadow)
        point_instrument = self.EMA(self.previous_point_point_instrument, self.parameters.alpha_point)

        # Calculate the shaft distance and the tip distance---Section VI and VII of Koyama et al. (2022)
        vec_1 = point_instrument - tip_instrument
        vec_2 = tip_shadow - tip_instrument
        shaft_dis = np.linalg.norm(vec_1.dot(vec_2)/(vec_1.dot(vec_1))*vec_1-vec_2)*ratio
        tip_dis = int(np.linalg.norm(tip_instrument-tip_shadow)*ratio)

        return ratio, tip_dis, shaft_dis, tip_instrument, tip_shadow, point_instrument

    @staticmethod
    def resize_image( image, parameters, width, height):
        """
        Resize the image to the specified size and crop out the image if size is not divisible by the stride of 16
        """
        # Resize the image to the specified size    
        resize_height = parameters.ROI_resize_size
        resize_width = int((parameters.ROI_resize_size * width) / height)
        image = cv2.resize(image, (resize_width, parameters.ROI_resize_size))

        # Crop out image if size is not divisible by the stride of 16 
        crop_height = resize_height % 16
        crop_width = resize_width % 16
        crop_height_top = crop_height // 2
        crop_height_bottom = crop_height - crop_height_top
        crop_width_left = crop_width // 2
        crop_width_right = crop_width - crop_width_left

        image = image[crop_height_top:resize_height-crop_height_bottom, crop_width_left:resize_width-crop_width_right]
        new_width, new_height = image.shape[1], image.shape[0]
        return image, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height

    @staticmethod
    def scale_keypoints(old_x, old_y, old_w, old_h, new_w, new_h):
        """
        Scale the keypoints to the original image size
        """
        new_x = old_x * (new_w / old_w)
        new_y = old_y * (new_h / old_h)
        return new_x, new_y

    @staticmethod
    def get_ROI_center(tip_instrument, tip_shadow):
        """
        get_ROI_center(self, tip_instrument, tip_shadow) returns the middle point of the instrument's tip and the
        shadow's tip as the center point of the next ROI image.
        """
        center = (tip_instrument + tip_shadow) / 2
        return center

    @staticmethod
    def max_distance(point1, point2, center):
        """
        max_distance(self, point1, point2, center) returns the longest distance of all the distances from the center
        to each point. This longest distance is used to decide the size of the next ROI image.
        """
        dis1 = np.linalg.norm(center-point1)
        dis2 = np.linalg.norm(center - point2)
        distances = np.array([dis1, dis2])
        return distances.max()

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
