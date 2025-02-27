#!/usr/bin/python3
# Import original files and messages
from tools import img_parameters
from robot_control.msg import ImgShowMsg
from robot_control.msg import CImage

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

# Import files from SAS library
from sas_datalogger import DataloggerInterface


def main():
    # Init "predict_4K" node
    rospy.init_node('predict_ROI')
    predict_node = PredictROINode()
    rospy.spin()


class PredictROINode:
    """
    PredictROINode subscribes microscopic images from "microscope" node, predicts the ROI center and max distance, and publishes
    the ROI parameters through the "predict" node. This provides better ROI for lower IoU keypoint detection models
    """
    def __init__(self):
        # load parameters from "img_parameters"
        self.parameters = img_parameters.ImageParameters
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load the ROI model and move to the GPU
        self.roi_model = torch.load(self.parameters.ROI_model)
        self.roi_model.to(self.device)
        self.roi_model.eval() # Set the model to evaluation mode
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

        # Set image parameters
        self.original_w = self.parameters.original_w
        self.original_h = self.parameters.original_h
        self.predict_size = self.parameters.predict_size
        self.output_size = self.parameters.output_size

        # Set ROI thresholds
        self.threshold1 = self.parameters.threshold1
        self.threshold2 = self.parameters.threshold2
        self.threshold3 = self.parameters.threshold3

        # ROI parameters
        self.previous_centers = deque(maxlen=self.parameters.SMA_smoothing_N)
        self.freeze_counter = 0
        self.margin = self.parameters.margin

        # Logging and debugging parameters
        self.print_time = self.parameters.print_predict_node_time
        self.print_time_from_get_to_predict = self.parameters.print_time_from_get_to_predict
        self.print_debugging_information = self.parameters.print_debugging_information

        self.cframe_pub_count = 0

        # Launch ROS publishers and ROS Subscribers
        self.publisher_ROI_parameter_ = rospy.Publisher("predict/ROI_parameter", ImgShowMsg, queue_size=1)
        self.subscriber_compressed_image_ = rospy.Subscriber("microscope/compressed_frame", CImage, self._get_compressed_image)

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

        # Store the compressed image information
        resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height = msg.cframe_values
        if self.print_debugging_information:
            print("frame:", np.shape(frame))
            print("resize_width: ", resize_width)
            print("resize_height: ", resize_height)
            print("crop_width_left: ", crop_width_left)
            print("crop_height_top: ", crop_height_top)
            print("new_width: ", new_width)
            print("new_height: ", new_height)
            print("Received compressed image")

        # Add frame to queue
        if len(self.frame_queue) < self.frame_queue.maxlen:
            self.frame_queue.append((frame, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height, header_time, start))
        else:
            self.frame_queue[0] = (frame, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height, header_time, start)        
        
        
    def process_frames(self):
        """
        Process the frames in the queue asynchronously.
        """
        while True:
            if len(self.frame_queue) > 0:
                frame_data = self.frame_queue.popleft()
                frame, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height, header_time, start = frame_data
        
                # Predict the region of interest (ROI) in the frame and calculate max distance
                ROI_center, self.max_dis, shaft_point_overall = self.predict_ROI(
                    frame, self.original_w, self.original_h, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height)

                # Check if the keypoints are are within the ROI and adjust the size of the ROI if necessary
                if self.max_dis + self.margin <= self.threshold1 / 2:
                    ROI_half_dis = self.threshold1 / 2
                elif self.max_dis + self.margin <= self.threshold2 / 2:
                    ROI_half_dis = self.threshold2 / 2
                else:
                    ROI_half_dis = self.threshold3 / 2

                if ROI_center[0] - ROI_half_dis < 0:
                    ROI_center[0] = ROI_half_dis
                if ROI_center[1] - ROI_half_dis < 0:
                    ROI_center[1] = ROI_half_dis
                if ROI_center[0] + ROI_half_dis >= self.original_w:
                    ROI_center[0] = self.original_w - ROI_half_dis
                if ROI_center[1] + ROI_half_dis >= self.original_h:
                    ROI_center[1] = self.original_h - ROI_half_dis

                # Convert the ROI center and distance to integers and calculate the left top and right bottom corners of the ROI
                ROI_center = ROI_center.astype('uint64')
                left_top = ROI_center - np.array([ROI_half_dis, ROI_half_dis])
                left_top = left_top.astype('uint64')

                # Calculate the shaft_point position relative to the ROI frame
                ratio = ROI_half_dis / self.predict_size * 2
                tip_instrument_scaled = shaft_point_overall - left_top
                point_inst_in_ROI = np.array([tip_instrument_scaled[1] / ratio, tip_instrument_scaled[0] / ratio])

                # Publish the ROI points
                ROI_points = np.vstack([ROI_center.reshape([2, 1]),
                                        ROI_half_dis,
                                        point_inst_in_ROI.reshape([2, 1]),
                                        shaft_point_overall.reshape([2, 1])])
                
                msg_ROI_parameter = ImgShowMsg(value=ROI_points)
                msg_ROI_parameter.header.stamp = header_time
                self.publisher_ROI_parameter_.publish(msg_ROI_parameter)

                # Save calculation time for debug
                self.data_logger.log("time_capture2predict", float(str(rospy.Time.now()-header_time)))
                self.data_logger.log("hz_predict", float(1/(time.time()-start)))

                if self.print_time_from_get_to_predict:
                    print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-header_time)+" nsec")

                if self.print_time:
                    print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")    

    
    def predict_ROI(self, frame, original_w, original_h, resize_width, resize_height, crop_width_left, crop_height_top, new_width, new_height):
        """
        Predict the region of interest (ROI) in the frame and smooth the results using a simple moving average (SMA) filter
        """
        # Preprocess the frame
        frame = self.preprocessing(image=frame)['image']

        # Convert the frame to a tensor and move to GPU
        frame_tensor = torch.from_numpy(frame).float().to(self.device).unsqueeze(0)
        
        # Forward pass the frame through the ROI model
        if self.parameters.mixed_precision:
             with torch.no_grad():
                with torch.cuda.amp.autocast():
                    pr_mask = self.roi_model(frame_tensor)  # torch.tensor
        else:    
            with torch.no_grad():
                pr_mask = self.roi_model.predict(frame_tensor)  # torch.tensor

        # Convert the tensor to a numpy array and transpose the dimensions
        pr_mask = np.transpose((pr_mask.squeeze().cpu().numpy()) * 255, (1, 2, 0))

        # Unravel indecies of the maximum value in each channel and calculate center, max distance
        tip_instrument = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 0]), pr_mask[:, :, 0].shape))
        tip_shadow = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 1]), pr_mask[:, :, 1].shape))
        shaft_point_unadjusted = np.array(np.unravel_index(np.argmax(pr_mask[:, :, 2]), pr_mask[:, :, 2].shape))
        ROI_center_unadjusted = self.get_ROI_center(tip_instrument, tip_shadow)
        max_dis = self.max_distance(tip_instrument, tip_shadow, ROI_center_unadjusted)

        # Adjust point to original image size considering cropped area
        (ROI_adjusted_x, ROI_adjusted_y
        ) = self.scale_keypoints(ROI_center_unadjusted[1], ROI_center_unadjusted[0], new_width, new_height, original_w, original_h, crop_width_left, crop_height_top, resize_width, resize_height)
        (point_inst_adjusted_x, point_inst_adjusted_y
        ) = self.scale_keypoints(shaft_point_unadjusted[1], shaft_point_unadjusted[0], new_width, new_height, original_w, original_h, crop_width_left, crop_height_top, resize_width, resize_height)
        ROI_center = np.array([ROI_adjusted_x, ROI_adjusted_y])
        shaft_point = np.array([point_inst_adjusted_x, point_inst_adjusted_y])

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

        return smooth_ROI_center, max_dis, shaft_point
   

    @staticmethod
    def scale_keypoints(old_x, old_y, old_w, old_h, new_w, new_h, crop_w_l, crop_h_t, resize_w, resize_h):
        """
        Scale the keypoints to the original image size
        """
        new_x = old_x * (new_w / old_w)
        new_y = old_y * (new_h / old_h)
        adjusted_x = new_x + crop_w_l * new_w / resize_w 
        adjusted_y = new_y + crop_h_t * new_h / resize_h

        return adjusted_x, adjusted_y


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
