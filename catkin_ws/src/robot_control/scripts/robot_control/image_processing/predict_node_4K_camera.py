#!/usr/bin/python3
# Import original files and messages
from tools import img_parameters
from robot_control.msg import ImgShowMsg
from robot_control.msg import ROI
from rosilo_datalogger.msg import AddValueMsg

# Import files for image processing
import torch
import cv2
import segmentation_models_pytorch as smp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import albumentations as albu

# Import other dependencies
import numpy as np
import rospy
import time
import os

# Import files from rosilo library
from rosilo_datalogger import DataloggerInterface


def main():
    # Init "predict_4K" node
    rospy.init_node('predict_4K')
    predict_node = PredictNode()
    rospy.spin()


class PredictNode:
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
        print("loaded!")
        self.ENCODER = 'resnet18'
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

        self.put_text_x = int(self.predict_size*250/512)
        self.put_text_y_1 = int(self.predict_size*420/512)
        self.put_text_y_2 = int(self.predict_size*460/512)
        self.put_text_y_3 = int(self.predict_size*500/512)
        self.fontsize = self.parameters.fontsize

        self.max_dis = self.threshold1/2
        self.margin = self.parameters.margin

        self.print_time = self.parameters.print_predict_node_time
        self.print_time_from_get_to_predict = self.parameters.print_time_from_get_to_predict
        self.print_img_size = self.parameters.print_img_size
        self.print_center = self.parameters.print_center

        self.roi_pub_count = 0
        self.next_ROI_half_size = 0

        self.ROI_center_init = self.parameters.ROI_center_init

        self.roi_param_publish = False
        rospy.set_param("/roi_param_publish", False)

        # Launch ROS publishers and ROS Subscribers
        self.publisher_distance_ = rospy.Publisher("predict/distances", ImgShowMsg, queue_size=1)
        self.publisher_ROI_parameter_ = rospy.Publisher("predict/ROI_parameter", ImgShowMsg, queue_size=1)
        self.publisher_tip_positions_ = rospy.Publisher("predict/tip_positions", AddValueMsg, queue_size=1)
        self.publisher_output_image_ = rospy.Publisher("img_show/ROI", Image, queue_size=1)
        self.subscriber_microscopic_image_ = rospy.Subscriber("microscope/capture", ROI, self._get_image)

        self.i = 0

        print("[" + rospy.get_name() + "]::Ready!")

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
        if self.print_img_size:
            print(np.shape(frame))

        # Store the center and the size of the received ROI image
        ROI_center = np.array([msg.roi_values[0], msg.roi_values[1]])
        ROI_half_size = msg.roi_values[2]

        # Get the ROI image after prediction and the center of the next ROI image
        next_ROI_center = self.predict_and_mark(frame, header_time, ROI_center, ROI_half_size)

        # Resize and publish the output ROI image for showing
        # ROI_resize = cv2.resize(ROI_img, (self.output_size, self.output_size))
        # ImgMsg = self.bridge_to_imgmsg(ROI_resize, "bgr8")
        # ImgMsg.header.stamp = header_time
        # self.publisher_output_image_.publish(ImgMsg)

        # Check if the center of the next ROI image is available (if the next ROI is inside the original image)
        if self.roi_pub_count == 1:
            if self.max_dis + self.margin <= self.threshold1/2:
                self.next_ROI_half_size = self.threshold1/2
            elif self.max_dis + self.margin <= self.threshold2/2:
                self.next_ROI_half_size = self.threshold2/2
            else:
                self.next_ROI_half_size = self.threshold3/2
        elif self.roi_pub_count == 10:
            self.roi_pub_count = 0

        if next_ROI_center[0] < self.next_ROI_half_size:
            next_ROI_center[0] = self.next_ROI_half_size
        if next_ROI_center[0] + self.next_ROI_half_size >= self.original_w:
            next_ROI_center[0] = self.original_w - self.next_ROI_half_size
        if next_ROI_center[1] < self.next_ROI_half_size:
            next_ROI_center[1] = self.next_ROI_half_size
        if next_ROI_center[1] + self.next_ROI_half_size >= self.original_h:
            next_ROI_center[1] = self.original_h - self.next_ROI_half_size

        # Publish the center and the size of the next ROI image
        self.roi_param_publish = rospy.get_param("/roi_param_publish")
        if self.roi_param_publish:
            self.i = self.i + 1
            if self.i < 100:
                msg_roi_parameter = ImgShowMsg(value=[self.ROI_center_init[0], self.ROI_center_init[1], 768/2])
                msg_roi_parameter.header.stamp = header_time
                self.publisher_ROI_parameter_.publish(msg_roi_parameter)
            else:
                msg_roi_parameter = ImgShowMsg(value=[next_ROI_center[0], next_ROI_center[1], self.next_ROI_half_size])
                msg_roi_parameter.header.stamp = header_time
                self.publisher_ROI_parameter_.publish(msg_roi_parameter)

        # Save calculation time for debug
        self.data_logger.log("time_capture2predict", float(str(rospy.Time.now()-header_time)))
        self.data_logger.log("hz_predict", float(1/(time.time()-start)))

        if self.print_time_from_get_to_predict:
            print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-header_time)+" nsec")

        if self.print_time:
            print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")

    def predict_and_mark(self, resized_img, header_time, ROI_center, ROI_half_size):
        """
        predict_and_mark(self, resized_img, header_time, ROI_center, ROI_half_size) predicts the positions of
        the instruments' tips and the shaft line of the surgical instrument, marks these values on the received image
        for showing, calculates the shaft distance and tip distance for autonomous positioning, and publishes
        the calculated distances.
        """
        # Predict the tips' positions and the shaft line
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

        # Calculate how far each point is from each other to decide the size of the next ROI image
        ratio = ROI_half_size / self.predict_size * 2
        self.max_dis = self.max_distance(tip_instrument, tip_shadow, center) * ratio

        # Mark values on the received ROI image
        # cv2.line(resized_img, (point_instrument[1], point_instrument[0]), (tip_instrument[1], tip_instrument[0]), (255, 0, 0), 4)
        # cv2.circle(resized_img, (tip_instrument[1], tip_instrument[0]), 5, (255, 0, 0), -1)
        # cv2.circle(resized_img, (tip_shadow[1], tip_shadow[0]), 5, (0, 0, 255), -1)
        # if self.print_center:
        #     cv2.circle(resized_img, (int(center[1]), int(center[0])), 10, (0, 255, 0), -1)

        # Calculate the shaft distance and the tip distance---Section VI and VII of Koyama et al. (2022)
        vec_1 = point_instrument - tip_instrument
        vec_2 = tip_shadow - tip_instrument
        shaft_dis = np.linalg.norm(vec_1.dot(vec_2)/(vec_1.dot(vec_1))*vec_1-vec_2)*ratio
        tip_dis = int(np.linalg.norm(tip_instrument-tip_shadow)*ratio)

        # cv2.putText(resized_img, 'tip_dis:'+str(tip_dis)+'px', (self.put_text_x, self.put_text_y_1), cv2.FONT_HERSHEY_SIMPLEX, self.fontsize, (0, 0, 0), 2)
        # if shaft_dis < 1000:
        #     cv2.putText(resized_img, 'shaft_dis:'+str(int(shaft_dis))+'px', (self.put_text_x, self.put_text_y_2), cv2.FONT_HERSHEY_SIMPLEX, self.fontsize, (0, 0, 0), 2)
        # cv2.putText(resized_img, 'ROI_size:'+str(int(ROI_half_size*2))+'px', (self.put_text_x, self.put_text_y_3), cv2.FONT_HERSHEY_SIMPLEX, self.fontsize, (0, 0, 0), 2)

        # Publish the calculated distances
        msg = ImgShowMsg(value=[tip_dis, shaft_dis])
        msg.header.stamp = header_time
        self.publisher_distance_.publish(msg)

        # Calculate the center of the next ROI image
        left_top = (ROI_center - np.array([ROI_half_size, ROI_half_size])).astype('uint64')
        next_ROI_Center = left_top+np.array([int(center[1]*ratio), int(center[0]*ratio)])
        tip_instrument_overall = left_top + np.array([int(tip_instrument[1]*ratio), int(tip_instrument[0]*ratio)])

        predicted_points = np.vstack([tip_instrument.reshape([2, 1]),
                                      tip_shadow.reshape([2, 1]),
                                      point_instrument.reshape([2, 1]),
                                      tip_instrument_overall.reshape([2, 1])])

        msg_tip = AddValueMsg(name="predicted_points", value=predicted_points)
        self.publisher_tip_positions_.publish(msg_tip)

        return next_ROI_Center

    def get_ROI_center(self, tip_instrument, tip_shadow):
        """
        get_ROI_center(self, tip_instrument, tip_shadow) returns the middle point of the instrument's tip and the
        shadow's tip as the center point of the next ROI image.
        """
        center = (tip_instrument + tip_shadow) / 2
        return center

    def max_distance(self, point1, point2, center):
        """
        max_distance(self, point1, point2, center) returns the longest distance of all the distances from the center
        to each point. This longest distance is used to decide the size of the next ROI image.
        """
        dis1 = np.linalg.norm(center-point1)
        dis2 = np.linalg.norm(center - point2)
        distances = np.array([dis1, dis2])
        return distances.max()

    def get_preprocessing(self, preprocessing_fn):
        """
        get_preprocessing(preprocessing_fn) defines preprocessing for input images.
        """
        _transform = [
            albu.Lambda(image=preprocessing_fn),
            albu.Lambda(image=self.to_tensor, mask=self.to_tensor),
        ]
        return albu.Compose(_transform)

    def to_tensor(self, x, **kwargs):
        """
        to_tensor(x, **kwargs) transposes input images for prediction.
        """
        return x.transpose(2, 0, 1).astype('float32')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
