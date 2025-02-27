#!/usr/bin/python3
import rospy
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tools import img_parameters
from robot_control.msg import ImgShowMsg
from robot_control.msg import ROI
import time
import numpy as np

def main():
    rospy.init_node('video_node', disable_signals=True)
    video_node = VideoNode()
    rospy.spin()

class VideoNode:
    """
    This class shows and extracts the frames from the video and publishes the ROI image and the overview image.
    """
    def __init__(self):
        # VIDEO ABSOLUTE OR RELATIVE PATH
        self.parameters = img_parameters.ImageParameters()
        video_dir = self.parameters.video_dir
        ids = os.listdir(video_dir)
        print(ids)
        video_fps = [os.path.join(video_dir, video_id) for video_id in ids]
        self.publisher_ROI_frame = rospy.Publisher("microscope/capture",ROI,queue_size=1)
        self.publisher_overview_frame = rospy.Publisher("img_show/overview",Image,queue_size=1)
        self.subscriber_ros_parameter_ = rospy.Subscriber("predict/ROI_parameter", ImgShowMsg, self._get_ROI_parameter)

        self.bridge = CvBridge()
        self.bridge_to_imgmsg = self.bridge.cv2_to_imgmsg
        self.print_time = self.parameters.print_video_node_time
        self.original_w = self.parameters.original_w
        self.original_h = self.parameters.original_h
        self.ROI_center = np.array([self.original_w/2, self.original_h/2])
        self.ROI_half_size = self.parameters.threshold2 / 2
        self.predict_size = self.parameters.predict_size
        self.output_size = self.parameters.output_size

        print("[" + rospy.get_name() + "]::Ready!")

        video_number = 0

        while video_number < len(video_fps):
            print('Opening video...')
            cap = cv2.VideoCapture(video_fps[video_number])
            total_frame = cap.get(cv2.CAP_PROP_FRAME_COUNT)

            i = 0
            r = rospy.Rate(60)
            while i < total_frame:
                start = time.time()
                i = i + 1
                if cap.isOpened():
                    ret, frame = cap.read()

                    ROI_center = self.ROI_center
                    ROI_half_size = self.ROI_half_size

                    left_top = (ROI_center - np.array([ROI_half_size, ROI_half_size])).astype('uint64')
                    right_bottom = (ROI_center + np.array([ROI_half_size, ROI_half_size])).astype('uint64')
                    frame_interested = frame[left_top[1]:right_bottom[1], left_top[0]:right_bottom[0]]
                    frame_interested = cv2.resize(frame_interested, (self.predict_size, self.predict_size))

                    cv2.rectangle(frame, (left_top[0], left_top[1]), (right_bottom[0], right_bottom[1]), (0, 255, 0), thickness=10)
                    frame_resize = cv2.resize(frame, (int(self.original_w / self.original_h * self.output_size), self.output_size))

                    # imgMsgROI = self.bridge_to_imgmsg(frame_interested, "bgr8")
                    # imgMsgROI.header.stamp = rospy.Time.now()
                    # self.publisher_ROI_frame.publish(imgMsgROI)

                    ROI_msg = ROI()
                    ROI_msg.roi_image = self.bridge_to_imgmsg(frame_interested, "bgr8")
                    ROI_msg.roi_image.header.stamp = rospy.Time.now()
                    ROI_msg.roi_values = [ROI_center[0],ROI_center[1],ROI_half_size]
                    self.publisher_ROI_frame.publish(ROI_msg)

                    imgMsgShow = self.bridge_to_imgmsg(frame_resize, "bgr8")
                    imgMsgShow.header.stamp = rospy.Time.now()
                    self.publisher_overview_frame.publish(imgMsgShow)

                    r.sleep()
                    if self.print_time:
                        print("[" + rospy.get_name() + "]:: " +str(int(1/(time.time()-start)))+" Hz")
            video_number = video_number + 1            

    def _get_ROI_parameter(self, msg):
        values = msg.value
        self.ROI_center[0] = values[0]
        self.ROI_center[1] = values[1]
        self.ROI_half_size = values[2]

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

