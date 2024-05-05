#!/usr/bin/python3
# Import files for image processing
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rosilo_datalogger.msg import AddValueMsg

# Import original files
import img_parameters
import contact_reporter_interface
import positioning_point_interface

# Import files from rosilo library
from rosilo_datalogger import DataloggerInterface

# Import other dependencies
import numpy as np
import rospy
import datetime
import time
import cv2


def main():
    # Init "img_show" node
    rospy.init_node('img_show')
    ing_show_node = ImgShowNode()
    rospy.spin()
    ing_show_node.video.release()
    print("[" + rospy.get_name() + "]:: Video was saved!")


class ImgShowNode:
    """
    This class shows microscopic view with its predicted image and saves it as video data.
    """
    def __init__(self):
        # Load parameters
        self.parameters = img_parameters.ImageParameters

        self.target_points_interface = positioning_point_interface.PositioningPointInterface()

        self.fontsize = self.parameters.fontsize*1.2
        self.predict_size = self.parameters.predict_size
        self.put_contact_x = int(self.predict_size * 250 / 512)
        self.put_contact_y = int(self.predict_size * 50 / 512)

        original_w = self.parameters.original_w
        original_h = self.parameters.original_h
        output_size = self.parameters.output_size

        self.print_time = self.parameters.print_image_show_time
        self.print_time_get2show = self.parameters.print_time_from_get_to_show
        self.print_center = self.parameters.print_center       

        # Prepare data_logger for logging
        self.data_logger = DataloggerInterface(10)

        # Launch ROS subscribers
        self.subscriber_ROI_image_ = rospy.Subscriber("img_show/ROI", Image, self._img_ROI_callback)
        self.subscriber_output_image_ = rospy.Subscriber("img_show/overview", Image, self._img_show_callback)
        self.subscriber_positioning_point_ = rospy.Subscriber("predict/positioning_point", AddValueMsg, self._positioning_point_callback)
        self.subscriber_tip_instrument_ = rospy.Subscriber("predict/tip_instrument", AddValueMsg, self._tip_instrument_callback)

        self.bridge = CvBridge()
        self.bridge_to_cv2 = self.bridge.imgmsg_to_cv2
        self.overview_image = np.array([])
        self.overview_ImgMsg_stamp = 0

        self.center_point = np.zeros([1, 2])
        self.instrument_tip = np.zeros([1, 2])

        # contact_reporter_interface
        self.contact_reporter_interface = contact_reporter_interface.ContactReporterInterface()

        # Prepare a video writer
        fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
        now = datetime.datetime.now()
        file_name = now.strftime('%Y_%m%d_%H%M')
        video_path = "/home/yuki/Videos/autonomous_positioning/"+file_name+".mp4"
        self.fps = self.parameters.video_save_fps
        self.video = cv2.VideoWriter(video_path, fourcc, self.fps, (int(original_w / original_h * output_size)+output_size, output_size))

        rospy.set_param("next_point", [0])
        rospy.set_param("step", ["Waiting"])

        print("[" + rospy.get_name() + "]:: Ready!")

    def _positioning_point_callback(self, msg):
        value = msg.value
        self.instrument_tip[0][0] = value[0]
        self.instrument_tip[0][1] = value[1]

    def _tip_instrument_callback(self, msg):
        value = msg.value
        self.center_point[0][0] = value[0]
        self.center_point[0][1] = value[1]

    def _img_show_callback(self, msg):
        """
        _img_show_callback(self, msg) stores the overview image received from "img_show/overview" node into class's
        variable "self.overview_image".
        """
        # Get values from the message and stores them
        self.overview_image = self.bridge_to_cv2(msg, "bgr8")
        self.overview_ImgMsg_stamp = msg.header.stamp
        if self.print_time_get2show:
            print("[" + rospy.get_name() + "]::overview: " + str(rospy.Time.now()-msg.header.stamp)+" nsec")

    def _img_ROI_callback(self, msg):
        """
        _img_ROI_callback(self, msg) receives the predicted ROI image, shows it with the overview image, and writes
        it to the video.
        """
        start = time.time()

        # Receive the predicted ROI image
        ROI_image = self.bridge_to_cv2(msg, "bgr8")

        if self.print_time_get2show:
            Hz_get2show = 1/(float(str(rospy.Time.now()-msg.header.stamp))*10**(-9))
            print("[" + rospy.get_name() + "]::ROI     : " + str(Hz_get2show)+" nsec")

        if self.contact_reporter_interface.contact:
            cv2.putText(ROI_image, 'Contact!!', (self.put_contact_x, self.put_contact_y), cv2.FONT_HERSHEY_SIMPLEX, self.fontsize, (0, 255, 0), 2)

        cv2.circle(self.overview_image, (int(self.target_points_interface.center[0][0]), int(self.target_points_interface.center[0][1])),
                   int(self.target_points_interface.radius), (255, 0, 0), 2)

        planar_error = np.linalg.norm(self.center_point-self.instrument_tip)
        step = rospy.get_param("step")

        cv2.putText(self.overview_image, 'Planar_error:' + str(int(planar_error)) + 'px', (10, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, self.fontsize, (255, 255, 0), 1)

        cv2.putText(self.overview_image, 'Step:' + step[0], (230, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, self.fontsize, (255, 255, 0), 1)

        # Generate an output image and show it
        output_image = cv2.hconcat([self.overview_image, ROI_image])

        cv2.imshow('microscopic_image', output_image)
        cv2.waitKey(1)

        # Write the output image to the video
        self.video.write(output_image)

        # For debug
        Hz_capture2show = 1/(float(str(rospy.Time.now()-msg.header.stamp))*10**(-9))
        self.data_logger.log("Hz_capture2show", Hz_capture2show)
        self.data_logger.log("hz_show", float(1/(time.time()-start)))

        if self.print_time_get2show:
            print("[" + rospy.get_name() + "]::show    : " + str(Hz_capture2show) + " Hz")

        if self.print_time:
            print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")

    def draw_circles(self, points_centers, points_radius, image, next_point):
        cv2.circle(image, (int(points_centers[next_point][0]), int(points_centers[next_point][1])), int(points_radius[next_point]), (255, 0, 0), 2)
        cv2.circle(image, (int(points_centers[next_point][0]), int(points_centers[next_point][1])), 3, (0, 255, 0), -1)
        cv2.putText(image, 'P' + str(next_point + 1), (int(points_centers[next_point][0]), int(points_centers[next_point][1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return image


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
