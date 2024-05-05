# Import files from rosilo library
from rosilo_datalogger.msg import AddValueMsg
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np

from dqrobotics import *

import rospy


class PositioningPointInterface:
    """

    """
    def __init__(self, converter_per_mm):
        self.subscriber_output_image_ = rospy.Subscriber("predict/positioning_points", AddValueMsg, self._positioning_points_callback)
        self.subscriber_instrument_tip_ = rospy.Subscriber("predict/tip_positions", AddValueMsg,  self._tip_positions_callback)
        self.publisher_lock_status_ = rospy.Publisher("predict/lock_or_not", Bool, queue_size=1)
        self.publisher_current_step_ = rospy.Publisher("predict/current_step", String, queue_size=10)
        self.publisher_planar_error_ = rospy.Publisher("predict/planar_error", AddValueMsg, queue_size=10)
        self.point_center = np.zeros([5, 2])
        self.target_point_positions = np.zeros([5, 4])
        self.instrument_tip = np.zeros([1, 2])
        self.point_lock = False

        self.converter_per_mm = converter_per_mm

    def _positioning_points_callback(self, msg):
        if not self.point_lock:
            self.point_center[0][0] = msg.value[0]
            self.point_center[0][1] = msg.value[1]
            self.point_center[1][0] = msg.value[2]
            self.point_center[1][1] = msg.value[3]
            self.point_center[2][0] = msg.value[4]
            self.point_center[2][1] = msg.value[5]
            self.point_center[3][0] = msg.value[6]
            self.point_center[3][1] = msg.value[7]
            self.point_center[4][0] = msg.value[8]
            self.point_center[4][1] = msg.value[9]

    def lock_desired_point(self):
        self.point_lock = True
        self.publisher_lock_status_.publish(self.point_lock)
        rospy.set_param("/target_lock", True)

    def unlock_desired_point(self):
        self.point_lock = False
        self.publisher_lock_status_.publish(self.point_lock)
        rospy.set_param("/target_lock", False)

    def publish_current_step(self, step_num):
        if step_num == 0:
            rospy.set_param("/step", "Waiting")
            self.publisher_current_step_.publish("Waiting")
        elif step_num == 1:
            rospy.set_param("/step", "Planar positioning")
            self.publisher_current_step_.publish("Planar positioning")
        elif step_num == 2:
            rospy.set_param("/step", "Overlap prevention")
            self.publisher_current_step_.publish("Overlap prevention")
        elif step_num == 3:
            rospy.set_param("/step", "Vertical positioning")
            self.publisher_current_step_.publish("Vertical positioning")
        elif step_num == 4:
            rospy.set_param("/step", "Pause")
            self.publisher_current_step_.publish("Pause")
        elif step_num == 5:
            rospy.set_param("/step", "Additional positioning")
            self.publisher_current_step_.publish("Additional positioning")
        elif step_num == 6:
            rospy.set_param("/step", "Returning")
            self.publisher_current_step_.publish("Returning")

    def _tip_positions_callback(self, msg):
        self.instrument_tip[0][0] = msg.value[6]
        self.instrument_tip[0][1] = msg.value[7]

    def get_translation_error(self, target_pixel, velocity, tau):
        error = (target_pixel - self.instrument_tip[0, :])/self.converter_per_mm*10**(-3)
        error = error[0]*i_ - error[1]*j_
        trajectory_length = np.linalg.norm(vec4(error))
        total_iteration = (trajectory_length * 10 ** 3 / velocity) // tau
        return error, total_iteration

    def get_planar_error(self, target_pixel):
        planar_error = np.linalg.norm(target_pixel - self.instrument_tip[0, :])
        msg = AddValueMsg(value=np.array([planar_error]))
        self.publisher_planar_error_.publish(msg)
        return planar_error

    def get_workspace(self, t_si):
        for i in range(5):
            relative_xy = (self.point_center[i, :] - self.instrument_tip[0, :])/self.converter_per_mm*10**(-3)
            self.target_point_positions[i, :] = vec4(t_si + relative_xy[0]*i_ - relative_xy[1]*j_).reshape([1, 4])


