# Import files from SAS library
from sas_datalogger.msg import AddValueMsg
from std_msgs.msg import Bool
from std_msgs.msg import String
import numpy as np

from dqrobotics import *

import rospy


class PositioningPointInterface:
    """
    This class is used to interface with the positioning points and instrument tip positions. The target points are retrieved from the
    predict/positioning_points topic and the instrument tip positions are retrieved from the predict/tip_positions topic. The class publishes
    the lock status of the target points to the predict/lock_or_not topic. When unlocked, the desired points are open for detection. The 
    class also publishes the current step to the predict/current_step. The translation and planar errors are calculated and published to the
    predict/planar_error topic. The class also calculates the workspace of the instrument tip and the target points.
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
            for i in range(5):                                # 5 points          
                self.point_center[i][0] = msg.value[2*i]      # x-coordinate
                self.point_center[i][1] = msg.value[2*i + 1]  # y-coordinate

    def lock_desired_point(self):
        self.point_lock = True
        self.publisher_lock_status_.publish(self.point_lock)
        rospy.set_param("/target_lock", True)

    def unlock_desired_point(self):
        self.point_lock = False
        self.publisher_lock_status_.publish(self.point_lock)
        rospy.set_param("/target_lock", False)

    def publish_current_step(self, step_num):
        step_messages = {
            0: "Waiting",
            1: "Planar positioning",
            2: "Overlap prevention",
            3: "Vertical positioning",
            4: "Pause",
            5: "Additional positioning",
            6: "Returning"
        }

        # Set the parameter and publish the message based on step_num
        message = step_messages.get(step_num, "Unknown Step")
        rospy.set_param("/step", message)
        self.publisher_current_step_.publish(message)

    def _tip_positions_callback(self, msg):
        self.instrument_tip[0][0] = msg.value[6]
        self.instrument_tip[0][1] = msg.value[7]

    def get_translation_error(self, target_pixel, velocity, tau):
        print("instrument tip: ", self.instrument_tip)
        error = (target_pixel - self.instrument_tip[0, :])/self.converter_per_mm*10**(-3)
        error = error[0]*i_ - error[1]*j_
        trajectory_length = np.linalg.norm(vec4(error))
        total_iteration = (trajectory_length * 10 ** 3 / velocity) // tau
        return error, total_iteration

    def get_planar_error(self, target_pixel):
        print("instrument tip: ", self.instrument_tip)
        planar_error = np.linalg.norm(target_pixel - self.instrument_tip[0, :])
        msg = AddValueMsg(value=np.array([planar_error]))
        self.publisher_planar_error_.publish(msg)
        return planar_error

    def get_workspace(self, t_si):
        for i in range(5):
            relative_xy = (self.point_center[i, :] - self.instrument_tip[0, :])/self.converter_per_mm*10**(-3)
            self.target_point_positions[i, :] = vec4(t_si + relative_xy[0]*i_ - relative_xy[1]*j_).reshape([1, 4])


