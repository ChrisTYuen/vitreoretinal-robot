from dqrobotics import *
import numpy as np
import rospy
from robot_control.msg import ImgShowMsg
from sas_datalogger import DataloggerInterface


class PredictInterface:
    def __init__(self):
        self.tip_distance = 100
        self.shaft_distance = 100
        self.counter_sum = 0
        self.data_logger = DataloggerInterface(10)
        self.subscriber_distance_ = rospy.Subscriber("predict/distances", ImgShowMsg, self._predict_callback)

    def _predict_callback(self, msg):
        self.tip_distance = msg.value[0]
        self.shaft_distance = msg.value[1]
        self.data_logger.log("time_from_get_to_distance", float(str(rospy.Time.now()-msg.header.stamp)))
        # if self.parameter.print_time_from_get_to_distance:
        #     print("[" + rospy.get_name() + "]:: " + str(rospy.Time.now()-msg.header.stamp)+" nsec")
