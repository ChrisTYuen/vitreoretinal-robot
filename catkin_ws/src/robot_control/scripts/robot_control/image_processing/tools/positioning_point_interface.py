# Import files from rosilo library
from rosilo_datalogger.msg import AddValueMsg
import numpy as np
import img_parameters

import rospy


class PositioningPointInterface:
    """

    """
    def __init__(self):
        self.parameters = img_parameters.ImageParameters
        self.subscriber_output_image_ = rospy.Subscriber("predict/positioning_point", AddValueMsg, self._positioning_points_callback)
        # self.subscriber_next_point_ = rospy.Subscriber("predict/next_point", AddValueMsg, self._next_point_callback)
        self.center = np.zeros([1, 2])
        self.radius = np.zeros([1, 1])
        self.ratio = self.parameters.original_h/self.parameters.output_size
        self.next_point = 0

    def _positioning_points_callback(self, msg):
        self.center[0][0] = msg.value[0]/self.ratio
        self.center[0][1] = msg.value[1]/self.ratio
        self.radius[0][0] = msg.value[2]/self.ratio

    # def _next_point_callback(self, msg):
    #     self.next_point = int(msg.value[0])
    #     print(self.next_point)
