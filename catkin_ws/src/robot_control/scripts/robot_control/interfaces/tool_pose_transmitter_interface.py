import socket
import time

import dqrobotics as dql

import rospy
from sas_operator_side_receiver import OperatorSideReceiverInterface


class ToolPoseTransmitterInterface:
    def __init__(self, config):
        self.socket = None
        self.config = config

        rospy.logwarn("[" + rospy.get_name() + "]::ToolPoseTransmitterInterface::Started")
        # Initialize the OperatorSideReceiverInterface
        self.osri = OperatorSideReceiverInterface()
        self.arm_manipulator_ns_s = [config['arm1_manipulator_ns'], config['arm2_manipulator_ns']]
        # Each master manipulator will have a label assigned to it.
        for manipulator_ns in self.arm_manipulator_ns_s:
            self.osri.add_manipulator_manager(manipulator_ns)
        # If you want any information from that master, retrieve it first
        self.manipulators = [self.osri.get_manipulator_manager_ptr(manipulator_ns)
                             for manipulator_ns in self.arm_manipulator_ns_s]

        self.target_ip = config['operator_side_ips'][0]
        self.target_port = config['operator_side_ports'][0]

    def initialize(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((self.target_ip, self.target_port))
        rospy.logwarn(
            "[" + rospy.get_name() + "]::ToolPoseTransmitterInterface::CONNECTED TO {}:{}".format(self.target_ip,
                                                                                                  self.target_port))
        for manipulator in self.manipulators:
            while not manipulator.is_enabled():
                rospy.sleep(0.1)
        rospy.loginfo("[" + rospy.get_name() + "]::ToolPoseTransmitterInterface::manipulator enabled")
        
    def send_tool_pose(self):
        arm_strs = []
        for i, manipulator in enumerate(self.manipulators):
            arm_t = 1000 * dql.translation(manipulator.get_pose()).vec3()  # tx ty tz, In millimeters
            arm_r = dql.rotation(manipulator.get_pose()).vec4()  # rw rx ry rz
            # print("arm_r: ", arm_r)
            arm_g = manipulator.get_button()  # button 0, 1, or 2
            arm_str = [i, arm_t[0], arm_t[1], arm_t[2], arm_r[0], arm_r[1], arm_r[2], arm_r[3], arm_g]
            # tx ty tz rx ry rz rw g
            arm_strs.append(arm_str)

        return arm_strs
        
    def __del__(self):
        if self.socket is not None:
            self.socket.close()
