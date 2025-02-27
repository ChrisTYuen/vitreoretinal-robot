from dqrobotics import *
# from dqrobotics import DQ_SerialManipulatorDH
# from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics import DQ_JsonReader
from dqrobotics import *
import numpy as np
import math
import time

import rospy
# from sas_robot_driver_denso import sas_robot_driver_denso
from sas_robot_driver import RobotDriverProvider
from .moonshot_drill_robot_configuration import MoonshotDrillRobotConfiguration
from .MoonshotDrillVrepRobot import MoonshotDrillVrepRobot


class MoonshotDrillRobot:

    def __init__(self, configuration):
        self.configuration_ = configuration
        self.vrep_interface_ = None
        self.robot_ = None
        self.vrep_robot = None
        self.robot_driver_provider_joint_dofs_ = None
        self.robot_driver_denso_ = configuration.robot_driver_denso_configuration
        self.clock_ = configuration.thread_sampling_time_nsec

        self.robot_driver_provider_joint_dofs_.reset = RobotDriverProvider(rospy.get_name() + "/joints/")
        self.vrep_robot_ = configuration.vrep_robot

    def _set_target_joint_positions(self):
        if self.is_enabled():
            self.desired_joint_positions_vectorxd = self.robot_driver_provider_joint_dofs_.get_target_joint_positions()
        else:
            if not self.configuration_.vrep_readonly:
                self.vrep_robot.send_q_to_vrep(self.desired_joint_positions_vectorxd)

    def _send_joint_positions(self):
        self.joint_positions_vectorxd(6)
        if self.configuration_.use_real_robot:
            self.joint_positions_vectorxd = self.robot_driver_denso_.get_joint_positions()
            if not self.configuration_.use_real_robot:
                self.vrep_robot.send_q_to_vrep(self.joint_positions_vectorxd)
        else:
            self.joint_positions_vectorxd = self.vrep_robot.get_q_from_vrep()
        self.robot_driver_provider_joint_dofs_.send_joint_positions(self.joint_positions_vectorxd)

    def _send_joint_limits(self):
        self.robot_driver_provider_joint_dofs_.send_joint_limits(self.robot_.lower_q_limit(),
                                                                 self.robot_.upper_q_limit())

    def _send_reference_frame(self):
        self.robot_driver_provider_joint_dofs_.send_reference_frame(self.vrep_robot_.get_reference_frame())

    def control_loop(self):
        try:
            self._initialize()

            while not self._should_shutdown():
                self.clock_.update_and_sleep()
                self._set_target_joint_positions()
                self._send_joint_positions()
                self._send_joint_limits()
                self._send_reference_frame()

        except Exception as exp:
            print(rospy.get_name() + "::Error or exception caught::", exp)
        return 0

    def is_enabled(self):
        return self.robot_driver_provider_joint_dofs_.is_enabled()

    def _should_shutdown(self):
        return

    def _initialize(self):
        print(
            "[" + rospy.get_name() + "]::Getting robot information from: " + self.configuration_.robot_parameter_file_path)
        self.robot_ = DQ_JsonReader.get_from_json(self.configuration_.robot_parameter_file_path)
        print(
            "[" + rospy.get_name() + "]::Obtained robot information with DoF =" + self.robot_.get_dim_configuration_space())

        print("[" + rospy.get_name() + "]::Connecting to VREP...")
        connect = self.vrep_interface_.connect(self.configuration_.vrep_ip, self.configuration_.vrep_port, 100, 10)
        print("V-REP connect is", connect)
        if connect is not True:
            self.vrep_interface_.disconnect_all()
            raise Exception(
                "[" + rospy.get_name() + "]:: Unable to connect to V-REP ip "
                + self.configuration_.vrep_ip
                + " port "
                + str(self.configuration_.vrep_port)
            )
        self.vrep_robot_.reset = MoonshotDrillVrepRobot(self.configuration_.vrep_robot_name, self.vrep_interface_)
        print("[" + rospy.get_name() + "]:: Connected to V-REP.")

        if self.configuration_.use_real_robot:
            print("[" + rospy.get_name() + "]::Waiting to connect with robot (over bCap)...")
            self.robot_driver_denso_.connect()
            self.robot_driver_denso_.initialize()
            print("[" + rospy.get_name() + "]::Connected to robot.")

    def __del__(self):
        self.robot_driver_denso_.deinitialize()
        self.robot_driver_denso_.disconnect()
        self.vrep_interface_.disconnect()
