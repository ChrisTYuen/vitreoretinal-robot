#!/usr/bin/python3
# from typing import Any
import traceback
import sys, os

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray as rosmsg_Float64MultiArray

import sas_conversions as sc
from sas_robot_driver import RobotDriverInterface

from end_effector_calibration_ui import EndEffectorCalibrationUI
from PyQt5.QtWidgets import QApplication

import dqrobotics as dql
from dqrobotics.interfaces.json11 import DQ_JsonReader


class AbbreviatedKinematicsInterface():
    def __init__(self, topic_prefix):
        self.tool_pose_ = None
        self.subscriber_tool_pose_ = rospy.Subscriber(topic_prefix + "get/pose",
                                                        PoseStamped,
                                                        self._callback_tool_pose)

    def is_enabled(self):
        if self.tool_pose_ is None:
            return False
        return True

    def _callback_tool_pose(self, msg):
        print("a")
        self.tool_pose_ = sc.geometry_msgs_pose_to_dq(msg.pose)

    def get_tool_pose(self):
        return self.tool_pose_

def calibration_ui_main():
    rosrate = rospy.Rate(200)
    if calibrationConfig['use_tool_pose']:
        robot1 = None
        robot2 = None
    else:
        def get_robot_from_json_and_set(robot_parameter_file_path):
            reader = DQ_JsonReader()
            robot = reader.get_serial_manipulator_denso_from_json(robot_parameter_file_path)
            # robot = robot_DH.kinematics()
            # Define Joint Limits ([rad])
            robot_q_minus = robot.get_lower_q_limit()
            robot_q_plus = robot.get_upper_q_limit()
            robot_dim = robot.get_dim_configuration_space()

            dq_change = dql.DQ([1, 0, 0, 0, 0, 0, 0, 0])
            robot.set_effector(dq_change)
            robot.set_reference_frame(dq_change)

            return robot
        
        robot1 = get_robot_from_json_and_set(calibrationConfig['arm1_parameter_file_path'])
        robot2 = get_robot_from_json_and_set(calibrationConfig['arm2_parameter_file_path'])
    
    try:
        rospy.loginfo("[" + name + "]::Waiting to connect to RobotDriverInterface...")
        robot1_interface = RobotDriverInterface(rosConfig['arm1_driver_ns'])
        robot2_interface = RobotDriverInterface(rosConfig['arm2_driver_ns'])
        while not robot1_interface.is_enabled() or not robot2_interface.is_enabled():
            rosrate.sleep()
        rospy.loginfo("[" + name + "]:: robot_interface enabled.")
        
        if calibrationConfig['use_tool_pose']:
            robot1_kinematics_interface = AbbreviatedKinematicsInterface(rosConfig['arm1_driver_ns']+'pose/')
            robot2_kinematics_interface = AbbreviatedKinematicsInterface(rosConfig['arm2_driver_ns']+'pose/')
            while not robot1_kinematics_interface.is_enabled() or not robot2_kinematics_interface.is_enabled():
                rosrate.sleep()
        else:
            robot1_kinematics_interface = None
            robot2_kinematics_interface = None

        rospy.loginfo("[" + name + "]:: robot_kinematics_interface enabled.")

        # robot.set_reference_frame(robot_interface.get_reference_frame())

    except Exception as exp:
        # traceback.print_exc()
        rospy.logerr("[" + name + "]::"+traceback.format_exc())
        rospy.signal_shutdown(name + ": ERROR on ros Init")

    robot1={
        "robot1": robot1,
        "robot1_interface": robot1_interface,
        "robot1_kinematics_interface1": robot1_kinematics_interface,
    }

    robot2={
        "robot2": robot2,
        "robot2_interface": robot2_interface,
        "robot2_kinematics_interface2": robot2_kinematics_interface,
    }
    
    rospy.loginfo("[" + name + "]::ROS setup complete.")

    app = QApplication([])
    widget = EndEffectorCalibrationUI(robot1, robot2, rosConfig, calibrationConfig)
    widget.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    rospy.init_node('end_effector_calibration_ui_node',
                    anonymous=False,
                    disable_signals=True)

    name = rospy.get_name()
    params = rospy.get_param(name)

    calibrationConfig = {}
    rosConfig = {}

    try:
        ## still run through everything to ensure all exist
        calibrationConfig['use_tool_pose'] = params.get('use_tool_pose', False)
        if calibrationConfig['use_tool_pose']:
            rospy.logwarn("[" + name + "]::calibrator is specified to use pose info from driver directly")
        else:
            rospy.logwarn("[" + name + "]::robot pose will be infered from robot parameter and joints")
            calibrationConfig['arm1_parameter_file_path'] = params['arm1_parameter_file_path']
            calibrationConfig['arm2_parameter_file_path'] = params['arm2_parameter_file_path']

        calibrationConfig['arm1_end_effector_info_save_path'] = params['arm1_end_effector_info_save_path']
        calibrationConfig['arm2_end_effector_info_save_path'] = params['arm2_end_effector_info_save_path']
        calibrationConfig['arm1_calibration_data_namespace'] = params['arm1_calibration_data_namespace']
        calibrationConfig['arm2_calibration_data_namespace'] = params['arm2_calibration_data_namespace']
        calibrationConfig['arm1_parameter_save_path_calibrated'] = params['arm1_parameter_save_path_calibrated']
        calibrationConfig['arm2_parameter_save_path_calibrated'] = params['arm2_parameter_save_path_calibrated']

        rospy.loginfo("[" + name + "]::calibrationConfig Parameter load OK.")

        rosConfig['arm1_driver_ns'] = params['arm1_driver_namespace']
        rosConfig['arm2_driver_ns'] = params['arm2_driver_namespace']

        rospy.loginfo("[" + name + "]::rosConfig Parameter load OK.")

    except KeyError as e:
        rospy.logerr(name + ": initialization ERROR on parameters")
        rospy.logerr(name + ": " + str(e))
        rospy.signal_shutdown(name + ": initialization ERROR on parameters")
        exit()

    ns = rospy.get_namespace()
    rosConfig.update({
        'name': name,
        'ns': ns,
    })

    calibration_ui_main()
