#!/usr/bin/python3
import numpy
import rospy

from sas_robot_kinematics import RobotKinematicsProvider

# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface

import time

def practice_phantom():
    try:
        rospy.init_node("practice_interface_device", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start node...")

        vi = DQ_VrepInterface()
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        robot_si_provider = RobotKinematicsProvider("/arm1_kinematics/")
        robot_lg_provider = RobotKinematicsProvider("/arm2_kinematics/")

        print("[" + rospy.get_name() + "]:: Waiting to RobotKinematicsProvider is enabled...")
        while not robot_lg_provider.is_enabled() or not robot_si_provider.is_enabled():
            robot_si_provider.send_pose(1+0.5*E_*k_)
            robot_lg_provider.send_pose(1+0.5*E_*(0.01*i_+k_))
            time.sleep(0.01)

        print("[" + rospy.get_name() + "]:: robot_provider enabled.")
        robot_si_provider.get_desired_pose()
        print("[" + rospy.get_name() + "]:: got desired pose of robot_instrument!!")
        robot_lg_provider.get_desired_pose()
        print("[" + rospy.get_name() + "]:: got desired pose of robot_light!!")

        print("[" + rospy.get_name() + "]:: Starting practicing interfaces...")


        ##############################
        # Control Loop
        ##############################

        # Run simulation until Keyboard Interrupt
        r = rospy.Rate(250)
        while True:
            # Get target pose from V-REP
            xd_si = robot_si_provider.get_desired_pose()
            xd_lg = robot_lg_provider.get_desired_pose()

            robot_si_provider.send_pose(xd_si)
            robot_lg_provider.send_pose(xd_lg)

            r.sleep()

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")

if __name__ == '__main__':
    practice_phantom()
