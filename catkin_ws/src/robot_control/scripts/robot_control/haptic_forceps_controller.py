#!/usr/bin/python3

from dqrobotics import *
from interfaces import tool_pose_transmitter_interface
from sas_datalogger import DataloggerInterface
import yaml

# Original
from kinematics.parameters import physical_parameters, control_parameters
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


# Some libraries for calculations
import numpy as np
import time

# ROS
import rospy

class ForcepsController:
    @classmethod
    def yaml_loader(cls, filepath):
        with open(filepath, 'r') as file_descriptor:
            data = yaml.safe_load(file_descriptor)
        return data
    
    @classmethod
    def haptic_forceps_setup(cls, setup):
        tool_pose = tool_pose_transmitter_interface.ToolPoseTransmitterInterface

        # Get current working directory and construct path to sas_operator_side_receiver.yaml
        sas_receiver_config = cls.yaml_loader(setup.sas_operator_side_reciever_path)

        tool_pose_instance = tool_pose(sas_receiver_config)

        return tool_pose_instance

    @classmethod
    def publish_data(cls, pub, data):
        # Create the message
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = ''
        dim.size = 0
        dim.stride = 0
        msg.layout.dim.append(dim)
        msg.layout.data_offset = 0
        msg.data = [data]  # Use the data argument

        # Publish the message
        pub.publish(msg)

    @classmethod
    def forceps_manipulation(cls, tool_pose, publish_forceps):
        button_si= tool_pose[0][8]
        print(button_si)
        # If button 1 is pressed, close forceps... If button 2 is pressed, open forceps
        if button_si == 1:
            print("Close forceps")
            cls.publish_data(publish_forceps, 1.0)
        elif button_si == 2:
            print("Open forceps")
            cls.publish_data(publish_forceps, 0.0)
    
    @classmethod
    def forceps_rotational_control(cls, tool_pose):
        # recieve the tool pose from the operator side
        rotation_si = DQ(np.array(tool_pose[0][4:8]))
        return rotation_si

    @classmethod
    def forceps_test(cls):
        try:
            rospy.init_node("tele_controller", disable_signals=True)
            print("[" + rospy.get_name() + "]:: Start forceps test...")

            pub_forceps_si = rospy.Publisher('escon_1/set/target_joint_positions', Float64MultiArray, queue_size=10)

            setup = physical_parameters.EyesurgerySetup()
            parameter = control_parameters.Parameters
            data_logger = DataloggerInterface(10)

            tool_pose_instance = cls.haptic_forceps_setup(setup)
            tool_pose_instance.initialize()

            ##############################
            # Control Loop
            ##############################
            iteration = 0
            # Run simulation until Keyboard Interrupt
            r = rospy.Rate(parameter.fps)
            while True:
                start = time.time()
                iteration = iteration + 1

                tool_pose = tool_pose_instance.send_tool_pose()
                # cls.forceps_manipulation(tool_pose, pub_forceps_si)
                rotation = cls.forceps_rotational_control(tool_pose)
                print(rotation)
                haptic_dq = rotation + 0.5 * E_ * DQ([1]) * rotation
                print(haptic_dq)
                if parameter.enable_sleep:
                    r.sleep()
                data_logger.log("hz_tele", float(1/(time.time()-start)))
                if parameter.print_time:
                    print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")

        except Exception as exp:
            print("[" + rospy.get_name() + "]:: {}.".format(exp))
        except KeyboardInterrupt:
            print("Keyboard Interrupt!!")

if __name__ == "__main__":
    ForcepsController.forceps_test()
    print("[" + rospy.get_name() + "]:: End of forceps test.")
    rospy.signal_shutdown("End of forceps test.")
    exit(0)