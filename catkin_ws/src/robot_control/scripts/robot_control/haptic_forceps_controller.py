#!/usr/bin/python3

from dqrobotics import *
from interfaces import tool_pose_transmitter_interface
from sas_datalogger import DataloggerInterface
import yaml

# Original
from kinematics.parameters import physical_parameters, control_parameters
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState

# Some libraries for calculations
import numpy as np
import time

# ROS
import rospy

class ForcepsController:
    """
    This class is used to control the forceps using the haptic device.
    """
    has_published = False  # Flag to check if the message has been published
    joint_state_msg = None  # Class variable to store the latest joint state message

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
    def joint_state_callback(cls, data):
        # rospy.loginfo(rospy.get_caller_id() + "Received joint states:\n%s", data)
        cls.joint_state_msg = data

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
    def forceps_manipulation(cls, tool_pose, joint_state_msg, publish_voltage_forceps, publish_position_forceps):
        position_array = joint_state_msg.position
        button_si = tool_pose[0][8]
        # print(position_array)
        # print(button_si)

        if control_parameters.Parameters.forceps_two_buttons:
            # If button 1 is pressed, close forceps... If button 2 is pressed, open forceps... Else, maintain the position
            if button_si == 1 and position_array[0] <= control_parameters.Parameters.max_closing:
                print("Close forceps")
                cls.publish_data(publish_voltage_forceps, control_parameters.Parameters.close_voltage)
                cls.has_published = True
            elif button_si == 2 and position_array[0] >= control_parameters.Parameters.max_opening:
                print("Open forceps")
                cls.publish_data(publish_voltage_forceps, control_parameters.Parameters.open_voltage)
                cls.has_published = True
            elif cls.has_published:
                cls.publish_data(publish_position_forceps, position_array[0])  # Publish current position once 
                cls.has_published = False
        else:
            # If button 1 or 2 is pressed, close forceps... If released, open forceps
            if button_si == (1 or 2) and position_array[0] <= control_parameters.Parameters.max_closing:
                print("Close forceps")
                cls.publish_data(publish_voltage_forceps, control_parameters.Parameters.close_voltage)
            elif position_array[0] >= control_parameters.Parameters.max_opening:
                print("Open forceps")
                cls.publish_data(publish_voltage_forceps, control_parameters.Parameters.open_voltage)
            else:
                cls.publish_data(publish_position_forceps, (control_parameters.Parameters.max_opening - 0.005))

    @classmethod 
    def forceps_rotational_control(cls, tool_pose):
        # receive the tool pose from the operator side
        rotation_si = DQ(np.array(tool_pose[0][4:8]))
        return rotation_si

    @classmethod
    def forceps_test(cls):
        """
        This function is used to test the forceps control using the haptic device. 
        """
        try:
            rospy.init_node("tele_controller", disable_signals=True)
            print("[" + rospy.get_name() + "]:: Start forceps test...")

            cls.pub_volt_forceps_si = rospy.Publisher('escon_1/set/target_joint_forces', Float64MultiArray, queue_size=10)
            cls.pub_pos_forceps_si = rospy.Publisher('escon_1/set/target_joint_positions', Float64MultiArray, queue_size=10)

            setup = physical_parameters.EyesurgerySetup()
            parameter = control_parameters.Parameters
            data_logger = DataloggerInterface(10)

            cls.tool_pose_instance = cls.haptic_forceps_setup(setup)
            cls.tool_pose_instance.initialize()

            rospy.Subscriber('escon_1/get/joint_states', JointState, cls.joint_state_callback)

            ##############################
            # Control Loop
            ##############################
            iteration = 0
            # Run simulation until Keyboard Interrupt
            r = rospy.Rate(parameter.fps)
            while True:
                start = time.time()
                iteration = iteration + 1

                cls.tool_pose = cls.tool_pose_instance.send_tool_pose()
                if cls.joint_state_msg is not None:
                    cls.forceps_manipulation(cls.tool_pose, cls.joint_state_msg, cls.pub_volt_forceps_si, cls.pub_pos_forceps_si)
                rotation = cls.forceps_rotational_control(cls.tool_pose)
                # print(rotation)
                haptic_dq = rotation + 0.5 * E_ * DQ([1]) * rotation
                # print(haptic_dq)
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
