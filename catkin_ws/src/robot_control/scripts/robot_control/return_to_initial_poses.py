#!/usr/bin/python3
# Import Relevant files from dqrobotics and sas
from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from sas_robot_driver import RobotDriverInterface
from robot_loader import Robot

# Import original files
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from eyesurgery_controllers import EyesurgeryControllers
import kinematics.kinematics_functions as kine_fun
from tools import functions

# For calculating the sampling time
import time

# ROS
import rospy

try:
    rospy.init_node("return_to_initial_poses", disable_signals=True)

    # Create VrepInterface object
    vi = DQ_VrepInterface()
    vi.connect("127.0.0.1", 19996, 100, 10)
    vi.start_simulation()

    # Get experimental configuration and control parameters
    setup = physical_parameters.EyesurgerySetup()
    sim_setup = physical_parameters.SimulationSetup()
    parameter = control_parameters.Parameters()
    eye_parameter = eyeball_parameter.EyeballParameters
    controller = EyesurgeryControllers()

    # Define robots: robot1 : surgical instrument, robot2: light guide
    robot_si = Robot(setup.robot_parameter_path_instrument).kinematics()
    robot_lg = Robot(setup.robot_parameter_path_light).kinematics()
    time.sleep(0.05)

    if functions.is_physical_robot():
        input("[" + rospy.get_name() + "]:: Start rcm_set with physical robots. OK?\n")
    else:
        print("[" + rospy.get_name() + "]:: Start rcm_set in simulation.\n")

    # Create robot arm object
    print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
    robot_si_interface = RobotDriverInterface("/arm2")
    robot_lg_interface = RobotDriverInterface("/arm1")
    while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
        time.sleep(0.01)
    print("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

    # Set the reference frame and the end_effector's position
    robot_si.set_reference_frame(robot_si_interface.get_reference_frame())
    time.sleep(0.02)
    robot_lg.set_reference_frame(robot_si_interface.get_reference_frame()*setup.robot_lg_base_rel)
    time.sleep(0.02)

    robot_si.set_effector(setup.robot_si_effector_dq)
    robot_lg.set_effector(setup.robot_lg_effector_dq)

    t_si_inserted = DQ(rospy.get_param("/t_si_inserted"))
    t_lg_inserted = DQ(rospy.get_param("/t_lg_inserted"))

    # Set the manipulators to the initial pose
    theta_si = robot_si_interface.get_joint_positions()
    theta_lg = robot_lg_interface.get_joint_positions()
    time.sleep(.5)

    x_si = robot_si.fkm(theta_si)
    x_lg = robot_lg.fkm(theta_lg)
    t_si = translation(x_si)
    t_lg = translation(x_lg)

    vi.set_object_pose(sim_setup.si_vrep_name, x_si)
    vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

    # Calculate rcm positions from the tool-tip positions
    rcm_si_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_si_rcm"))
    rcm_lg_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_lg_rcm"))
    vi.set_object_pose(sim_setup.rcm_si_vrep_name, rcm_si_dq)
    vi.set_object_pose(sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
    time.sleep(.5)

    # Get eyeball position based on the rcm positions and create an Eyeball object
    eyeball_dq = kine_fun.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius, parameter.port_angle)
    vi.set_object_pose(sim_setup.eyeball_vrep_name, eyeball_dq)
    print("[" + rospy.get_name() + "]:: Calculated eyeball position from rcm positions!")

    eye = eyeball.Eyeball(eyeball_dq, rcm_si_dq, rcm_lg_dq, eye_parameter)

    # Show the eyeground
    vi.set_object_pose(sim_setup.workspace_vrep_name, eye.ws_dq)

    if functions.is_physical_robot():
        input("[" + rospy.get_name() + "]:: Push Enter to move the tips to the initial positions...\n")
    else:
        print("[" + rospy.get_name() + "]:: Moving the tips to the initial positions...\n")

    # Set instrument-tips to the starting points
    controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, [t_lg_inserted - eye.eyeball_t],
                                                          eye.rcm_lg_t, eye.eyeball_t, sim_setup.lg_vrep_name, vi)

    controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, [t_si_inserted - eye.eyeball_t],
                                                          eye.rcm_si_t, eye.eyeball_t, sim_setup.si_vrep_name, vi)

    print("Returned to initial poses!!")

except Exception as exp:
    print("[" + rospy.get_name() + "]:: {}.".format(exp))

except KeyboardInterrupt:
    print("Keyboard Interrupt!!")

