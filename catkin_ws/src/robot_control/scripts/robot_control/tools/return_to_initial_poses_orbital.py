#!/usr/bin/python3
# Add the parent directory to sys.path
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))

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
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics
import kinematics.kinematics_functions as kine_func

# For calculating the sampling time
import time
import numpy as np
import math

# ROS
import rospy

try:
    """
    This script returns the instruments to the initial positions when using orbital manipulation.
    """
    rospy.init_node("return_to_initial_point", disable_signals=True)

    vi = DQ_VrepInterface()
    while not vi.connect("127.0.0.1", 19996, 100, 10):
        time.sleep(1)
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

    # Set the end_effector positions
    robot_si.set_effector(setup.robot_si_effector_dq)
    robot_lg.set_effector(setup.robot_lg_effector_dq)

    # Robot driver interface
    print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
    robot_si_interface = RobotDriverInterface("/arm2")
    robot_lg_interface = RobotDriverInterface("/arm1") 
    while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
        time.sleep(0.01)
    print("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

    # Set the reference frame and the end_effector's position
    robot_si.set_reference_frame(vi.get_object_pose("VS050_reference#2"))
    time.sleep(0.02)
    robot_lg.set_reference_frame(vi.get_object_pose("VS050_reference#2")*setup.robot_lg_base_rel)
    time.sleep(0.02)

    # Initial tooltip pose (DQ)
    t_si_inserted = DQ(rospy.get_param("/t_si_inserted"))
    t_lg_inserted = DQ(rospy.get_param("/t_lg_inserted"))
    r_si_inserted = DQ(rospy.get_param("/r_si_inserted"))
    r_lg_inserted = DQ(rospy.get_param("/r_lg_inserted"))

    # Set the manipulators to the initial pose
    theta_si = robot_si_interface.get_joint_positions()
    theta_lg = robot_lg_interface.get_joint_positions()
    time.sleep(.5)

    x_si = robot_si.fkm(theta_si)
    x_lg = robot_lg.fkm(theta_lg)
    t_si = translation(x_si)
    t_lg = translation(x_lg)
    axis = k_
    l_si = normalize(r_si_inserted * axis * conj(r_si_inserted))
    l_lg = normalize(r_lg_inserted * axis * conj(r_lg_inserted))

    vi.set_object_pose('instrument_tip', x_si)
    vi.set_object_pose('light_tip', x_lg)

    # Calculate rcm positions from the tool-tip positions
    rcm_si_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_si_rcm"))
    rcm_lg_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_lg_rcm"))
    vi.set_object_pose(sim_setup.rcm_si_vrep_name, rcm_si_dq)
    vi.set_object_pose(sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
    time.sleep(.5)

    rcm_t_si_init = translation(rcm_si_dq)
    rcm_t_lg_init = translation(rcm_lg_dq)

    # Get eyeball position based on the rcm positions and create an Eyeball object
    eyeball_dq = kine_fun.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius, parameter.port_angle)
    vi.set_object_pose(sim_setup.eyeball_vrep_name, eyeball_dq)
    print("[" + rospy.get_name() + "]:: Calculated eyeball position from rcm positions!")

    eye = eyeball.Eyeball(eyeball_dq, rcm_si_dq, rcm_lg_dq, eye_parameter)
    current_rcm_t_si, current_rcm_t_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg, eye.eyeball_t, eye.eyeball_radius)
    r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_t_si_init, rcm_t_lg_init,
                                               current_rcm_t_si, current_rcm_t_lg)

    D_rcm_init = np.linalg.norm(vec4(rcm_t_si_init - rcm_t_lg_init)) ** 2

    [constrained_plane_list_si, constrained_plane_list_lg] = kine_func.get_constrained_plane_list(eye.eyeball_t,
                                                                                                  setup.celling_height,
                                                                                                  setup.floor_height)

    rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
    rotation_c_n = k_
    rotation_c_plane_2 = rotation_c_n + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, rotation_c_n)
    rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

    normal1 = normalize(math.cos(np.deg2rad(parameter.theta_safe_eye_plane)) * i_ + math.sin(
        np.deg2rad(parameter.theta_safe_eye_plane)) * j_)
    normal2 = normalize(-1 * math.cos(np.deg2rad(parameter.theta_safe_eye_plane)) * i_ + math.sin(
        np.deg2rad(parameter.theta_safe_eye_plane)) * j_)
    rotation_c_plane_unified_1 = normal1 + E_ * dot(eye.eyeball_t, normal1)
    rotation_c_plane_unified_2 = normal2 + E_ * dot(eye.eyeball_t, normal2)
    rotation_c_plane_unified_list = [rotation_c_plane_unified_1, rotation_c_plane_unified_2]

    # Show the eyeground
    vi.set_object_pose(sim_setup.workspace_vrep_name, eye.ws_dq)

    # Create constrained planes
    if functions.is_physical_robot():
        input("[" + rospy.get_name() + "]:: Push Enter to move the tips to the initial positions...\n")
    else:
        print("[" + rospy.get_name() + "]:: Moving the tips to the initial positions...\n")

    ############################################################
    # Set tool-tips to the starting points
    ############################################################
    controller.translation_controller_with_rcm_orbital(robot_si, robot_lg, robot_si_interface, robot_lg_interface,
                                                       t_si_inserted, t_lg_inserted, r_si_inserted, r_lg_inserted,
                                                       eye, rcm_t_si_init, rcm_t_lg_init, D_rcm_init,
                                                       constrained_plane_list_si, constrained_plane_list_lg,
                                                       rotation_c_plane_list, rotation_c_plane_unified_list, vi, 2)
    print("Returned to initial points!!")

except Exception as exp:
    print("[" + rospy.get_name() + "]:: {}.".format(exp))

except KeyboardInterrupt:
    print("Keyboard Interrupt!!")

