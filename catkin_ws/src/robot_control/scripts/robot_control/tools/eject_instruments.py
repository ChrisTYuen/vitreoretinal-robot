#!/usr/bin/python3
# Add the parent directory to sys.path
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))

# Import Relevant files from dqrobotics and sas
from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.solvers import DQ_QuadprogSolver
from sas_robot_driver import RobotDriverInterface
from robot_loader import Robot

# Import original files
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from eyesurgery_controllers import EyesurgeryControllers
import kinematics.kinematics_functions as kine_func
from tools import functions

# For calculating the sampling time
import time
import numpy as np

# ROS
import rospy

try:
    """
    This script ejects the instruments from the eye model. 
    """
    rospy.init_node("return_to_initial_poses", disable_signals=True)

    # Create VrepInterface object
    vi = DQ_VrepInterface()
    while not vi.connect("127.0.0.1", 19996, 100, 10):
        time.sleep(1)
    vi.start_simulation()

    # Get experimental configuration and control parameters
    setup = physical_parameters.EyesurgerySetup()
    sim_setup = physical_parameters.SimulationSetup()
    parameter = control_parameters.Parameters()

    qp_solver = DQ_QuadprogSolver()

    # Define robots: denso_robot_light : left hand, denso_robot_instrument with attachment: right hand
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

    # Create robot arm object
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

    # Set the manipulators to the initial pose
    theta_si = robot_si_interface.get_joint_positions()
    theta_lg = robot_lg_interface.get_joint_positions()
    time.sleep(.5)

    x_si = robot_si.fkm(theta_si)
    x_lg = robot_lg.fkm(theta_lg)
    jointx_si = theta_si.size
    jointx_lg = theta_lg.size
    jointx_comb = jointx_si + jointx_lg
    t_init_si = translation(x_si)
    t_init_lg = translation(x_lg)
    r_si = rotation(x_si)
    r_lg = rotation(x_lg)

    eject_dis = 0.05
    td_si = t_init_si - r_si * k_ * conj(r_si) * eject_dis
    td_lg = t_init_lg - r_lg * k_ * conj(r_lg) * eject_dis

    vi.set_object_pose(sim_setup.si_vrep_name, x_si)
    vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

    if functions.is_physical_robot():
        input("[" + rospy.get_name() + "]:: Push Enter to eject instruments...\n")
    else:
        print("[" + rospy.get_name() + "]:: Ejecting instruments...\n")

    # Calculate the number of iterations needed to move the tip to the desired translation at a constant velocity
    total_iteration = kine_func.get_p2p_iteration(t_init_si, td_si, parameter.tau, parameter.setup_velocity)

    # Loop till the target reaches the initial point
    i = 0
    r = rospy.Rate(parameter.fps)
    while i < total_iteration:
        start = time.time()
        i = i + 1

        # Update the desired translation
        td_intermediate_si = t_init_si + DQ(vec4(td_si - t_init_si) * i / total_iteration)
        td_intermediate_lg = t_init_lg + DQ(vec4(td_lg - t_init_lg) * i / total_iteration)

        # Get the current pose, translation, and Jacobians
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        J_si = robot_si.pose_jacobian(theta_si)
        J_lg = robot_lg.pose_jacobian(theta_lg)
        Jt_si = DQ_Kinematics.translation_jacobian(J_si, x_si)
        Jt_lg = DQ_Kinematics.translation_jacobian(J_lg, x_lg)

        vi.set_object_pose(sim_setup.si_vrep_name, x_si)
        vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

        # Quadratic programming

        e_si = np.array([vec4(t_si - td_intermediate_si)]).T
        e_lg = np.array([vec4(t_lg - td_intermediate_lg)]).T
        e = np.vstack([e_si,
                       e_lg])
        A = np.vstack([np.hstack([Jt_si, np.zeros([4, jointx_lg])]),
                       np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
        H = A.T @ A
        C = 2 * parameter.n_initialize * (A.T @ e)
        C = C.reshape(13)

        delta_thetas = qp_solver.solve_quadratic_program(2 * (H + parameter.damping_initialize * np.eye(jointx_comb)),
                                                         C, np.zeros([1, jointx_comb]), np.zeros([1, 1]),
                                                         np.zeros([1, jointx_comb]), np.zeros([1, 1]))

        # Update joint position
        theta_si = theta_si + delta_thetas[:jointx_si] * parameter.tau
        theta_lg = theta_lg + delta_thetas[jointx_si:jointx_comb] * parameter.tau
        theta_si.reshape([jointx_si, 1])
        theta_lg.reshape([jointx_lg, 1])
        robot_si_interface.send_target_joint_positions(theta_si)
        robot_lg_interface.send_target_joint_positions(theta_lg)

        if parameter.enable_sleep:
            r.sleep()
        if parameter.print_time:
            print(time.time() - start)

    print("Ejected instruments!!")

except Exception as exp:
    print("[" + rospy.get_name() + "]:: {}.".format(exp))

except KeyboardInterrupt:
    print("Keyboard Interrupt!!")

