#!/usr/bin/python3
# Import Relevant files from dqrobotics and sas
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from sas_robot_driver import RobotDriverInterface
from sas_robot_kinematics import RobotKinematicsProvider
from sas_datalogger import DataloggerInterface
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# Original
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
import kinematics.kinematics_functions as kine_func
from tools import functions
from tools.robot_loader import Robot
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from interfaces import store_interface
from haptic_forceps_controller import ForcepsController
from eyesurgery_controllers import EyesurgeryControllers
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics
from positioning_helper_functions import PositioningHelper as pos_help

# Some libraries for calculations
import numpy as np
import time
import math
import traceback

# ROS
import rospy

def tele_operation():
    """
    This script performs teleoperation of the surgical instruments using the proposed control method. The RCM positions must be set
    within the eye model before this script is to be run. The light guide can be controlled manually or automatically. The orbital 
    manipulation can be enabled or disabled. The surgical instrument can be controlled with or without end-effector rotation. 
    CPLEX is the recommended solver for this script.
    """
    try:
        rospy.init_node("tele_controller", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start kinematics...")

        vi = DQ_VrepInterface()
        while not vi.connect("127.0.0.1", 19996, 100, 10):
            time.sleep(1)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = physical_parameters.EyesurgerySetup()
        sim_setup = physical_parameters.SimulationSetup
        parameter = control_parameters.Parameters()
        eye_parameter = eyeball_parameter.EyeballParameters
        store = store_interface.StoreInterface()
        data_logger = DataloggerInterface(10)
        controller = EyesurgeryControllers()
        forceps = ForcepsController()

        # Define robots: denso_robot_light : left hand, denso_robot_instrument with attachment: right hand
        robot_si = Robot(setup.robot_parameter_path_instrument).kinematics()
        robot_lg = Robot(setup.robot_parameter_path_light).kinematics()
        time.sleep(0.05)

        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Start tele_operation with physical robots. OK?\n")
            if parameter.forceps_control:
                forceps_control = forceps.haptic_forceps_setup(setup)
                forceps_control.initialize()
                pub_volt_forceps_si = rospy.Publisher('escon_1/set/target_joint_forces', Float64MultiArray, queue_size=10)
                pub_pos_forceps_si = rospy.Publisher('escon_1/set/target_joint_positions', Float64MultiArray, queue_size=10)
                rospy.Subscriber('escon_1/get/joint_states', JointState, ForcepsController.joint_state_callback)


        else:
            print("[" + rospy.get_name() + "]:: Start tele_operation in simulation.\n")
        
        # Set the end_effector positions
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq) 

        # Define Solver
        if parameter.solver == 0:
            qp_solver = DQ_QuadprogSolver()
        else:
            qp_solver = DQ_CPLEXSolver()

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting for the RobotDriverInterfaces to be enabled.")
        robot_si_interface = RobotDriverInterface("/arm2")
        robot_lg_interface = RobotDriverInterface("/arm1")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        input("[" + rospy.get_name() + "]:: Enabled. Press enter to continue...")

        robot_si_provider = RobotKinematicsProvider("/arm1_kinematics")
        robot_lg_provider = RobotKinematicsProvider("/arm2_kinematics")

        # Set robot base
        robot_si.set_reference_frame(vi.get_object_pose("VS050_reference#2"))
        time.sleep(0.02)
        robot_lg.set_reference_frame(vi.get_object_pose("VS050_reference#2")*setup.robot_lg_base_rel)
        time.sleep(0.02)

        # Get joint positions
        theta_si = np.array(robot_si_interface.get_joint_positions())
        theta_lg = np.array(robot_lg_interface.get_joint_positions())

        # Initial tooltip pose (DQ)
        x_si_inserted = robot_si.fkm(theta_si)
        x_lg_inserted = robot_lg.fkm(theta_lg)
        t_si_inserted = translation(x_si_inserted)
        t_lg_inserted = translation(x_lg_inserted)

        vi.set_object_pose(sim_setup.si_vrep_name, x_si_inserted)
        vi.set_object_pose(sim_setup.lg_vrep_name, x_lg_inserted)

        # Define RCM points　(translation)
        # Set RCM 0.5mm above the robot tooltip
        # Calculate rcm positions from the tool-tip positions
        rcm_si_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_si_rcm"))
        rcm_lg_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_lg_rcm"))
        vi.set_object_pose(sim_setup.rcm_si_vrep_name, rcm_si_dq)
        vi.set_object_pose(sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
        print("[" + rospy.get_name() + "]:: Calculated rcm positions from the tip positions!")
        time.sleep(.5)

        # Calculate the eyeball position based on the rcm positions and create an Eyeball object
        eyeball_dq = kine_func.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius, parameter.port_angle)
        eye = eyeball.Eyeball(eyeball_dq, rcm_si_dq, rcm_lg_dq, eye_parameter)
        eyeball_variables = np.vstack(  # Store eyeball parameters
            [vec4(eye.rcm_si_t).reshape([4, 1]), vec4(eye.rcm_lg_t).reshape([4, 1]), vec4(eye.ws_t).reshape([4, 1]),
             vec4(eye.eyeball_t).reshape([4, 1])])
        store.send_store_data("eyeball_variables", eyeball_variables)
        time.sleep(.5)

        # Set the eyeball position and workspace position in Vrep
        vi.set_object_pose(sim_setup.eyeball_vrep_name, eyeball_dq)
        vi.set_object_translation(sim_setup.workspace_vrep_name, eye.ws_t)

        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to move the tool-tips to the start positions...\n")
        else:
            print("[" + rospy.get_name() + "]:: Move the tool-tips to the start positions...\n")

        controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, parameter.td_init_set_si,
                                                              eye.rcm_si_t, eye.eyeball_t,
                                                              sim_setup.si_vrep_name, vi)

        controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, parameter.td_init_set_lg,
                                                              eye.rcm_lg_t, eye.eyeball_t,
                                                              sim_setup.lg_vrep_name, vi)

        if not EyeVFI.constraints_are_satisfied(robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye, parameter):
            print("--------------------------------------------------")
            input("[" + rospy.get_name() + "]:: Positioning was quit.")
            print("--------------------------------------------------")
            controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, [t_lg_inserted - eye.eyeball_t],
                                                                  eye.rcm_lg_t, eye.eyeball_t,
                                                                  sim_setup.lg_vrep_name, vi)
            controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, [t_si_inserted - eye.eyeball_t],
                                                                  eye.rcm_si_t, eye.eyeball_t,
                                                                  sim_setup.si_vrep_name, vi)
            exit()

        theta_si = robot_si_interface.get_joint_positions()
        theta_lg = robot_lg_interface.get_joint_positions()
        forceps_grasp_joint_states = np.zeros(3)

        # Get initial poses and translations after the instruments are inserted into an eyeball model
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)
        axis = k_
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))
        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)
        t_rcm_si, t_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                        eye.eyeball_t, eye.eyeball_radius)
        D_rcm_init = np.linalg.norm(vec4(t_rcm_si - t_rcm_lg)) ** 2  # D_OM

        rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
        rotation_c_n = k_
        rotation_c_plane_2 = rotation_c_n + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, rotation_c_n)
        rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

        normal1 = normalize(math.cos(np.deg2rad(parameter.theta_safe_eye_plane)) * i_ + math.sin(np.deg2rad(parameter.theta_safe_eye_plane)) * j_)
        normal2 = normalize(-1 * math.cos(np.deg2rad(parameter.theta_safe_eye_plane)) * i_ + math.sin(np.deg2rad(parameter.theta_safe_eye_plane)) * j_)
        rotation_c_plane_unified_1 = normal1 + E_ * dot(eye.eyeball_t, normal1)
        rotation_c_plane_unified_2 = normal2 + E_ * dot(eye.eyeball_t, normal2)
        rotation_c_plane_unified_list = [rotation_c_plane_unified_1, rotation_c_plane_unified_2]

        [constrained_plane_list_si, constrained_plane_list_lg] = kine_func.get_constrained_plane_list(eye.eyeball_t,
                                                                                                      setup.celling_height,
                                                                                                      setup.floor_height)

        print("[" + rospy.get_name() + "]:: Waiting for RobotKinematicsProvider to be enabled...")
        while not robot_lg_provider.is_enabled() or not robot_si_provider.is_enabled():
            robot_si_provider.send_pose(x_si)
            robot_si_provider.send_reference_frame(vi.get_object_pose("VS050_reference#2"))
            robot_lg_provider.send_pose(x_lg)
            robot_lg_provider.send_reference_frame(vi.get_object_pose("VS050_reference#2")*setup.robot_lg_base_rel)
            time.sleep(0.01)

        print("[" + rospy.get_name() + "]:: robot_provider enabled.")
        robot_si_provider.get_desired_pose()
        print("[" + rospy.get_name() + "]:: got desired pose of robot_instrument!!")
        robot_lg_provider.get_desired_pose()
        print("[" + rospy.get_name() + "]:: got desired pose of robot_light!!")
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to start control loop....\n")
            print("[" + rospy.get_name() + "]:: Starting control loop...")
        else:
            print("\n[" + rospy.get_name() + "]:: Starting control loop....\n")

        ##############################
        # Control Loop
        ##############################
        iteration = 0
        # Run simulation until Keyboard Interrupt
        r = rospy.Rate(parameter.fps)
        while True:
            start = time.time()
            iteration = iteration + 1

            # Get target pose from V-REP
            xd_si = robot_si_provider.get_desired_pose()
            td_si = translation(xd_si)
            rd_si = rotation(xd_si) #.normalize()

            if parameter.lg_automation:
                xd_lg = vi.get_object_pose(sim_setup.lg_vrep_name)
            else:
                xd_lg = robot_lg_provider.get_desired_pose()

            vi.set_object_pose(sim_setup.si_xd_vrep_name, xd_si)
            vi.set_object_pose(sim_setup.lg_xd_vrep_name, xd_lg)

            # Get the pose of the current tooltip pose
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            jointx_si = theta_si.size
            jointx_lg = theta_lg.size
            jointx_comb = jointx_si + jointx_lg
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            l_si = normalize(r_si * axis * conj(r_si))  # direction of z-axis of the instrument
            l_lg = normalize(r_lg * axis * conj(r_lg))  # direction of z-axis of the light guide

            vi.set_object_pose(sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

            robot_si_provider.send_pose(x_si)
            robot_lg_provider.send_pose(x_lg)

            # Get the current RCM positions, eyeball rotation and set the positions          
            (rcm_current_si, rcm_current_lg, r_o_e 
            ) = pos_help.calculate_and_set_rcm_positions(
            t_si, t_lg, l_si, l_lg, eye, rcm_init_si, rcm_init_lg, om_kinematics, sim_setup, vi)

            # Get the shadow tip position and set the position
            shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            pos_help.set_tip_positions(sim_setup, vi, x_si, x_lg, shadow_tip_dq)

            # Get Jacobians related to the current tooltip poses
            (J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg
            ) = pos_help.calculate_jacobians(robot_si, robot_lg, theta_si, theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si)
           
            # Calculate the errors of the eyeball and instruments
            td_eye, e_si_t, e_si_r, e_lg_t = pos_help.calculate_errors(xd_si, xd_lg, td_si, x_si, t_si, t_lg, r_o_e, eye, kine_func)

            if parameter.print_error:
                print("instrument translation error:", np.linalg.norm(e_si_t))
                print("instrument rotation error:", np.linalg.norm(e_si_r))   

            # Quadratic programming (without the proposed constraints)
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg, r_o_e)
            
            W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t, parameter)

            if parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(robot_si, robot_lg, theta_si,
                                                                                  theta_lg,
                                                                                  eye.eyeball_t, eye.eyeball_radius,
                                                                                  parameter, D_rcm_init,
                                                                                  rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(robot_si, robot_lg, theta_si, theta_lg,
                                                                         eye.eyeball_t, eye.eyeball_radius, parameter,
                                                                         D_rcm_init, r_o_e, rcm_init_si, rcm_init_lg,
                                                                         rotation_c_plane_unified_list)
           
            W = np.vstack([W_vitreo,
                           W_conical,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_conical,
                           w_om])

            # Calculate the eye jacobians
            (eye_rotation_jacobian, eyeball_jacobian_t, eyeball_jacobian_r
            ) = pos_help.get_eye_jacobians(Jt_si, Jt_lg, Jl_si, Jr_rd_si, Jl_lg, t_si, t_lg, l_si, l_lg, rcm_current_si, rcm_current_lg, 
                                           eye, rcm_init_si, rcm_init_lg, td_eye, jointx_si, jointx_lg, om_kinematics)
            
            # Calculate the decision variables
            H, c = pos_help.decision_variable_calculation(eyeball_jacobian_t, eyeball_jacobian_r, eye_rotation_jacobian, Jt_lg, e_si_t, 
                                                          e_lg_t, e_si_r, jointx_comb, jointx_si, parameter.n, parameter.damping, parameter)

            if parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(jointx_comb)

            delta_thetas = qp_solver.solve_quadratic_program(H, c, W, w, np.zeros([1, jointx_comb]), np.zeros([1, 1]))

            # Update the theta joint positions and send the target joint positions to the robots
            theta_si, theta_lg = pos_help.update_joint_positions(theta_si, theta_lg, delta_thetas, jointx_comb, 
                                                                 robot_si_interface, robot_lg_interface, parameter)
            
            # Control the opening and closure of foceps
            if functions.is_physical_robot() and parameter.forceps_control and ForcepsController.joint_state_msg is not None:
                haptic_pose_si = forceps_control.send_tool_pose()
                forceps.forceps_manipulation(haptic_pose_si, ForcepsController.joint_state_msg, pub_volt_forceps_si, pub_pos_forceps_si)
                forceps_grasp_joint_states = ForcepsController.joint_state_msg.position[0:3]

            ##################################
            # Logging
            ##################################
            data_logger.log("error", np.array([np.linalg.norm(vec4(td_si - t_si))]))

            l_eye = normalize(r_o_e * k_ * conj(r_o_e))
            tilt_angle = np.rad2deg(math.acos(np.dot(vec4(l_eye), vec4(k_))))
            data_logger.log("tilt_angle", np.array([tilt_angle]))

            init_turn = vec4(rcm_init_si - eye.eyeball_t)
            init_turn_xy = normalize(init_turn[1] * i_ + init_turn[2] * j_)
            current_turn = vec4(rcm_current_si - eye.eyeball_t)
            current_turn_xy = normalize(current_turn[1] * i_ + current_turn[2] * j_)
            sign = vec4(current_turn_xy)[2] - vec4(init_turn_xy)[2]
            if sign == 0:
                sign = 0
            else:
                sign = sign / abs(sign)
            turning_angle = sign * np.rad2deg(math.acos(np.dot(vec4(init_turn_xy), vec4(current_turn_xy))))
            data_logger.log("turning_angle", np.array([turning_angle]))

            dot_eye_rotation = np.linalg.norm(eye_rotation_jacobian @ delta_thetas)
            data_logger.log("dot_eye_rotation", np.array([dot_eye_rotation]))

            store_data = np.hstack(
                [theta_si.T, theta_lg.T, delta_thetas, vec8(x_si), vec8(x_lg), vec8(td_si),
                 vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e), forceps_grasp_joint_states])
            store.send_store_data("kinematics", store_data)

            if parameter.enable_sleep:
                r.sleep()

            data_logger.log("hz_tele", float(1/(time.time()-start)))

            if parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
        traceback.print_exc()
        print("please check the error message above.")

    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    tele_operation()
