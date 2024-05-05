#!/usr/bin/python3
import rospy

from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
import kinematics.kinematics_functions as kine_func
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from eyesurgery_controllers import EyesurgeryControllers
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics
from tools import functions
from interfaces import store_interface

from sas_robot_driver import RobotDriverInterface
from sas_datalogger import DataloggerInterface

# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.interfaces.json11 import DQ_JsonReader
from dqrobotics.utils import DQ_Geometry

# Some libraries for calculations
import numpy as np

# For calculating the sampling time
import time
import math
import sys
sys.path.append('/home/yuki/git/ykoyama2017/catkin_ws_unified/devel_release/lib/python3/dist-packages')
sys.path.insert(1, '/home/nml/git/ykoyama2017/catkin_ws_unified/src/sas/sas_datalogger/src/sas_datalogger/__init__.py')


def orbital_manipulation_trajectory_control():
    try:
        rospy.init_node("orbital_manipulation_trajectory", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start kinematics...")

        vi = DQ_VrepInterface()
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = physical_parameters.EyesurgerySetup()
        sim_setup = physical_parameters.SimulationSetup
        parameter = control_parameters.Parameters()
        eye_parameter = eyeball_parameter.EyeballParameters
        controller = EyesurgeryControllers()
        data_logger = DataloggerInterface(10)
        store = store_interface.StoreInterface()

        # Define robots: denso_robot_light : left hand, denso_robot_instrument: right hand
        reader = DQ_JsonReader()
        robot_si = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_instrument)
        robot_lg = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_light)

        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Start tele-operation with physical robots. OK?\n")
        else:
            print("[" + rospy.get_name() + "]:: Start tele-operation in simulation.\n")

        # Define Solver
        if parameter.solver == 0:
            qp_solver = DQ_QuadprogSolver()
        else:
            qp_solver = DQ_CPLEXSolver()

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
        robot_si_interface = RobotDriverInterface("/arm2/joints/")
        robot_lg_interface = RobotDriverInterface("/arm1/joints/")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        print("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

        # Set robot base
        robot_si.set_reference_frame(robot_si_interface.get_reference_frame())
        time.sleep(0.02)
        robot_lg.set_reference_frame(robot_si_interface.get_reference_frame()*setup.robot_lg_base_rel)
        time.sleep(0.02)

        # Set robot effector
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq)

        theta_si, theta_lg = controller.set_manipulators_initial_thetas(robot_si, robot_lg, vi,
                                                                        robot_si_interface, robot_lg_interface)
        time.sleep(.5)

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
        rcm_si_dq = x_si_inserted * (1 - 0.5 * E_ * setup.insertion_distance * k_)
        rcm_lg_dq = x_lg_inserted * (1 - 0.5 * E_ * setup.insertion_distance * k_)
        vi.set_object_pose("x3", rcm_si_dq)
        vi.set_object_pose("x4", rcm_lg_dq)
        print("[" + rospy.get_name() + "]:: Calculated rcm positions from the tip positions!")
        time.sleep(.5)

        eyeball_dq = kine_func.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius,
                                                             parameter.port_angle)
        eye = eyeball.Eyeball(eyeball_dq, rcm_si_dq, rcm_lg_dq, eye_parameter)
        eyeball_variables = np.vstack(
            [vec4(eye.rcm_si_t).reshape([4, 1]), vec4(eye.rcm_lg_t).reshape([4, 1]), vec4(eye.ws_t).reshape([4, 1]),
             vec4(eye.eyeball_t).reshape([4, 1])
             ]
        )
        store.send_store_data("eyeball_variables", eyeball_variables)
        time.sleep(.5)
        vi.set_object_pose(sim_setup.eyeball_vrep_name, eyeball_dq)
        # vi.set_object_translation(sim_setup.workspace_vrep_name, eye.ws_t)

        [constrained_plane_list_si, constrained_plane_list_lg] = kine_func.get_constrained_plane_list(eye.eyeball_t, 10, 10)

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

        if not EyeVFI.constraints_are_satisfied(robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye,
                                                parameter):
            print("--------------------------------------------------")
            input("[" + rospy.get_name() + "]:: Positioning was quit.")
            print("--------------------------------------------------")
            controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface,
                                                                  [t_lg_inserted - eye.eyeball_t],
                                                                  eye.rcm_lg_t, eye.eyeball_t,
                                                                  sim_setup.lg_vrep_name, vi)
            controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface,
                                                                  [t_si_inserted - eye.eyeball_t],
                                                                  eye.rcm_si_t, eye.eyeball_t,
                                                                  sim_setup.si_vrep_name, vi)
            exit()

        time.sleep(1)

        theta_si = robot_si_interface.get_joint_positions()
        theta_lg = robot_lg_interface.get_joint_positions()

        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)
        t_si_init = t_si

        axis = k_
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))

        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)

        ##############################
        # Control Loop
        ##############################
        trajectory_point_list = [t_si_init - eye.eyeball_t] + parameter.om_trajectory_point_list
        trajectory_point_num = len(trajectory_point_list)

        point = 0
        velocity = 1
        data_logger.log("eye_damping", np.array([parameter.eyeball_damping]))

        t_rcm_si, t_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg, eye.eyeball_t, eye.eyeball_radius)
        D_rcm_init = np.linalg.norm(vec4(t_rcm_si-t_rcm_lg))**2

        print("start loop")
        while point < trajectory_point_num:
            if point == trajectory_point_num - 1:
                td_eye = trajectory_point_list[1] * eye.eyeball_radius
                total_iteration = kine_func.get_p2p_iteration(trajectory_point_list[point] * eye.eyeball_radius,
                                                              td_eye, parameter.tau, velocity)
            elif point == 0:
                td_eye = trajectory_point_list[point + 1] * eye.eyeball_radius
                total_iteration = kine_func.get_p2p_iteration(trajectory_point_list[point],
                                                              td_eye, parameter.tau, velocity)
            else:
                td_eye = trajectory_point_list[point + 1] * eye.eyeball_radius
                total_iteration = kine_func.get_p2p_iteration(trajectory_point_list[point] * eye.eyeball_radius,
                                                              td_eye, parameter.tau, velocity)

            i = 0

            r = rospy.Rate(parameter.fps)
            while i < total_iteration:
                start = time.time()
                i = i + 1

                # Get the pose of the current tooltip pose
                x_si = robot_si.fkm(theta_si)
                x_lg = robot_lg.fkm(theta_lg)
                t_si = translation(x_si)
                t_lg = translation(x_lg)
                r_si = rotation(x_si)
                r_lg = rotation(x_lg)

                l_si = normalize(r_si * axis * conj(r_si))
                l_lg = normalize(r_lg * axis * conj(r_lg))
                vi.set_object_pose(sim_setup.si_vrep_name, x_si)
                vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

                current_rcm_si, current_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                            eye.eyeball_t, eye.eyeball_radius)
                vi.set_object_translation("rcm1_current", current_rcm_si)
                vi.set_object_translation("rcm2_current", current_rcm_lg)

                r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                           current_rcm_si, current_rcm_lg)
                vi.set_object_rotation(sim_setup.eyeball_vrep_name, r_o_e)

                if point == 0:
                    td_e_si = trajectory_point_list[point] + DQ(
                        vec4(td_eye - trajectory_point_list[point]) * i / total_iteration)
                    td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                    td = eye.eyeball_t + r_o_e * td_eye * conj(r_o_e)
                elif point == trajectory_point_num - 1:
                    td_e_si = trajectory_point_list[point] * eye.eyeball_radius + DQ(
                        eye.eyeball_radius * vec4(trajectory_point_list[1] - trajectory_point_list[point]) * i / total_iteration)
                    td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                    td = eye.eyeball_t + r_o_e * td_eye * conj(r_o_e)
                else:
                    td_e_si = trajectory_point_list[point] * eye.eyeball_radius + DQ(
                        eye.eyeball_radius * vec4(trajectory_point_list[point + 1] - trajectory_point_list[point]) * i / total_iteration)
                    td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                    td = eye.eyeball_t + r_o_e * td_eye * conj(r_o_e)

                xd_lg = vi.get_object_pose(sim_setup.lg_vrep_name)

                vi.set_object_translation("xd1", td_si)
                vi.set_object_translation("xd2", td)
                vi.set_object_translation("x3", eye.eyeball_t + r_o_e * (rcm_init_si - eye.eyeball_t) * conj(r_o_e))
                vi.set_object_translation("x4", eye.eyeball_t + r_o_e * (rcm_init_lg - eye.eyeball_t) * conj(r_o_e))

                # Get Jacobians related to the current tooltip poses
                J_si = robot_si.pose_jacobian(theta_si)
                Jt_si = robot_si.translation_jacobian(J_si, x_si)
                Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
                J_lg = robot_lg.pose_jacobian(theta_lg)
                Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
                Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)

                Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
                Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg

                # Define errors
                e_si = np.array([vec4(t_si - td_si)])
                e_lg = np.array([vec4(t_lg - translation(xd_lg))])

                if parameter.print_error:
                    print(np.linalg.norm(e_si))

                # Quadratic programming (without the proposed constraints)
                W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                                   eye.rcm_lg_t, eye.eyeball_t, parameter,
                                                                   constrained_plane_list_si, constrained_plane_list_lg)
                W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t, parameter)
                if parameter.om_version_icra_ver:
                    W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(robot_si, robot_lg, theta_si, theta_lg,
                                                                                      eye.eyeball_t, eye.eyeball_radius,
                                                                                      parameter, D_rcm_init, rotation_c_plane_list)
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

                eye_rotation_jacobian = om_kinematics.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg, t_si, t_lg, l_si, l_lg,
                                                                                    current_rcm_si, current_rcm_lg,
                                                                                    eye.eyeball_t, eye.eyeball_radius,
                                                                                    rcm_init_si, rcm_init_lg)

                eyeball_jacobian = om_kinematics.get_eyeball_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                                      t_si, t_lg, l_si, l_lg, current_rcm_si, current_rcm_lg,
                                                                      eye.eyeball_t, eye.eyeball_radius, rcm_init_si,
                                                                      rcm_init_lg, td_eye)

                H1 = parameter.beta * eyeball_jacobian.T @ eyeball_jacobian
                A1 = np.vstack([np.zeros([4, 12]),
                                np.hstack([np.zeros([4, 6]), Jt_lg])])
                H2 = (1 - parameter.beta) * A1.T @ A1
                H3 = parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian
                H = H1 + H2 + H3

                c1 = 2 * parameter.beta * parameter.n * (eyeball_jacobian.T @ e_si.T)
                A2 = np.vstack([np.zeros([6, 1]),
                                Jt_lg.T @ e_lg.T])
                c2 = 2 * (1 - parameter.beta) * parameter.n * A2
                c = c1 + c2

                if parameter.solver == 0:
                    w = w.reshape(w.shape[0])
                    c = c.reshape(12)

                delta_thetas = qp_solver.solve_quadratic_program(2 * (H + parameter.damping * parameter.B_12),
                                                                 c, W, w, np.zeros([1, 12]), np.zeros([1, 1]))

                theta_si = theta_si + delta_thetas[:6] * parameter.tau
                theta_lg = theta_lg + delta_thetas[6:12] * parameter.tau
                theta_si.reshape([6, 1])
                theta_lg.reshape([6, 1])

                ################################
                # V-REP related
                ################################
                # Set joint position
                robot_si_interface.send_target_joint_positions(theta_si)
                robot_lg_interface.send_target_joint_positions(theta_lg)

                ##################################
                # Logging
                ##################################
                data_logger.log("error", np.array([np.linalg.norm(vec4(td-t_si))]))

                l_eye = normalize(r_o_e * k_ * conj(r_o_e))
                tilt_angle = np.rad2deg(math.acos(np.dot(vec4(l_eye), vec4(k_))))
                data_logger.log("tilt_angle", np.array([tilt_angle]))

                # init_turn = vec4(rcm_init_si-eye.eyeball_t)
                # init_turn_xy = normalize(init_turn[1]*i_+init_turn[2]*j_)
                # current_turn = vec4(current_rcm_si - eye.eyeball_t)
                # current_turn_xy = normalize(current_turn[1] * i_ + current_turn[2] * j_)
                # sign = vec4(current_turn_xy)[2] - vec4(init_turn_xy)[2]
                # if sign == 0:
                #     sign = 0
                # else:
                #     sign = sign/abs(sign)
                # turning_angle = sign * np.rad2deg(math.acos(np.dot(vec4(init_turn_xy), vec4(current_turn_xy))))
                d_rot_plane_si = DQ_Geometry.point_to_plane_distance(current_rcm_si, rotation_c_plane_unified_list[0])
                current_turn = vec4(current_rcm_si - eye.eyeball_t)
                current_turn_xy = current_turn[1] * i_ + current_turn[2] * j_
                turning_angle = (90-parameter.theta_safe_eye_plane) - np.rad2deg(eye_parameter.port_angle) - np.rad2deg(math.asin(d_rot_plane_si/np.linalg.norm(vec4(current_turn_xy))))
                data_logger.log("turning_angle", np.array([turning_angle]))

                d_rot_plane_si = DQ_Geometry.point_to_plane_distance(current_rcm_si, rotation_c_plane_unified_list[0])
                current_turn = vec4(current_rcm_si - eye.eyeball_t)
                current_turn_xy = current_turn[1] * i_ + current_turn[2] * j_
                d_rot_plane_si_angle = np.rad2deg(math.asin(d_rot_plane_si / np.linalg.norm(vec4(current_turn_xy))))
                data_logger.log("d_rot_plane_si_angle", np.array([d_rot_plane_si_angle]))

                d_rot_plane_lg = DQ_Geometry.point_to_plane_distance(current_rcm_lg, rotation_c_plane_unified_list[1])
                current_turn = vec4(current_rcm_lg - eye.eyeball_t)
                current_turn_xy = current_turn[1] * i_ + current_turn[2] * j_
                d_rot_plane_lg_angle = np.rad2deg(math.asin(d_rot_plane_lg / np.linalg.norm(vec4(current_turn_xy))))
                data_logger.log("d_rot_plane_lg_angle", np.array([d_rot_plane_lg_angle]))

                dot_eye_rotation = np.linalg.norm(eye_rotation_jacobian@delta_thetas)
                data_logger.log("dot_eye_rotation", np.array([dot_eye_rotation]))

                store_data = np.hstack(
                    [theta_si.T, theta_lg.T, delta_thetas, vec8(x_si), vec8(x_lg), vec8(td_si),
                     vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
                store.send_store_data("kinematics", store_data)

                if parameter.enable_sleep:
                    r.sleep()

                if parameter.print_time:
                    print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")

                # input("a")

            point = point + 1

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    orbital_manipulation_trajectory_control()
