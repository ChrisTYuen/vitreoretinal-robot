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
        data_logger = DataloggerInterface(1000)
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
        vi.set_object_translation(sim_setup.workspace_vrep_name, eye.ws_t)

        [constrained_plane_list_si, constrained_plane_list_lg] = kine_func.get_constrained_plane_list(eye.eyeball_t, 10, 10)

        rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
        rotation_c_n = k_
        rotation_c_plane_2 = rotation_c_n + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, rotation_c_n)
        rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

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
        theta_si_init = theta_si
        theta_lg_init = theta_lg

        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)
        t_si_init = t_si

        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)

        ##############################
        # Control Loop
        ##############################
        point = 0
        threshold = 0.0001
        eyeball_damping = 0.001
        data_logger.log("eye_damping", np.array([eyeball_damping]))

        t_rcm_si, t_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
        D_rcm_init = np.linalg.norm(vec4(t_rcm_si-t_rcm_lg))**2

        positioning_point_list = []
        positioning_point_list_xy = []
        eyeball_radius = parameter.eyeball_radius * 10 ** 3
        positioning_radius = 7  # mm
        data_logger.log("positioning_radius", positioning_radius)

        i = positioning_radius

        while i >= -2*positioning_radius:
            j = 0
            while (i**2+j**2)<=positioning_radius**2:
                k = math.sqrt(eyeball_radius**2-(i**2+j**2))
                if j == 0:
                    positioning_point_list = positioning_point_list + [normalize(i * i_ + j * j_ - k * k_)]
                    data_logger.log("positioning_point", vec4(normalize(i * i_ + j * j_ - k * k_)))
                    positioning_point_list_xy = positioning_point_list_xy + [i * i_ + j * j_]
                    data_logger.log("positioning_point_xy", vec4(i * i_ + j * j_))
                else:
                    positioning_point_list = positioning_point_list + [normalize(i * i_ + j * j_ - k * k_)]
                    data_logger.log("positioning_point", vec4(normalize(i * i_ + j * j_ - k * k_)))
                    positioning_point_list = positioning_point_list + [normalize(i * i_ - j * j_ - k * k_)]
                    data_logger.log("positioning_point", vec4(normalize(i * i_ - j * j_ - k * k_)))
                    positioning_point_list_xy = positioning_point_list_xy + [i * i_ + j * j_]
                    data_logger.log("positioning_point_xy", vec4(i * i_ + j * j_))
                    positioning_point_list_xy = positioning_point_list_xy + [i * i_ - j * j_]
                    data_logger.log("positioning_point_xy", vec4(i * i_ - j * j_))
                j = j + 1
            i = i - 1
        trajectory_point_num = len(positioning_point_list)

        print("start loop")
        while point < trajectory_point_num:
            print("[" + rospy.get_name() + "]:: Positioning to " + str(point + 1) + "th/" + str(
                trajectory_point_num) + " point....")
            theta_si = robot_si_interface.get_joint_positions()
            theta_lg = robot_lg_interface.get_joint_positions()
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)

            vi.set_object_pose(sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

            td_eye = positioning_point_list[point] * eye.eyeball_radius
            td_e = eye.eyeball_t + positioning_point_list[point] * eye.eyeball_radius
            trajectory_length = np.linalg.norm(vec4(td_e - t_si))
            total_iteration = (trajectory_length / 0.005) // parameter.tau

            axis = k_
            l_si = normalize(r_si * axis * conj(r_si))
            l_lg = normalize(r_lg * axis * conj(r_lg))
            vi.set_object_pose(sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

            current_rcm_si, current_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t,
                                                                                        eye.eyeball_radius)

            d_rcm = np.linalg.norm(vec4(current_rcm_si - current_rcm_lg))
            store_data = np.hstack(
                [np.array([theta_si]).reshape(6), np.array([theta_lg]).reshape(6), vec8(x_si), vec8(x_lg),
                 np.array([d_rcm])])
            data_logger.log("initial_state", store_data)

            i = 0
            t_si_init = t_si
            error = t_si_init - td_e
            r = rospy.Rate(parameter.fps)
            while np.linalg.norm(vec4(error)) > threshold:
                start = time.time()
                i = i + 1

                # Get the pose of the current tooltip pose
                t_si = translation(x_si)
                t_lg = translation(x_lg)
                r_si = rotation(x_si)
                r_lg = rotation(x_lg)

                axis = k_
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

                if i < total_iteration:
                    td_e_si = t_si_init - eye.eyeball_t + DQ(vec4(td_e - t_si_init) * i / total_iteration)
                else:
                    td_e_si = td_e - eye.eyeball_t

                td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                td = r_o_e * (td_e - eye.eyeball_t) * conj(r_o_e)

                vi.set_object_translation("xd1", td_si)
                vi.set_object_translation("xd2", eye.eyeball_t + td)
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
                e_error = e_si.T

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
                                                                             D_rcm_init, r_o_e, rcm_init_si, rcm_init_lg)
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
                H = eyeball_jacobian.T @ eyeball_jacobian
                H2 = eye_rotation_jacobian.T @ eye_rotation_jacobian
                e = e_error
                c = 2 * parameter.n * (eyeball_jacobian.T @ e)

                if parameter.solver == 0:
                    w = w.reshape(w.shape[0])
                    c = c.reshape(12)

                delta_thetas = qp_solver.solve_quadratic_program(2 * (H + 0.0001 * parameter.B_12 + eyeball_damping*H2),
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

                x_si = robot_si.fkm(theta_si)
                x_lg = robot_lg.fkm(theta_lg)

                error = translation(x_si) - (eye.eyeball_t + td)

                ##################################
                # Logging
                ##################################
                # data_logger.log("error", np.array([np.linalg.norm(vec4(td - t_si))]))
                #
                # l_eye = normalize(r_o_e * k_ * conj(r_o_e))
                # tilt_angle = np.rad2deg(math.acos(np.dot(vec4(l_eye), vec4(k_))))
                # data_logger.log("tilt_angle", np.array([tilt_angle]))
                #
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
                # data_logger.log("turning_angle", np.array([turning_angle]))
                #
                # dot_eye_rotation = np.linalg.norm(eye_rotation_jacobian@delta_thetas)
                # data_logger.log("dot_eye_rotation", np.array([dot_eye_rotation]))
                #
                # store_data = np.hstack(
                #     [theta_si.T, theta_lg.T, delta_thetas, vec8(x_si), vec8(x_lg), vec8(td_si),
                #      vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
                # store.send_store_data("kinematics", store_data)

                if parameter.enable_sleep:
                    r.sleep()

                if parameter.print_time:
                    print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")

                # input("a")

            point = point + 1
            data_logger.log("error", np.array([np.linalg.norm(vec4(td - t_si))]))
            l_eye = normalize(r_o_e * k_ * conj(r_o_e))
            tilt_angle = np.rad2deg(math.acos(np.dot(vec4(l_eye), vec4(k_))))
            data_logger.log("tilt_angle", np.array([tilt_angle]))

            init_turn = vec4(rcm_init_si-eye.eyeball_t)
            init_turn_xy = normalize(init_turn[1]*i_+init_turn[2]*j_)
            current_turn = vec4(current_rcm_si - eye.eyeball_t)
            current_turn_xy = normalize(current_turn[1] * i_ + current_turn[2] * j_)
            sign = vec4(current_turn_xy)[2] - vec4(init_turn_xy)[2]
            if sign == 0:
                sign = 0
            else:
                sign = sign/abs(sign)
            turning_angle = sign * np.rad2deg(math.acos(np.dot(vec4(init_turn_xy), vec4(current_turn_xy))))
            data_logger.log("turning_angle", np.array([turning_angle]))

            dot_eye_rotation = np.linalg.norm(eye_rotation_jacobian@delta_thetas)
            data_logger.log("dot_eye_rotation", np.array([dot_eye_rotation]))

            store_data = np.hstack(
                [theta_si.T, theta_lg.T, delta_thetas, vec8(x_si), vec8(x_lg), vec8(td_si),
                 vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
            store.send_store_data("kinematics", store_data)

            data_logger.log("iterations_with", i)
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            J_si = robot_si.pose_jacobian(theta_si)
            Jt_si = robot_si.translation_jacobian(J_si, x_si)
            Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
            Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)
            Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
            Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg
            eyeball_jacobian = om_kinematics.get_eyeball_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                                  t_si, t_lg, l_si, l_lg, current_rcm_si,
                                                                  current_rcm_lg,
                                                                  eye.eyeball_t, eye.eyeball_radius, rcm_init_si,
                                                                  rcm_init_lg, td_eye)
            data_logger.log("J_eyeball", np.hstack(
                [eyeball_jacobian[0, :], eyeball_jacobian[1, :], eyeball_jacobian[2, :], eyeball_jacobian[3, :]]))
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            current_rcm_si, current_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t,
                                                                                        eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       current_rcm_si, current_rcm_lg)
            store_data = np.hstack([vec8(x_si), vec8(x_lg), vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
            data_logger.log("positioned_with", store_data)

            robot_si_interface.send_target_joint_positions(theta_si_init)
            robot_lg_interface.send_target_joint_positions(theta_lg_init)
            time.sleep(1)


    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    orbital_manipulation_trajectory_control()
