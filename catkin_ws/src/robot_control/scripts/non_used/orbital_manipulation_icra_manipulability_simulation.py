#!/usr/bin/python3
import rospy

from tools.DQ_eyesurgery_VFI import DQEyesurgeryVFI as EyeVFI
import tools.eyesurgery_functions as functions
from tools import eyeball, config, parameters_eye_manipulation, store_interface
import tools.eyesurgery_controllers as controller
from tools.eyeball_manipulation import EyeballManipulation as eye_manipulation

from sas_robot_driver import RobotDriverInterface
from sas_datalogger import DataloggerInterface

# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.json11 import DQ_JsonReader
from dqrobotics.robot_modeling import DQ_Kinematics

# Some libraries for calculations
import numpy as np

# For calculating the sampling time
import time
import sys
import math
sys.path.append('/home/yuki/git/ykoyama2017/catkin_ws_teleoperation/devel_release/lib/python3/dist-packages')
sys.path.insert(1, '/home/nml/git/ykoyama2017/catkin_ws_teleoperation/src/sas/sas_datalogger/src/sas_datalogger/__init__.py')

def eyeball_manipulation():
    try:
        rospy.init_node("eyeball_manipulability", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start kinematics...")

        vi = DQ_VrepInterface()
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = config.EyesurgerySetup()
        sim_setup = config.SimulationSetup()
        parameter = parameters_eye_manipulation.Parameters()
        store = store_interface.StoreInterface()
        data_logger = DataloggerInterface(1000)

        # Define Solver
        if parameter.solver == 0:
            qp_solver = DQ_QuadprogSolver()
        else:
            qp_solver = DQ_CPLEXSolver()

        # Define robots: denso_robot_light : left hand, denso_robot_instrument: right hand
        reader = DQ_JsonReader()
        robot_si = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_instrument)
        robot_lg = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_light)

        print("[" + rospy.get_name() + "]:: Start  in simulation.\n")

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
        robot_si_interface = RobotDriverInterface("/arm2/joints/")
        robot_lg_interface = RobotDriverInterface("/arm1/joints/")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        input("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

        # Set robot base
        robot_si.set_reference_frame(robot_si_interface.get_reference_frame())
        time.sleep(0.02)
        robot_lg.set_reference_frame(robot_si_interface.get_reference_frame()*setup.robot_lg_base_rel)
        time.sleep(0.02)

        # Set robot effector
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq)

        theta_si, theta_lg = controller.set_manipulators_initial_thetas(robot_si, robot_lg, vi, robot_si_interface,
                                                                        robot_lg_interface)

        # Initial tooltip pose (DQ)
        x_si_inserted = robot_si.fkm(theta_si)
        x_lg_inserted = robot_lg.fkm(theta_lg)
        t_si_inserted = translation(x_si_inserted)
        t_lg_inserted = translation(x_lg_inserted)

        vi.set_object_pose("instrument_tip", x_si_inserted)
        vi.set_object_pose("light_tip", x_lg_inserted)

        # Define RCM points
        rcm_si_dq = x_si_inserted * (1 - 0.5 * E_ * setup.insertion_distance * k_)
        rcm_lg_dq = x_lg_inserted * (1 - 0.5 * E_ * setup.insertion_distance * k_)

        vi.set_object_pose("x3", rcm_si_dq)
        vi.set_object_pose("x4", rcm_lg_dq)
        print("[" + rospy.get_name() + "]:: Calculated rcm positions from the tip positions!")
        time.sleep(.5)

        eyeball_dq = functions.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius, parameter.port_angle)
        eye = eyeball.Eyeball(eyeball_dq, parameter.eyeball_radius, parameter.eyeground_radius, rcm_si_dq, rcm_lg_dq)
        eyeball_variables = np.vstack([vec4(eye.rcm_si_t).reshape([4,1]), vec4(eye.rcm_lg_t).reshape([4,1]), vec4(eye.ws_t).reshape([4,1]), vec4(eye.eyeball_t).reshape([4,1])])
        store.send_store_data("eyeball_variables", eyeball_variables)
        time.sleep(.5)
        vi.set_object_pose("Eyeball", eyeball_dq)

        [restricted_plane_list_si, restricted_plane_list_lg] = functions.get_constrained_plane_list(eye.eyeball_t, 10, 10)

        if parameter.simulation:
            print("[" + rospy.get_name() + "]:: Move the tool-tips to the start positions...\n")
        else:
            input("[" + rospy.get_name() + "]:: Push Enter to move the tool-tips to the start positions...\n")

        controller.translation_controller_with_rcm_constraint(robot_si, parameter.td_init_set_si, eye.rcm_si_t,
                                                              robot_si_interface, eye, restricted_plane_list_si,
                                                              "instrument_tip")

        time.sleep(.1)

        controller.translation_controller_with_rcm_constraint(robot_lg, parameter.td_init_set_lg, eye.rcm_lg_t,
                                                              robot_lg_interface, eye, restricted_plane_list_lg,
                                                              "light_tip")

        if not functions.constraints_are_satisfied(robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye, parameter, restricted_plane_list_si, restricted_plane_list_lg):
            print("--------------------------------------------------")
            input("[" + rospy.get_name() + "]:: Positioning was quit.")
            print("--------------------------------------------------")

            controller.translation_controller_with_rcm_constraint(robot_lg, [t_lg_inserted - eye.eyeball_t],
                                                                  eye.rcm_lg_t, robot_lg_interface, eye,
                                                                  [restricted_plane_list_si, restricted_plane_list_lg],
                                                                  "light_tip")
            controller.translation_controller_with_rcm_constraint(robot_si, [t_si_inserted - eye.eyeball_t],
                                                                  eye.rcm_si_t, robot_si_interface, eye,
                                                                  [restricted_plane_list_si, restricted_plane_list_lg],
                                                                  "instrument_tip")
            exit()

        time.sleep(1)

        theta_si = robot_si_interface.get_joint_positions()
        theta_lg = robot_lg_interface.get_joint_positions()
        theta_si_init = theta_si
        theta_lg_init = theta_lg

        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)
        D_rcm_init = np.linalg.norm(vec4(rcm_init_si - rcm_init_lg)) ** 2

        ##############################
        # Control Loop
        ##############################
        point = 0
        threshold = 0.0001
        positioning_point_list = []
        positioning_point_list_xy = []
        eyeball_radius = parameter.eyeball_radius*10**3
        positioning_radius = 7 #mm
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

        rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
        rotation_c_plane_2 = k_ + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, k_)
        rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

        time.sleep(1)

        while point < trajectory_point_num:
            print("[" + rospy.get_name() + "]:: Positioning to " + str(point + 1) + "th/" + str(trajectory_point_num) + " point....")
            theta_si = robot_si_interface.get_joint_positions()
            theta_lg = robot_lg_interface.get_joint_positions()
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)

            vi.set_object_pose("instrument_tip", x_si)
            vi.set_object_pose("light_tip", x_lg)

            td_e = eye.eyeball_t + positioning_point_list[point] * eye.eyeball_radius
            trajectory_length = np.linalg.norm(vec4(td_e - t_si))
            total_iteration = (trajectory_length / 0.001) // parameter.tau

            current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)

            if parameter.orbital_manipulation:
                d_rcm = np.linalg.norm(vec4(current_rcm_si - current_rcm_lg))
                store_data = np.hstack([np.array([theta_si]).reshape(6), np.array([theta_lg]).reshape(6), vec8(x_si), vec8(x_lg), np.array([d_rcm])])
                data_logger.log("initial_state_with", store_data)
            else:
                store_data = np.hstack([np.array([theta_si]).reshape(6), np.array([theta_lg]).reshape(6), vec8(x_si), vec8(x_lg), vec4(current_rcm_si), vec4(current_rcm_lg)])
                data_logger.log("initial_state_without", store_data)

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
                vi.set_object_pose("instrument_tip", x_si)
                vi.set_object_pose("light_tip", x_lg)

                # Get Jacobians related to the current tooltip poses
                J_si = robot_si.pose_jacobian(theta_si)
                Jt_si = robot_si.translation_jacobian(J_si, x_si)
                Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
                J_lg = robot_lg.pose_jacobian(theta_lg)
                Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
                Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)

                if parameter.orbital_manipulation_icra_ver:
                    if i < total_iteration:
                        # td_si = t_si_init + DQ(vec4(td_e - t_si_init) * i / total_iteration)
                        current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                        r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                        td_e_si = t_si_init - eye.eyeball_t + DQ(vec4(td_e - t_si_init) * i / total_iteration)
                        td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                        td = r_o_e * (td_e - eye.eyeball_t) * conj(r_o_e)
                    else:
                        current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                        r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                        td_e_si = td_e - eye.eyeball_t
                        td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                        td = r_o_e * (td_e - eye.eyeball_t) * conj(r_o_e)
                    vi.set_object_translation("xd1", td_si)
                    vi.set_object_translation("xd2", eye.eyeball_t + td)

                    J_d_rcm = EyeVFI.get_rcm_jacobians_for_eyeball_manipulation_simulation_icra(robot_si, robot_lg, theta_si, theta_lg, eye.eyeball_t, eye.eyeball_radius, parameter)

                    task_jacobian = np.vstack([np.hstack([Jt_si, np.zeros([4, 6])]),
                                               np.hstack([np.zeros([4, 6]), Jt_lg]),
                                               J_d_rcm])
                    e = np.vstack([np.array([vec4(t_si - td_si)]).T,
                                   np.zeros([4, 1]),
                                   np.zeros([J_d_rcm.shape[0], 1])])

                    delta_thetas = (-1 * parameter.n * np.linalg.pinv(task_jacobian) @ e).reshape(12)

                    theta_si = theta_si + delta_thetas[:6] * parameter.tau
                    theta_lg = theta_lg + delta_thetas[6:12] * parameter.tau

                    robot_si_interface.send_target_joint_positions(theta_si)
                    robot_lg_interface.send_target_joint_positions(theta_lg)

                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)

                    # error = translation(x_si) - td_e
                    error = translation(x_si) - (eye.eyeball_t + td)
                    # Logging
                    current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                    r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                    vi.set_object_rotation("Eyeball", r_o_e)
                    vi.set_object_translation("x3", eye.eyeball_t + r_o_e * (rcm_init_si - eye.eyeball_t) * conj(r_o_e))
                    vi.set_object_translation("x4", eye.eyeball_t + r_o_e * (rcm_init_lg - eye.eyeball_t) * conj(r_o_e))
                    d_rcm = np.linalg.norm(vec4(current_rcm_si - current_rcm_lg))
                    store_data = np.hstack([theta_si.T, theta_lg.T, vec8(x_si), vec8(x_lg), vec4(td_si), vec4(current_rcm_si), vec4(current_rcm_lg), np.array([d_rcm]), vec4(r_o_e)])
                    data_logger.log("kinematics_with", store_data)

                    if parameter.orbital_manipulation:
                        current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                        r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                        vi.set_object_rotation("Eyeball", r_o_e)
                        vi.set_object_translation("x3", eye.eyeball_t + r_o_e * (rcm_init_si - eye.eyeball_t) * conj(r_o_e))
                        vi.set_object_translation("x4", eye.eyeball_t + r_o_e * (rcm_init_lg - eye.eyeball_t) * conj(r_o_e))
                        d_rcm = np.linalg.norm(vec4(current_rcm_si - current_rcm_lg))
                        store_data = np.hstack([theta_si.T, theta_lg.T, vec8(x_si), vec8(x_lg), vec4(td_si), vec4(current_rcm_si), vec4(current_rcm_lg), np.array([d_rcm]), vec4(r_o_e)])
                        data_logger.log("kinematics_with", store_data)
                    else:
                        current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                        r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                        d_rcm = np.linalg.norm(vec4(current_rcm_si - current_rcm_lg))
                        store_data = np.hstack([theta_si.T, theta_lg.T, vec8(x_si), vec8(x_lg), vec4(td_si), vec4(current_rcm_si), vec4(current_rcm_lg), np.array([d_rcm]), vec4(r_o_e)])
                        data_logger.log("kinematics_without", store_data)
                else:
                    if i < total_iteration:
                        current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                        r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                        td_e_si = t_si_init - eye.eyeball_t + DQ(vec4(td_e - t_si_init) * i / total_iteration)
                        td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                        td = r_o_e * (td_e - eye.eyeball_t) * conj(r_o_e)
                    else:
                        current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                        r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                        td_e_si = td_e - eye.eyeball_t
                        td_si = eye.eyeball_t + r_o_e * td_e_si * conj(r_o_e)
                        td = r_o_e * (td_e - eye.eyeball_t) * conj(r_o_e)
                    vi.set_object_translation("xd1", td_si)
                    vi.set_object_translation("xd2", eye.eyeball_t+td)

                    # J_r_eye = eye_manipulation.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jr_si, Jr_lg, t_si, t_lg, r_si, r_lg, eye.eyeball_t,
                    #                                                          eye.eyeball_radius, rcm_init_si, rcm_init_lg)
                    eyeball_jacobian = eye_manipulation.get_eyeball_jacobian(Jt_si, Jt_lg, Jr_si, Jr_lg, t_si, t_lg, r_si, r_lg, eye.eyeball_t,
                                                                             eye.eyeball_radius, rcm_init_si, rcm_init_lg, td_e_si)
                    # W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs_for_orbital_manipulation(robot_si, robot_lg, theta_si, theta_lg, eye.eyeball_t,
                    #                                                                             eye.eyeball_radius, restricted_plane_list_si, restricted_plane_list_lg,
                    #                                                                             parameter, D_rcm_init, r_o_e, J_r_eye, rotation_c_plane_list)
                    # W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t, parameter)
                    #
                    # W = np.vstack([W_vitreo,
                    #                W_conical
                    #                ])
                    # w = np.vstack([w_vitreo,
                    #                w_conical
                    #                ])
                    #
                    # H = eyeball_jacobian.T @ eyeball_jacobian
                    # e_si = np.array([vec4(t_si - td_si)])
                    # e_error = e_si.T
                    # e = e_error
                    # c = 2 * parameter.n * (eyeball_jacobian.T @ e)
                    #
                    # if parameter.solver == 0:
                    #     w = w.reshape(w.shape[0])
                    #     c = c.reshape(12)
                    #
                    # delta_thetas = qp_solver.solve_quadratic_program(2 * (H + parameter.damping * parameter.B_12), c,
                    #                                                  W, w, np.zeros([1, 12]), np.zeros([1, 1]))

                    J_d_rcm = EyeVFI.get_rcm_jacobians_for_eyeball_manipulation_simulation_icra(robot_si, robot_lg, theta_si, theta_lg, eye.eyeball_t,
                                                                                                eye.eyeball_radius, parameter)
                    task_jacobian = np.vstack([eyeball_jacobian,
                                               J_d_rcm])

                    e = np.vstack([np.array([vec4(t_si - td_si)]).T,
                                   np.zeros([J_d_rcm.shape[0], 1])])

                    delta_thetas = (-1 * 100 * np.linalg.pinv(task_jacobian) @ e).reshape(12)

                    theta_si = theta_si + delta_thetas[:6] * parameter.tau
                    theta_lg = theta_lg + delta_thetas[6:12] * parameter.tau
                    theta_si.reshape([6, 1])
                    theta_lg.reshape([6, 1])

                    robot_si_interface.send_target_joint_positions(theta_si)
                    robot_lg_interface.send_target_joint_positions(theta_lg)

                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)

                    error = translation(x_si) - (eye.eyeball_t+td)

                    # Logging
                    current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                    r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                    vi.set_object_rotation("Eyeball", r_o_e)
                    vi.set_object_translation("x3", eye.eyeball_t + r_o_e * (rcm_init_si - eye.eyeball_t) * conj(r_o_e))
                    vi.set_object_translation("x4", eye.eyeball_t + r_o_e * (rcm_init_lg - eye.eyeball_t) * conj(r_o_e))
                    d_rcm = np.linalg.norm(vec4(current_rcm_si - current_rcm_lg))
                    store_data = np.hstack([theta_si.T, theta_lg.T, vec8(x_si), vec8(x_lg), vec4(td_si), vec4(current_rcm_si), vec4(current_rcm_lg), np.array([d_rcm]), vec4(r_o_e)])
                    data_logger.log("kinematics", store_data)

                # if parameter.enable_sleep:
                #     r.sleep()

                data_logger.log("hz_tele", float(1/(time.time()-start)))

                if parameter.print_time:
                    print("[" + rospy.get_name() + "]:: " +str(int(1/(time.time()-start)))+" Hz")


            if parameter.orbital_manipulation_icra_ver:
                if parameter.orbital_manipulation:
                    data_logger.log("iterations_with", i)
                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)
                    J_si = robot_si.pose_jacobian(theta_si)
                    Jt_si = robot_si.translation_jacobian(J_si, x_si)
                    J_lg = robot_lg.pose_jacobian(theta_lg)
                    Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
                    J_d_rcm = EyeVFI.get_rcm_jacobians_for_eyeball_manipulation_simulation_icra(robot_si, robot_lg, theta_si, theta_lg, eye.eyeball_t, eye.eyeball_radius, parameter)
                    data_logger.log("Jt_si_with", np.hstack([Jt_si[0, :], Jt_si[1, :], Jt_si[2, :], Jt_si[3, :]]))
                    data_logger.log("Jt_lg_with", np.hstack([Jt_lg[0, :], Jt_lg[1, :], Jt_lg[2, :], Jt_lg[3, :]]))
                    data_logger.log("J_d_rcm_with", J_d_rcm)
                    t_si = translation(x_si)
                    t_lg = translation(x_lg)
                    r_si = rotation(x_si)
                    r_lg = rotation(x_lg)
                    current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                    r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                    store_data = np.hstack([vec8(x_si), vec8(x_lg), vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
                    data_logger.log("positioned_with_icra_ver", store_data)
                else:
                    data_logger.log("iterations_without", i)
                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)
                    J_si = robot_si.pose_jacobian(theta_si)
                    Jt_si = robot_si.translation_jacobian(J_si, x_si)
                    J_lg = robot_lg.pose_jacobian(theta_lg)
                    Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
                    J_d_rcm = EyeVFI.get_rcm_jacobians_for_eyeball_manipulation_simulation_icra(robot_si, robot_lg, theta_si, theta_lg, eye.eyeball_t, eye.eyeball_radius, parameter)
                    data_logger.log("Jt_si_without", np.hstack([Jt_si[0, :], Jt_si[1, :], Jt_si[2, :], Jt_si[3, :]]))
                    data_logger.log("Jt_lg_without", np.hstack([Jt_lg[0, :], Jt_lg[1, :], Jt_lg[2, :], Jt_lg[3, :]]))
                    data_logger.log("J_d_rcm_without", np.hstack([J_d_rcm[0, :],J_d_rcm[1, :],J_d_rcm[2, :],J_d_rcm[3, :]]))
                    t_si = translation(x_si)
                    t_lg = translation(x_lg)
                    r_si = rotation(x_si)
                    r_lg = rotation(x_lg)
                    current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                    r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                    store_data = np.hstack( [vec8(x_si), vec8(x_lg), vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
                    data_logger.log("positioned_without", store_data)
            else:
                data_logger.log("iterations_with", i)
                x_si = robot_si.fkm(theta_si)
                x_lg = robot_lg.fkm(theta_lg)
                J_si = robot_si.pose_jacobian(theta_si)
                Jt_si = robot_si.translation_jacobian(J_si, x_si)
                Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
                J_lg = robot_lg.pose_jacobian(theta_lg)
                Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
                Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)
                td_e_si = td_e - eye.eyeball_t
                eyeball_jacobian = eye_manipulation.get_eyeball_jacobian(Jt_si, Jt_lg, Jr_si, Jr_lg, t_si, t_lg, r_si, r_lg, eye.eyeball_t,
                                                                         eye.eyeball_radius, rcm_init_si, rcm_init_lg, td_e_si)
                data_logger.log("J_eyeball", np.hstack([eyeball_jacobian[0, :], eyeball_jacobian[1, :], eyeball_jacobian[2, :], eyeball_jacobian[3, :]]))
                t_si = translation(x_si)
                t_lg = translation(x_lg)
                r_si = rotation(x_si)
                r_lg = rotation(x_lg)
                current_rcm_si, current_rcm_lg = eye_manipulation.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eye.eyeball_t, eye.eyeball_radius)
                r_o_e = eye_manipulation.get_eyeball_rotation(eye.eyeball_t, rcm_init_si, rcm_init_lg, current_rcm_si, current_rcm_lg, eye.eyeball_radius)
                store_data = np.hstack([vec8(x_si), vec8(x_lg), vec4(current_rcm_si), vec4(current_rcm_lg), vec4(r_o_e)])
                data_logger.log("positioned_with", store_data)

            robot_si_interface.send_target_joint_positions(theta_si_init)
            robot_lg_interface.send_target_joint_positions(theta_lg_init)
            time.sleep(1)
            point = point + 1


    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")

if __name__ == '__main__':
    eyeball_manipulation()
