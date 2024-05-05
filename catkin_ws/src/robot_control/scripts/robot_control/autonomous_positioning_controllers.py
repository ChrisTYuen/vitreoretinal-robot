# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.robot_modeling import DQ_Kinematics

# Import original files
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
from kinematics import kinematics_functions as kine_func
from kinematics.parameters import physical_parameters, control_parameters
from tools import functions
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics

# Import other dependencies
import numpy as np
import time
import rospy
"""
This file contains some controllers 
"""


class AutonomousPositioningControllers:
    def __init__(self):
        self.parameter = control_parameters.Parameters()
        if self.parameter.solver == 0:
            self.qp_solver = DQ_QuadprogSolver()
        else:
            self.qp_solver = DQ_CPLEXSolver()
        self.setup = physical_parameters.EyesurgerySetup()
        self.sim_setup = physical_parameters.SimulationSetup
        self.fail_position_list = [np.zeros(4)]

    def planar_positioning_controller(self, robot_si, robot_lg, robot_si_interface, robot_lg_interface,
                                      theta_si, theta_lg,
                                      t_start, total_iteration, td_error, planar_error, target_pixel,
                                      eye, rcm_init_si, rcm_init_lg, D_rcm_init,
                                      constrained_plane_list_si, constrained_plane_list_lg,
                                      rotation_c_plane_list, rotation_c_plane_unified_list,
                                      predict, store, data_logger, target_points, vi, theta_rotation_angle):
        """
        """
        i = 0
        planar_iteration = 0
        r = rospy.Rate(self.parameter.fps)
        axis = k_
        norm_delta_theta = 0
        while planar_error > self.parameter.threshold_planar_positioning_pixel:
            # print(planar_error)
            start = time.time()
            i = i + 1

            planar_iteration = planar_iteration + 1
            if planar_iteration > 10000:
                break

            if i < total_iteration:
                td_si = t_start + DQ(vec4(td_error) * i / total_iteration)
            else:
                td_si = t_start + td_error

            xd_lg = vi.get_object_pose(self.sim_setup.lg_vrep_name)
            td_lg = translation(xd_lg)

            # Get current poses and Jacobians
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            l_si = normalize(r_si * axis * conj(r_si))
            l_lg = normalize(r_lg * axis * conj(r_lg))

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t, eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       rcm_current_si, rcm_current_lg)
            vi.set_object_rotation(self.sim_setup.eyeball_vrep_name, r_o_e)
            vi.set_object_translation(self.sim_setup.rcm_si_vrep_name, rcm_current_si)
            vi.set_object_translation(self.sim_setup.rcm_lg_vrep_name, rcm_current_lg)

            shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            vi.set_object_pose(self.sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(self.sim_setup.lg_vrep_name, x_lg)
            vi.set_object_pose(self.sim_setup.shadow_vrep_name, shadow_tip_dq)

            # if i > total_iteration / 2:
            #     i = 0
            #     t_start = t_si
            #     td_error, total_iteration = target_points.get_translation_error(target_pixel,
            #                                                                     self.parameter.si_velocity_planar,
            #                                                                     self.parameter.tau)
            #     vi.set_object_translation("tool_tip_d2", t_si + td_error)

            # Get Jacobians related to the current tooltip poses
            J_si = robot_si.pose_jacobian(theta_si)
            Jt_si = robot_si.translation_jacobian(J_si, x_si)
            Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
            Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
            Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)
            Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg

            # Define errors
            td_eye = conj(r_o_e) * (td_si - eye.eyeball_t) * r_o_e
            e_si = np.array([vec4(t_si - td_si)])
            e_lg = np.array([vec4(t_lg - td_lg)])

            # Get the inequality constraints for safety
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, self.parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg)
            W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t, self.parameter)
            if self.parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(robot_si, robot_lg, theta_si,
                                                                                  theta_lg,
                                                                                  eye.eyeball_t, eye.eyeball_radius,
                                                                                  self.parameter, D_rcm_init,
                                                                                  rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(robot_si, robot_lg, theta_si, theta_lg,
                                                                         eye.eyeball_t, eye.eyeball_radius, self.parameter,
                                                                         D_rcm_init, r_o_e, rcm_init_si, rcm_init_lg,
                                                                         rotation_c_plane_unified_list, theta_rotation_angle)
            W = np.vstack([W_vitreo,
                           W_conical,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_conical,
                           w_om])

            eye_rotation_jacobian = om_kinematics.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg, t_si, t_lg,
                                                                                l_si, l_lg,
                                                                                rcm_current_si, rcm_current_lg,
                                                                                eye.eyeball_t, eye.eyeball_radius,
                                                                                rcm_init_si, rcm_init_lg)

            eyeball_jacobian = om_kinematics.get_eyeball_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                                  t_si, t_lg, l_si, l_lg, rcm_current_si,
                                                                  rcm_current_lg,
                                                                  eye.eyeball_t, eye.eyeball_radius, rcm_init_si,
                                                                  rcm_init_lg, td_eye)
            H1 = self.parameter.beta * eyeball_jacobian.T @ eyeball_jacobian
            A1 = np.vstack([np.zeros([4, 12]),
                            np.hstack([np.zeros([4, 6]), Jt_lg])])
            H2 = (1 - self.parameter.beta) * A1.T @ A1
            H3 = self.parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian
            H = H1 + H2 + H3

            c1 = 2 * self.parameter.beta * self.parameter.n * (eyeball_jacobian.T @ e_si.T)
            A2 = np.vstack([np.zeros([6, 1]),
                            Jt_lg.T @ e_lg.T])
            c2 = 2 * (1 - self.parameter.beta) * self.parameter.n * A2
            c = c1 + c2

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(12)

            delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H + self.parameter.damping * self.parameter.B_12),
                                                                  c, W, w, np.zeros([1, 12]), np.zeros([1, 1]))

            theta_si = theta_si + delta_thetas[:6] * self.parameter.tau
            theta_lg = theta_lg + delta_thetas[6:12] * self.parameter.tau
            theta_si.reshape([6, 1])
            theta_lg.reshape([6, 1])

            norm_delta_theta = norm_delta_theta + np.linalg.norm(delta_thetas)

            # Send info to the robots
            # robot_si_interface.send_target_joint_positions(theta_si)
            # robot_lg_interface.send_target_joint_positions(theta_lg)

            # Store values
            # shaft_distance = predict.shaft_distance
            # tip_distance = predict.tip_distance

            # store_data = np.vstack(
            #     [np.array([theta_si]).T, np.array([theta_lg]).T, delta_thetas.reshape([12, 1]), shaft_distance,
            #      tip_distance, predict.counter_sum, 0, vec4(td_si).reshape([4, 1]), 0])
            # store.send_store_data("kinematics", store_data)

            x_si = robot_si.fkm(theta_si)
            t_si = translation(x_si)

            planar_error = (np.linalg.norm(vec4(t_start + td_error - t_si)) * 10 ** 3) * self.parameter.converter_per_mm

            # if self.parameter.enable_sleep:
            #     r.sleep()

            # data_logger.log("hz_planar", float(1 / (time.time() - start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")

        return planar_iteration, theta_si, theta_lg, norm_delta_theta

    def overlap_prevention_controller(self, robot_si, robot_lg, robot_si_interface, robot_lg_interface,
                                      theta_si, theta_lg,
                                      rcm_init_si, rcm_init_lg,
                                      threshold_overlap_prevention, t_current_si, target_pixel, eye, shaft_distance,
                                      constrained_plane_list_si, constrained_plane_list_lg,
                                      predict, store, data_logger, target_points, vi):
        """
        """
        i = 0
        r = rospy.Rate(self.parameter.fps)
        axis = k_
        while shaft_distance < threshold_overlap_prevention:
            # print(shaft_distance)
            target_points.publish_current_step(2)
            start = time.time()

            # if the overlap prevention step is not completed after 10000 iterations, this step is finished.
            if i > 10000:
                self.fail_position_list = np.vstack([self.fail_position_list,
                                                     vec4(target_pixel)])
                break

            i = i + 1

            # Set the desired translation
            td_si = t_current_si

            # Get current poses and Jacobians
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            J_si = robot_si.pose_jacobian(theta_si)
            Jt_si = robot_si.translation_jacobian(J_si, x_si)
            Jr_1 = DQ_Kinematics.rotation_jacobian(J_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)

            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            l_si = normalize(r_si * axis * conj(r_si))
            l_lg = normalize(r_lg * axis * conj(r_lg))

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t,
                                                                                        eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       rcm_current_si, rcm_current_lg)

            shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            vi.set_object_pose(self.sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(self.sim_setup.lg_vrep_name, x_lg)
            vi.set_object_pose(self.sim_setup.shadow_vrep_name, shadow_tip_dq)

            # Get the inequality constraints for safety
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, self.parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg)
            W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t,
                                                           self.parameter)

            # Get the equality constraints
            Aeq = np.vstack([np.hstack([Jt_si, np.zeros([4, 6])]),
                             np.zeros([4, 12])])
            beq = np.zeros([8, 1])

            # Quadratic programming for the overlap prevention
            W = np.vstack([W_vitreo,
                           W_conical])
            w = np.vstack([w_vitreo,
                           w_conical])

            J_second_task = EyeVFI.get_second_task_distance_jacobian(Jt_si, Jt_lg, Jr_1, t_si, t_lg, r_si)
            H1 = J_second_task.T @ J_second_task
            c = -2 * J_second_task.T * self.parameter.D_velocity

            H2 = np.vstack([np.hstack([self.parameter.damping_overlap_instrument * np.eye(6), np.zeros([6, 6])]),
                            np.hstack([np.zeros([6, 6]), self.parameter.damping_overlap_light * np.eye(6)])])

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(12)

            delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2), c, W, w, Aeq, beq)

            # Update thetas
            theta_si = theta_si + delta_thetas[:6] * self.parameter.tau
            theta_lg = theta_lg + delta_thetas[6:12] * self.parameter.tau
            theta_si.reshape([6, 1])
            theta_lg.reshape([6, 1])

            # Send info to the robots
            robot_si_interface.send_target_joint_positions(theta_si)
            robot_lg_interface.send_target_joint_positions(theta_lg)

            # Store values
            shaft_distance = (eye.get_shaft_shadow_tip_distance(shadow_tip_dq, x_si)*10**3)*self.parameter.converter_per_mm
            tip_distance = eye.get_tip_shadow_tip_distance(shadow_tip_dq, x_si)
            # if not functions.is_physical_robot():
            #     shaft_distance = eye.get_shaft_shadow_tip_distance(shadow_tip_dq, x_si)
            #     if self.parameter.print_error:
            #         print(shaft_distance)
            #     tip_distance = eye.get_tip_shadow_tip_distance(shadow_tip_dq, x_si)
            # else:
            #     shaft_distance = predict.shaft_distance
            #     tip_distance = predict.tip_distance

            # planar_error = target_points.get_planar_error(target_pixel)

            # second_task_velocity = J_second_task @ delta_thetas.reshape([12, 1])
            # store_data = np.vstack(
            #     [np.array([theta_si]).T, np.array([theta_lg]).T, delta_thetas.reshape([12, 1]), shaft_distance,
            #      tip_distance, planar_error, predict.counter_sum, 1, vec4(td_si).reshape([4, 1]),
            #      second_task_velocity])
            # store.send_store_data("kinematics", store_data)

            if self.parameter.enable_sleep:
                r.sleep()

            data_logger.log("hz_overlap", float(1 / (time.time() - start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")

        return self.fail_position_list, i

    def vertical_positioning_controller(self, robot_si, robot_lg, robot_si_interface, robot_lg_interface,
                                        theta_si, theta_lg,
                                        threshold_vertical_positioning, total_iteration, td_error, planar_error, target_pixel,
                                        t_current_si,
                                        eye, rcm_init_si, rcm_init_lg, D_rcm_init,
                                        constrained_plane_list_si, constrained_plane_list_lg,
                                        rotation_c_plane_list, rotation_c_plane_unified_list,
                                        predict, store, data_logger, target_points, vi):
        """
        """
        tip_distance = predict.tip_distance
        i = 0
        j = 0

        td_error, total_planar_iteration = target_points.get_translation_error(target_pixel,
                                                                               self.parameter.si_velocity_planar2,
                                                                               self.parameter.tau)
        ex_td_error = td_error

        t1_above_target = t_current_si
        print(total_planar_iteration)
        depth = eye.get_vertical_depth(t1_above_target)
        total_iteration = kine_func.get_p2p_iteration(self.parameter.tau, depth * k_,
                                                      self.parameter.si_velocity_vertical)
        t_target = t1_above_target - depth * k_ + td_error
        vi.set_object_translation("tool_tip_d2", t1_above_target - depth * k_ + td_error)

        r = rospy.Rate(self.parameter.fps)
        while tip_distance >= threshold_vertical_positioning:
            start = time.time()

            # if the vertical positioning step is not completed after 10000 iterations, this step is finished.
            if i > 10000:
                break

            # Count how many times the tip distance achieves the threshold
            current_counter_sum = predict.counter_sum

            # Continue vertical positioning till the tip distance achieves the threshold certain times
            if current_counter_sum <= self.parameter.tip_dis_count:
                i = i + 1
                j = j + 1

                # Set the desired translation
                if i < total_iteration + 500:
                    td_si = t1_above_target - (depth / total_iteration * i) * k_ + DQ(
                        vec4(td_error) * j / total_planar_iteration)
                    # td_si = t1_above_target - (depth / total_iteration * i) * k_
                else:
                    # td_si = td_si + DQ(vec4(td_error) * j / total_planar_iteration)
                    td_si = td_si

                vi.set_object_translation("tool_tip_d1", td_si)

                # Get current poses and Jacobians
                x_si = robot_si.fkm(theta_si)
                x_lg = robot_lg.fkm(theta_lg)
                t_si = translation(x_si)
                t_lg = translation(x_lg)
                r1 = rotation(x_si)
                J_si = robot_si.pose_jacobian(theta_si)
                Jt_si = robot_si.translation_jacobian(J_si, x_si)
                Jr_1 = DQ_Kinematics.rotation_jacobian(J_si)
                J_lg = robot_lg.pose_jacobian(theta_lg)
                Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)

                shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg)
                vi.set_object_pose("instrument_tip", x_si)
                vi.set_object_pose("light_tip", x_lg)
                vi.set_object_pose("shadow_tip", shadow_tip_dq)

                if j == total_planar_iteration:
                    j = 0
                    i = 0
                    t1_above_target = td_si
                    td_error, total_planar_iteration = target_points.get_translation_error(target_pixel,
                                                                                           self.parameter.si_velocity_planar2,
                                                                                           self.parameter.tau)
                    vi.set_object_translation("tool_tip_d2", t_target + td_error)
                    t_target = t_target + td_error

                # Get the inequality constraints for safety
                W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs_for_two_manipulator(robot_si, robot_lg, theta_si,
                                                                                       theta_lg, eye.rcm_si_t,
                                                                                       eye.rcm_lg_t,
                                                                                       eye.ws_t,
                                                                                       constrained_planes_si,
                                                                                       constrained_planes_lg,
                                                                                       self.parameter)

                # Get the inequality constraints for the proposed method
                W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t,
                                                               self.parameter)

                # Quadratic programming for the first task of the vertical positioning
                W = np.vstack([W_vitreo,
                               W_conical])
                w = np.vstack([w_vitreo,
                               w_conical])

                A = np.vstack([np.hstack([Jt_si, np.zeros([4, 6])]),
                               np.zeros([4, 12])])
                H1 = A.T @ A
                e = np.vstack([np.array([vec4(t_si - td_si)]).T,
                               np.zeros([4, 1])])
                c = 2 * self.parameter.n_vertical * (A.T @ e)
                c = c.reshape(12)

                H2 = np.vstack([np.hstack([self.parameter.damping_vertical_instrument * np.eye(6), np.zeros([6, 6])]),
                                np.hstack([np.zeros([6, 6]), self.parameter.damping_vertical_light * np.eye(6)])])

                delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2), c, W, w_reshape,
                                                                      np.zeros([1, 12]), np.zeros([1, 1]))

                # Quadratic programming for the second task of the vertical positioning
                J_second_task = EyeVFI.get_second_task_distance_jacobian(Jt_si, Jt_lg, Jr_1, t_si, t_lg, r1)
                H1 = J_second_task.T @ J_second_task
                c = -2 * J_second_task.T * self.parameter.D_velocity

                Aeq = A
                beq = A @ delta_thetas.reshape([12, 1])

                if self.parameter.solver == 0:
                    c = c.reshape(12)
                    w = w.reshape(w.shape[0])

                delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2), c, W, w, Aeq, beq)

                # Update thetas
                theta_si = theta_si + delta_thetas[:6] * self.parameter.tau
                theta_lg = theta_lg + delta_thetas[6:12] * self.parameter.tau
                theta_si.reshape([6, 1])
                theta_lg.reshape([6, 1])

                # Send info to the robots
                robot_si_interface.send_target_joint_positions(theta_si)
                robot_lg_interface.send_target_joint_positions(theta_lg)

                # Store values
                if not functions.is_physical_robot():
                    shaft_distance = eye.get_shaft_shadow_tip_distance(shadow_tip_dq, x_si)
                    tip_distance = eye.get_tip_shadow_tip_distance(shadow_tip_dq, x_si)
                    if self.parameter.print_distances:
                        print(tip_distance)
                else:
                    shaft_distance = predict.shaft_distance
                    tip_distance = predict.tip_distance

                planar_error = target_points.get_planar_error(target_pixel)

                second_task_velocity = J_second_task @ delta_thetas.reshape([12, 1])
                # store_data = np.vstack(
                #     [np.array([theta_si]).T, np.array([theta_lg]).T, delta_thetas.reshape([12, 1]), shaft_distance,
                #      tip_distance, planar_error, current_counter_sum, 2, vec4(td_si).reshape([4, 1]),
                #      second_task_velocity])
                # store.send_store_data("kinematics", store_data)
            else:
                # store_data = np.vstack(
                #     [np.array([theta_si]).T, np.array([theta_lg]).T, delta_thetas.reshape([12, 1]), shaft_distance,
                #      tip_distance, planar_error, current_counter_sum, 3, vec4(td_si).reshape([4, 1]), 0])
                # store.send_store_data("kinematics", store_data)
                break

            if self.parameter.enable_sleep:
                r.sleep()

            data_logger.log("hz_vertical", float(1 / (time.time() - start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")