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
from eyeball import eyeball
from positioning_helper_functions import PositioningHelper as pos_help

# Import other dependencies
import numpy as np
import time
import rospy
import math

"""
This file contains controller functions for autonomous positioning of the robots with orbital manipulation.
Planar positioning, overlap prevention, and vertical positioning are implemented, and additional positioning are
included in this file. Rotation of a forceps device has currently NOT been tested at this time, but the foundation 
is set for future testing. See Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal_Surgery
described in V of Koyama et al. (2022),
"""

class AutonomousPositioningControllers:
    def __init__(self, robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye, vi, target_points,
                 data_logger, predict, store, rcm_init_si, rcm_init_lg, D_rcm_init, 
                 constrained_plane_list_si, constrained_plane_list_lg, rotation_c_plane_list, rotation_c_plane_unified_list):
        
        self.parameter = control_parameters.Parameters()
        if self.parameter.solver == 0:
            self.qp_solver = DQ_QuadprogSolver()
        else:
            self.qp_solver = DQ_CPLEXSolver()
        self.setup = physical_parameters.EyesurgerySetup()
        self.sim_setup = physical_parameters.SimulationSetup
        self.fail_position_list = [np.zeros(4)]

        # Initialize the robots, eye, and other parameters
        self.robot_si = robot_si
        self.robot_lg = robot_lg
        self.robot_si_interface = robot_si_interface
        self.robot_lg_interface = robot_lg_interface
        self.eye = eye
        self.vi = vi
        self.target_points = target_points
        self.data_logger = data_logger
        self.predict = predict
        self.store = store

        # Initialize other variables that were passed as parameters or returned
        self.theta_si = None
        self.theta_lg = None
        self.t_start = None         # Revisit regarding the total iteration lines
        self.td_error = None
        self.planar_error = None
        self.target_pixel = None
        self.rcm_init_si = rcm_init_si
        self.rcm_init_lg = rcm_init_lg
        self.D_rcm_init = D_rcm_init
        self.constrained_plane_list_si = constrained_plane_list_si
        self.constrained_plane_list_lg = constrained_plane_list_lg
        self.rotation_c_plane_list = rotation_c_plane_list
        self.rotation_c_plane_unified_list = rotation_c_plane_unified_list

    def planar_positioning_controller(self, task_iteration, planar_error):
        """
        The controller moves the instrument from its initial position to t_planar,d (td_si), which is projected to the
        target position in the retina. The light guide autonomously move in order to keep the constraints while “softly” prioritizing the
        instrument motion. The inequality constraints simultaneously enforce the vitreoretinal task constraints and shadow-based
        autonomous positioning constraints using VFIs. This step converges successfully when the error norm goes below the threshold.
        See Section V of Koyama et al. (2022) for more details.
        """
        i = 0
        planar_iteration = 0
        r = rospy.Rate(self.parameter.fps)
        axis = self.parameter.axis
        norm_delta_theta = 0

        while planar_error > self.parameter.threshold_planar_positioning_pixel:
            # print(planar_error)
            start = time.time()
            i += 1

            # Stop the planar positioning if the iteration exceeds 10000
            planar_iteration = planar_iteration + 1
            if planar_iteration > 10000:
                break

            # Gradually move the tooltip to the target position
            if i < task_iteration:
                td_si = self.t_start + DQ(vec4(self.td_error) * i / task_iteration)
            else:
                td_si = self.t_start + self.td_error  # Ensure the tooltip reaches the target position
            
            self.vi.set_object_translation(self.sim_setup.td_1_vrep_name, td_si)

            # Get current poses
            (xd_si, xd_lg, td_lg, rd_si, x_si, x_lg, jointx_si, jointx_lg, 
             jointx_comb, t_si, t_lg, r_si, r_lg, l_si, l_lg
            ) = pos_help.calculate_poses(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, self.sim_setup, self.vi)

            # Get the current RCM positions, eyeball rotation and set the positions          
            (rcm_current_si, rcm_current_lg, r_o_e 
            ) = pos_help.calculate_and_set_rcm_positions(
            t_si, t_lg, l_si, l_lg, self.eye, self.rcm_init_si, self.rcm_init_lg, om_kinematics, self.sim_setup, self.vi)

            # Get the shadow tip position and set the position
            shadow_tip_dq = self.eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            pos_help.set_tip_positions(self.sim_setup, self.vi, x_si, x_lg, shadow_tip_dq)

            # Update the td_error, target point, start position, and task_iteration to improve the accuracy and efficiency of reaching the target position.
            if functions.is_physical_robot():
                if i > task_iteration / 2:
                    i = 0
                    self.t_start = t_si
                    self.td_error, task_iteration = self. target_points.get_translation_error(self.target_pixel,
                                                                                              self.parameter.si_velocity_planar,
                                                                                              self.parameter.tau)
                    self.vi.set_object_translation(self.sim_setup.td_2_vrep_name, t_si + self.td_error)

            # Get Jacobians related to the current tooltip poses
            (J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg
            ) = pos_help.calculate_jacobians(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si)
           
            # Calculate the errors of the eyeball and instruments
            td_eye, e_si_t, e_si_r, e_lg_t = pos_help.calculate_errors(xd_si, xd_lg, td_si, x_si, t_si, t_lg, r_o_e, self.eye, kine_func)

            # Get the inequality constraints for safety
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.rcm_si_t,
                                                               self.eye.rcm_lg_t, self.eye.eyeball_t, self.parameter,
                                                               self.constrained_plane_list_si, self.constrained_plane_list_lg, r_o_e)
            
            # Get the inequality constraints for the proposed method
            W_conical, w_conical = EyeVFI.get_conical_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.ws_t, self.parameter)

            # Get the inequality constraints for orbital manipulation
            if self.parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(self.robot_si, self.robot_lg, self.theta_si,
                                                                                  self.theta_lg,
                                                                                  self.eye.eyeball_t, self.eye.eyeball_radius,
                                                                                  self.parameter, self.D_rcm_init,
                                                                                  self.rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg,
                                                                         self.eye.eyeball_t, self.eye.eyeball_radius, self.parameter,
                                                                         self.D_rcm_init, r_o_e, self.rcm_init_si, self.rcm_init_lg,
                                                                         self.rotation_c_plane_unified_list)
            W = np.vstack([W_vitreo,
                           W_conical,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_conical,
                           w_om])

            # Calculate the eye jacobians
            (eye_rotation_jacobian, eyeball_jacobian_t, eyeball_jacobian_r
            ) = pos_help.get_eye_jacobians(Jt_si, Jt_lg, Jl_si, Jr_rd_si, Jl_lg, t_si, t_lg, l_si, l_lg, rcm_current_si, rcm_current_lg, 
                                           self.eye, self.rcm_init_si, self.rcm_init_lg, td_eye, jointx_si, jointx_lg, om_kinematics)
            
            # Calculate the decision variables
            H, c = pos_help.decision_variable_calculation(eyeball_jacobian_t, eyeball_jacobian_r, eye_rotation_jacobian, Jt_lg, e_si_t, e_lg_t, e_si_r,
                                                          jointx_comb, jointx_si, self.parameter.n_planar, self.parameter.damping_planar, self.parameter)

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(jointx_comb)

            # Quadratic programming for the planar positioning
            delta_thetas = self.qp_solver.solve_quadratic_program(H, c, W, w, np.zeros([1, jointx_comb]), np.zeros([1, 1]))

            # Update the theta joint positions and send the target joint positions to the robots
            self.theta_si, self.theta_lg = pos_help.update_joint_positions(self.theta_si, self.theta_lg, delta_thetas, jointx_comb, 
                                                                           self.robot_si_interface, self.robot_lg_interface, self.parameter)

            norm_delta_theta = norm_delta_theta + np.linalg.norm(delta_thetas)

            # Calculate distances and store the values 
            shaft_distance, tip_distance = pos_help.store_distances(x_si, shadow_tip_dq, self.eye, self.parameter, functions, self.predict)
            
            if functions.is_physical_robot():
                planar_error = self.target_points.get_planar_error(self.target_pixel)
            else:
                x_si = self.robot_si.fkm(self.theta_si)
                t_si = translation(x_si)
                planar_error = (np.linalg.norm(vec4(self.t_start + self.td_error - t_si)) * 10 ** 3) * self.parameter.converter_per_mm

            store_data = np.hstack(
                [self.theta_si.T, self.theta_lg.T, delta_thetas, shaft_distance, tip_distance, planar_error, self.predict.counter_sum, 1,
                0, vec8(x_si), vec8(x_lg), vec8(td_si), vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
            self.store.send_store_data("kinematics", store_data)

            if self.parameter.enable_sleep:
                r.sleep()

            self. data_logger.log("hz_planar", float(1 / (time.time() - start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")

        return planar_iteration, norm_delta_theta, shadow_tip_dq


    def overlap_prevention_controller(self, t_si_above_target, threshold_overlap_prevention, shaft_distance, tip_distance):
        """
        The controller prevents the overlap between the instrument and its shadow. This can be achieved by moving the 
        light guide as far as possible from constrained plane_OP. See Section VI of Koyama et al. (2022) for more details.
        """
        i = 0
        r = rospy.Rate(self.parameter.fps)
        axis = self.parameter.axis

        while shaft_distance < threshold_overlap_prevention:
            # print(shaft_distance)
            start = time.time()
            i += 1

            # Stop the overlap prevention if the iteration exceeds 10000
            if i > 10000:
                self.fail_position_list = np.vstack([self.fail_position_list,
                                                     vec4(DQ(self.target_pixel))])
                break

            # Set the instrument's desired translation to current position as the light guide will translate
            td_si = t_si_above_target

            # Get current poses
            (xd_si, xd_lg, td_lg, rd_si, x_si, x_lg, jointx_si, jointx_lg, 
             jointx_comb, t_si, t_lg, r_si, r_lg, l_si, l_lg
            ) = pos_help.calculate_poses(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, self.sim_setup, self.vi)

           # Get the current RCM positions, eyeball rotation and set the positions          
            (rcm_current_si, rcm_current_lg, r_o_e 
            ) = pos_help.calculate_and_set_rcm_positions(
            t_si, t_lg, l_si, l_lg, self.eye, self.rcm_init_si, self.rcm_init_lg, om_kinematics, self.sim_setup, self.vi)

            # Get the shadow tip position and set the position
            shadow_tip_dq = self.eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            pos_help.set_tip_positions(self.sim_setup, self.vi, x_si, x_lg, shadow_tip_dq)

            # Get Jacobians related to the current tooltip poses
            (J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg
             ) = pos_help.calculate_jacobians(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si)

            # Get the inequality constraints for safety
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.rcm_si_t,
                                                               self.eye.rcm_lg_t, self.eye.eyeball_t, self.parameter,
                                                               self.constrained_plane_list_si, self.constrained_plane_list_lg)
            W_conical, w_conical = EyeVFI.get_conical_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.ws_t,
                                                           self.parameter)

            # Get the equality constraints
            Aeq = np.vstack([np.hstack([Jt_si, np.zeros([4, jointx_lg])]),
                             np.zeros([4, jointx_comb])])
            beq = np.zeros([8, 1])

            W = np.vstack([W_vitreo,
                           W_conical])
            w = np.vstack([w_vitreo,
                           w_conical])

            J_second_task = EyeVFI.get_second_task_distance_jacobian(Jt_si, Jt_lg, Jr_si, t_si, t_lg, r_si, jointx_comb)
            H1 = J_second_task.T @ J_second_task
            c = -2 * J_second_task.T * self.parameter.D_velocity
            
            H2 = np.vstack([np.hstack([self.parameter.damping_overlap_instrument * np.eye(jointx_si), np.zeros([jointx_si, jointx_lg])]),
                            np.hstack([np.zeros([jointx_lg, jointx_si]), self.parameter.damping_overlap_light * np.eye(jointx_lg)])])
            H = 2 * (H1 + H2)

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(jointx_comb)

            # Quadratic programming for the overlap prevention
            delta_thetas = self.qp_solver.solve_quadratic_program(H, c, W, w, Aeq, beq)

            # Update the theta joint positions and send the target joint positions to the robots
            self.theta_si, self.theta_lg = pos_help.update_joint_positions(self.theta_si, self.theta_lg, delta_thetas, jointx_comb, 
                                                                           self.robot_si_interface, self.robot_lg_interface, self.parameter)

            # Calculate distances, errors and store the values 
            shaft_distance, tip_distance = pos_help.store_distances(x_si, shadow_tip_dq, self.eye, self.parameter, functions, self.predict)
            if functions.is_physical_robot():
                planar_error = self.target_points.get_planar_error(self.target_pixel)
            else:
                planar_error = (np.linalg.norm(vec4(self.td_error))*10**3)*self.parameter.converter_per_mm

            second_task_velocity = J_second_task @ delta_thetas

            store_data = np.hstack(
                [self.theta_si.T, self.theta_lg.T, delta_thetas, shaft_distance, tip_distance, planar_error, self.predict.counter_sum, 2,
                second_task_velocity, vec8(x_si), vec8(x_lg), vec8(td_si), vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
            self.store.send_store_data("kinematics", store_data)

            if self.parameter.enable_sleep:
                r.sleep()

            self. data_logger.log("hz_overlap", float(1 / (time.time() - start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")

        if i == 0:
            print("Overlap prevention is not required.")

        return tip_distance, i


    def vertical_positioning_controller(self, t_current_si, tip_distance, depth, t_target, threshold_vertical_positioning, task_iteration, total_planar_iteration):
        """
        This controller's main goal is to get the tip of the surgical instrument onto the retina precisely, so we prioritize 
        the motion of the instrument over that of the light guide. See Section VII of Koyama et al. (2022) for more details.
        """
        i = 0
        j = 0
        r = rospy.Rate(self.parameter.fps)
        axis = self.parameter.axis

        while tip_distance >= threshold_vertical_positioning:
            start = time.time()

            # if the vertical positioning step is not completed after 10000 iterations, this step is finished.
            if i > 10000:
                break

            # Count how many times the tip distance achieves the threshold
            current_counter_sum = self.predict.counter_sum

            # Continue vertical positioning till the tip distance achieves the threshold certain times
            if current_counter_sum <= self.parameter.tip_dis_count:
                i += 1
                j += 1

                # Set the desired translation
                if i < task_iteration + 500:
                    if functions.is_physical_robot():
                        td_si = t_current_si - (depth / task_iteration * i) * k_ + DQ(vec4(self.td_error) * j / total_planar_iteration)
                    else: 
                        td_si = t_current_si - (depth / task_iteration * i) * k_
                else:
                    # td_si = td_si + DQ(vec4(self.td_error) * j / total_planar_iteration)
                    td_si = td_si

                self.vi.set_object_translation(self.sim_setup.td_1_vrep_name, td_si)

                # Get current poses
                (xd_si, xd_lg, td_lg, rd_si, x_si, x_lg, jointx_si, jointx_lg, 
                jointx_comb, t_si, t_lg, r_si, r_lg, l_si, l_lg
                ) = pos_help.calculate_poses(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, self.sim_setup, self.vi)

                # Get the current RCM positions, eyeball rotation and set the positions          
                (rcm_current_si, rcm_current_lg, r_o_e 
                ) = pos_help.calculate_and_set_rcm_positions(
                t_si, t_lg, l_si, l_lg, self.eye, self.rcm_init_si, self.rcm_init_lg, om_kinematics, self.sim_setup, self.vi)

                # Get the shadow tip position and set the position
                shadow_tip_dq = self.eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
                pos_help.set_tip_positions(self.sim_setup, self.vi, x_si, x_lg, shadow_tip_dq)

                if functions.is_physical_robot():
                    if j == total_planar_iteration:
                        j = 0
                        i = 0
                        t_current_si = td_si
                        self.td_error, total_planar_iteration = self.target_points.get_translation_error(self.target_pixel,
                                                                                                         self.parameter.si_velocity_planar2,
                                                                                                         self.parameter.tau)
                        self.vi.set_object_translation(self.sim_setup.td_2_vrep_name, t_target + self.td_error)
                        t_target = t_target + self.td_error

                # Get Jacobians related to the current tooltip poses
                (J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg
                ) = pos_help.calculate_jacobians(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si)
            
                # Calculate the errors of the eyeball and instruments
                td_eye, e_si_t, e_si_r, e_lg_t = pos_help.calculate_errors(xd_si, xd_lg, td_si, x_si, t_si, t_lg, r_o_e, self.eye, kine_func)

                # Get the inequality constraints for safety
                W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.rcm_si_t,
                                                                   self.eye.rcm_lg_t, self.eye.eyeball_t, self.parameter,
                                                                   self.constrained_plane_list_si, self.constrained_plane_list_lg)

                # Get the inequality constraints for safety
                W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.rcm_si_t,
                                                                   self.eye.rcm_lg_t, self.eye.eyeball_t, self.parameter,
                                                                   self.constrained_plane_list_si, self.constrained_plane_list_lg)
                
                # Get the inequality constraints for the proposed method
                W_conical, w_conical = EyeVFI.get_conical_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.ws_t, self.parameter)

                if self.parameter.om_version_icra_ver:
                    W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(self.robot_si, self.robot_lg, self.theta_si,
                                                                                      self.theta_lg,
                                                                                      self.eye.eyeball_t, self.eye.eyeball_radius,
                                                                                      self.parameter, self.D_rcm_init,
                                                                                      self.rotation_c_plane_list)
                else:
                    W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg,
                                                                             self.eye.eyeball_t, self.eye.eyeball_radius, self.parameter,
                                                                             self.D_rcm_init, r_o_e, self.rcm_init_si, self.rcm_init_lg,
                                                                             self.rotation_c_plane_unified_list)
                W = np.vstack([W_vitreo,
                               W_conical,
                               W_om])
                w = np.vstack([w_vitreo,
                               w_conical,
                               w_om])
                
                # Calculate the eye jacobians
                (eye_rotation_jacobian, eyeball_jacobian_t, eyeball_jacobian_r
                ) = pos_help.get_eye_jacobians(Jt_si, Jt_lg, Jl_si, Jr_rd_si, Jl_lg, t_si, t_lg, l_si, l_lg, rcm_current_si, rcm_current_lg, 
                                               self.eye, self.rcm_init_si, self.rcm_init_lg, td_eye, jointx_si, jointx_lg, om_kinematics)
                
                
                # Calculate the decision variables for vertical positioning. Instrument is calculated first, then light guide in second task.
                if self.parameter.end_effector_rotation:
                    # Quadratic coefficients of the decision variables
                    A1 = self.parameter.alpha * eyeball_jacobian_t.T @ eyeball_jacobian_t
                    A2 = (1 - self.parameter.alpha) * eyeball_jacobian_r.T @ eyeball_jacobian_r
                    H1 = A1 + A2                                                                           # instrument                                           # light guide
                    H2 = self.parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation

                    # Linear coefficients of the decision variables
                    A3 = self.parameter.alpha * eyeball_jacobian_t.T @ e_si_t.T  # instrument
                    A4 = (1 - self.parameter.alpha) * eyeball_jacobian_r.T @ e_si_r.T
                    c1 = A3 + A4  # instrument
                    c = 2 *  self.parameter.n_vertical * c1
                    
                else:
                    H1 = eyeball_jacobian_t.T @ eyeball_jacobian_t  # instrument
                    H2 = self.parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation

                    c1 = eyeball_jacobian_t.T @ e_si_t.T  # instrument
                    c = 2 * self.parameter.n_vertical * c1

                if self.parameter.solver == 0:
                    w = w.reshape(w.shape[0])
                    c = c.reshape(jointx_comb)
                
                H3 = np.vstack([np.hstack([self.parameter.damping_vertical_instrument * np.eye(jointx_si), np.zeros([jointx_si, jointx_lg])]),
                                np.hstack([np.zeros([jointx_lg, jointx_si]), self.parameter.damping_vertical_light * np.eye(jointx_lg)])])

                delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2 + H3), c, W, w, np.zeros([1, jointx_comb]), np.zeros([1, 1]))

                # Quadratic programming for the second task of the vertical positioning
                J_second_task = EyeVFI.get_second_task_distance_jacobian(Jt_si, Jt_lg, Jr_si, t_si, t_lg, r_si, jointx_comb)
                H1 = J_second_task.T @ J_second_task

                c = -(2 * J_second_task.T * self.parameter.D_velocity).flatten()

                if self.parameter.end_effector_rotation:
                    Aeq = (eyeball_jacobian_t + eyeball_jacobian_r)  #(eyeball_jacobian_t + eyeball_jacobian_r + eye_rotation_jacobian) 
                    beq = (eyeball_jacobian_t + eyeball_jacobian_r) @ delta_thetas.reshape([jointx_comb, 1])  #(eyeball_jacobian_t + eyeball_jacobian_r + eye_rotation_jacobian)
                else:
                    Aeq = eyeball_jacobian_t  #(eyeball_jacobian_t + eye_rotation_jacobian)
                    beq = eyeball_jacobian_t @ delta_thetas.reshape([jointx_comb, 1])  #(eyeball_jacobian_t + eye_rotation_jacobian)

                delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2 + H3), c, W, w, Aeq, beq)

                # Update the theta joint positions and send the target joint positions to the robots
                self.theta_si, self.theta_lg = pos_help.update_joint_positions(self.theta_si, self.theta_lg, delta_thetas, jointx_comb, 
                                                                               self.robot_si_interface, self.robot_lg_interface, self.parameter)

                # Calculate distances and store the values
                shaft_distance, tip_distance = pos_help.store_distances(x_si, shadow_tip_dq, self.eye, self.parameter, functions, self.predict)
                if functions.is_physical_robot():
                    planar_error = self.target_points.get_planar_error(self.target_pixel)
                else:
                    planar_error = (np.linalg.norm(vec4(self.td_error))*10**3)*self.parameter.converter_per_mm
                second_task_velocity = 0 # J_second_task @ delta_thetas

                store_data = np.hstack(
                    [self.theta_si.T, self.theta_lg.T, delta_thetas, shaft_distance, tip_distance, planar_error, self.predict.counter_sum, 3,
                    second_task_velocity, vec8(x_si), vec8(x_lg), vec8(td_si), vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
                self.store.send_store_data("kinematics", store_data)
            else:
                store_data = np.hstack(
                    [self.theta_si.T, self.theta_lg.T, delta_thetas, shaft_distance, tip_distance, planar_error, self.predict.counter_sum, 3,
                    0, vec8(x_si), vec8(x_lg), vec8(td_si), vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
                self.store.send_store_data("kinematics", store_data)
                break

            if self.parameter.enable_sleep:
                r.sleep()

            self. data_logger.log("hz_vertical", float(1 / (time.time() - start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1 / (time.time() - start))) + " Hz")
        
        return delta_thetas, td_si
            
    def pause_before_additional(self, delta_thetas, threshold_vertical_positioning, td_si):
        """
        This function pauses the robot before the additional positioning step to calculate the 
        additional distance that the instrument should move.
        """
        i = 0
        axis = self.parameter.axis

        while i < 1000:
            i = i + 1
            
            x_si = self.robot_si.fkm(self.theta_si)
            x_lg = self.robot_lg.fkm(self.theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            l_si = normalize(r_si * axis * conj(r_si))
            l_lg = normalize(r_lg * axis * conj(r_lg))

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg, 
                                                                                        self.eye.eyeball_t, self.eye.eyeball_radius)
        
            r_o_e = om_kinematics.get_eyeball_rotation(self.eye.eyeball_t, self.eye.eyeball_radius, 
                                                       self.rcm_init_si, self.rcm_init_lg, rcm_current_si, rcm_current_lg)
            shadow_tip_dq = self.eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            tip_distance = self.eye.get_tip_shadow_tip_distance(shadow_tip_dq, x_si)
            
            if functions.is_physical_robot():
                shaft_distance = self.predict.shaft_distance
                tip_distance = self.predict.tip_distance
                planar_error = self.target_points.get_planar_error(self.target_pixel)
            else:
                x_si = self.robot_si.fkm(self.theta_si)
                t_si = translation(x_si)
                shaft_distance, tip_distance = pos_help.store_distances(x_si, shadow_tip_dq, self.eye, self.parameter, functions, self.predict)
                planar_error = (np.linalg.norm(vec4(self.td_error))*10**3)*self.parameter.converter_per_mm
            
            store_data = np.hstack(
                [self.theta_si.T, self.theta_lg.T, delta_thetas, shaft_distance, tip_distance, planar_error, self.predict.counter_sum, 4,
                0, vec8(x_si), vec8(x_lg), vec8(td_si), vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
            self.store.send_store_data("kinematics", store_data)

        # Calculate the additional distance
        t1_vec4 = vec4(translation(self.robot_si.fkm(self.robot_si_interface.get_joint_positions())))
        t2_vec4 = vec4(translation(self.robot_lg.fkm(self.robot_lg_interface.get_joint_positions())))
        additional_depth = threshold_vertical_positioning / math.sqrt((t1_vec4[1] - t2_vec4[1])**2 + 
                           (t1_vec4[2] - t2_vec4[2])**2) * (t2_vec4[3] - t1_vec4[3]) / self.parameter.converter_per_mm
        additional_depth = additional_depth / 1000 + self.parameter.additional_positioning_margin

        # Store the additional distance
        store_data = np.array([additional_depth])
        self.store.send_store_data("additional_depth", store_data)
        time.sleep(.5)
    
        return additional_depth

    
    def additional_positioning_controller(self, t_current, additional_depth, task_iteration, total_planar_iteration):
        """
        This controller is used to add an additional distance to the vertical positioning step to ensure contact with the retina.
        """
        i = 0
        j = 0
        r = rospy.Rate(self.parameter.fps)
        axis = self.parameter.axis

        while i < task_iteration:
        # while not contact_reporter_interface.contact:
            start = time.time()
            i += 1
            j += 1

            # Set the desired translation
            # td_si = t_current - i * additional_depth/task_iteration*k_ + DQ(vec4(self.td_error) * j / total_planar_iteration)
            td_si = t_current - i * additional_depth / task_iteration
            self.vi.set_object_translation(self.sim_setup.td_1_vrep_name, td_si)

             # Get current poses
            (xd_si, xd_lg, td_lg, rd_si, x_si, x_lg, jointx_si, jointx_lg, 
             jointx_comb, t_si, t_lg, r_si, r_lg, l_si, l_lg
            ) = pos_help.calculate_poses(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, self.sim_setup, self.vi)

            # Get the current RCM positions, eyeball rotation and set the positions          
            (rcm_current_si, rcm_current_lg, r_o_e 
            ) = pos_help.calculate_and_set_rcm_positions(
            t_si, t_lg, l_si, l_lg, self.eye, self.rcm_init_si, self.rcm_init_lg, om_kinematics, self.sim_setup, self.vi)

            # Get the shadow tip position and set the position
            shadow_tip_dq = self.eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            pos_help.set_tip_positions(self.sim_setup, self.vi, x_si, x_lg, shadow_tip_dq)

            if functions.is_physical_robot():
                if j == total_planar_iteration:
                    j = 0
                    # i = 0
                    # t_current = t_si
                    self.td_error, total_planar_iteration = self.target_points.get_translation_error(self.target_pixel,
                                                                                           self.parameter.si_velocity_planar2,
                                                                                           self.parameter.tau)

            # Get Jacobians related to the current tooltip poses
            (J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg
            ) = pos_help.calculate_jacobians(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si)
           
            # Calculate the errors of the eyeball and instruments
            td_eye, e_si_t, e_si_r, e_lg_t = pos_help.calculate_errors(xd_si, xd_lg, td_si, x_si, t_si, t_lg, r_o_e, self.eye, kine_func)

             # Get the inequality constraints for safety
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.rcm_si_t,
                                                               self.eye.rcm_lg_t, self.eye.eyeball_t, self.parameter,
                                                               self.constrained_plane_list_si, self.constrained_plane_list_lg)
            
            # Get the inequality constraints for the proposed method
            W_conical, w_conical = EyeVFI.get_conical_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.ws_t, self.parameter)

            if self.parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(self.robot_si, self.robot_lg, self.theta_si,
                                                                                  self.theta_lg,
                                                                                  self.eye.eyeball_t, self.eye.eyeball_radius,
                                                                                  self.parameter, self.D_rcm_init,
                                                                                  self.rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg,
                                                                         self.eye.eyeball_t, self.eye.eyeball_radius, self.parameter,
                                                                         self.D_rcm_init, r_o_e, self.rcm_init_si, self.rcm_init_lg,
                                                                         self.rotation_c_plane_unified_list)
            W = np.vstack([W_vitreo,
                            W_conical,
                            W_om])
            w = np.vstack([w_vitreo,
                            w_conical,
                            w_om])
            
            # Calculate the eye jacobians
            (eye_rotation_jacobian, eyeball_jacobian_t, eyeball_jacobian_r
            ) = pos_help.get_eye_jacobians(Jt_si, Jt_lg, Jl_si, Jr_rd_si, Jl_lg, t_si, t_lg, l_si, l_lg, rcm_current_si, rcm_current_lg, 
                                           self.eye, self.rcm_init_si, self.rcm_init_lg, td_eye, jointx_si, jointx_lg, om_kinematics)
            
            # Calculate the decision variables for vertical positioning. Instrument is calculated first, then light guide in second task.
            if self.parameter.end_effector_rotation:
                # Quadratic coefficients of the decision variables
                A1 = self.parameter.alpha * eyeball_jacobian_t.T @ eyeball_jacobian_t
                A2 = (1 - self.parameter.alpha) * eyeball_jacobian_r.T @ eyeball_jacobian_r
                H1 = A1 + A2                                                                           # instrument    
                H2 = self.parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation

                # Linear coefficients of the decision variables
                A3 = self.parameter.alpha * eyeball_jacobian_t.T @ e_si_t.T  # instrument
                A4 = (1 - self.parameter.alpha) * eyeball_jacobian_r.T @ e_si_r.T
                c1 = A3 + A4  # instrument
                c = 2 *  self.parameter.n_vertical * c1
                
            else:
                H1 = eyeball_jacobian_t.T @ eyeball_jacobian_t  # instrument
                H2 = self.parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation

                c1 = eyeball_jacobian_t.T @ e_si_t.T  # instrument
                c = 2 * self.parameter.n_vertical * c1

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(jointx_comb)
            
            H3 = np.vstack([np.hstack([self.parameter.damping_vertical_instrument * np.eye(jointx_si), np.zeros([jointx_si, jointx_lg])]),
                            np.hstack([np.zeros([jointx_lg, jointx_si]), self.parameter.damping_vertical_light * np.eye(jointx_lg)])])

            delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2 + H3), c, W, w, np.zeros([1, jointx_comb]), np.zeros([1, 1]))

            # Quadratic programming for the second task of the vertical positioning
            J_second_task = EyeVFI.get_second_task_distance_jacobian(Jt_si, Jt_lg, Jr_si, t_si, t_lg, r_si, jointx_comb)
            H1 = J_second_task.T @ J_second_task

            c = -(2 * J_second_task.T * self.parameter.D_velocity).flatten()

            if self.parameter.end_effector_rotation:
                Aeq = (eyeball_jacobian_t + eyeball_jacobian_r)  #(eyeball_jacobian_t + eyeball_jacobian_r + eye_rotation_jacobian) 
                beq = (eyeball_jacobian_t + eyeball_jacobian_r) @ delta_thetas.reshape([jointx_comb, 1])  #(eyeball_jacobian_t + eyeball_jacobian_r + eye_rotation_jacobian)
            else:
                Aeq = eyeball_jacobian_t  #(eyeball_jacobian_t + eye_rotation_jacobian)
                beq = eyeball_jacobian_t @ delta_thetas.reshape([jointx_comb, 1])  #(eyeball_jacobian_t + eye_rotation_jacobian)

            delta_thetas = self.qp_solver.solve_quadratic_program(2 * (H1 + H2 + H3), c, W, w, Aeq, beq)

            # Update the theta joint positions and send the target joint positions to the robots
            self.theta_si, self.theta_lg = pos_help.update_joint_positions(self.theta_si, self.theta_lg, delta_thetas, jointx_comb, 
                                                                           self.robot_si_interface, self.robot_lg_interface, self.parameter)

            # Store values
            shaft_distance, tip_distance = pos_help.store_distances(x_si, shadow_tip_dq, self.eye, self.parameter, functions, self.predict)
            if functions.is_physical_robot():
                planar_error = self.target_points.get_planar_error(self.target_pixel)
            else:
                planar_error = (np.linalg.norm(vec4(self.td_error))*10**3)*self.parameter.converter_per_mm
            second_task_velocity = J_second_task @ delta_thetas

            store_data = np.hstack(
                [self.theta_si.T, self.theta_lg.T, delta_thetas, shaft_distance, tip_distance, planar_error, self.predict.counter_sum, 5,
                second_task_velocity, vec8(x_si), vec8(x_lg), vec8(td_si), vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
            self.store.send_store_data("kinematics", store_data)

            if self.parameter.enable_sleep:
                r.sleep()

            self. data_logger.log("hz_additional", float(1/(time.time()-start)))

            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")

    
    def return_to_above_point(self, t_current, trajectory_translation, task_iteration):
        """
        This function moves the surgical instrument back to the point above the target point.
        """
        i = 0
        r = rospy.Rate(self.parameter.fps)
        axis = self.parameter.axis

        while i < task_iteration:
            start = time.time()
            i += 1

            # Set the desired translation
            td_si = t_current + DQ(vec4(trajectory_translation) * i / task_iteration)
            self.vi.set_object_translation(self.sim_setup.td_1_vrep_name, td_si)
            
            # Get current poses
            (xd_si, xd_lg, td_lg, rd_si, x_si, x_lg, jointx_si, jointx_lg, 
             jointx_comb, t_si, t_lg, r_si, r_lg, l_si, l_lg
            ) = pos_help.calculate_poses(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, self.sim_setup, self.vi)

            # Get the current RCM positions, eyeball rotation and set the positions          
            (rcm_current_si, rcm_current_lg, r_o_e 
            ) = pos_help.calculate_and_set_rcm_positions(
            t_si, t_lg, l_si, l_lg, self.eye, self.rcm_init_si, self.rcm_init_lg, om_kinematics, self.sim_setup, self.vi)

            # Get the shadow tip position and set the position
            shadow_tip_dq = self.eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            pos_help.set_tip_positions(self.sim_setup, self.vi, x_si, x_lg, shadow_tip_dq)

            if functions.is_physical_robot():
                if i > task_iteration / 2:
                    i = 0
                    self.t_start = t_si
                    self.td_error, task_iteration = self.target_points.get_translation_error(self.target_pixel,
                                                                                             self.parameter.si_velocity_vertical,
                                                                                             self.parameter.tau)
                    self.vi.set_object_translation(self.sim_setup.td_2_vrep_name, t_si + self.td_error)

            # Get Jacobians related to the current tooltip poses
            (J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg
            ) = pos_help.calculate_jacobians(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si)
           
            # Calculate the errors of the eyeball and instruments
            td_eye, e_si_t, e_si_r, e_lg_t = pos_help.calculate_errors(xd_si, xd_lg, td_si, x_si, t_si, t_lg, r_o_e, self.eye, kine_func)

            # Get the inequality constraints for safety
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.rcm_si_t,
                                                               self.eye.rcm_lg_t, self.eye.eyeball_t, self.parameter,
                                                               self.constrained_plane_list_si, self.constrained_plane_list_lg)
            
            # Get the inequality constraints for the proposed method
            W_conical, w_conical = EyeVFI.get_conical_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg, self.eye.ws_t, self.parameter)

            if self.parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(self.robot_si, self.robot_lg, self.theta_si,
                                                                                  self.theta_lg,
                                                                                  self.eye.eyeball_t, self.eye.eyeball_radius,
                                                                                  self.parameter, self.D_rcm_init,
                                                                                  self.rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(self.robot_si, self.robot_lg, self.theta_si, self.theta_lg,
                                                                         self.eye.eyeball_t, self.eye.eyeball_radius, self.parameter,
                                                                         self.D_rcm_init, r_o_e, self.rcm_init_si, self.rcm_init_lg,
                                                                         self.rotation_c_plane_unified_list)
            W = np.vstack([W_vitreo,
                           W_conical,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_conical,
                           w_om])

            # Calculate the eye jacobians
            (eye_rotation_jacobian, eyeball_jacobian_t, eyeball_jacobian_r
            ) = pos_help.get_eye_jacobians(Jt_si, Jt_lg, Jl_si, Jr_rd_si, Jl_lg, t_si, t_lg, l_si, l_lg, rcm_current_si, rcm_current_lg, 
                                           self.eye, self.rcm_init_si, self.rcm_init_lg, td_eye, jointx_si, jointx_lg, om_kinematics)
            
            # Calculate the decision variables
            H, c = pos_help.decision_variable_calculation(eyeball_jacobian_t, eyeball_jacobian_r, eye_rotation_jacobian, Jt_lg, e_si_t, e_lg_t, e_si_r,
                                                          jointx_comb, jointx_si, self.parameter.n_vertical, self.parameter.damping_planar, self.parameter)

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(jointx_comb)

            delta_thetas = self.qp_solver.solve_quadratic_program(H, c, W, w, np.zeros([1, jointx_comb]), np.zeros([1, 1]))

            # Update the theta joint positions and send the target joint positions to the robots
            self.theta_si, self.theta_lg = pos_help.update_joint_positions(self.theta_si, self.theta_lg, delta_thetas, jointx_comb, 
                                                                           self.robot_si_interface, self.robot_lg_interface, self.parameter)

            if self.parameter.enable_sleep:
                r.sleep()
            if self.parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")