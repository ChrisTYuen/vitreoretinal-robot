# Import Relevant files from dqrobotics
import dqrobotics as dq
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
# from typing import List, Union

class EyesurgeryControllers:
    def __init__(self):
        self.parameter = control_parameters.Parameters()
        if self.parameter.solver == 0:
            self.qp_solver = DQ_QuadprogSolver()
        else:
            self.qp_solver = DQ_CPLEXSolver()
        self.setup = physical_parameters.EyesurgerySetup()
        self.sim_setup = physical_parameters.SimulationSetup

    def translation_controller_with_rcm_constraint(self, robot, interface, td_list, rcm_t, eyeball_t,
                                                   vrep_instrument_name, vi):
        """
        translation_controller_with_rcm(manipulator, td_set, rcm_t, interface, eyeball, constrained_plane_list, tool_tip,
         vi) controls a "manipulator" from the current position to the points stored in "td_set" in order
        satisfying the rcm constraint around the point "rcm_t" and the collision avoidance constraint for the constrained
        planes "constrained_plane_list".
        """
        # Move the instrument's tip to the points in "td_set" in order
        for td in td_list:
            # Set the desired translation
            td = eyeball_t + td

            # Get the initial translation
            theta = interface.get_joint_positions()
            jointx = theta.size
            x = robot.fkm(theta)
            t_init = translation(x)

            # Calculate the number of iterations needed to move the tip to the desired translation at a constant velocity
            total_iteration = kine_func.get_p2p_iteration(t_init, td, self.parameter.tau, self.parameter.setup_velocity)

            # Loop till the target reaches the initial point
            i = 0
            r = rospy.Rate(self.parameter.fps)
            while i < total_iteration:
                start = time.time()
                i = i + 1

                # Update the desired translation
                td_intermediate = t_init + DQ(vec4(td - t_init) * i / total_iteration)

                # Get the current pose, translation, and Jacobians
                x = robot.fkm(theta)
                t = translation(x)
                J = robot.pose_jacobian(theta)
                Jt = DQ_Kinematics.translation_jacobian(J, x)

                # Quadratic programming
                W, w = EyeVFI.get_vitreoretinal_VFIs_for_one_manipulator(robot, theta, rcm_t, eyeball_t, self.parameter)

                e = np.array([vec4(t - td_intermediate)]).T
                H = Jt.T @ Jt
                C = 2 * self.parameter.n_initialize * (Jt.T @ e)

                if self.parameter.solver == 0:
                    w = w.reshape(w.shape[0])
                    C = C.reshape(jointx)

                delta_thetas = self.qp_solver.solve_quadratic_program(2*(H + self.parameter.damping_initialize * np.eye(jointx)), C, W, w,
                                                                      np.zeros([1, jointx]), np.zeros([1, 1]))

                # Update joint position
                theta = theta + delta_thetas * self.parameter.tau
                theta.reshape([jointx, 1])
                interface.send_target_joint_positions(theta)

                # Show the updated position of the instrument's tip in the simulation
                x = robot.fkm(theta)
                vi.set_object_pose(vrep_instrument_name, x)

                if self.parameter.enable_sleep:
                    r.sleep()
                if self.parameter.print_time:
                    print(time.time()-start)

            # Show how accurate the controller could move the instrument
            print("The error of " + vrep_instrument_name + " is " + str(
                format(np.linalg.norm(vec4(translation(x) - td)) * 10 ** 3, '.3f')) + " mm.")

            if functions.is_physical_robot():
                input("[" + rospy.get_name() + "]:: Push Enter to continue...\n")
            else:
                print("[" + rospy.get_name() + "]:: Continue...\n")

    def translation_controller_with_rcm_orbital(self, robot_si, robot_lg, interface_si, interface_lg,
                                                td_si, td_lg, rd_si, rd_lg,
                                                eye, rcm_init_si, rcm_init_lg, D_rcm_init,
                                                constrained_plane_list_si, constrained_plane_list_lg,
                                                rotation_c_plane_list, rotation_c_plane_unified_list, vi, velocity):
        """
        translation_controller_with_rcm(manipulator, td_set, rcm_t, interface, eyeball, constrained_plane_list, tool_tip,
         vi) controls a "manipulator" from the current position to the points stored in "td_set" in order
        satisfying the rcm constraint around the point "rcm_t" and the collision avoidance constraint for the constrained
        planes "constrained_plane_list".
        """
        # Get the initial translation
        theta_si = interface_si.get_joint_positions()
        theta_lg = interface_lg.get_joint_positions()
        jointx_si = theta_si.size
        jointx_lg = theta_lg.size
        jointx_comb = jointx_si + jointx_lg
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        t_si_init = t_si
        t_lg_init = t_lg

        control_priority = 0.9995

        # Calculate the number of iterations needed to move the tip to the desired translation at a constant velocity
        trajectory_translation = td_si - t_si_init
        trajectory_length = np.linalg.norm(vec4(trajectory_translation))
        total_iteration = (trajectory_length * 10 ** 3 / velocity) // self.parameter.tau
        print("[" + rospy.get_name() + "]:: Tool-tip will finish to move in " + str(format(total_iteration*self.parameter.tau, '.4f')) + "s.")

        # Loop till the target reaches the initial point
        i = 0
        r = rospy.Rate(self.parameter.fps)
        while i < total_iteration:
            start = time.time()
            i = i + 1

            # Update the desired translation
            td_intermediate_si = t_si_init + DQ(vec4(td_si - t_si_init) / total_iteration * i)
            td_intermediate_lg = t_lg

            # Get the current pose, translation, and Jacobians
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            l_si = normalize(r_si * k_ * conj(r_si))
            l_lg = normalize(r_lg * k_ * conj(r_lg))
            J_si = robot_si.pose_jacobian(theta_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_si = DQ_Kinematics.translation_jacobian(J_si, x_si)
            Jt_lg = DQ_Kinematics.translation_jacobian(J_lg, x_lg)
            Jr_si = haminus4(rd_si) @ C4() @ DQ_Kinematics.rotation_jacobian(J_si)
            Jr_lg = haminus4(rd_lg) @ C4() @ DQ_Kinematics.rotation_jacobian(J_lg)

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t, eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       rcm_current_si, rcm_current_lg)
            vi.set_object_rotation(self.sim_setup.eyeball_vrep_name, r_o_e)
            vi.set_object_translation("rcm1_current", rcm_current_si)
            vi.set_object_translation("rcm2_current", rcm_current_lg)

            # Show the updated position of the instrument's tip in the simulation
            vi.set_object_pose(self.sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(self.sim_setup.lg_vrep_name, x_lg)

            # Quadratic programming
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, self.parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg, r_o_e)
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
                                                                         rotation_c_plane_unified_list)

            W = np.vstack([W_vitreo,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_om])

            e_t_si = np.array([vec4(t_si - td_intermediate_si)]).T
            e_t_lg = np.array([vec4(t_lg - td_intermediate_lg)]).T
            e_r_si = np.array([vec4(kine_func.closest_invariant_rotation_error(r_si, rd_si))]).T
            e_r_lg = np.array([vec4(kine_func.closest_invariant_rotation_error(r_lg, r_lg))]).T

            A_t = np.vstack([np.hstack([Jt_si, np.zeros([4, jointx_lg])]),
                             np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
            A_r = np.vstack([np.hstack([Jr_si, np.zeros([4, jointx_lg])]),
                             np.hstack([np.zeros([4, jointx_si]), Jr_lg])])
            H_t = A_t.T @ A_t
            H_r = A_r.T @ A_r

            e_t = np.vstack([e_t_si,
                             e_t_lg])
            e_r = np.vstack([e_r_si,
                             e_r_lg])
            C_t = 2 * 100 * (A_t.T @ e_t)
            C_r = 2 * 100 * (A_r.T @ e_r)

            H = control_priority * H_t + (1 - control_priority) * H_r + 0.01 * np.eye(jointx_comb)
            C = control_priority * C_t + (1 - control_priority) * C_r

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                C = C.reshape(jointx_comb)

            delta_thetas = self.qp_solver.solve_quadratic_program(2 * H, C, W, w,
                                                                  np.zeros([1, jointx_comb]), np.zeros([1, 1]))

            # Update joint position
            theta_si = theta_si + delta_thetas[:jointx_si] * self.parameter.tau
            theta_lg = theta_lg + delta_thetas[jointx_si:jointx_comb] * self.parameter.tau
            theta_si.reshape([jointx_si, 1])
            theta_lg.reshape([jointx_lg, 1])

            # Set joint position
            interface_si.send_target_joint_positions(theta_si)
            interface_lg.send_target_joint_positions(theta_lg)

            if self.parameter.enable_sleep:
                r.sleep()
            if self.parameter.print_time:
                print(time.time()-start)

        # Show how accurate the controller could move the instrument
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to continue...\n")
        else:
            print("[" + rospy.get_name() + "]:: Continue...\n")

        # Get the initial translation
        theta_si = interface_si.get_joint_positions()
        theta_lg = interface_lg.get_joint_positions()
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        t_lg_init = t_lg

        trajectory_translation = td_lg - t_lg_init
        trajectory_length = np.linalg.norm(vec4(trajectory_translation))
        total_iteration = (trajectory_length * 10 ** 3 / velocity) // self.parameter.tau
        print("[" + rospy.get_name() + "]:: Tool-tip will finish to move in " + str(
            format(total_iteration * self.parameter.tau, '.4f')) + "s.")

        # Loop till the target reaches the initial point
        i = 0
        r = rospy.Rate(self.parameter.fps)
        while i < total_iteration:
            start = time.time()
            i = i + 1

            # Update the desired translation
            td_intermediate_si = td_si
            td_intermediate_lg = t_lg_init + DQ(vec4(td_lg - t_lg_init) / total_iteration * i)

            # Get the current pose, translation, and Jacobians
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            l_si = normalize(r_si * k_ * conj(r_si))
            l_lg = normalize(r_lg * k_ * conj(r_lg))
            J_si = robot_si.pose_jacobian(theta_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_si = DQ_Kinematics.translation_jacobian(J_si, x_si)
            Jt_lg = DQ_Kinematics.translation_jacobian(J_lg, x_lg)
            Jr_si = haminus4(rd_si) @ C4() @ DQ_Kinematics.rotation_jacobian(J_si)
            Jr_lg = haminus4(rd_lg) @ C4() @ DQ_Kinematics.rotation_jacobian(J_lg)

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t,
                                                                                        eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       rcm_current_si, rcm_current_lg)
            vi.set_object_rotation(self.sim_setup.eyeball_vrep_name, r_o_e)
            vi.set_object_translation("rcm1_current", rcm_current_si)
            vi.set_object_translation("rcm2_current", rcm_current_lg)

            # Show the updated position of the instrument's tip in the simulation
            vi.set_object_pose(self.sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(self.sim_setup.lg_vrep_name, x_lg)

            # Quadratic programming
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, self.parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg, r_o_e)
            if self.parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(robot_si, robot_lg, theta_si,
                                                                                  theta_lg,
                                                                                  eye.eyeball_t, eye.eyeball_radius,
                                                                                  self.parameter, D_rcm_init,
                                                                                  rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(robot_si, robot_lg, theta_si, theta_lg,
                                                                         eye.eyeball_t, eye.eyeball_radius,
                                                                         self.parameter,
                                                                         D_rcm_init, r_o_e, rcm_init_si, rcm_init_lg,
                                                                         rotation_c_plane_unified_list)

            W = np.vstack([W_vitreo,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_om])

            e_t_si = np.array([vec4(t_si - td_intermediate_si)]).T
            e_t_lg = np.array([vec4(t_lg - td_intermediate_lg)]).T
            e_r_si = np.array([vec4(kine_func.closest_invariant_rotation_error(r_si, rd_si))]).T
            e_r_lg = np.array([vec4(kine_func.closest_invariant_rotation_error(r_lg, rd_lg))]).T

            A_t = np.vstack([np.hstack([Jt_si, np.zeros([4, jointx_lg])]),
                             np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
            A_r = np.vstack([np.hstack([Jr_si, np.zeros([4, jointx_lg])]),
                             np.hstack([np.zeros([4, jointx_si]), Jr_lg])])
            H_t = A_t.T @ A_t
            H_r = A_r.T @ A_r

            e_t = np.vstack([e_t_si,
                             e_t_lg])
            e_r = np.vstack([e_r_si,
                             e_r_lg])
            C_t = 2 * 100 * (A_t.T @ e_t)
            C_r = 2 * 100 * (A_r.T @ e_r)

            H = control_priority * H_t + (1 - control_priority) * H_r + 0.01 * np.eye(jointx_comb)
            C = control_priority * C_t + (1 - control_priority) * C_r

            if self.parameter.solver == 0:
                w = w.reshape(w.shape[0])
                C = C.reshape(jointx_comb)

            delta_thetas = self.qp_solver.solve_quadratic_program(2 * H, C, W, w,
                                                                  np.zeros([1, jointx_comb]), np.zeros([1, 1]))

            # Update joint position
            theta_si = theta_si + delta_thetas[:jointx_si] * self.parameter.tau
            theta_lg = theta_lg + delta_thetas[jointx_si:jointx_comb] * self.parameter.tau
            theta_si.reshape([jointx_si, 1])
            theta_lg.reshape([jointx_lg, 1])

            # Set joint position
            interface_si.send_target_joint_positions(theta_si)
            interface_lg.send_target_joint_positions(theta_lg)

            if self.parameter.enable_sleep:
                r.sleep()
            if self.parameter.print_time:
                print(time.time() - start)

        # Show how accurate the controller could move the instrument
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to continue...\n")
        else:
            print("[" + rospy.get_name() + "]:: Continue...\n")

    def set_manipulators_initial_thetas(self, robot_si, robot_lg, vi, si_interface, lg_interface):
        """
        initialize_manipulator(manipulator_si, manipulator_lg, vi, interface_si, interface_lg, tool_tip_si, tool_tip_lg,
        theta_init_si, theta_init_lg, initialize_velocity) gets both "manipulator_si" and "manipulator_lg"
        to the ideal initial poses to start a simulation study and returns the initialized joint positions
        of both manipulators. This function is used only in simulation.
        """
        if functions.is_physical_robot():
            if self.parameter.is_open_space:
                input("[" + rospy.get_name() + "]:: Move manipulators in an open space??!\n")
                theta_si = si_interface.get_joint_positions()
                theta_lg = lg_interface.get_joint_positions()

                t_si = translation(robot_si.fkm(theta_si))
                eyeball_position = 1 + 0.5 * E_ * (t_si + 0.01 * k_)

                vi.set_object_pose(self.sim_setup.eyeball_vrep_name, eyeball_position)
                rcm_si_dq, rcm_lg_dq = kine_func.get_rcm_positions(self.parameter.port_angle, self.sim_setup.port_radius,
                                                                   self.parameter.eyeball_radius, translation(eyeball_position))
                vi.set_object_pose(self.sim_setup.rcm_si_vrep_name, rcm_si_dq)
                vi.set_object_pose(self.sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
                time.sleep(.5)

                # Calculate the desired poses of both instruments' tips
                td_si, rd_si = kine_func.get_initial_pose_in_eyeball(eyeball_position, self.parameter.eyeball_radius,
                                                                     self.setup.insertion_distance, rcm_si_dq)
                td_lg, rd_lg = kine_func.get_initial_pose_in_eyeball(eyeball_position, self.parameter.eyeball_radius,
                                                                     self.setup.insertion_distance, rcm_lg_dq)

                # Calculate the number of iterations needed to move the instrument's tip to the desired pose at a constant velocity
                theta_si = self.pose_controller(theta_si, td_si, rd_si, robot_si, si_interface, vi, self.sim_setup.si_vrep_name,
                                                self.parameter.tau, self.sim_setup.initialize_velocity)
                theta_lg = self.pose_controller(theta_lg, td_lg, rd_lg, robot_lg, lg_interface, vi, self.sim_setup.lg_vrep_name,
                                                self.parameter.tau, self.sim_setup.initialize_velocity)

                print("[" + rospy.get_name() + "]:: Completed!\n")

            else:
                theta_si = si_interface.get_joint_positions()
                theta_lg = lg_interface.get_joint_positions()
        else:
            if self.parameter.is_open_space:
                print("[" + rospy.get_name() + "]:: Move manipulators in an open space??!\n")
                theta_si = si_interface.get_joint_positions()
                theta_lg = lg_interface.get_joint_positions()

                t_si = translation(robot_si.fkm(theta_si))
                eyeball_position = 1 + 0.5 * E_ * (t_si + 0.01 * k_)

                vi.set_object_pose(self.sim_setup.eyeball_vrep_name, eyeball_position)
                rcm_si_dq, rcm_lg_dq = kine_func.get_rcm_positions(self.parameter.port_angle, self.sim_setup.port_radius,
                                                                   self.parameter.eyeball_radius, translation(eyeball_position))
                vi.set_object_pose(self.sim_setup.rcm_si_vrep_name, rcm_si_dq)
                vi.set_object_pose(self.sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
                time.sleep(.5)

                # Calculate the desired poses of both instruments' tips
                td_si, rd_si = kine_func.get_initial_pose_in_eyeball(eyeball_position, self.parameter.eyeball_radius,
                                                                     self.setup.insertion_distance, rcm_si_dq)
                td_lg, rd_lg = kine_func.get_initial_pose_in_eyeball(eyeball_position, self.parameter.eyeball_radius,
                                                                     self.setup.insertion_distance, rcm_lg_dq)

                # Calculate the number of iterations needed to move the instrument's tip to the desired pose at a constant velocity
                theta_si = self.pose_controller(theta_si, td_si, rd_si, robot_si, si_interface, vi, self.sim_setup.si_vrep_name,
                                                self.parameter.tau, self.sim_setup.initialize_velocity_sim)
                theta_lg = self.pose_controller(theta_lg, td_lg, rd_lg, robot_lg, lg_interface, vi, self.sim_setup.lg_vrep_name,
                                                self.parameter.tau, self.sim_setup.initialize_velocity_sim)

                print("[" + rospy.get_name() + "]:: Completed!\n")

            else:
                print("[" + rospy.get_name() + "]:: Initialize manipulators in simulation ...")

                # Set the manipulators to the joint positions, "theta_init_si" and "theta_init_lg", where the initialization starts
                theta_init_si = self.sim_setup.theta_init_si
                theta_init_lg = self.sim_setup.theta_init_lg
                theta_si = theta_init_si
                theta_lg = theta_init_lg
                jointx_si = theta_si.size
                jointx_lg = theta_lg.size
                theta_init_si.reshape([jointx_si, 1])
                theta_init_lg.reshape([jointx_lg, 1])

                # Set the eyeball to the ideal position "eyeball_position" in simulation and calculate the rcm positions
                eyeball_position = self.sim_setup.eyeball_position
                vi.set_object_pose(self.sim_setup.eyeball_vrep_name, eyeball_position)
                rcm_si_dq, rcm_lg_dq = kine_func.get_rcm_positions(self.parameter.port_angle, self.sim_setup.port_radius,
                                                                   self.parameter.eyeball_radius, translation(eyeball_position))
                vi.set_object_pose(self.sim_setup.rcm_si_vrep_name, rcm_si_dq)
                vi.set_object_pose(self.sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
                time.sleep(.5)

                # Calculate the desired poses of both instruments' tips
                td_si, rd_si = kine_func.get_initial_pose_in_eyeball(eyeball_position, self.parameter.eyeball_radius,
                                                                     self.setup.insertion_distance, rcm_si_dq)
                td_lg, rd_lg = kine_func.get_initial_pose_in_eyeball(eyeball_position, self.parameter.eyeball_radius,
                                                                     self.setup.insertion_distance, rcm_lg_dq)

                # Calculate the number of iterations needed to move the instrument's tip to the desired pose at a constant velocity
                theta_si = self.pose_controller(theta_si, td_si, rd_si, robot_si, si_interface, vi, self.sim_setup.si_vrep_name,
                                                self.parameter.tau, self.sim_setup.initialize_velocity_sim)
                theta_lg = self.pose_controller(theta_lg, td_lg, rd_lg, robot_lg, lg_interface, vi, self.sim_setup.lg_vrep_name,
                                                self.parameter.tau, self.sim_setup.initialize_velocity_sim)

                print("[" + rospy.get_name() + "]:: Completed!\n")

        return theta_si, theta_lg

    def pose_controller(self, theta_init, td, rd, robot, robot_interface, vi, vrep_instrument_name, tau, velocity):
        x_init = robot.fkm(theta_init)
        t_init = translation(x_init)
        total_iteration = kine_func.get_p2p_iteration(t_init, td, tau, velocity)

        # Loop till the surgical instrument reaches the initial pose
        theta = theta_init
        jointx = theta.size
        print("This robot has this many joints:", jointx)
        i = 0
        rate = rospy.Rate(self.parameter.fps)

        # Loop till the target reaches the initial point
        while i < total_iteration:
            i = i + 1

            # Update the desired pose
            td_intermediate = t_init + DQ(vec4(td - t_init) / total_iteration * i)

            # Get the current translation and rotation
            x = robot.fkm(theta)
            t = translation(x)
            r = rotation(x)

            vi.set_object_pose(vrep_instrument_name, x)
            Jx = robot.pose_jacobian(theta)

            theta_limit_minu_joint = self.parameter.theta_limit_minu[0, 0: jointx]
            theta_limit_plu_joint = self.parameter.theta_limit_plu[0, 0: jointx]
            J_theta_limit = EyeVFI.get_joint_limit_jacobian(7)
            d_theta_limit = EyeVFI.get_joint_limit_distance(theta_limit_minu_joint,
                                                            theta_limit_plu_joint, theta)
          
            W = np.vstack([
                -np.eye(jointx),
                np.eye(jointx)
            ])
            w = np.hstack([
                -(theta_limit_minu_joint - theta), (theta_limit_minu_joint - theta)
            ])

            Jr = DQ_Kinematics.rotation_jacobian(Jx)
            Jt = DQ_Kinematics.translation_jacobian(Jx, x)
            tdx = DQ(vec4(td))
            rdx = DQ(vec4(rd))
            
            xd = rdx + 0.5 * E_ * tdx * rdx

            Jr_rd = haminus4(rotation(xd)) @ C4() @ Jr

            Ht = 2 * (Jt.T @ Jt)
            Hr = 2 * (Jr_rd.T @ Jr_rd)
            Hj = self.parameter.damping * np.identity(jointx)  # jointx dim unit matrix
            H = self.parameter.alpha * Ht + (1 - self.parameter.alpha) * Hr + Hj

            def closest_invariant_rotation_error(x, xd):
                er_plus_norm = np.linalg.norm(dq.vec4(dq.conj(dq.rotation(x)) * dq.rotation(xd) - 1))
                er_minus_norm = np.linalg.norm(dq.vec4(dq.conj(dq.rotation(x)) * dq.rotation(xd) + 1))

                if er_plus_norm < er_minus_norm:
                    er = dq.conj(dq.rotation(x)) * dq.rotation(xd) - 1

                else:
                    er = dq.conj(dq.rotation(x)) * dq.rotation(xd) + 1

                return er

            err_t_vec = vec4(translation(x) - translation(xd))
            err_r_vec = vec4(closest_invariant_rotation_error(x, xd))

            ct = 2 * self.parameter.n * err_t_vec.T @ Jt
            cr = 2 * self.parameter.n * err_r_vec.T @ Jr_rd
            
            C = np.hstack([
                (self.parameter.alpha) * ct + (1 - self.parameter.alpha) * cr
            ])
            C = C.reshape(jointx, )

            W = np.vstack([
                W,
            ])
            w = np.hstack([
                w,
            ])

            u = self.qp_solver.solve_quadratic_program(2*H, C, np.zeros([1, jointx]), np.zeros([1, 1]),
                                                                  np.zeros([1, jointx]), np.zeros([1, 1]))

            theta = theta + u * self.parameter.tau

            robot_interface.send_target_joint_positions(theta) 

            if self.parameter.enable_sleep:
                rate.sleep()
    
        # Show how accurate the initialization of the surgical instrument was performed
        x = robot.fkm(theta)
        t = translation(x)
        print("The error of " + vrep_instrument_name + " is " + str(
            format(np.linalg.norm(vec4(t - td_intermediate))*10**3, '.3f')) + " mm.")

        return theta
