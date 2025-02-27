# Import necessary dependencies
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_Kinematics
import rospy
import numpy as np

"""
This script contains helper functions for positioning the robots. Poses, jacobians, and errors are calculated to determine the decision variables for 
quadratic programming. The optimization function is calculated and is updated for the robots to move to the desired positions.
"""
class PositioningHelper:
    def calculate_poses(robot_si, robot_lg, theta_si, theta_lg, axis, sim_setup, vi):
        xd_si = vi.get_object_pose(sim_setup.si_vrep_name)
        xd_lg = vi.get_object_pose(sim_setup.lg_vrep_name)
        td_lg = translation(xd_lg)
        rd_si = rotation(xd_si)
        
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        jointx_si = theta_si.size
        jointx_lg = theta_lg.size
        jointx_comb = jointx_si + jointx_lg
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))

        return xd_si, xd_lg, td_lg, rd_si, x_si, x_lg, jointx_si, jointx_lg, jointx_comb, t_si, t_lg, r_si, r_lg, l_si, l_lg


    def calculate_and_set_rcm_positions(t_si, t_lg, l_si, l_lg, eye, rcm_init_si, rcm_init_lg, om_kinematics, sim_setup, vi):
        rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg, 
                                                                                    eye.eyeball_t, eye.eyeball_radius)
        
        r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, 
                                                rcm_init_si, rcm_init_lg, rcm_current_si, rcm_current_lg)
        
        vi.set_object_rotation(sim_setup.eyeball_vrep_name, r_o_e)
        vi.set_object_translation(sim_setup.rcm_si_vrep_name, rcm_current_si)
        vi.set_object_translation(sim_setup.rcm_lg_vrep_name, rcm_current_lg)

        return rcm_current_si, rcm_current_lg, r_o_e


    def set_tip_positions(sim_setup, vi, x_si, x_lg, shadow_tip_dq):
        vi.set_object_pose(sim_setup.si_vrep_name, x_si)
        vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)
        vi.set_object_pose(sim_setup.shadow_vrep_name, shadow_tip_dq)


    def calculate_jacobians(robot_si, robot_lg, theta_si, theta_lg, axis, x_si, x_lg, r_si, r_lg, rd_si):
        J_si = robot_si.pose_jacobian(theta_si)
        Jt_si = robot_si.translation_jacobian(J_si, x_si)
        Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
        Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
        Jr_rd_si = (haminus4(rd_si) @ C4()) @ Jr_si
        J_lg = robot_lg.pose_jacobian(theta_lg)
        Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
        Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)
        Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg

        return J_si, Jt_si, Jr_si, Jl_si, Jr_rd_si, J_lg, Jt_lg, Jr_lg, Jl_lg


    def calculate_errors(xd_si, xd_lg, td_si, x_si, t_si, t_lg, r_o_e, eye, kine_func):
        td_eye = conj(r_o_e)*(td_si-eye.eyeball_t)*r_o_e                                    # Translation error of eyeball
        e_si_t = np.array([vec4(t_si - td_si)])                                             # Translation error of instrument
        e_si_r = np.array([vec4(kine_func.closest_invariant_rotation_error(x_si, xd_si))])  # Rotation error of instrument
        e_lg_t = np.array([vec4(t_lg - translation(xd_lg))])

        return td_eye, e_si_t, e_si_r, e_lg_t


    def get_eye_jacobians(Jt_si, Jt_lg, Jl_si, Jr_rd_si, Jl_lg, t_si, t_lg, l_si, l_lg, rcm_current_si, rcm_current_lg, 
                        eye, rcm_init_si, rcm_init_lg, td_eye, jointx_si, jointx_lg, om_kinematics):
        
        eye_rotation_jacobian = om_kinematics.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg, t_si, t_lg, l_si, l_lg, 
                                                                            rcm_current_si, rcm_current_lg, eye.eyeball_t, eye.eyeball_radius, 
                                                                            rcm_init_si, rcm_init_lg, jointx_si, jointx_lg)
        
        eyeball_jacobian_t = om_kinematics.get_eyeball_jacobian_translation(Jt_si, Jt_lg, Jl_si, Jl_lg, t_si, t_lg, l_si, l_lg, 
                                                                            rcm_current_si, rcm_current_lg, eye.eyeball_t, eye.eyeball_radius, 
                                                                            rcm_init_si, rcm_init_lg, td_eye, jointx_si, jointx_lg)
        
        eyeball_jacobian_r = (np.hstack([Jr_rd_si, np.zeros([4, jointx_lg])]))

        return eye_rotation_jacobian, eyeball_jacobian_t, eyeball_jacobian_r


    def decision_variable_calculation(eyeball_jacobian_t, eyeball_jacobian_r, eye_rotation_jacobian, Jt_lg, 
                                      e_si_t, e_lg_t, e_si_r, jointx_comb, jointx_si, n, damping, parameter):
        
        if parameter.end_effector_rotation:
            # Quadratic coefficients of the decision variables
            A1 = parameter.alpha * eyeball_jacobian_t.T @ eyeball_jacobian_t
            A2 = (1 - parameter.alpha) * eyeball_jacobian_r.T @ eyeball_jacobian_r
            H1 = parameter.beta * (A1 + A2)                                                   # instrument
            A3 = np.vstack([np.zeros([4, jointx_comb]),
                            np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
            H2 = (1 - parameter.beta) * A3.T @ A3                                             # light guide
            H3 = parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation
            H = 2 * ((H1 + H2 + H3) + damping * parameter.B_13)

            # Linear coefficients of the decision variables
            A4 = parameter.alpha * eyeball_jacobian_t.T @ e_si_t.T
            A5 = (1 - parameter.alpha) * eyeball_jacobian_r.T @ e_si_r.T
            c1 = parameter.beta * (A4 + A5)  # instrument
            A6 = np.vstack([np.zeros([jointx_si, 1]),
                            Jt_lg.T @ e_lg_t.T])
            c2 = (1 - parameter.beta) * A6   # light guide
            c = 2 *  n * (c1 + c2)
            
        else:
            H1 = parameter.beta * eyeball_jacobian_t.T @ eyeball_jacobian_t                   # instrument
            A1 = np.vstack([np.zeros([4, jointx_comb]),                             
                            np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
            H2 = (1 - parameter.beta) * A1.T @ A1                                             # light guide
            H3 = parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation
            H = 2 * ((H1 + H2 + H3) + damping * parameter.B_13)

            c1 = parameter.beta * (eyeball_jacobian_t.T @ e_si_t.T)  # instrument
            A2 = np.vstack([np.zeros([jointx_si, 1]),    
                            Jt_lg.T @ e_lg_t.T])
            c2 = (1 - parameter.beta) * A2  # light guide
            c = 2 * n * (c1 + c2)

        return H, c


    def update_joint_positions(theta_si, theta_lg, delta_thetas, jointx_comb, 
                            robot_si_interface, robot_lg_interface, parameter):
        
        theta_si = theta_si + delta_thetas[:theta_si.size] * parameter.tau
        theta_lg = theta_lg + delta_thetas[theta_si.size:jointx_comb] * parameter.tau
        theta_si.reshape([theta_si.size, 1])
        theta_lg.reshape([theta_lg.size, 1])

        robot_si_interface.send_target_joint_positions(theta_si)
        robot_lg_interface.send_target_joint_positions(theta_lg)

        return theta_si, theta_lg


    def store_distances(x_si, shadow_tip_dq, eye, parameter, functions, predict):
        
        if functions.is_physical_robot():
            shaft_distance = predict.shaft_distance
            tip_distance = predict.tip_distance
        else:
            shaft_distance = eye.get_shaft_shadow_tip_distance(shadow_tip_dq, x_si)
            if parameter.print_error:
                print("shaft distance: " + str(shaft_distance))
            tip_distance = eye.get_tip_shadow_tip_distance(shadow_tip_dq, x_si)
            # shaft_distance = (eye.get_shaft_shadow_tip_distance(shadow_tip_dq, x_si)*10**3)*self.parameter.converter_per_mm
            # tip_distance = eye.get_tip_shadow_tip_distance(shadow_tip_dq, x_si)
        
        return shaft_distance, tip_distance
