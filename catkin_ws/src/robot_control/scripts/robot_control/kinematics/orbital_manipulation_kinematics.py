# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.utils import DQ_Geometry
from dqrobotics.robot_modeling import DQ_Kinematics

# Import other dependencies
import numpy as np
import math


class OrbitalManipulationKinematics:
    """

    """

    @classmethod
    def get_current_inserted_length(cls, t, l, t_eye, eye_radius):
        """
        The distances between the tips and the RCM positions as d_i. See also Vitreoretinal Surgical Robotic System with 
        Autonomous Orbital Manipulation using Vector-Field Inequalities described in V of Koyama et al. (2023), Equation 11.
        """
        g = np.dot(vec4(t - t_eye), vec4(l))
        c = g ** 2 + eye_radius ** 2 - (np.linalg.norm(vec4(t - t_eye))) ** 2
        d = g + math.sqrt(c)
        return d

    @classmethod
    def get_current_inserted_length_jacobian(cls, Jt, Jl, t, l, t_eye, eye_radius):
        """
        The Jacobian distance between the tips and the RCM positions as Jd_i. See also Vitreoretinal Surgical Robotic System with 
        Autonomous Orbital Manipulation using Vector-Field Inequalities described in V of Koyama et al. (2023), Equation 18.
        """
        # d_si, d_lg (d_R1, d_R2)
        g = np.dot(vec4(t - t_eye), vec4(l))
        h1 = math.sqrt(g ** 2 + eye_radius ** 2 - (np.linalg.norm(vec4(t - t_eye))) ** 2)  # Equation 11

        Jh_2 = vec4(l) @ Jt + vec4(t - t_eye) @ Jl  # Equation 16
        Jh_1 = 1/h1*(g*Jh_2-vec4(t - t_eye) @ Jt)  # Equation 17 from 15 and 16
        J_d = Jh_1 + Jh_2
        return J_d

    @classmethod
    def get_current_rcm_translations(cls, t_si, t_lg, l_si, l_lg, t_eye, eye_radius):
        """
        The translations of the RCM points of both instruments as t_o_i. See also Vitreoretinal Surgical Robotic System with 
        Autonomous Orbital Manipulation using Vector-Field Inequalities described in V of Koyama et al. (2023), Equation 10.
        """
        # d_si, d_lg (d_R1, d_R2)
        d_si = cls.get_current_inserted_length(t_si, l_si, t_eye, eye_radius)
        d_lg = cls.get_current_inserted_length(t_lg, l_lg, t_eye, eye_radius)

        # si_rcmt, lg_rcmt (t_o_rcm1, t_o_rcm2)
        t_o_rcm1 = t_si - d_si * l_si
        t_o_rcm2 = t_lg - d_lg * l_lg
        return t_o_rcm1, t_o_rcm2

    @classmethod
    def get_current_rcm_translation_jacobians(cls, Jt_si, Jt_lg, Jl_si, Jl_lg, J_d_si, J_d_lg,
                                              l_si, l_lg, d_si, d_lg, jointx_si, jointx_lg):
        """
        The Jacobians of the RCM translations of both instruments as Jt_o_i. See also Vitreoretinal Surgical Robotic System with 
        Autonomous Orbital Manipulation using Vector-Field Inequalities described in V of Koyama et al. (2023), Equations 19.
        """
        J_t_o_rcm1 = Jt_si - vec4(l_si).reshape([4, 1]) @ J_d_si.reshape([1, jointx_si]) - d_si * Jl_si
        J_t_o_rcm2 = Jt_lg - vec4(l_lg).reshape([4, 1]) @ J_d_lg.reshape([1, jointx_lg]) - d_lg * Jl_lg
        return J_t_o_rcm1, J_t_o_rcm2

    @staticmethod
    def get_eyeball_rotation(eyeball_center, eyeball_radius, rcm_si_init, rcm_lg_init, rcm_si_current, rcm_lg_current):
        """
        Calculate the rotation of the eyeball by using the geometry of the eye and the RCM points.
        """
        # RCMs in the eyeball coordinate
        u_om1_init = normalize(rcm_si_init - eyeball_center)
        u_om2_init = normalize(rcm_lg_init - eyeball_center)
        u_o_om1 = normalize(rcm_si_current - eyeball_center)
        u_o_om2 = normalize(rcm_lg_current - eyeball_center)

        # r_o_1
        dot_rcm_si = np.dot(vec4(u_om1_init), vec4(u_o_om1))
        r_o_m_cos = np.sqrt((dot_rcm_si + 1) / 2)
        cross_rcm_si = cross(u_om1_init, u_o_om1)
        r_o_m = normalize(r_o_m_cos + DQ(vec4(cross_rcm_si) / (2 * r_o_m_cos)))

        # r_1_e
        h = np.dot(vec4(u_om1_init), vec4(u_om2_init)) * eyeball_radius * u_om1_init
        u_m_h_init = normalize(eyeball_radius * u_om2_init - h)
        u_m_h = normalize(eyeball_radius * conj(r_o_m) * u_o_om2 * r_o_m - h)
        dot_rcm_lg = np.dot(vec4(u_m_h_init), vec4(u_m_h))
        cross_rcm_lg = cross(u_m_h_init, u_m_h)
        r_m_e_cos = np.sqrt((dot_rcm_lg + 1) / 2)
        r_m_e = normalize(r_m_e_cos + DQ(vec4(cross_rcm_lg) / (2 * r_m_e_cos)))

        # r_o_e
        r_o_e = normalize(r_o_m * r_m_e)

        return r_o_e

    @classmethod
    def get_eyeball_rotation_jacobian(cls, Jt_si, Jt_lg, Jl_si, Jl_lg,
                                      t_si, t_lg, l_si, l_lg, t_o_rcm1, t_o_rcm2,
                                      t_eye, eye_radius, rcm_init_si, rcm_init_lg, jointx_si, jointx_lg):
        # d_si, d_lg (d_R1, d_R2)
        d_si = cls.get_current_inserted_length(t_si, l_si, t_eye, eye_radius)
        J_d_si = cls.get_current_inserted_length_jacobian(Jt_si, Jl_si, t_si, l_si, t_eye, eye_radius)
        d_lg = cls.get_current_inserted_length(t_lg, l_lg, t_eye, eye_radius)
        J_d_lg = cls.get_current_inserted_length_jacobian(Jt_lg, Jl_lg, t_lg, l_lg, t_eye, eye_radius)

        # si_rcmt, lg_rcmt (t_o_rcm1, t_o_rcm2)
        J_t_o_rcm1, J_t_o_rcm2 = cls.get_current_rcm_translation_jacobians(Jt_si, Jt_lg, Jl_si, Jl_lg, J_d_si, J_d_lg,
                                                                           l_si, l_lg, d_si, d_lg, jointx_si, jointx_lg)

        # unit vectors
        u_om1_init = normalize(rcm_init_si - t_eye)
        u_om2_init = normalize(rcm_init_lg - t_eye)
        u_o_om1 = normalize(t_o_rcm1 - t_eye)
        J_u_o_om1 = J_t_o_rcm1 / eye_radius

        u_o_om2 = normalize(t_o_rcm2 - t_eye)
        J_u_o_om2 = J_t_o_rcm2 / eye_radius

        # eye rotation first (r_o_m)
        dot_rcm_si = np.dot(vec4(u_om1_init), vec4(u_o_om1))
        a_1 = cross(u_om1_init, u_o_om1)
        a_2 = np.sqrt((dot_rcm_si + 1) / 2)
        r_o_m = normalize(a_2 + DQ(vec4(a_1) / (2 * a_2)))

        # eye rotation second (r_m_e)
        h = np.dot(vec4(u_om1_init), vec4(u_om2_init)) * eye_radius * u_om1_init
        u_m_h_init = normalize(eye_radius * u_om2_init - h)
        u_m_om2 = conj(r_o_m) * u_o_om2 * r_o_m
        u_m_h = normalize(eye_radius * u_m_om2 - h)
        dot_rcm_lg = np.dot(vec4(u_m_h_init), vec4(u_m_h))
        b_1 = cross(u_m_h_init, u_m_h)
        b_2 = np.sqrt((dot_rcm_lg + 1) / 2)
        r_m_e = normalize(b_2 + DQ(vec4(b_1) / (2 * b_2)))

        # time derivative eye rotation first (r_o_m)
        A_3 = (np.array([1, 0, 0, 0])-1/(2*a_2**2)*vec4(a_1)).reshape([4, 1])
        A_4 = crossmatrix4(u_om1_init)
        A_5 = (1/(4*a_2)*vec4(u_om1_init)).reshape([1, 4])

        J_r_o_m = (A_3@A_5+1/(2*a_2) * A_4) @ J_u_o_om1

        # lg_rcm_unit_second (u_m_om2)
        B_6 = haminus4(u_o_om2 * r_o_m) @ C4() + hamiplus4(conj(r_o_m) * u_o_om2)
        J_u_m_om2 = np.hstack([B_6 @ J_r_o_m, haminus4(r_o_m) @ hamiplus4(conj(r_o_m)) @ J_u_o_om2])

        # eye rotation second (r_m_e)
        c_1 = eye_radius * u_m_om2 - h
        c_1_norm = np.linalg.norm(vec4(c_1))

        J_u_m_h = (np.eye(4)/c_1_norm-vec4(c_1).reshape([4, 1])@vec4(c_1).reshape([1, 4])/(c_1_norm**3))*eye_radius@J_u_m_om2

        B_3 = (np.array([1, 0, 0, 0])-1/(2*b_2**2)*vec4(b_1)).reshape([4, 1])
        B_4 = crossmatrix4(u_m_h_init)
        B_5 = (1/(4*b_2)*vec4(u_m_h_init)).reshape([1, 4])

        J_r_m_e = (B_3 @ B_5 + 1/(2*b_2)*B_4) @ J_u_m_h

        # eye rotation (r_o_e)
        J_r_o_e = np.hstack([haminus4(r_m_e) @ J_r_o_m, np.zeros([4, jointx_lg])]) + hamiplus4(r_o_m) @ J_r_m_e

        return J_r_o_e

    @classmethod
    def get_eyeball_jacobian_translation(cls, Jt_si, Jt_lg, Jl_si, Jl_lg,
                                         t_si, t_lg, l_si, l_lg, t_o_rcm1, t_o_rcm2,
                                         t_eye, eye_radius, rcm_init_si, rcm_init_lg, td_eye, jointx_si, jointx_lg):
        """

        """
        # eye rotation (r_o_e)
        r_o_e = cls.get_eyeball_rotation(t_eye, rcm_init_si, rcm_init_lg, t_o_rcm1, t_o_rcm2, eye_radius)
        J_r_o_e = cls.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg, t_si, t_lg,
                                                    l_si, l_lg, t_o_rcm1, t_o_rcm2,
                                                    t_eye, eye_radius, rcm_init_si, rcm_init_lg, jointx_si, jointx_lg)

        # desired translation (t_o_d)
        J_t_o_d = (haminus4(td_eye*conj(r_o_e))+hamiplus4(r_o_e*td_eye)@C4())@J_r_o_e

        # eyeball jacobian
        J_eyeball = np.hstack([Jt_si, np.zeros([4, jointx_lg])]) - J_t_o_d

        return J_eyeball

    @classmethod
    def get_rcm_points_squared_distance(cls, t_si, t_lg, r_si, r_lg, t_eye, eye_radius, rcm_init_si, rcm_init_lg):
        """

        """
        # d_si, d_lg (d_R1, d_R2)
        g_si = np.dot(vec4(t_si - t_eye), vec4(r_si * k_ * conj(r_si)))
        c_si = g_si ** 2 + eye_radius ** 2 - (np.linalg.norm(vec4(t_si - t_eye))) ** 2
        d_si = g_si + math.sqrt(c_si)

        g_lg = np.dot(vec4(t_lg - t_eye), vec4(r_lg * k_ * conj(r_lg)))
        c_lg = g_lg ** 2 + eye_radius ** 2 - (np.linalg.norm(vec4(t_lg - t_eye))) ** 2
        d_lg = g_lg + math.sqrt(c_lg)

        # si_rcmt, lg_rcmt (t_o_rcm1, t_o_rcm2)
        t_o_rcm1 = t_si - d_si * (r_si * k_ * conj(r_si))
        t_o_rcm2 = t_lg - d_lg * (r_lg * k_ * conj(r_lg))

        D = np.linalg.norm(vec4(t_o_rcm1-t_o_rcm2))**2 - np.linalg.norm(vec4(rcm_init_si-rcm_init_lg))**2
        return D

    @classmethod
    def get_eye_rotation_constraint_jacobian(cls, r_eye, J_r_eye, safe_theta):
        """

        """
        # eyeball axis
        l_eye = r_eye * k_ * conj(r_eye)
        J_l_eye = (haminus4(k_ * conj(r_eye)) + hamiplus4(r_eye * k_) @ C4()) @ J_r_eye
        J = 2 * np.dot(vec4(l_eye), vec4(k_)) * np.array([vec4(k_)]) @ J_l_eye

        return J

    @classmethod
    def get_eye_rotation_constraint_distance(cls, r_eye, safe_theta):
        """

        """
        # eyeball axis
        l_eye = normalize(r_eye * k_ * conj(r_eye))
        d = (np.dot(vec4(l_eye), vec4(k_)))**2 - (math.cos(np.deg2rad(safe_theta)))**2

        return d

    @classmethod
    def get_tip_to_moving_trocar_distance_error(cls, t_trocar, t_tip, D_safe_trocar):
        """
        Distance error between the corresponding RCM and tool tip.
        """
        # Calculate the squared distance between the tip and the trocar
        D_trocar_to_tip = (np.linalg.norm(vec4(t_trocar - t_tip))) ** 2

        # Calculate the error
        D_error = D_trocar_to_tip - D_safe_trocar
        return D_error

    @classmethod
    def get_tip_to_moving_trocar_distance_jacobian(cls, Jt_trocar, Jt_tip, t_trocar, t_tip, jointx):
        """
        Jacobian of the distance between the corresponding RCM and tool tip.
        """
        # Calculate the trocar-to-tip distance Jacobian
        J = vec4(t_trocar - t_tip) @ (Jt_trocar - Jt_tip)
        return J.reshape([1, jointx])

    @classmethod
    def get_orbital_manipulation_VFIs_icra2023(cls, denso_robot_si, denso_robot_lg, theta_si, theta_lg,
                                               eyeball_t, eyeball_radius, parameter, D_rcm_init,
                                               rotation_c_plane_list):
        # Get the pose of the current tooltip pose
        x_si = denso_robot_si.fkm(theta_si)
        x_lg = denso_robot_lg.fkm(theta_lg)
        jointx_si = theta_si.size
        jointx_lg = theta_lg.size
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)

        axis = k_
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))
        d_si = cls.get_current_inserted_length(t_si, l_si, eyeball_t, eyeball_radius)
        d_lg = cls.get_current_inserted_length(t_lg, l_lg, eyeball_t, eyeball_radius)
        t_trocar_si, t_trocar_lg = cls.get_current_rcm_translations(t_si, t_lg, l_si, l_lg, eyeball_t, eyeball_radius)

        # Get Jacobians related to the current tooltip poses
        J_si = denso_robot_si.pose_jacobian(theta_si)
        J_lg = denso_robot_lg.pose_jacobian(theta_lg)
        Jt_si = denso_robot_si.translation_jacobian(J_si, x_si)
        Jt_lg = denso_robot_lg.translation_jacobian(J_lg, x_lg)
        Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
        Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)

        Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
        Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg
        J_d_si = cls.get_current_inserted_length_jacobian(Jt_si, Jl_si, t_si, l_si, eyeball_t, eyeball_radius)
        J_d_lg = cls.get_current_inserted_length_jacobian(Jt_lg, Jl_lg, t_lg, l_lg, eyeball_t, eyeball_radius)

        Jt_trocar_si, Jt_trocar_lg = cls.get_current_rcm_translation_jacobians(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                                               J_d_si, J_d_lg, l_si, l_lg,
                                                                               d_si, d_lg, jointx_si, jointx_lg)

        # Get jacobians and distances for vitreoretinal environment constraints
        J_trocar_si = cls.get_tip_to_moving_trocar_distance_jacobian(Jt_trocar_si, Jt_si, t_trocar_si, t_si, jointx_si)
        J_trocar_lg = cls.get_tip_to_moving_trocar_distance_jacobian(Jt_trocar_lg, Jt_lg, t_trocar_lg, t_lg, jointx_lg)

        J_D_om = 2 * vec4(t_trocar_si - t_trocar_lg) @ np.hstack([Jt_trocar_si, -1 * Jt_trocar_lg])  # Equation 20
        J_rot_plane_1_si = DQ_Kinematics.point_to_plane_distance_jacobian(Jt_trocar_si, t_trocar_si, rotation_c_plane_list[0])
        J_rot_plane_1_lg = DQ_Kinematics.point_to_plane_distance_jacobian(Jt_trocar_lg, t_trocar_lg, rotation_c_plane_list[0])
        J_rot_plane_2_si = DQ_Kinematics.point_to_plane_distance_jacobian(Jt_trocar_si, t_trocar_si, rotation_c_plane_list[1])
        J_rot_plane_2_lg = DQ_Kinematics.point_to_plane_distance_jacobian(Jt_trocar_lg, t_trocar_lg, rotation_c_plane_list[1])

        D_om = np.linalg.norm(vec4(t_trocar_si - t_trocar_lg)) ** 2  # Equation 8
        D_trocar_si = cls.get_tip_to_moving_trocar_distance_error(t_trocar_si, t_si, parameter.D_safe_trocar)
        D_trocar_lg = cls.get_tip_to_moving_trocar_distance_error(t_trocar_lg, t_lg, parameter.D_safe_trocar)
        d_rot_plane_1_si = DQ_Geometry.point_to_plane_distance(t_trocar_si, rotation_c_plane_list[0])
        d_rot_plane_1_lg = DQ_Geometry.point_to_plane_distance(t_trocar_lg, rotation_c_plane_list[0])
        d_rot_plane_2_si = DQ_Geometry.point_to_plane_distance(t_trocar_si, rotation_c_plane_list[1])
        d_rot_plane_2_lg = DQ_Geometry.point_to_plane_distance(t_trocar_lg, rotation_c_plane_list[1])
        # print("W")
        # print("1", np.hstack([-1 * J_trocar_si, np.zeros([1, jointx_lg])]) * parameter.om_trocar_si)
        # print("2", np.hstack([np.zeros([1, jointx_si]), -1 * J_trocar_lg]) * parameter.om_trocar_lg)
        # print("3", J_D_om.reshape([1, jointx_si + jointx_lg]) * parameter.om_D_rcm)
        # print("4", -1 * J_D_om.reshape([1, jointx_si + jointx_lg]) * parameter.om_D_rcm)
        # print("5", np.hstack([-1 * J_rot_plane_1_si, np.zeros([1, jointx_lg])]) * parameter.om_rot_plane)
        # print("6", np.hstack([np.zeros([1, jointx_si]), -1 * J_rot_plane_1_lg]) * parameter.om_rot_plane)
        # print("7", np.hstack([-1 * J_rot_plane_2_si, np.zeros([1, jointx_lg])]) * parameter.om_rot_plane)
        # print("8", np.hstack([np.zeros([1, jointx_si]), -1 * J_rot_plane_2_lg]) * parameter.om_rot_plane)
        # print("w")
        # print("1", np.array([parameter.nd_trocar * D_trocar_si]) * parameter.om_trocar_si)
        # print("2", np.array([parameter.nd_trocar * D_trocar_lg]) * parameter.om_trocar_lg)
        # print("3", np.array([-1 * parameter.nd_rcm * (D_om - D_rcm_init - parameter.D_safe_rcm_om)]) * parameter.om_D_rcm)
        # print("4", np.array([parameter.nd_rcm * (D_om - D_rcm_init + parameter.D_safe_rcm_om)]) * parameter.om_D_rcm)
        # print("5", np.array([parameter.nd_rotation_constraint * d_rot_plane_1_si]) * parameter.om_rot_plane)
        # print("6", np.array([parameter.nd_rotation_constraint * d_rot_plane_1_lg]) * parameter.om_rot_plane)
        # print("7", np.array([parameter.nd_rotation_constraint * d_rot_plane_2_si]) * parameter.om_rot_plane)
        # print("8", np.array([parameter.nd_rotation_constraint * d_rot_plane_2_lg]) * parameter.om_rot_plane)

        W = np.vstack([np.hstack([-1 * J_trocar_si, np.zeros([1, jointx_lg])]) * parameter.om_trocar_si,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_trocar_lg]) * parameter.om_trocar_lg,
                       J_D_om.reshape([1, jointx_si + jointx_lg]) * parameter.om_D_rcm,
                       -1 * J_D_om.reshape([1, jointx_si + jointx_lg]) * parameter.om_D_rcm,
                       np.hstack([-1 * J_rot_plane_1_si, np.zeros([1, jointx_lg])]) * parameter.om_rot_plane,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_rot_plane_1_lg]) * parameter.om_rot_plane,
                       np.hstack([-1 * J_rot_plane_2_si, np.zeros([1, jointx_lg])]) * parameter.om_rot_plane,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_rot_plane_2_lg]) * parameter.om_rot_plane,
                       ])
        w = np.vstack([np.array([parameter.nd_trocar * D_trocar_si]) * parameter.om_trocar_si,
                       np.array([parameter.nd_trocar * D_trocar_lg]) * parameter.om_trocar_lg,
                       np.array([-1 * parameter.nd_rcm * (D_om - D_rcm_init - parameter.D_safe_rcm_om)]) * parameter.om_D_rcm,
                       np.array([parameter.nd_rcm * (D_om - D_rcm_init + parameter.D_safe_rcm_om)]) * parameter.om_D_rcm,
                       np.array([parameter.nd_rotation_constraint * d_rot_plane_1_si]) * parameter.om_rot_plane,
                       np.array([parameter.nd_rotation_constraint * d_rot_plane_1_lg]) * parameter.om_rot_plane,
                       np.array([parameter.nd_rotation_constraint * d_rot_plane_2_si]) * parameter.om_rot_plane,
                       np.array([parameter.nd_rotation_constraint * d_rot_plane_2_lg]) * parameter.om_rot_plane,
                       ])
        # print("orbital manipulation VFIs are calculated (ICRA)")

        return W, w

    @classmethod
    def get_orbital_manipulation_VFIs(cls, denso_robot_si, denso_robot_lg, theta_si, theta_lg,
                                      eyeball_t, eyeball_radius, parameter, D_rcm_init, r_eye,
                                      rcm_init_si, rcm_init_lg, rotation_c_plane_unified_list):
        # Get the pose of the current tooltip pose
        x_si = denso_robot_si.fkm(theta_si)
        x_lg = denso_robot_lg.fkm(theta_lg)
        jointx_si = theta_si.size
        jointx_lg = theta_lg.size
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)

        axis = k_
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))
        d_si = cls.get_current_inserted_length(t_si, l_si, eyeball_t, eyeball_radius)
        d_lg = cls.get_current_inserted_length(t_lg, l_lg, eyeball_t, eyeball_radius)
        rcm_current_si, rcm_current_lg = cls.get_current_rcm_translations(t_si, t_lg, l_si, l_lg, eyeball_t, eyeball_radius)

        # Get Jacobians related to the current tooltip poses
        J_si = denso_robot_si.pose_jacobian(theta_si)
        J_lg = denso_robot_lg.pose_jacobian(theta_lg)
        Jt_si = denso_robot_si.translation_jacobian(J_si, x_si)
        Jt_lg = denso_robot_lg.translation_jacobian(J_lg, x_lg)
        Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
        Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)

        Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
        Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg
        J_d_si = cls.get_current_inserted_length_jacobian(Jt_si, Jl_si, t_si, l_si, eyeball_t, eyeball_radius)
        J_d_lg = cls.get_current_inserted_length_jacobian(Jt_lg, Jl_lg, t_lg, l_lg, eyeball_t, eyeball_radius)
        Jt_trocar_si, Jt_trocar_lg = cls.get_current_rcm_translation_jacobians(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                                               J_d_si, J_d_lg, l_si, l_lg,
                                                                               d_si, d_lg, jointx_si, jointx_lg)
        # print(7)
        J_r_eye = cls.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                    t_si, t_lg, l_si, l_lg,
                                                    rcm_current_si, rcm_current_lg,
                                                    eyeball_t, eyeball_radius, rcm_init_si, rcm_init_lg, jointx_si, jointx_lg)
        # print(7)

        # Get jacobians and distances for vitreoretinal environment constraints
        J_trocar_si = cls.get_tip_to_moving_trocar_distance_jacobian(Jt_trocar_si, Jt_si, rcm_current_si, t_si, jointx_si)
        J_trocar_lg = cls.get_tip_to_moving_trocar_distance_jacobian(Jt_trocar_lg, Jt_lg, rcm_current_lg, t_lg, jointx_lg)
        J_D_om = 2 * vec4(rcm_current_si - rcm_current_lg) @ np.hstack([Jt_trocar_si, -1 * Jt_trocar_lg])
        J_rotation_constraint = cls.get_eye_rotation_constraint_jacobian(r_eye, J_r_eye, parameter.theta_safe_eye_rotation)
        # J_rotation_constraint = cls.get_eye_rotation_constraint_jacobian(r_eye, J_r_eye, theta_rotation_angle)
        J_rot_plane_si = DQ_Kinematics.point_to_plane_distance_jacobian(Jt_trocar_si, rcm_current_si,
                                                                        rotation_c_plane_unified_list[0])
        J_rot_plane_lg = DQ_Kinematics.point_to_plane_distance_jacobian(Jt_trocar_lg, rcm_current_lg,
                                                                        rotation_c_plane_unified_list[1])

        D_trocar_si = cls.get_tip_to_moving_trocar_distance_error(rcm_current_si, t_si, parameter.D_safe_trocar)
        D_trocar_lg = cls.get_tip_to_moving_trocar_distance_error(rcm_current_lg, t_lg, parameter.D_safe_trocar)
        D_om = np.linalg.norm(vec4(rcm_current_si - rcm_current_lg)) ** 2
        D_rotation_constraint = cls.get_eye_rotation_constraint_distance(r_eye, parameter.theta_safe_eye_rotation)
        # D_rotation_constraint = cls.get_eye_rotation_constraint_distance(r_eye, theta_rotation_angle)
        d_rot_plane_si = DQ_Geometry.point_to_plane_distance(rcm_current_si, rotation_c_plane_unified_list[0])
        d_rot_plane_lg = DQ_Geometry.point_to_plane_distance(rcm_current_lg, rotation_c_plane_unified_list[1])

        # Quadratic programming (without the proposed constraints)
        W = np.vstack([np.hstack([-1 * J_trocar_si, np.zeros([1, jointx_lg])]) * parameter.trocar_si,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_trocar_lg]) * parameter.trocar_lg,
                       J_D_om.reshape([1, jointx_si + jointx_lg]) * parameter.om_D_rcm,
                       -1 * J_D_om.reshape([1, jointx_si + jointx_lg]) * parameter.om_D_rcm,
                       -1 * J_rotation_constraint.reshape([1, jointx_si + jointx_lg]) * parameter.om_rot,
                       np.hstack([-1 * J_rot_plane_si, np.zeros([1, jointx_lg])]) * parameter.om_rot_plane,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_rot_plane_lg]) * parameter.om_rot_plane,
                       ])

        w = np.vstack([np.array([parameter.nd_trocar * D_trocar_si]) * parameter.trocar_si,
                       np.array([parameter.nd_trocar * D_trocar_lg]) * parameter.trocar_lg,
                       np.array([-1 * parameter.nd_rcm * (D_om - D_rcm_init - parameter.D_safe_rcm_om)]) * parameter.om_D_rcm,
                       np.array([parameter.nd_rcm * (D_om - D_rcm_init + parameter.D_safe_rcm_om)]) * parameter.om_D_rcm,
                       np.array([parameter.nd_rotation_constraint * D_rotation_constraint]) * parameter.om_rot,
                       np.array([parameter.nd_rotation_constraint * d_rot_plane_si]) * parameter.om_rot_plane,
                       np.array([parameter.nd_rotation_constraint * d_rot_plane_lg]) * parameter.om_rot_plane,
                       ])
        # print("orbital manipulation VFI's calculated")

        return W, w
