# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.utils import DQ_Geometry

# Import other dependencies
import numpy as np
import math


class EyesurgeryVFIs:
    """
    This class contains Vector Field Inequalities (VFI) functions developed for eye surgery.
    As for the VFI method, see Marinho, M. M., Adorno, B. V., Harada, K., and Mitsuishi, M. (2018).
    Dynamic Active Constraints for Surgical Robots using Vector Field Inequalities.
    http://arxiv.org/abs/1804.11270

    Some of the functions depend on DQ Robotics. DQ Robotics is free software.
    DQ Robotics website: dqrobotics.github.io
    """

    @staticmethod
    def point_to_pyramid_jacobian(Jt_si, Jt_lg, p_1, p_2, t_si, t_lg):
        """
        point_to_pyramid_jacobian(Jt_si, Jt_lg, p_1, p_2, t_si, t_lg) returns the Jacobian 'J' that relates the joint
        velocities (delta_thetas) to the time derivative of the signed distance between the instrument's tip (t_si) and
        the plane spanned by p_1, p_2, and t_lg.
        For more details, see Koyama, Y. , Marinho, M. M., and et al. (2019).
        Towards the Automatic Control of the Light Guide in Vitreoretinal Surgery.
        The 15th Asian Conference on Computer Aided Surgery (ACCAS)
        """
        # Calculate plane normal and its norm
        plane_normal = cross(p_1 - t_lg, p_2 - t_lg)
        plane_normal_norm = np.linalg.norm(vec4(plane_normal))
        unit_plane_normal = np.array([vec4(plane_normal)]) / plane_normal_norm

        # Calculate the point-to-pyramid Jacobian
        J_plane_normal = -(crossmatrix4(p_1 - t_lg) + crossmatrix4(p_2 - t_lg).T) @ Jt_lg
        J_pyramid = J_plane_normal / plane_normal_norm - np.array([vec4(plane_normal)]).T @ np.array([vec4(plane_normal)]) @ J_plane_normal / (2 * plane_normal_norm ** 3)
        J_pyramid_distance = np.array([vec4(t_si)]) @ J_pyramid - unit_plane_normal @ Jt_lg - np.array([vec4(t_lg)]) @ J_pyramid
        J = np.hstack([unit_plane_normal @ Jt_si, J_pyramid_distance])
        return J

    @staticmethod
    def point_to_pyramid_signed_distance(p_1, p_2, t_si, t_lg):
        """
        point_to_pyramid_signed_distance(p_1, p_2, t_si, t_lg) returns the signed distance "d" between
        the instrument's tip (t_si) and the plane spanned by p_1, p_2, and t_lg.
        For more details, see Koyama, Y., Marinho, M. M., and et al. (2019).
        Towards the Automatic Control of the Light Guide in Vitreoretinal Surgery.
        The 15th Asian Conference on Computer Aided Surgery (ACCAS)
        """
        # Calculate plane normal and its norm
        plane_normal = cross(p_1 - t_lg, p_2 - t_lg)
        plane_normal_norm = np.linalg.norm(vec4(plane_normal))
        unit_plane_normal = np.array([vec4(plane_normal)]) / plane_normal_norm

        # Calculate the point-to-pyramid-signed-distance
        d = (np.array([vec4(t_si)]) - np.array([vec4(t_lg)])) @ unit_plane_normal.T
        return d

    @classmethod
    def get_point_to_pyramid_jacobians(cls, eyeground_points, number, Jt_si, Jt_lg, t_si, t_lg):
        """
        get_point_to_pyramid_jacobians(cls, eyeground_points, number, Jt_si, Jt_lg, t_si, t_lg) returns the array of
        the point-to-pyramid Jacobians "J_array".
        For more details, see Koyama, Y., Marinho, M. M., and et al. (2019).
        Towards the Automatic Control of the Light Guide in Vitreoretinal Surgery.
        The 15th Asian Conference on Computer Aided Surgery (ACCAS)
        """
        i = 0
        J_array = np.array([np.zeros(13)])

        # Calculate the point-to-pyramid Jacobians for each plane of the pyramid
        while i < number:
            if i == number - 1:
                J_i = cls.point_to_pyramid_jacobian(Jt_si, Jt_lg, translation(eyeground_points[i]), translation(eyeground_points[0]), t_si, t_lg)
            else:
                J_i = cls.point_to_pyramid_jacobian(Jt_si, Jt_lg, translation(eyeground_points[i]), translation(eyeground_points[i+1]), t_si, t_lg)
            J_array = np.vstack([J_array, J_i])
            i = i + 1
        J_array = np.delete(J_array, 0, 0)
        return J_array

    @classmethod
    def get_point_to_pyramid_signed_distances(cls, eyeground_points, number, t_si, t_lg):
        """
        get_point_to_pyramid_signed_distances(cls, eyeground_points, number, t_si, t_lg) returns the array of
        the point-to-pyramid signed distances "d_array".
        For more details, see Koyama, Y., Marinho, M. M., and et al. (2019).
        Towards the Automatic Control of the Light Guide in Vitreoretinal Surgery.
        The 15th Asian Conference on Computer Aided Surgery (ACCAS)
        """
        i = 0
        d_array = np.array([np.zeros(1)])

        # Calculate the point-to-pyramid signed distances for each plane of the pyramid
        while i < number:
            if i == number - 1:
                d_i = cls.point_to_pyramid_signed_distance(translation(eyeground_points[i]), translation(eyeground_points[0]), t_si, t_lg)
            else:
                d_i = cls.point_to_pyramid_signed_distance(translation(eyeground_points[i]), translation(eyeground_points[i+1]), t_si, t_lg)
            d_array = np.vstack([d_array, d_i])
            i = i + 1
        d_array = np.delete(d_array, 0, 0)
        return d_array

    @staticmethod
    def point_to_cone_signed_distance(p_R2_e, p_lg_c, t_lg_si):
        """
        - point_to_cone_signed_distance(p_R2_e, p_lg_c, t_lg_si) returns the signed distance "d" that takes positive
        values if the instrument's tip is inside the conical region.
        - This function is used to enforce the constraint C1 of Section II-D of Koyama et al. (2022).
        - For more details, see Koyama, Y., Marinho, M. M., Mitsuishi, M., and Harada, K. (2022).
        Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal Surgery.
        Transactions on Medical Robotics and Bionics (TMRB)
        """
        # Calculate the point-to-cone signed distance---Eq. 28 of Koyama et al. (2022)
        d = (np.linalg.norm(vec4(p_R2_e)) ** 2) * ((np.dot(vec4(p_lg_c), vec4(t_lg_si))) ** 2) \
            - (np.linalg.norm(vec4(t_lg_si)) ** 2) * ((np.dot(vec4(p_lg_c), vec4(p_R2_e))) ** 2)
        return d

    @staticmethod
    def point_to_cone_jacobian(Jt_si, Jt_lg, ws_radius, t_lg_si, p_lg_c, p_R2_e, p_a_si):
        """
        - point_to_cone_jacobian(Jt_si, Jt_lg, ws_radius, t_lg_si, p_lg_c, p_R2_e, p_a_si) returns the Jacobian "J"
        that relates the joint velocities (delta_thetas) to the time derivative of the point-to-cone signed distance.
        - This function is used to enforce the constraint C1 of Section II-D of Koyama et al. (2022).
        - For more details, see Koyama, Y., Marinho, M. M., Mitsuishi, M., and Harada, K. (2022).
        Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal Surgery.
        Transactions on Medical Robotics and Bionics (TMRB)
        """
        # Calculate the point-to-cone Jacobian---Section IX-C of Koyama et al. (2022)
        Jc_1 = (np.eye(4)-np.array([vec4(p_lg_c)]).T@np.array([vec4(k_)])/np.dot(vec4(p_lg_c), vec4(k_)))@Jt_si
        Jc_2 = (-np.eye(4)+np.array([vec4(p_lg_c)]).T@np.array([vec4(k_)])/np.dot(vec4(p_lg_c), vec4(k_))
                + (np.dot(vec4(t_lg_si), vec4(k_))*np.dot(vec4(p_lg_c), vec4(k_))*np.eye(4)
                   - np.dot(vec4(t_lg_si), vec4(k_))*np.array([vec4(p_lg_c)]).T@np.array([vec4(k_)]))
                / ((np.dot(vec4(p_lg_c), vec4(k_)))**2))@Jt_lg
        J_1_1 = 2*ws_radius*(np.dot(vec4(p_lg_c), vec4(t_lg_si)))**2*np.array([vec4(p_R2_e)]) \
                @ (np.eye(4)/np.linalg.norm(vec4(p_a_si))-np.array([vec4(p_a_si)]).T@np.array([vec4(p_a_si)])
                   / ((np.linalg.norm(vec4(p_a_si)))**3))@Jc_1
        J_1_2 = 2*(np.dot(vec4(p_lg_c), vec4(t_lg_si)))**2*np.array([vec4(p_R2_e)]) \
                @ (ws_radius*(np.eye(4)/np.linalg.norm(vec4(p_a_si))-np.array([vec4(p_a_si)]).T
                              @ np.array([vec4(p_a_si)])/((np.linalg.norm(vec4(p_a_si)))**3))@Jc_2-Jt_lg)
        J_1 = np.hstack([J_1_1, J_1_2])
        J_2_1 = np.linalg.norm(vec4(p_R2_e))**2*2*np.dot(vec4(p_lg_c), vec4(t_lg_si))*np.array([vec4(p_lg_c)])@Jt_si
        J_2_2 = -1*np.linalg.norm(vec4(p_R2_e))**2*2*np.dot(vec4(p_lg_c), vec4(t_lg_si)) \
                * (np.array([vec4(t_lg_si)])+np.array([vec4(p_lg_c)]))@Jt_lg
        J_2 = np.hstack([J_2_1, J_2_2])
        J_3_1 = 2*(np.dot(vec4(p_lg_c), vec4(p_R2_e)))**2*np.array([vec4(t_lg_si)])@Jt_si
        J_3_2 = -2*(np.dot(vec4(p_lg_c), vec4(p_R2_e)))**2*np.array([vec4(t_lg_si)])@Jt_lg
        J_3 = np.hstack([J_3_1, J_3_2])
        J_4_1 = ws_radius*np.linalg.norm(vec4(t_lg_si))**2*2*np.dot(vec4(p_lg_c), vec4(p_R2_e)) \
                * np.array([vec4(p_lg_c)])@(np.eye(4)/np.linalg.norm(vec4(p_a_si))-np.array([vec4(p_a_si)]).T
                                            @ np.array([vec4(p_a_si)])/((np.linalg.norm(vec4(p_a_si)))**3))@Jc_1
        J_4_2 = ws_radius*np.linalg.norm(vec4(t_lg_si))**2*2*np.dot(vec4(p_lg_c), vec4(p_R2_e)) \
                * np.array([vec4(p_lg_c)])@(np.eye(4)/np.linalg.norm(vec4(p_a_si))-np.array([vec4(p_a_si)]).T
                                            @ np.array([vec4(p_a_si)])/((np.linalg.norm(vec4(p_a_si)))**3))@Jc_2 \
                - np.linalg.norm(vec4(t_lg_si))**2*2*np.dot(vec4(p_lg_c), vec4(p_R2_e)) \
                * (np.array([vec4(p_R2_e)])+np.array([vec4(p_lg_c)]))@Jt_lg
        J_4 = np.hstack([J_4_1, J_4_2])
        J = J_1 + J_2 - J_3 - J_4
        return J

    @staticmethod
    def point_to_illumination_signed_distance(lz_2, t_lg_si, gamma):
        """
        - point_to_illumination_signed_distance(lz_2, t_lg_si, gamma) returns the signed distance "d" that takes positive
        values if the instrument's tip is inside the illumination range of the light guide.
        - This function is used to enforce the constraint C2 of Section II-D of Koyama et al. (2022).
        - For more details, see Koyama, Y., Marinho, M. M., Mitsuishi, M., and Harada, K. (2022).
        Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal Surgery.
        Transactions on Medical Robotics and Bionics (TMRB)
        """
        d = (np.dot(vec4(lz_2), vec4(t_lg_si)) ** 2) \
            - (np.linalg.norm(vec4(t_lg_si)) ** 2) * ((math.cos(gamma)) ** 2)
        return d

    @staticmethod
    def point_to_illumination_jacobian(r_lg, Jr_lg, lz_2, t_lg_si, gamma, Jt_si, Jt_2):
        """
        - point_to_illumination_jacobian(_lg, J_lg, lz_2, t_lg_si, gamma, Jt_si, Jt_2) returns the Jacobian "J" that
        relates the joint velocities (delta_thetas) to the time derivative of the point-to-illumination signed distance.
        - This function is used to enforce the constraint C2 of Section II-D of Koyama et al. (2022).
        - For more details, see Koyama, Y., Marinho, M. M., Mitsuishi, M., and Harada, K. (2022).
        Autonomous Coordinated Control of the Light Guide for Positioning in Vitreoretinal Surgery.
        Transactions on Medical Robotics and Bionics (TMRB)
        """
        # Calculate the point-to-illumination Jacobian---Section IX-E of Koyama et al. (2022)
        Jrz_2 = (haminus4(k_ * conj(r_lg)) + hamiplus4(r_lg * k_) @ C4()) @ Jr_lg
        J_1 = 2 * (np.dot(vec4(lz_2), vec4(t_lg_si)) * np.array([vec4(lz_2)]) - (math.cos(gamma)) ** 2 * np.array([vec4(t_lg_si)])) @ Jt_si
        J_2 = 2 * (np.dot(vec4(lz_2), vec4(t_lg_si)) * np.array([vec4(t_lg_si)]) @ Jrz_2
                   - (np.dot(vec4(lz_2), vec4(t_lg_si)) * np.array([vec4(lz_2)]) - (math.cos(gamma)) ** 2 * np.array([vec4(t_lg_si)])) @ Jt_2)
        J = np.hstack([J_1, J_2])
        return J

    @staticmethod
    def get_shaft_to_rcm_distance_error(x, rcm_t, D_safe_rcm):
        """
        - get_shaft_to_rcm_distance_error(x, rcm_t, D_safe_rcm) returns the error "D_error" between the safe
        distance (D_safe_rcm) and the squared distance between the shaft of the instrument and the corresponding
        insertion point (rcm_t).
        - This function is used to enforce the constraint C_R of Section II-C of Koyama et al. (2022).
        """
        # Calculate the rotation and the translation of the instrument
        r = rotation(x)
        t = translation(x)

        # Calculate the Plucker line that corresponds to the instrument's shaft
        lz = r * k_ * conj(r)
        mz = cross(t, lz)

        # Calculate the squared distance between the shaft of the instrument and the insertion point (rcm_t)
        D_rcm = (np.linalg.norm(vec4(cross(rcm_t, lz) - mz))) ** 2

        # Calculate the error
        D_error = D_safe_rcm - D_rcm
        return D_error

    @staticmethod
    def get_shaft_to_rcm_distance_jacobian(x, Jx, rcm_t):
        """
        - get_shaft_to_rcm_distance_jacobian(x, Jx, rcm_t) returns the Jacobian "J" that relates the joint
        velocities (delta_thetas) to the time derivative of the shaft-to-rcm distance error.
        - This function is used to enforce the constraint C_R of Section II-C of Koyama et al. (2022).
        - See also the line-to-point distance Jacobian of Murilo et al. (2018).
        """
        # Calculate the rotation and the translation of the instrument
        r = rotation(x)
        t = translation(x)

        # Calculate the Plucker line that corresponds to the instrument's shaft
        lz = r * k_ * conj(r)
        mz = cross(t, lz)
        lz_dq = lz + E_ * mz

        # Calculate the shaft-to-rcm distance Jacobian
        Jlz = DQ_Kinematics.line_jacobian(Jx, x, k_)
        J = DQ_Kinematics.line_to_point_distance_jacobian(Jlz, lz_dq, rcm_t)
        return J

    @staticmethod
    def get_light_tip_to_eyeball_distance_error(x, eyeball_center_t, D_safe_eyeball):
        """
        - get_light_tip_to_eyeball_distance_error(x, eyeball_center_t, D_safe_eyeball) returns the error "D_error"
        between the safe distance (D_safe_eyeball) and the squared distance between the tip of the instrument and
        the inner surface of the eyeball.
        - This function is used to enforce the constraint C_r of Section II-C of Koyama et al. (2022).
        """
        # Calculate the translation of the instrument
        t = translation(x)

        # Calculate the squared distance between the tip of the instrument and the inner surface of the eyeball
        D_eyeball = (np.linalg.norm(vec4(t - eyeball_center_t))) ** 2

        # Calculate the error
        D_error = D_safe_eyeball - D_eyeball
        return D_error

    @staticmethod
    def get_light_tip_to_eyeball_distance_jacobian(x, Jt, eyeball_center_t):
        """
        - get_light_tip_to_eyeball_distance_jacobian(x, Jt, eyeball_center_t) returns the Jacobian "J" that relates
        the joint velocities (delta_thetas) to the time derivative of the light_tip-to-eyeball distance error.
        - This function is used to enforce the constraint C_r of Section II-C of Koyama et al. (2022).
        - See also point-to-point distance Jacobian of Murilo et al. (2018).
        """
        # Calculate the translation of the instrument
        t = translation(x)

        # Calculate the light_tip-to-eyeball distance Jacobian
        J = DQ_Kinematics.point_to_point_distance_jacobian(Jt, t, eyeball_center_t)
        return J

    @staticmethod
    def get_light_tip_to_instrument_shaft_distance_error(x1, x2, D_safe_collision):
        """
        - get_light_tip_to_instrument_shaft_distance_error(x1, x2, D_safe_collision) returns the error "D_error"
        between the safe distance (D_safe_collision) and the squared distance between the tip of the light guide and
        the shaft of the surgical instrument.
        - This function is used to enforce the constraint C_s of Section II-C of Koyama et al. (2022).
        """
        # Calculate the rotations and the translations of the instruments
        r_1 = rotation(x1)
        t_1 = translation(x1)
        t_2 = translation(x2)

        # Calculate the Plucker line that corresponds to the surgical instrument's shaft
        lz_1 = r_1 * k_ * conj(r_1)
        mz_1 = cross(t_1, lz_1)

        # Calculate the squared distance between the tip of the light guide and the shaft of the surgical instrument
        D_collision = (np.linalg.norm(vec4(cross(t_2, lz_1) - mz_1))) ** 2

        # Calculate the error
        D_error = D_safe_collision - D_collision
        return D_error

    @staticmethod
    def get_light_tip_to_instrument_shaft_distance_jacobian(x2, Jt_2, x1):
        """
        - get_light_tip_to_instrument_shaft_distance_jacobian(x2, Jt_2, x1) returns the Jacobian "J" that relates
        the joint velocities (delta_thetas) to the time derivative of the light_tip-to-instrument_shaft distance error.
        - This function is used to enforce the constraint C_s of Section II-C of Koyama et al. (2022).
        - See also point-to-line distance Jacobian of Murilo et al. (2018).
        """
        # Calculate the rotations and the translations of the instruments
        r_1 = rotation(x1)
        t_1 = translation(x1)
        t_2 = translation(x2)

        # Calculate the Plucker line that corresponds to the surgical instrument's shaft
        lz_1 = r_1 * k_ * conj(r_1)
        mz_1 = cross(t_1, lz_1)
        lz_collision_dq = lz_1 + E_ * mz_1

        # Calculate the light_tip-to-instrument_shaft distance Jacobian
        J = DQ_Kinematics.point_to_line_distance_jacobian(Jt_2, t_2, lz_collision_dq)
        return J

    @staticmethod
    def get_joint_limit_distance(limit_minus, limit_plus, theta):
        """
        - get_joint_limit_distance(limit_minus, limit_plus, theta) returns the array "d_array" that stores
        the errors between the current joint positions and the limits of the joints.
        - This function is used to enforce the constraint C_j of Section II-C of Koyama et al. (2022).
        """
        # Store the errors between the current joint positions and the limits of the joints
        d_array = np.concatenate([-1 * (limit_minus - theta).T,
                             (limit_plus - theta).T]).reshape(-1, 1)
        return d_array

    @staticmethod
    def get_joint_limit_jacobian(joint_number):
        """
        - get_joint_limit_jacobian(joint_number) returns the Jacobian "J" that relates the joint velocities
        (delta_thetas) to the time derivative of the joint limit distances.
        - This function is used to enforce the constraint C_j of Section II-C of Koyama et al. (2022).
        """
        # Calculate the joint-limit Jacobian
        J = np.vstack([-1 * np.eye(joint_number),
                       np.eye(joint_number)])
        return J

    @staticmethod
    def get_robot_to_view_distance_error(x, eyeball_center_t, D_safe_view):
        """
        - get_robot_to_view_distance_error(x, eyeball_center_t, D_safe_view) returns the error "D_error" between
        the safe distance (D_safe_view) and the squared distance between a point in the robot and the center line of
        the microscopic view.
        - This function is used to enforce the constraint C_m of Section II-C of Koyama et al. (2022).
        """
        # Calculate the translation of the point in the robot
        t = translation(x)

        # Calculate the Plucker line that corresponds to the center line of the microscopic view
        lz = k_
        mz = cross(eyeball_center_t, lz)

        # Calculate the squared distance between a point in the robot and the center line of the microscopic view
        D_view = (np.linalg.norm(vec4(cross(t, lz) - mz))) ** 2

        # Calculate the error
        D_error = D_safe_view - D_view
        return D_error

    @staticmethod
    def get_robot_to_view_distance_jacobian(x, Jt, eyeball_center_t):
        """
        - get_robot_to_view_distance_jacobian(x, Jt, eyeball_center_t) returns the Jacobian "J" that relates
        the joint velocities (delta_thetas) to the time derivative of the robot-to-view distance error.
        - This function is used to enforce the constraint C_m of Section II-C of Koyama et al. (2022).
        - See also point-to-line distance Jacobian of Murilo et al. (2018).
        """
        # Calculate the translation of the point in the robot
        t = translation(x)

        # Calculate the Plucker line that corresponds to the center line of the microscopic view
        lz = k_
        mz = cross(eyeball_center_t, lz)
        lz_view_dq = lz + E_ * mz

        # Calculate the robot-to-view Jacobian
        J = DQ_Kinematics.point_to_line_distance_jacobian(Jt, t, lz_view_dq)
        return J

    @staticmethod
    def get_robot_to_constrained_planes_distance(theta, robot, plane_list):
        """
        - get_robot_to_constrained_planes_distance(theta, robot, plane_list) returns the array "d_array"
        that store the signed distances between the joint positions of the robot and the constrained planes.
        - This function is used to enforce the constraint C_ro of Section II-C of Koyama et al. (2022).
        """
        # Number of joints
        n = len(theta)

        # Store the signed distances between the joint positions of the robot and the constrained planes
        d = np.zeros([1, 1])
        joint_counter = 3
        while joint_counter < n:
            x = robot.get_reference_frame() * robot.raw_fkm(theta, joint_counter)
            t = translation(x)

            plane_counter = 0
            while plane_counter < len(plane_list):
                plane = plane_list[plane_counter]
                dp_pi = np.dot(vec4(t), vec4(P(plane))) - np.linalg.norm(vec4(D(plane)))
                d = np.vstack((d, dp_pi))
                plane_counter = plane_counter + 1
            joint_counter = joint_counter + 1
        d_array = np.delete(d, 0, 0)
        return d_array

    @staticmethod
    def get_robot_to_constrained_planes_distance_jacobian(theta, robot, plane_list):
        """
        - get_robot_to_constrained_planes_distance_jacobian(theta, robot, plane_list) returns the Jacobian "J" that
        relates the joint velocities (delta_thetas) to the time derivative of the robot-to-constrained_plane distances.
        - This function is used to enforce the constraint C_ro of Section II-C of Koyama et al. (2022).
        - See also point-to-plane distance Jacobian of Murilo et al. (2018).
        """
        # Number of joints
        n = len(theta)

        # Store the robot-to-constrained_plane Jacobians
        J = np.zeros([1, n])
        joint_counter = 3
        while joint_counter < n:
            Jx = hamiplus8(robot.get_reference_frame()) @ robot.raw_pose_jacobian(theta, joint_counter)
            x = robot.get_reference_frame() * robot.raw_fkm(theta, joint_counter)
            Jt = robot.translation_jacobian(Jx, x)
            t = translation(x)

            plane_counter = 0
            while plane_counter < len(plane_list):
                plane = plane_list[plane_counter]
                Jp_pi = np.hstack((DQ_Kinematics.point_to_plane_distance_jacobian(Jt, t, plane),
                                   np.zeros([1, n - 1 - joint_counter])))
                J = np.vstack((J, Jp_pi))
                plane_counter = plane_counter + 1
            joint_counter = joint_counter + 1
        J = np.delete(J, 0, 0)
        return J

    @staticmethod
    def get_tip_to_trocar_distance_error(t, rcm_t, D_safe_trocar):
        """
        - get_tip_to_trocar_distance_error(t, rcm_t, D_safe_trocar) returns the error "D_error" between the safe
        distance (D_safe_trocar) and the squared distance between the tip of the instrument and
        its insertion point (trocar point).
        - This function is used to enforce the constraint C_tr of Section II-C of Koyama et al. (2022).
        """
        # Calculate the squared distance between the tip of the instrument and its insertion point
        D_trocar = (np.linalg.norm(vec4(t - rcm_t))) ** 2

        # Calculate the error
        D_error = D_trocar - D_safe_trocar
        return D_error

    @staticmethod
    def get_tip_to_trocar_distance_jacobian(Jt, t, rcm_t):
        """
        - get_tip_to_trocar_distance_jacobian(Jt, t, rcm_t) returns the Jacobian "J" that relates
        the joint velocities (delta_thetas) to the time derivative of the tip-to-trocar distance error.
        - This function is used to enforce the constraint C_tr of Section II-C of Koyama et al. (2022).
        - See also point-to-point distance Jacobian of Murilo et al. (2018).
        """
        # Calculate the tip-to-trocar distance Jacobian
        J = DQ_Kinematics.point_to_point_distance_jacobian(Jt, t, rcm_t)
        return J

    @staticmethod
    def get_second_task_distance(t_1, t_2, r_1):
        """
        - get_second_task_distance(t_1, t_2, r_1) returns the signed distance "d" between the tip of the light guide
        and the plane that contains the z-axis of the world frame and the shaft of the surgical instrument.
        - This function is used for the optimization problems of the overlap prevention step and the vertical
        positioning step. See also Section VI-A of Koyama et al. (2022).
        """
        # Get the direction of the shaft of the surgical instrument
        axis = r_1 * k_ * conj(r_1)

        # Calculate the unit plane normal of the plane that contains the z-axis of the world frame and the shaft
        # ---Eq. 7 of Koyama et al. (2022)
        plane_normal = vec4(cross(axis, k_))
        unit_plane_normal = plane_normal / np.linalg.norm(plane_normal)

        # Calculate the second task distance---Eq. 8 of Koyama et al. (2022)
        d = np.dot(unit_plane_normal, vec4(t_1 - t_2))
        return d

    @staticmethod
    def get_second_task_distance_jacobian(Jt_1, Jt_2, Jr_1, t_1, t_2, r_1, jointx_comb):
        """
        - get_second_task_distance_jacobian(Jt_1, Jt_2, Jr_1, t_1, t_2, r_1) returns the task Jacobian "J" that relates
        the joint velocities (delta_thetas) to the time derivative of the second task distance.
        - This function is used for the optimization problems of the overlap prevention step and the vertical
        positioning step. See also Section VI-B of Koyama et al. (2022).
        """
        # Get the direction of the shaft of the surgical instrument
        axis = r_1 * k_ * conj(r_1)

        # Calculate the unit plane normal of the plane that contains the z-axis of the world frame and the shaft
        plane_normal = vec4(cross(axis, k_)).reshape([4, 1])
        unit_plane_normal = plane_normal / np.linalg.norm(plane_normal)

        # See II-A of Koyama et al. (2022)
        C_4 = np.diag([1., -1., -1., -1.])

        # Calculate the task Jacobian of the second task distance---Section VI-B of Koyama et al. (2022)
        t_R2_R1 = t_1 - t_2
        u_1 = np.eye(4) / np.linalg.norm(plane_normal) - plane_normal @ plane_normal.T / (
                np.linalg.norm(plane_normal) ** 3)
        J_n = crossmatrix4(k_).T @ (haminus4(k_ * conj(r_1)) + hamiplus4(r_1 * k_) @ C_4) @ Jr_1
        J_1 = unit_plane_normal.T @ Jt_1 + vec4(t_R2_R1) @ u_1 @ J_n
        J_2 = -unit_plane_normal.T @ Jt_2
        J = np.hstack([J_1, J_2]).reshape([1, jointx_comb])
        return J

    @staticmethod
    def get_second_task_squared_distance(t_1, t_2, r_1):
        """
        get_second_task_squared_distance(t_1, t_2, r_1) returns the squared distance "D" of the second task distance.
        """
        # Get the direction of the shaft of the surgical instrument
        axis = r_1 * k_ * conj(r_1)

        # Calculate the unit plane normal of the plane that contains the z-axis of the world frame and the shaft
        unit_plane_normal = normalize(cross(axis, k_))

        # Calculate the squared distance of the second task distance
        d = np.dot(vec4(unit_plane_normal), vec4(t_1 - t_2))
        D_second = d ** 2
        return D_second

    @staticmethod
    def get_second_task_squared_distance_jacobian(Jt_1, Jt_2, Jr_1, t_1, t_2, r_1):
        """
        get_second_task_squared_distance_jacobian(Jt_1, Jt_2, Jr_1, t_1, t_2, r_1) returns the Jacobian "J" that
        relates the joint velocities (delta_thetas) to the time derivative of the second task squared distance.
        """
        # Get the direction of the shaft of the surgical instrument
        axis = r_1 * k_ * conj(r_1)

        # Calculate the unit plane normal of the plane that contains the z-axis of the world frame and the shaft
        plane_normal = cross(axis, k_)
        unit_plane_normal = normalize(plane_normal)

        # See II-A of Koyama et al. (2022)
        C_4 = np.diag([1., -1., -1., -1.])

        # Calculate the second-task-squared-distance Jacobian
        d = np.dot(vec4(unit_plane_normal), vec4(t_1 - t_2))
        J_rz = haminus4(k_ * conj(r_1)) @ Jr_1 + hamiplus4(r_1 * k_) @ C_4 @ Jr_1

        J_1 = 2 * d * (vec4(unit_plane_normal) @ Jt_1
                       + vec4(t_1 - t_2) @ (np.eye(4) / np.linalg.norm(vec4(plane_normal))
                                            - vec4(plane_normal).reshape([4, 1]) @ vec4(plane_normal).reshape([1, 4])
                                            / (np.linalg.norm(vec4(plane_normal)) ** 3))
                       @ crossmatrix4(k_).T @ J_rz)
        J_2 = -2 * d * vec4(unit_plane_normal) @ Jt_2
        J = np.hstack([J_1, J_2])
        return J

    @staticmethod
    def get_tip_to_tip_distance_error(x, t_si, D_safe_tip):
        """
        - get_tip_to_tip_distance_error(x, t_si, D_safe_second_sphere) returns the error "D_error" between
        the safe distance (D_safe_tip) and the squared distance between the light guide's tip and the surgical
        instrument's tip.
        - This function is used to enforce the constraint C_t of Section II-D of Koyama et al. (2022).
        """
        # Calculate the translation of the light guide
        t = translation(x)

        # Calculate the squared distance between the light guide's tip and the surgical instrument's tip
        D_eyeball = (np.linalg.norm(vec4(t - t_si))) ** 2

        # Calculate the error
        D_error = D_safe_tip - D_eyeball
        return D_error

    @staticmethod
    def get_tip_to_tip_distance_jacobian(Jt_1, Jt_2, t_1, t_2, jointx_1, jointx_2):
        """
        - get_tip_to_tip_distance_jacobian(Jt_1, Jt_2, t_1, t_2) returns the Jacobian "J" that relates
        the joint velocities (delta_thetas) to the time derivative of the tip-to-tip distance error.
        - This function is used to enforce the constraint C_t of Section II-C of Koyama et al. (2022).
        """
        # Calculate the tip-to-tip distance Jacobian
        J_1 = 2 * vec4(t_2 - t_1) @ Jt_1
        J_2 = -2 * vec4(t_2 - t_1) @ Jt_2
        J = np.hstack([J_1, J_2]).reshape([1, jointx_1 + jointx_2])
        return J
    
    @staticmethod
    def get_shaft_to_eyeball_normal_angle_error(x, r_eye, A_safe_forceps):
        """
        - get_shaft_to_eyeball_normal_angle_error(x, r_eye, A_safe_forceps) returns the error "A_error" between 
        the safe angle (A_safe_forceps) and the angle between the top face of the surgical instrument (forceps) 
        and the direction of the eyeball's normal. 
        """
        # Calculate the rotations of the instruments
        r_1 = rotation(x)

        # Calculate the Plucker line direction that corresponds to the upper face of the surgical instrument's shaft
        lx_1 = r_1 * i_ * conj(r_1)

        # Get the direction of the eyeball's normal
        l_eye = normalize(r_eye * k_ * conj(r_eye))

        # Calculate the angle between the top face of the surgical instrument and the direction of the eyeball's normal
        A_forceps = DQ_Geometry.line_to_line_angle(lx_1, l_eye)

        # Calculate the error
        A_error = A_safe_forceps - A_forceps
        return A_error


    @staticmethod
    def get_shaft_to_eyeball_normal_jacobian(x, Jx, r_eye):
        """
        - get_shaft_to_eyeball_normal_jacobian(x, Jx, r_eye) returns the Jacobian "J" that relates to the angle between 
        the top face of the surgical instrument (forceps) and the direction of the eyeball's normal.
        - This function is used to enforce the forceps rotation constraint to the eyeball's surface.
        """
        # Calculate the rotations and the translations of the instruments
        r_1 = rotation(x)
        t_1 = translation(x)

        # Calculate the Plucker line that corresponds to the upper face of the surgical instrument's shaft
        lx_1 = r_1 * i_ * conj(r_1)
        mx_1 = cross(t_1, lx_1)
        lx_dq = lx_1 + E_ * mx_1
        
        # Get the direction of the eyeball's normal
        l_eye = normalize(r_eye * k_ * conj(r_eye))

        # Calculate the line-to-line angle Jacobian
        Jlz = DQ_Kinematics.line_jacobian(Jx, x, i_)
        J = DQ_Kinematics.line_to_line_angle_jacobian(Jlz, lx_dq, l_eye)
        return J
        


    @classmethod
    def get_vitreoretinal_VFIs_for_one_manipulator(cls, manipulator, theta, rcm_t, eyeball_t, parameter):
        """
        - get_vitreoretinal_VFIs_for_one_manipulator(cls, manipulator, theta, rcm_t, eyeball_t, constrained_plane_list,
        parameter) returns the array of Jacobians "W" and the array of corresponding distances "w" to enforce
        inequality constraints for the safe control of one robotic manipulator under the vitreoretinal environment.
        - This function is used to move a robotic arm to the initial position.
        """
        # Get poses and Jacobians
        x = manipulator.fkm(theta)
        jointx = theta.size
        jointx_min = theta.size - 1
        x_jxx = manipulator.raw_fkm(theta, jointx_min)
    
        J = manipulator.pose_jacobian(theta)
        J_jxx = manipulator.raw_pose_jacobian(theta, jointx_min)
        Jt = DQ_Kinematics.translation_jacobian(J, x)
        Jt_jxx = manipulator.translation_jacobian(J_jxx, x_jxx)

        # Calculate Jacobians for safety constraints
        J_l_rcm = cls.get_shaft_to_rcm_distance_jacobian(x, J, rcm_t)
        J_p_eyeball = cls.get_light_tip_to_eyeball_distance_jacobian(x, Jt, eyeball_t)
        J_theta_limit = cls.get_joint_limit_jacobian(jointx)
        J_view = cls.get_robot_to_view_distance_jacobian(x_jxx, Jt_jxx, eyeball_t)

        # Calculate distances for safety constraints
        D_rcm_error = cls.get_shaft_to_rcm_distance_error(x, rcm_t, parameter.D_safe_rcm)
        D_eyeball_error = cls.get_light_tip_to_eyeball_distance_error(x, eyeball_t, parameter.D_safe_eyeball)

        d_theta_limit = cls.get_joint_limit_distance([parameter.theta_limit_minu[0, 0: jointx]], [parameter.theta_limit_plu[0, 0: jointx]], theta)
        D_view_error = cls.get_robot_to_view_distance_error(x_jxx, eyeball_t, parameter.D_safe_view)

        # Store them in arrays
        W = np.vstack([J_l_rcm,
                       J_p_eyeball,
                       J_theta_limit,
                       -1 * J_view,
                       ])
        w = np.vstack([parameter.nd_rcm * D_rcm_error,
                       parameter.nd_eyeball * D_eyeball_error,
                       parameter.nd_limit * d_theta_limit,
                       -1 * parameter.nd_view * D_view_error,
                       ])
        return W, w

    @classmethod 
    def get_vitreoretinal_VFIs(cls, robot_si, robot_lg, theta_si, theta_lg, rcm_si, rcm_lg,
                               eyeball_t, parameter, constrained_planes_si, constrained_planes_lg, r_eye):
        """
        - get_vitreoretinal_VFIs_for_two_manipulator(cls, robot_si, robot_lg, theta_si, theta_lg, rcm_si, rcm_lg,
        eyeball_t, constrained_planes_si, constrained_planes_lg, parameter) returns the array of Jacobians "W" and
        the array of corresponding distances "w" to enforce inequality constraints for the safe control of
        two robotic manipulators under the vitreoretinal environment.
        - This function is used to get Vitreoretinal Task Constraints described in VIII of Koyama et al. (2022).
        """
        ## Modified by Christian Yuen to be more flexible with the number of joints of each robot
        # Get the poses and translations
        jointx_si = theta_si.size
        jointx_lg = theta_lg.size
        jointx_si_min = theta_si.size - 1
        jointx_lg_min = theta_lg.size - 1
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        x_si_jxx = robot_si.raw_fkm(theta_si, jointx_si_min)
        x_lg_jxx = robot_lg.raw_fkm(theta_lg, jointx_lg_min)
        t_si = translation(x_si)
        t_lg = translation(x_lg)

        # Get Jacobians
        J_si = robot_si.pose_jacobian(theta_si)
        Jt_si = robot_si.translation_jacobian(J_si, x_si)
        J_si_jxx = robot_si.raw_pose_jacobian(theta_si, jointx_si_min)
        Jt_si_jxx = robot_si.translation_jacobian(J_si_jxx, x_si_jxx)
        J_lg = robot_lg.pose_jacobian(theta_lg)
        Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
        J_lg_jxx = robot_lg.raw_pose_jacobian(theta_lg, jointx_lg_min)
        Jt_lg_jxx = robot_lg.translation_jacobian(J_lg_jxx, x_lg_jxx)

        # Calculate Jacobians for Vitreoretinal Task Constraints---Section VIII of Koyama et al. (2022)
        J_l_rcm_si = cls.get_shaft_to_rcm_distance_jacobian(x_si, J_si, rcm_si)
        J_l_rcm_lg = cls.get_shaft_to_rcm_distance_jacobian(x_lg, J_lg, rcm_lg)
        J_trocar_si = cls.get_tip_to_trocar_distance_jacobian(Jt_si, t_si, rcm_si)
        J_trocar_lg = cls.get_tip_to_trocar_distance_jacobian(Jt_lg, t_lg, rcm_lg)
        J_p_eyeball = cls.get_light_tip_to_eyeball_distance_jacobian(x_lg, Jt_lg, eyeball_t)
        J_collision = cls.get_light_tip_to_instrument_shaft_distance_jacobian(x_lg, Jt_lg, x_si)
        J_theta_limit_si = cls.get_joint_limit_jacobian(jointx_si)
        J_theta_limit_lg = cls.get_joint_limit_jacobian(jointx_lg)
        J_view_si = cls.get_robot_to_view_distance_jacobian(x_si_jxx, Jt_si_jxx, eyeball_t)
        J_view_lg = cls.get_robot_to_view_distance_jacobian(x_lg_jxx, Jt_lg_jxx, eyeball_t)
        J_plane_si = cls.get_robot_to_constrained_planes_distance_jacobian(theta_si, robot_si, constrained_planes_si)
        J_plane_lg = cls.get_robot_to_constrained_planes_distance_jacobian(theta_lg, robot_lg, constrained_planes_lg)
        J_forceps = cls.get_shaft_to_eyeball_normal_jacobian(x_si, J_si, r_eye)

        # Calculate distances for Vitreoretinal Task Constraints---Section VIII of Koyama et al. (2022)
        D_rcm_error_si = cls.get_shaft_to_rcm_distance_error(x_si, rcm_si, parameter.D_safe_rcm)
        D_rcm_error_lg = cls.get_shaft_to_rcm_distance_error(x_lg, rcm_lg, parameter.D_safe_rcm)
        D_trocar_si = cls.get_tip_to_trocar_distance_error(t_si, rcm_si, parameter.D_safe_trocar)
        D_trocar_lg = cls.get_tip_to_trocar_distance_error(t_lg, rcm_lg, parameter.D_safe_trocar)
        D_eyeball_error = cls.get_light_tip_to_eyeball_distance_error(x_lg, eyeball_t, parameter.D_safe_eyeball)
        D_collision_error = cls.get_light_tip_to_instrument_shaft_distance_error(x_si, x_lg, parameter.D_safe_collision)
        d_theta_limit_si = cls.get_joint_limit_distance(parameter.theta_limit_minu[0, 0: jointx_si], parameter.theta_limit_plu[0, 0: jointx_si], theta_si)
        d_theta_limit_lg = cls.get_joint_limit_distance(parameter.theta_limit_minu[0, 0: jointx_lg], parameter.theta_limit_plu[0, 0: jointx_lg], theta_lg)
        D_view_error_si = cls.get_robot_to_view_distance_error(x_si_jxx, eyeball_t, parameter.D_safe_view)
        D_view_error_lg = cls.get_robot_to_view_distance_error(x_lg_jxx, eyeball_t, parameter.D_safe_view)
        D_plane_si = cls.get_robot_to_constrained_planes_distance(theta_si, robot_si, constrained_planes_lg)
        D_plane_lg = cls.get_robot_to_constrained_planes_distance(theta_lg, robot_lg, constrained_planes_si)
        D_forceps_angle = cls.get_shaft_to_eyeball_normal_angle_error(x_si, r_eye, parameter.A_safe_forceps)

        # Store them in arrays
        W = np.vstack([np.hstack([J_l_rcm_si, np.zeros([1, jointx_lg])]) * parameter.rcm_si,
                       np.hstack([np.zeros([1, jointx_si]), J_l_rcm_lg]) * parameter.rcm_lg,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_collision]) * parameter.collision,
                       np.hstack([np.zeros([1, jointx_si]), J_p_eyeball]) * parameter.eyeball,
                       np.hstack([J_theta_limit_si, np.zeros([(2 * jointx_si), jointx_lg])]),
                       np.hstack([np.zeros([(2 * jointx_lg), jointx_si]), J_theta_limit_lg]),
                       np.hstack([-1 * J_trocar_si, np.zeros([1, jointx_lg])]) * parameter.trocar_si,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_trocar_lg]) * parameter.trocar_lg,
                       np.hstack([-1 * J_view_si, np.zeros([1, jointx_lg])]) * parameter.view,
                       np.hstack([np.zeros([1, jointx_si]), -1 * J_view_lg]) * parameter.view,
                       np.hstack([-1 * J_plane_si, np.zeros([J_plane_si.shape[0], jointx_lg])]) * parameter.plane,
                       np.hstack([-1 * np.zeros([J_plane_lg.shape[0], jointx_si]), J_plane_lg]) * parameter.plane,
                       np.hstack([J_forceps, np.zeros([1, jointx_lg])]) * parameter.forceps,
                       ])
        w = np.vstack([np.array([parameter.nd_rcm * D_rcm_error_si]) * parameter.rcm_si,
                       np.array([parameter.nd_rcm * D_rcm_error_lg]) * parameter.rcm_lg,
                       np.array([-1 * parameter.nd_collision * D_collision_error]) * parameter.collision,
                       np.array([parameter.nd_eyeball * D_eyeball_error]) * parameter.eyeball,
                       parameter.nd_limit * d_theta_limit_si,
                       parameter.nd_limit * d_theta_limit_lg,
                       np.array([parameter.nd_trocar * D_trocar_si]) * parameter.trocar_si,
                       np.array([parameter.nd_trocar * D_trocar_lg]) * parameter.trocar_lg,
                       np.array([-1 * parameter.nd_view * D_view_error_si]) * parameter.view,
                       np.array([-1 * parameter.nd_view * D_view_error_lg]) * parameter.view,
                       parameter.nd_plane * D_plane_si * parameter.plane,
                       parameter.nd_plane * D_plane_lg * parameter.plane,
                       np.array([parameter.nd_forceps * D_forceps_angle]) * parameter.forceps,
                       ])

        return W, w

    # @classmethod
    # def get_rcm_jacobians_for_eyeball_manipulation_simulation_icra(cls, denso_robot_si, denso_robot_lg, theta_si, theta_lg, eyeball_t, eyeball_radius, parameter):
    #     # Get the pose of the current tooltip pose
    #     x_si = denso_robot_si.fkm(theta_si)
    #     x_lg = denso_robot_lg.fkm(theta_lg)
    #     t_si = translation(x_si)
    #     t_lg = translation(x_lg)
    #     r_si = rotation(x_si)
    #     r_lg = rotation(x_lg)
    #     t_trocar_si, t_trocar_lg = eye_kinematics.get_current_rcm_translations(t_si, t_lg, r_si, r_lg, eyeball_t, eyeball_radius)
    #
    #     # Get Jacobians related to the current tooltip poses
    #     J_si = denso_robot_si.pose_jacobian(theta_si)
    #     J_lg = denso_robot_lg.pose_jacobian(theta_lg)
    #     Jt_si = denso_robot_si.translation_jacobian(J_si, x_si)
    #     Jt_lg = denso_robot_lg.translation_jacobian(J_lg, x_lg)
    #     Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
    #     Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)
    #     d_si, d_lg, J_d_si, J_d_lg = eye_kinematics.get_current_inserted_length_and_jacobian(Jt_si, Jt_lg, Jr_si, Jr_lg, t_si, t_lg, r_si, r_lg, eyeball_t, eyeball_radius)
    #     Jt_trocar_si, Jt_trocar_lg = eye_kinematics.get_current_rcm_translation_jacobians(Jt_si, Jt_lg, Jr_si, Jr_lg, J_d_si, J_d_lg, r_si, r_lg, d_si, d_lg)
    #
    #     if parameter.orbital_manipulation:
    #         J_d_rcm = (1 / np.linalg.norm(vec4(t_trocar_si - t_trocar_lg))) * vec4(t_trocar_si - t_trocar_lg) @ np.hstack([Jt_trocar_si, -1 * Jt_trocar_lg]).reshape([4, 13])
    #         J_d_rcm = J_d_rcm.reshape([1, 13])
    #
    #     else:
    #         Jt_trocar_si, Jt_trocar_lg = eye_kinematics.get_current_rcm_translation_jacobians(Jt_si, Jt_lg, Jr_si, Jr_lg, J_d_si, J_d_lg, r_si, r_lg, d_si, d_lg)
    #         J_d_rcm = np.hstack([Jt_trocar_si, Jt_trocar_lg])
    #
    #     return J_d_rcm
    #

    @classmethod
    def get_conical_VFIs(cls, manipulator_si, manipulator_lg, theta_si, theta_lg, ws_t, parameter):
        """
        - get_conical_VFIs(cls, manipulator_si, manipulator_lg, theta_si, theta_lg, ws_t, parameter) returns the array
        of Jacobians "W" and the array of corresponding distances "w" to enforce inequality constraints for
        the proposed autonomous positioning method.
        - This function is used to get Shadow-based Positioning Constraints described in IX of Koyama et al. (2022).
        """
        # Get the poses, translation, and rotations
        x_si = manipulator_si.fkm(theta_si)
        x_lg = manipulator_lg.fkm(theta_lg)
        jointx_si = theta_si.size
        jointx_lg = theta_lg.size
        t_si = translation(x_si)
        r_lg = rotation(x_lg)
        t_lg = translation(x_lg)

        # Get Jacobians
        J_si = manipulator_si.pose_jacobian(theta_si)
        Jt_si = manipulator_si.translation_jacobian(J_si, x_si)
        J_lg = manipulator_lg.pose_jacobian(theta_lg)
        Jt_lg = manipulator_lg.translation_jacobian(J_lg, x_lg)
        Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)

        # Prepare quaternions necessary to define the point-to-cone distance
        lz_lg = r_lg * k_ * conj(r_lg)
        t_lg_si = t_si - t_lg
        p_lg_c = ws_t - t_lg
        p_lg_a = (np.linalg.norm(vec4(dot(t_lg_si, k_))) / np.linalg.norm(vec4(dot(p_lg_c, k_)))) * p_lg_c
        p_a_si = t_lg_si - p_lg_a
        if p_a_si == 0:
            p_a_si = parameter.ws_radius / 100
        p_R2_e = p_lg_c + (parameter.ws_radius / np.linalg.norm(vec4(p_a_si))) * p_a_si

        # Get Calculate Jacobians for Shadow-based Positioning Constraints---Section IX of Koyama et al. (2022)
        J_shadow_cone = cls.point_to_cone_jacobian(Jt_si, Jt_lg, parameter.ws_radius, t_lg_si, p_lg_c, p_R2_e,
                                                   p_a_si)
        J_illumination_cone = cls.point_to_illumination_jacobian(r_lg, Jr_lg, lz_lg, t_lg_si, parameter.gamma, Jt_si,
                                                                 Jt_lg)
        J_tip = cls.get_tip_to_tip_distance_jacobian(Jt_si, Jt_lg, t_si, t_lg, jointx_si, jointx_lg)

        # Get Calculate distances for Shadow-based Positioning Constraints---Section IX of Koyama et al. (2022)
        d_shadow_cone = cls.point_to_cone_signed_distance(p_R2_e, p_lg_c, t_lg_si)
        d_illumination_cone = cls.point_to_illumination_signed_distance(lz_lg, t_lg_si, parameter.gamma)
        D_tip = cls.get_tip_to_tip_distance_error(x_lg, t_si, parameter.D_safe_tip)

        # Store them in arrays
        W = np.vstack([-1 * J_shadow_cone * 10 ** 10 * parameter.shadow_cone,
                       -1 * J_illumination_cone * parameter.illumination_cone,
                       -1 * J_tip * parameter.tip
                       ])
        w = np.vstack([np.array([parameter.nd_cone * d_shadow_cone * 10 ** 10]) * parameter.shadow_cone,
                       np.array([parameter.nd_illumination * d_illumination_cone]) * parameter.illumination_cone,
                       np.array([parameter.nd_tip * D_tip]) * parameter.tip
                       ])
        return W, w

    @classmethod
    def constraints_are_satisfied(cls, manipulator_si, manipulator_lg, robot_si_interface, robot_lg_interface,
                                  eye, parameter):
        """
        constraints_are_satisfied(manipulator_si, manipulator_lg, robot_si_interface, robot_lg_interface, eye, parameter,
        constrained_planes_si, constrained_planes_lg) stores "true" in the bool value "satisfied" if the all constraints
        are satisfied. The constraints must be satisfied at the beginning of the automation motion. See also Section II-B
        of Koyama et al. (2022).
        """
        # Prepare a bool
        satisfied = True

        # Get current status
        theta_si = robot_si_interface.get_joint_positions()
        theta_lg = robot_lg_interface.get_joint_positions()
        x_si = manipulator_si.fkm(theta_si)
        x_lg = manipulator_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_lg = rotation(x_lg)
        lz_lg = r_lg * k_ * conj(r_lg)

        # Get other values needed to check if the constraints are satisfied
        rcm_si = eye.rcm_si_t
        rcm_lg = eye.rcm_lg_t
        eyeball_t = eye.eyeball_t
        ws_t = eye.ws_t

        # Calculate the distances related to each constraint---Section VIII and IX of Koyama et al. (2022)
        t_R2_R1 = t_si - t_lg
        p_R2_c = ws_t - t_lg
        p_R2_a = (np.linalg.norm(vec4(dot(t_R2_R1, k_))) / np.linalg.norm(vec4(dot(p_R2_c, k_)))) * p_R2_c
        p_a_R1 = t_R2_R1 - p_R2_a
        if p_a_R1 == 0:
            p_a_R1 = parameter.ws_radius / 100
        p_R2_e = p_R2_c + (parameter.ws_radius / np.linalg.norm(vec4(p_a_R1))) * p_a_R1

        D_rcm_error_si = cls.get_shaft_to_rcm_distance_error(x_si, rcm_si, parameter.D_safe_rcm)
        D_rcm_error_lg = cls.get_shaft_to_rcm_distance_error(x_lg, rcm_lg, parameter.D_safe_rcm)
        D_trocar_si = cls.get_tip_to_trocar_distance_error(t_si, rcm_si, parameter.D_safe_trocar)
        D_trocar_lg = cls.get_tip_to_trocar_distance_error(t_lg, rcm_lg, parameter.D_safe_trocar)
        D_eyeball_error = cls.get_light_tip_to_eyeball_distance_error(x_lg, eyeball_t, parameter.D_safe_eyeball)
        D_collision_error = cls.get_light_tip_to_instrument_shaft_distance_error(x_si, x_lg,
                                                                                    parameter.D_safe_collision)
        d_shadow_cone = cls.point_to_cone_signed_distance(p_R2_e, p_R2_c, t_R2_R1)
        d_illumination_cone = cls.point_to_illumination_signed_distance(lz_lg, t_R2_R1, parameter.gamma)
        D_tip = cls.get_tip_to_tip_distance_error(x_lg, t_si, parameter.D_safe_tip)

        if parameter.rcm_si == 1 and not D_rcm_error_si >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: RCM constraint for robot1 is not OK... D_rcm_error = " + str(
                format(D_rcm_error_si, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.rcm_lg == 1 and not D_rcm_error_lg >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: RCM constraint for robot2 is not OK... D_rcm_error = " + str(
                format(D_rcm_error_lg, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.trocar_si == 1 and not D_trocar_si >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: Trocar constraint for robot1 is not OK... D_trocar_error = " + str(
                format(D_trocar_si, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.trocar_lg == 1 and not D_trocar_lg >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: Trocar constraint for robot2 is not OK... D_trocar_error = " + str(
                format(D_trocar_lg, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.eyeball == 1 and not D_eyeball_error >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: Eyeball constraint for robot2 is not OK... D_eyeball_error = " + str(
                format(D_eyeball_error, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.collision == 1 and not D_collision_error <= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: Collision constraint is not OK... D_collision_error = " + str(
                format(D_collision_error, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.shadow_cone == 1 and not d_shadow_cone >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print(
                "[eyesurgery_function.py]:: Conical constraint for microscopic view is not OK... d_p_to_cone_error = " + str(
                    format(d_shadow_cone, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.illumination_cone == 1 and not d_illumination_cone >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print(
                "[eyesurgery_function.py]:: Conical constraint for illumination range is not OK... d_p_to_illumination_error = " + str(
                    format(d_illumination_cone, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if parameter.tip == 1 and not D_tip >= 0:
            satisfied = False
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: Tip constraint is not OK... D_tip_error = " + str(
                format(D_tip, '.4f')) + ".")
            print("--------------------------------------------------\n")
        if satisfied:
            print("--------------------------------------------------")
            print("[eyesurgery_function.py]:: All constraints are OK.")
            print("--------------------------------------------------\n")
        return satisfied
