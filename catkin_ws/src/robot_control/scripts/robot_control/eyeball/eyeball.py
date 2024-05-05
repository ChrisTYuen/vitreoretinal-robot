# Import dqrobotics for calculation of dual quaternion algebra
from dqrobotics import *

# Import other dependencies
import numpy as np
import math


class Eyeball:
    """
    This class contains eyeball related parameters such as its position, radius, etc...
    This object also has some functions that uses eyeball related parameters.
    """
    def __init__(self, eyeball_dq, rcm_si_dq, rcm_lg_dq, eyeball_parameter):
        self.eyeball_radius = eyeball_parameter.eyeball_radius
        self.__eyeball_dq = eyeball_dq
        self.eyeball_t = translation(eyeball_dq)
        self.ws_radius = eyeball_parameter.ws_radius
        self.ws_dq = self._get_ws_dq()
        # The height of the workspace depends on the eye model, so it can be changed by adding "eyeground_plus_height"
        self.ws_t = translation(self.ws_dq) + eyeball_parameter.eyeground_plus_height * k_
        self.rcm_si_dq = rcm_si_dq
        self.rcm_lg_dq = rcm_lg_dq
        self.rcm_si_t = translation(rcm_si_dq)
        self.rcm_lg_t = translation(rcm_lg_dq)
        print("[eyeball.py]::An Eyeball object has been created!\n")

    def _get_ws_dq(self):
        """
        _get_ws_dq(self) returns the dual quaternion "ws_dq" that represents the pose of the workspace.
        """
        # Height is the distance from the center of the eyeball to the workspace
        height = np.sqrt(self.eyeball_radius ** 2 - self.ws_radius ** 2)
        ws_dq = 1 + 0.5 * E_ * (self.eyeball_t - height * k_)
        return ws_dq

    def get_eyeground_points(self, number):
        """
        get_eyeground_points(self, number) returns the array of dual quaternions "points" that represent the poses of
        the points that divide the circumference of the circular workspace into "number" equal parts.
        """
        i = 0
        points = []
        while i < number:
            # Get the translation of the i-th point
            p = self.ws_t + self.ws_radius * math.cos((i + 1) * 2 * math.pi / number) * i_ \
                + self.ws_radius * math.sin((i + 1) * 2 * math.pi / number) * j_
            # Get the pose of the i-th point letting the rotation be equal to 1
            p_dq = 1 + 0.5 * E_ * p
            points.append(p_dq)
            i = i + 1
        print("[Eyeball.py]::Got positions of " + str(number) + " points on the eyeground!\n")
        return points

    def get_shadow_tip_position(self, si_tip_dq, lg_tip_dq, r_o_e):
        """
        get_shadow_tip_position(self, si_tip_dq, lg_tip_dq) returns the dual quaternion "shadow_tip_dq" that represents
        the pose of tip of the instrument's shadow projected on the workspace.
        """
        # Get the translations of the tips
        si_tip_t = translation(si_tip_dq)
        lg_tip_t = translation(lg_tip_dq)

        # Get necessary vectors
        ws_si_vec = vec3(conj(r_o_e) * (si_tip_t - ((r_o_e * (self.ws_t - self.eyeball_t) * conj(r_o_e)) + self.eyeball_t)) * r_o_e)
        si_lg_vec = vec3(conj(r_o_e) * (lg_tip_t - si_tip_t) * r_o_e)
        si_vec = vec3(conj(r_o_e) * (si_tip_t - self.eyeball_t) * r_o_e)
        ws_vec = vec3(self.ws_t - self.eyeball_t)

        # Get the shadow tip translation
        shadow_x = si_vec[0] - si_lg_vec[0] * np.abs(ws_si_vec[2]) / np.abs(si_lg_vec[2])
        shadow_y = si_vec[1] - si_lg_vec[1] * np.abs(ws_si_vec[2]) / np.abs(si_lg_vec[2])
        shadow_z = ws_vec[2]

        shadow_tip_t = (r_o_e * (shadow_x * i_ + shadow_y * j_ + shadow_z * k_) * conj(r_o_e)) + self.eyeball_t

        # Calculate the pose of the shadow's tip letting the rotation be equal to 1
        shadow_tip_dq = 1 + 0.5 * E_ * shadow_tip_t
        return shadow_tip_dq

    def get_initial_tip_poses_inside_eyeball(self, insertion_distance):
        """
        get_initial_tip_poses_inside_eyeball(self, insertion_distance) returns the desired initial poses of the tips
        of the instruments, "xd_si" and "xd_lg", inside the eyeball. This function is used to define the initial poses
        of the tips in simulation studies.
        """
        # Calculate the desired translations of the tips so that the instruments point to the center of the eyeball
        c_to_rcm_si = vec4(self.rcm_si_t - self.eyeball_t)
        from_center_si = c_to_rcm_si * (self.eyeball_radius - insertion_distance) / self.eyeball_radius
        from_center_si = DQ(from_center_si)
        c_to_rcm_lg = vec4(self.rcm_lg_t - self.eyeball_t)
        from_center_lg = c_to_rcm_lg * (self.eyeball_radius - insertion_distance) / self.eyeball_radius
        from_center_lg = DQ(from_center_lg)

        td_si = self.eyeball_t + from_center_si
        td_lg = self.eyeball_t + from_center_lg

        # Calculate the desired rotations of the instruments so that the shaft lines pass through the insertion points
        # and the center of the eyeball
        rotation_si = math.cos(math.pi / 2) + k_ * math.sin(math.pi / 2)
        rotation_lg = math.cos(math.pi / 4) - j_ * math.sin(math.pi / 4)

        rotation_3_theta_si = math.atan(-1 * c_to_rcm_si[2] / c_to_rcm_si[1])
        rotation_3_si = math.cos(rotation_3_theta_si / 2) - i_ * math.sin(rotation_3_theta_si / 2)
        rotation_4_theta_si = math.atan(c_to_rcm_si[3] / math.sqrt(c_to_rcm_si[1] ** 2 + c_to_rcm_si[2] ** 2))
        rotation_4_si = math.cos(rotation_4_theta_si / 2) - j_ * math.sin(rotation_4_theta_si / 2)

        rotation_3_theta_lg = math.atan(c_to_rcm_lg[2] / c_to_rcm_lg[1])
        rotation_3_lg = math.cos(rotation_3_theta_lg / 2) + i_ * math.sin(rotation_3_theta_lg / 2)

        rotation_4_theta_lg = math.atan(c_to_rcm_lg[3] / math.sqrt(c_to_rcm_lg[1] ** 2 + c_to_rcm_lg[2] ** 2))
        rotation_4_lg = math.cos(rotation_4_theta_lg / 2) - j_ * math.sin(rotation_4_theta_lg / 2)

        rd_si = rotation_lg * rotation_3_si * rotation_4_si
        rd_lg = rotation_si * rotation_lg * rotation_3_lg * rotation_4_lg

        # Calculate the desired poses of both instruments
        xd_si = rd_si + 0.5 * E_ * td_si * rd_si
        xd_lg = rd_lg + 0.5 * E_ * td_lg * rd_lg
        return xd_si, xd_lg

    def get_tip_store_translations(self, s_tip_dq, i_tip_dq, l_tip_dq):
        """
        get_tip_store_translations(self, s_tip_dq, i_tip_dq, l_tip_dq) returns the positions of the surgical
        instrument's tip, the light guide's tip, and the shadow's tip, "s_tip", "l_tip", and "s_tip",
        in the coordinate of the workspace. This function is used to store the values for logging.
        """
        i_tip = np.array([vec4(translation(i_tip_dq) - self.ws_t)])
        l_tip = np.array([vec4(translation(l_tip_dq) - self.ws_t)])
        s_tip = np.array([vec4(translation(s_tip_dq) - self.ws_t)])
        return s_tip, i_tip, l_tip

    def get_other_store_translations(self, i_tip_dq, l_tip_dq):
        """
        get_other_store_translations(self, i_tip_dq, l_tip_dq) returns other points, "t_i_another", "t_l_another", and
        "t_s_another", needed to make animations of the instrument's movements. This function is used to store these
        values for logging and making animations.
        """
        # Get the position of another point in the instrument shaft
        # We get another point so that its x component becomes -12 mm(eyeball's radius).
        t_i_tip = np.array([vec4(translation(i_tip_dq) - self.ws_t)])
        x_i_another = i_tip_dq * (1 + 0.5 * E_ * (-1) * 0.012 * k_)
        t_i_another = np.array([vec4(translation(x_i_another) - self.ws_t)])
        t = (self.eyeball_radius - t_i_tip[0, 1]) / (t_i_another[0, 1] - t_i_tip[0, 1])
        t_i_another[0, 1] = t_i_tip[0, 1] + t * (t_i_another[0, 1] - t_i_tip[0, 1])
        t_i_another[0, 2] = t_i_tip[0, 2] + t * (t_i_another[0, 2] - t_i_tip[0, 2])
        t_i_another[0, 3] = t_i_tip[0, 3] + t * (t_i_another[0, 3] - t_i_tip[0, 3])

        # Get the position of another point in the light guide shaft
        # We get another point so that its x component becomes 12 mm(eyeball's radius).
        t_l_tip = np.array([vec4(translation(l_tip_dq) - self.ws_t)])
        x_l_another = l_tip_dq * (1 + 0.5 * E_ * (-1) * 0.012 * k_)
        t_l_another = np.array([vec4(translation(x_l_another) - self.ws_t)])
        t = (-self.eyeball_radius - t_l_tip[0, 1]) / (t_l_another[0, 1] - t_l_tip[0, 1])
        t_l_another[0, 1] = t_l_tip[0, 1] + t * (t_l_another[0, 1] - t_l_tip[0, 1])
        t_l_another[0, 2] = t_l_tip[0, 2] + t * (t_l_another[0, 2] - t_l_tip[0, 2])
        t_l_another[0, 3] = t_l_tip[0, 3] + t * (t_l_another[0, 3] - t_l_tip[0, 3])

        # Get the position of another point in the shadow
        t_e_i = DQ(t_i_another[0]) - self.ws_t
        t_e_i_vec = vec3(t_e_i)
        t_i_l = DQ(t_l_another[0]) - DQ(t_i_another[0])
        t_i_l_vec = vec3(t_i_l)
        t_si_tip_vec = vec3(DQ(t_i_another[0]))
        eyeground_vec = vec3(self.ws_t)
        shadow_x = t_si_tip_vec[0] - t_i_l_vec[0] * np.abs(t_e_i_vec[2]) / np.abs(t_i_l_vec[2])
        shadow_y = t_si_tip_vec[1] - t_i_l_vec[1] * np.abs(t_e_i_vec[2]) / np.abs(t_i_l_vec[2])
        shadow_z = eyeground_vec[2]
        t_s_another = np.array([vec4(shadow_x * i_ + shadow_y * j_ + shadow_z * k_)])
        return t_i_another, t_l_another, t_s_another

    def get_vertical_depth(self, t1_above_target):
        """
        get_vertical_depth(t1_above_target, ws_t) returns the distance "depth" between the current tip position and the
        workspace on the retina calculated using a kinematic models of the robots.
        """
        # Calculate the distance
        vertical = vec4(t1_above_target - self.ws_t)
        depth = abs(vertical[3])
        return depth

    def get_shaft_shadow_tip_distance(self, shadow_tip_dq, x1):
        """
        get_shaft_shadow_tip_distance(shadow_tip_dq, x1) returns the distance "shaft_distance" between the instrument's
        shaft and the tip of the shadow in the microscopic view. This distance is used to judge when to stop the overlap
        prevention step.
        See also Section VI of Koyama et al. (2022)
        """
        # Get the position of the shadow's tip in the microscopic view
        shadow_tip_vec = vec4(translation(shadow_tip_dq))
        shadow_tip_retina = np.array([shadow_tip_vec[1], shadow_tip_vec[2]])

        # Get the position of the surgical instrument's tip in the microscopic view
        si_tip_vec = vec4(translation(x1))
        si_tip_retina = np.array([si_tip_vec[1], si_tip_vec[2]])

        # Calculate the normal of the instrument's shaft line in the microscopic view
        si_shaft_vec = vec4(rotation(x1) * k_ * conj(rotation(x1)))
        si_shaft_retina = np.array([si_shaft_vec[1], si_shaft_vec[2]])
        normal_si_shaft_retina = np.array([si_shaft_vec[2] / np.linalg.norm(si_shaft_retina),
                                           -1 * si_shaft_vec[1] / np.linalg.norm(si_shaft_retina)])

        # Calculate the distance
        distance_vec = shadow_tip_retina - si_tip_retina
        shaft_distance = np.linalg.norm(np.dot(distance_vec, normal_si_shaft_retina))
        return shaft_distance

    def get_tip_shadow_tip_distance(self, shadow_tip_dq, x1):
        """
        get_tip_shadow_tip_distance(shadow_tip_dq, x1) returns the distance "tip_distance" between the instrument's tip
        and the shadow's tip in the microscopic view. This distance is used to judge when to stop the vertical positioning
        step.
        See also Section VII of Koyama et al. (2022)
        """
        # Get the position of the shadow's tip in the microscopic view
        shadow_tip_vec = vec4(translation(shadow_tip_dq))
        shadow_tip_retina = np.array([shadow_tip_vec[1], shadow_tip_vec[2]])

        # Get the position of the surgical instrument's tip in the microscopic view
        si_tip_vec = vec4(translation(x1))
        si_tip_retina = np.array([si_tip_vec[1], si_tip_vec[2]])

        # Calculate the distance
        distance_vec = shadow_tip_retina - si_tip_retina
        tip_distance = np.linalg.norm(distance_vec)
        return tip_distance

