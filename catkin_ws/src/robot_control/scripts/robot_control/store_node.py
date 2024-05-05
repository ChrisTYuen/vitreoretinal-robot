#!/usr/bin/python3
# Import Relevant files from dqrobotics
from dqrobotics import *

# Import original files
from kinematics.parameters import control_parameters, physical_parameters
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI

# Import files from sas library
from sas_datalogger import DataloggerInterface
from sas_datalogger.msg import AddValueMsg

# Import message
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

# Import other dependencies
import numpy as np
import rospy
import time
import sys

# sys.path.append('/home/yuki/git/ykoyama2017/catkin_ws_unified/devel_release/lib/python3/dist-packages')
# sys.path.insert(1, '/home/nml/git/ykoyama2017/catkin_ws_unified/src/sas/sas_datalogger/src/sas_datalogger/__init__.py')


def main():
    # Init "store" node
    rospy.init_node('store')
    store_node = StoreNode()
    rospy.spin()


class StoreNode:
    """
    This class saves data for logging and making figures.
    """
    def __init__(self):
        self.setup = physical_parameters.EyesurgerySetup()
        self.parameter = control_parameters.Parameters()
        self.eye_parameters_name = {"rcm_1_t", "rcm_2_t", "ws_t", "eyeball_t"}

        self.data_logger = DataloggerInterface(10)

        self.subscriber_data_patient_side = rospy.Subscriber("store/store_data", AddValueMsg, self._store_callback_patient_side)
        self.subscriber_clutch = rospy.Subscriber("sss/operator_side_receiver_interface/clutch", Bool, self._store_callback_clutch)
        self.subscriber_motion_scaling = rospy.Subscriber("sss/operator_side_receiver_interface/motion_scaling", Float64, self._store_callback_motion_scaling)
        self.subscriber_right_pose = rospy.Subscriber("sss/operator_side_receiver_interface/right/pose", Pose, self._store_callback_right_pose)
        self.subscriber_right_twist = rospy.Subscriber("sss/operator_side_receiver_interface/right/twist", Twist, self._store_callback_right_twist)
        self.subscriber_left_pose = rospy.Subscriber("sss/operator_side_receiver_interface/left/pose", Pose, self._store_callback_left_pose)
        self.subscriber_left_twist = rospy.Subscriber("sss/operator_side_receiver_interface/left/twist", Twist, self._store_callback_left_twist)

        print("[" + rospy.get_name() + "]:: Ready!")

    def _store_callback_patient_side(self, msg):
        """
        _store_callback(self, msg) saves the values of the "msg" using the interface "data_logger" of "sas_datalogger".
        """
        log_start = time.time()

        # Storing process changes depending on the message
        if msg.name == "eyeball_variables":
            print("[" + rospy.get_name() + "]:: Got eyeball variables...\n")
            eyeball_variables = msg.value
            self.rcm_1_t = DQ(np.array([eyeball_variables[:4]]).reshape([4, 1]))
            self.rcm_2_t = DQ(np.array([eyeball_variables[4:8]]).reshape([4, 1]))
            self.ws_t = DQ(np.array([eyeball_variables[8:12]]).reshape([4, 1]))
            self.eyeball_t = DQ(np.array([eyeball_variables[12:16]]).reshape([4, 1]))
            self.eyeball_radius = self.parameter.eyeball_radius
            rotation_c_plane_1 = -1 * j_ + E_ * dot(self.eyeball_t, -1 * j_)
            rotation_c_n = k_
            rotation_c_plane_2 = rotation_c_n + E_ * dot(self.eyeball_t + self.eyeball_radius / 2 * k_, rotation_c_n)
            self.rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

            # Eye variables
            self.data_logger.log("rcm_1_t", vec4(self.rcm_1_t))
            self.data_logger.log("rcm_2_t", vec4(self.rcm_2_t))
            self.data_logger.log("workspace_t", vec4(self.ws_t))
            self.data_logger.log("eyeball_t", vec4(self.eyeball_t))

            # Condition
            self.data_logger.log("lg_automation", str(self.parameter.lg_automation))
            self.data_logger.log("orbital_manipulation", str(self.parameter.orbital_manipulation))
            self.data_logger.log("orbital_manipulation_ver", str(self.parameter.om_version_icra_ver))

            # Other conditions
            self.data_logger.log("solver", float(self.parameter.solver))

            # Constraints enforced or not
            self.data_logger.log("shadow_cone_enforce", float(self.parameter.shadow_cone))
            self.data_logger.log("illumination_cone_enforce", float(self.parameter.illumination_cone))
            self.data_logger.log("tip_enforce", float(self.parameter.tip))

            self.data_logger.log("om_trocar_si_enforce", float(self.parameter.om_trocar_si))
            self.data_logger.log("om_trocar_lg_enforce", float(self.parameter.om_trocar_lg))
            self.data_logger.log("om_D_rcm_enforce", float(self.parameter.om_D_rcm))
            self.data_logger.log("om_rot_plane_enforce", float(self.parameter.om_rot_plane))
            self.data_logger.log("om_rot_enforce", float(self.parameter.om_rot))

            self.data_logger.log("rcm_si_enforce", float(self.parameter.rcm_si))
            self.data_logger.log("rcm_lg_enforce", float(self.parameter.rcm_lg))
            self.data_logger.log("trocar_si_enforce", float(self.parameter.trocar_si))
            self.data_logger.log("trocar_lg_enforce", float(self.parameter.trocar_lg))
            self.data_logger.log("collision_enforce", float(self.parameter.collision))
            self.data_logger.log("eyeball_enforce", float(self.parameter.eyeball))
            self.data_logger.log("view_enforce", float(self.parameter.view))
            self.data_logger.log("plane_enforce", float(self.parameter.plane))

            # configurations
            self.data_logger.log("td_init_set_instrument_1", vec4(self.parameter.td_init_set_si[0]))
            self.data_logger.log("td_init_set_instrument_2", vec4(self.parameter.td_init_set_si[1]))
            self.data_logger.log("td_init_set_light_1", vec4(self.parameter.td_init_set_lg[0]))
            self.data_logger.log("td_init_set_light_2", vec4(self.parameter.td_init_set_lg[1]))

            self.data_logger.log("om_trajectory_point_list1", vec4(self.parameter.om_trajectory_point_list[0]))
            self.data_logger.log("om_trajectory_point_list2", vec4(self.parameter.om_trajectory_point_list[1]))
            self.data_logger.log("om_trajectory_point_list3", vec4(self.parameter.om_trajectory_point_list[2]))
            self.data_logger.log("om_trajectory_point_list4", vec4(self.parameter.om_trajectory_point_list[3]))

            self.data_logger.log("eyeball_radius", float(self.parameter.eyeball_radius))
            self.data_logger.log("port_angle ", float(self.parameter.port_angle))
            self.data_logger.log("eyeground_radius", float(self.parameter.eyeground_radius))
            self.data_logger.log("eyeground_plus_height", float(self.parameter.eyeground_plus_height))
            self.data_logger.log("trocar_outer_radius", float(self.parameter.trocar_outer_radius))
            self.data_logger.log("trocar_inner_radius", float(self.parameter.trocar_inner_radius))
            self.data_logger.log("radius_bias", float(self.parameter.radius_bias))
            self.data_logger.log("ws_radius", float(self.parameter.ws_radius))

            self.data_logger.log("control_priority", float(self.parameter.control_priority))
            self.data_logger.log("fps", float(self.parameter.fps))
            self.data_logger.log("n", float(self.parameter.n))
            self.data_logger.log("damping", float(self.parameter.damping))
            self.data_logger.log("alpha", float(self.parameter.alpha))
            self.data_logger.log("beta", float(self.parameter.beta))

            self.data_logger.log("tan_gamma", float(self.parameter.tan_gamma))
            self.data_logger.log("tan_bias", float(self.parameter.tan_bias))
            self.data_logger.log("gamma", float(self.parameter.gamma))

            self.data_logger.log("rotation_constraint_theta_icra", float(self.parameter.rotation_c_theta_icra))

            self.data_logger.log("nd_rcm", float(self.parameter.nd_rcm))
            self.data_logger.log("nd_trocar", float(self.parameter.nd_trocar))
            self.data_logger.log("nd_eyeball", float(self.parameter.nd_eyeball))
            self.data_logger.log("nd_collision", float(self.parameter.nd_collision))
            self.data_logger.log("nd_view", float(self.parameter.nd_view))
            self.data_logger.log("nd_limit", float(self.parameter.nd_limit))
            self.data_logger.log("nd_plane", float(self.parameter.nd_plane))
            self.data_logger.log("nd_cone", float(self.parameter.nd_cone))
            self.data_logger.log("nd_illumination", float(self.parameter.nd_illumination))
            self.data_logger.log("nd_tip", float(self.parameter.nd_tip))
            self.data_logger.log("nd_rotation_constraint", float(self.parameter.nd_rotation_constraint))

            self.data_logger.log("d_safe_rcm", float(self.parameter.d_safe_rcm))
            self.data_logger.log("d_safe_trocar", float(self.parameter.d_safe_trocar))
            self.data_logger.log("d_safe_eyeball", float(self.parameter.d_safe_eyeball))
            self.data_logger.log("d_safe_collision", float(self.parameter.d_safe_collision))
            self.data_logger.log("d_safe_view", float(self.parameter.d_safe_view))
            self.data_logger.log("d_safe_tip", float(self.parameter.d_safe_tip))
            self.data_logger.log("theta_safe_eye_rotation", float(self.parameter.theta_safe_eye_rotation))

            self.data_logger.log("insertion_distance", float(self.setup.insertion_distance))

            print("[" + rospy.get_name() + "]:: Load all of the parameters!")

        elif msg.name == "kinematics":
            store_data = msg.value

            theta_1 = np.array([store_data[:7]]).reshape([7, 1])
            theta_2 = np.array([store_data[7:13]]).reshape([6, 1])
            self.data_logger.log("theta_si", theta_1)
            self.data_logger.log("theta_lg", theta_2)
            self.data_logger.log("delta_thetas", np.array([store_data[13:26]]).reshape([13, 1]))

            x1 = DQ(np.array([store_data[26:34]]).reshape([8, 1]))
            x2 = DQ(np.array([store_data[34:42]]).reshape([8, 1]))
            xd_si = DQ(np.array([store_data[42:50]]).reshape([8, 1]))
            xd_lg = DQ(np.array([store_data[50:58]]).reshape([8, 1]))

            t1 = translation(x1)
            t2 = translation(x2)

            shadow_t = self.get_shadow_tip_translation(t1,t2)
            r2 = rotation(x2)
            lz_2 = r2 * k_ * conj(r2)
            self.store_translations_and_rotations(shadow_t, x1, x2, r2, lz_2, vec4(t1)[3], vec8(xd_si), vec8(xd_lg))

            D_rcm_error_1 = EyeVFI.get_shaft_to_rcm_distance_error(x1, self.rcm_1_t, self.parameter.D_safe_rcm)
            D_rcm_error_2 = EyeVFI.get_shaft_to_rcm_distance_error(x2, self.rcm_2_t, self.parameter.D_safe_rcm)
            D_trocar_1 = EyeVFI.get_tip_to_trocar_distance_error(t1, self.rcm_1_t, self.parameter.D_safe_trocar)
            D_trocar_2 = EyeVFI.get_tip_to_trocar_distance_error(t2, self.rcm_2_t, self.parameter.D_safe_trocar)
            D_eyeball_error = EyeVFI.get_light_tip_to_eyeball_distance_error(x2, self.eyeball_t,self.parameter.D_safe_eyeball)
            D_collision_error = EyeVFI.get_light_tip_to_instrument_shaft_distance_error(x1, x2,self.parameter.D_safe_collision)
            D_tip = EyeVFI.get_tip_to_tip_distance_error(x2, t1, self.parameter.D_safe_tip)

            self.store_distances(np.hstack([D_rcm_error_1, D_rcm_error_2, D_eyeball_error, D_collision_error,
                                            D_trocar_1, D_trocar_2, D_tip]))

            self.data_logger.log("hz_store", float(1/(time.time()-log_start)))

        if self.parameter.print_datalog_time:
            print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-log_start)))+" Hz")

    def _store_callback_clutch(self, msg):
        if msg.data:
            self.data_logger.log("clutch", np.array([1, rospy.get_time()]))
        else:
            self.data_logger.log("clutch", np.array([0, rospy.get_time()]))

    def _store_callback_motion_scaling(self, msg):
        self.data_logger.log("motion_scaling", msg.data)

    def _store_callback_right_pose(self, msg):
        self.data_logger.log("right_pose", np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))

    def _store_callback_right_twist(self, msg):
        self.data_logger.log("right_twist", np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]))

    def _store_callback_left_pose(self, msg):
        self.data_logger.log("left_pose", np.array([msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))

    def _store_callback_left_twist(self, msg):
        self.data_logger.log("left_twist", np.array([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]))

    def get_shadow_tip_translation(self, si_tip_t, lg_tip_t):
        # Get necessary vectors
        ws_si_vec = vec3(si_tip_t - self.ws_t)
        si_lg_vec = vec3(lg_tip_t - si_tip_t)
        si_vec = vec3(si_tip_t)
        ws_vec = vec3(self.ws_t)

        # Get the shadow tip position
        shadow_x = si_vec[0] - si_lg_vec[0] * np.abs(ws_si_vec[2]) / np.abs(si_lg_vec[2])
        shadow_y = si_vec[1] - si_lg_vec[1] * np.abs(ws_si_vec[2]) / np.abs(si_lg_vec[2])
        shadow_z = ws_vec[2]
        shadow_tip_t = shadow_x * i_ + shadow_y * j_ + shadow_z * k_
        return shadow_tip_t

    def store_translations_and_rotations(self, shadow_tip, si_tip_dq, lg_tip_dq, r2, lz_2, si_z, xd_si,xd_lg):
        i_another, l_another, s_another = self.get_other_store_translations(si_tip_dq, lg_tip_dq)
        si_tip = translation(si_tip_dq)
        lg_tip = translation(lg_tip_dq)
        td_si = translation(DQ(xd_si))
        rd_si = rotation(DQ(xd_si))
        td_lg = translation(DQ(xd_lg))
        rd_lg = rotation(DQ(xd_lg))
        translations_and_rotations = np.hstack([vec4(shadow_tip), vec4(si_tip), vec4(lg_tip), i_another, l_another, s_another, vec4(r2), vec4(lz_2), si_z, vec4(td_si), vec4(rd_si), vec4(td_lg), vec4(rd_lg)])
        self.data_logger.log("translations_and_rotations", translations_and_rotations)

    def get_other_store_translations(self,si_tip_dq,lg_tip_dq):
        # Get another point's position on the instrument shaft. This is necessary to make an animation of results.
        # We get another point so that its x component becomes -12 mm(eyeball's radius).
        t_i_tip = vec4(translation(si_tip_dq) - self.ws_t)
        x_i_another = si_tip_dq * (1 + 0.5 * E_ * (-1) * 0.012 * k_)
        t_i_another = vec4(translation(x_i_another) - self.ws_t)
        t = (self.parameter.eyeball_radius - t_i_tip[1]) / (t_i_another[1] - t_i_tip[1])
        t_i_another[1] = t_i_tip[1] + t * (t_i_another[1] - t_i_tip[1])
        t_i_another[2] = t_i_tip[2] + t * (t_i_another[2] - t_i_tip[2])
        t_i_another[3] = t_i_tip[3] + t * (t_i_another[3] - t_i_tip[3])

        # Get another point's position on the light guide shaft. This is necessary to make an animation of results
        # We get another point so that its x component becomes 12 mm(eyeball's radius).
        t_l_tip = vec4(translation(lg_tip_dq) - self.ws_t)
        x_l_another = lg_tip_dq * (1 + 0.5 * E_ * (-1) * 0.012 * k_)
        t_l_another = vec4(translation(x_l_another) - self.ws_t)
        t = (-self.parameter.eyeball_radius - t_l_tip[1]) / (t_l_another[1] - t_l_tip[1])
        t_l_another[1] = t_l_tip[1] + t * (t_l_another[1] - t_l_tip[1])
        t_l_another[2] = t_l_tip[2] + t * (t_l_another[2] - t_l_tip[2])
        t_l_another[3] = t_l_tip[3] + t * (t_l_another[3] - t_l_tip[3])

        # Get another point's position on the shadow. This is necessary to make an animation of results
        t_e_i_vec = vec3(DQ(t_i_another) - self.ws_t)
        t_i_l_vec = vec3(DQ(t_l_another) - DQ(t_i_another))
        t_si_tip_vec = vec3(DQ(t_i_another))
        eyeground_vec = vec3(self.ws_t)
        shadow_x = t_si_tip_vec[0] - t_i_l_vec[0] * np.abs(t_e_i_vec[2]) / np.abs(t_i_l_vec[2])
        shadow_y = t_si_tip_vec[1] - t_i_l_vec[1] * np.abs(t_e_i_vec[2]) / np.abs(t_i_l_vec[2])
        shadow_z = eyeground_vec[2]
        t_s_another = vec4(shadow_x * i_ + shadow_y * j_ + shadow_z * k_)
        return t_i_another, t_l_another, t_s_another

    def store_distances(self, safe_distances):
        self.data_logger.log("safe_distances", safe_distances)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
