#!/usr/bin/python3
# Import Relevant files from dqrobotics and sas
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from sas_robot_driver import RobotDriverInterface
from sas_robot_kinematics import RobotKinematicsProvider
from sas_datalogger import DataloggerInterface
from robot_loader import Robot
from std_msgs.msg import Float64MultiArray

# Original
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
import kinematics.kinematics_functions as kine_func
from tools import functions
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from interfaces import store_interface
from haptic_forceps_controller import ForcepsController
from eyesurgery_controllers import EyesurgeryControllers
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics

# Some libraries for calculations
import numpy as np
import time
import math

# ROS
import rospy

def tele_operation():
    try:
        rospy.init_node("tele_controller", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start kinematics...")

        vi = DQ_VrepInterface()
        while not vi.connect("127.0.0.1", 19996, 100, 10):
            time.sleep(1)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = physical_parameters.EyesurgerySetup()
        sim_setup = physical_parameters.SimulationSetup
        parameter = control_parameters.Parameters()
        eye_parameter = eyeball_parameter.EyeballParameters
        store = store_interface.StoreInterface()
        data_logger = DataloggerInterface(10)
        controller = EyesurgeryControllers()
        forceps = ForcepsController()


        # Define robots: denso_robot_light : left hand, denso_robot_instrument with attachment: right hand
        robot_si = Robot(setup.robot_parameter_path_instrument).kinematics()
        robot_lg = Robot(setup.robot_parameter_path_light).kinematics()
        time.sleep(0.05)

        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Start tele_operation with physical robots. OK?\n")
            forceps_control = forceps.haptic_forceps_setup(setup)
            forceps_control.initialize()
            pub_forceps_si_closure = rospy.Publisher('escon_1/set/target_joint_positions', Float64MultiArray, queue_size=10)
        else:
            print("[" + rospy.get_name() + "]:: Start tele_operation in simulation.\n")
        
        # Set the end_effector positions
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq)

        # Define Solver
        if parameter.solver == 0:
            qp_solver = DQ_QuadprogSolver()
        else:
            qp_solver = DQ_CPLEXSolver()

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting for the RobotDriverInterfaces to be enabled.")
        robot_si_interface = RobotDriverInterface("/arm2")
        robot_lg_interface = RobotDriverInterface("/arm1")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        input("[" + rospy.get_name() + "]:: Enabled. Press enter to continue...")

        robot_si_provider = RobotKinematicsProvider("/arm1_kinematics")
        robot_lg_provider = RobotKinematicsProvider("/arm2_kinematics")

        # Set robot base
        robot_si.set_reference_frame(vi.get_object_pose("VS050_reference#2"))
        time.sleep(0.02)
        robot_lg.set_reference_frame(vi.get_object_pose("VS050_reference#2")*setup.robot_lg_base_rel)
        time.sleep(0.02)

        # Get joint positions
        theta_si = np.array(robot_si_interface.get_joint_positions())
        theta_lg = np.array(robot_lg_interface.get_joint_positions())

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
        rcm_si_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_si_rcm"))
        rcm_lg_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_lg_rcm"))
        vi.set_object_pose(sim_setup.rcm_si_vrep_name, rcm_si_dq)
        vi.set_object_pose(sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
        print("[" + rospy.get_name() + "]:: Calculated rcm positions from the tip positions!")
        time.sleep(.5)

        eyeball_dq = kine_func.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius, parameter.port_angle)
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

        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to move the tool-tips to the start positions...\n")
        else:
            print("[" + rospy.get_name() + "]:: Move the tool-tips to the start positions...\n")
        # print(1)

        controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, parameter.td_init_set_si,
                                                              eye.rcm_si_t, eye.eyeball_t,
                                                              sim_setup.si_vrep_name, vi)

        # print(1)
        controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, parameter.td_init_set_lg,
                                                              eye.rcm_lg_t, eye.eyeball_t,
                                                              sim_setup.lg_vrep_name, vi)

        # print(1)
        if not EyeVFI.constraints_are_satisfied(robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye, parameter):
            print("--------------------------------------------------")
            input("[" + rospy.get_name() + "]:: Positioning was quit.")
            print("--------------------------------------------------")
            controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, [t_lg_inserted - eye.eyeball_t],
                                                                  eye.rcm_lg_t, eye.eyeball_t,
                                                                  sim_setup.lg_vrep_name, vi)
            controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, [t_si_inserted - eye.eyeball_t],
                                                                  eye.rcm_si_t, eye.eyeball_t,
                                                                  sim_setup.si_vrep_name, vi)
            exit()

        theta_si = robot_si_interface.get_joint_positions()
        theta_lg = robot_lg_interface.get_joint_positions()

        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)

        print("[" + rospy.get_name() + "]:: Waiting for RobotKinematicsProvider to be enabled...")
        while not robot_lg_provider.is_enabled() or not robot_si_provider.is_enabled():
            robot_si_provider.send_pose(x_si)
            robot_si_provider.send_reference_frame(vi.get_object_pose("VS050_reference#2"))
            robot_lg_provider.send_pose(x_lg)
            robot_lg_provider.send_reference_frame(vi.get_object_pose("VS050_reference#2")*setup.robot_lg_base_rel)
            time.sleep(0.01)

        print("[" + rospy.get_name() + "]:: robot_provider enabled.")
        robot_si_provider.get_desired_pose()
        print("[" + rospy.get_name() + "]:: got desired pose of robot_instrument!!")
        robot_lg_provider.get_desired_pose()
        print("[" + rospy.get_name() + "]:: got desired pose of robot_light!!")
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to start control loop....\n")
            print("[" + rospy.get_name() + "]:: Starting control loop...")
        else:
            print("\n[" + rospy.get_name() + "]:: Starting control loop....\n")

        # Parameters for orbital manipulation
        t_si = translation(x_si)
        t_lg = translation(x_lg)
        r_si = rotation(x_si)
        r_lg = rotation(x_lg)
        axis = k_
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))
        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)
        t_rcm_si, t_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                        eye.eyeball_t, eye.eyeball_radius)
        D_rcm_init = np.linalg.norm(vec4(t_rcm_si - t_rcm_lg)) ** 2

        rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
        rotation_c_n = k_
        rotation_c_plane_2 = rotation_c_n + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, rotation_c_n)
        rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

        normal1 = normalize(math.cos(np.deg2rad(parameter.theta_safe_eye_plane)) * i_ + math.sin(np.deg2rad(parameter.theta_safe_eye_plane)) * j_)
        normal2 = normalize(-1 * math.cos(np.deg2rad(parameter.theta_safe_eye_plane)) * i_ + math.sin(np.deg2rad(parameter.theta_safe_eye_plane)) * j_)
        rotation_c_plane_unified_1 = normal1 + E_ * dot(eye.eyeball_t, normal1)
        rotation_c_plane_unified_2 = normal2 + E_ * dot(eye.eyeball_t, normal2)
        rotation_c_plane_unified_list = [rotation_c_plane_unified_1, rotation_c_plane_unified_2]

        [constrained_plane_list_si, constrained_plane_list_lg] = kine_func.get_constrained_plane_list(eye.eyeball_t,
                                                                                                      setup.celling_height,
                                                                                                      setup.floor_height)

        ##############################
        # Control Loop
        ##############################
        iteration = 0
        # Run simulation until Keyboard Interrupt
        r = rospy.Rate(parameter.fps)
        while True:
            start = time.time()
            iteration = iteration + 1

            # Get target pose from V-REP
            xd_si = robot_si_provider.get_desired_pose()
            td_si = translation(xd_si)
            # rd_si = r_si * normalize(1 - 0.05 * k_)
            rd_si = rotation(xd_si) #.normalize()

            if parameter.lg_automation:
                xd_lg = vi.get_object_pose(sim_setup.lg_vrep_name)
            else:
                xd_lg = robot_lg_provider.get_desired_pose()

            # print(np.rad2deg(theta_si))

            vi.set_object_pose(sim_setup.si_xd_vrep_name, xd_si)
            vi.set_object_pose(sim_setup.lg_xd_vrep_name, xd_lg)
            # input("[" + rospy.get_name() + "]:: Tested xd_lg object pose. Press enter to continue...\n")

            # Get the pose of the current tooltip pose
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            jointx_si = theta_si.size
            jointx_lg = theta_lg.size
            jointx_comb = jointx_si + jointx_lg
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)

            l_si = normalize(r_si * axis * conj(r_si))  # direction of z-axis of the instrument
            l_lg = normalize(r_lg * axis * conj(r_lg))  # direction of z-axis of the light guide

            vi.set_object_pose(sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

            robot_si_provider.send_pose(x_si)
            robot_lg_provider.send_pose(x_lg)
            # input("[" + rospy.get_name() + "]:: Tested x_lg object pose. Press enter to continue...\n")

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t, eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       rcm_current_si, rcm_current_lg)

            shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
            vi.set_object_pose(sim_setup.shadow_vrep_name, shadow_tip_dq)

            vi.set_object_rotation(sim_setup.eyeball_vrep_name, r_o_e)
            vi.set_object_translation(sim_setup.rcm_si_vrep_name, rcm_current_si)
            vi.set_object_translation(sim_setup.rcm_lg_vrep_name, rcm_current_lg)

            # Get Jacobians related to the current tooltip poses
            J_si = robot_si.pose_jacobian(theta_si)
            Jt_si = robot_si.translation_jacobian(J_si, x_si)
            Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
            Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)
            Jl_si = (haminus4(axis * conj(r_si)) + hamiplus4(r_si * axis) @ C4()) @ Jr_si
            Jl_lg = (haminus4(axis * conj(r_lg)) + hamiplus4(r_lg * axis) @ C4()) @ Jr_lg
            Jr_rd_si = (haminus4(rd_si) @ C4()) @ Jr_si

            # Define errors
            td_eye = conj(r_o_e)*(td_si-eye.eyeball_t)*r_o_e                                    # Translation error of eyeball
            e_si_t = np.array([vec4(t_si - td_si)])                                             # Translation error of instrument
            e_si_r = np.array([vec4(kine_func.closest_invariant_rotation_error(x_si, xd_si))])  # Rotation error of instrument
            e_lg_t = np.array([vec4(t_lg - translation(xd_lg))])

            if parameter.print_error:
                print("instrument translation error:", np.linalg.norm(e_si_t))
                print("instrument rotation error:", np.linalg.norm(e_si_r))   

            # Quadratic programming (without the proposed constraints)
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg)
            # input("[" + rospy.get_name() + "]:: Tested vitreo VFI's. Press enter to continue...\n")
            W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t, parameter)
            # input("[" + rospy.get_name() + "]:: Tested conical VFI's. Press enter to continue...\n")
            if parameter.om_version_icra_ver:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(robot_si, robot_lg, theta_si,
                                                                                  theta_lg,
                                                                                  eye.eyeball_t, eye.eyeball_radius,
                                                                                  parameter, D_rcm_init,
                                                                                  rotation_c_plane_list)
            else:
                W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs(robot_si, robot_lg, theta_si, theta_lg,
                                                                         eye.eyeball_t, eye.eyeball_radius, parameter,
                                                                         D_rcm_init, r_o_e, rcm_init_si, rcm_init_lg,
                                                                         rotation_c_plane_unified_list)
            # input("[" + rospy.get_name() + "]:: Tested orbital manipulation VFI's. Press enter to continue...\n")
            # print("W_vitreo", W_vitreo.shape,"/n", "w_vitreo", w_vitreo.shape)
            # print("W_conical", W_conical.shape,"/n", "w_conical", w_conical.shape)
            # print("W_om", W_om.shape, "/n", "w_om", w_om.shape)
            W = np.vstack([W_vitreo,
                           W_conical,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_conical,
                           w_om])
            # print("W", W.shape, W)
            # print("w", w.shape, w)

            eye_rotation_jacobian = om_kinematics.get_eyeball_rotation_jacobian(Jt_si, Jt_lg, Jl_si, Jl_lg, t_si, t_lg,
                                                                                l_si, l_lg, jointx_lg,
                                                                                rcm_current_si, rcm_current_lg,
                                                                                eye.eyeball_t, eye.eyeball_radius,
                                                                                rcm_init_si, rcm_init_lg)

            eyeball_jacobian_t = om_kinematics.get_eyeball_jacobian_translation(Jt_si, Jt_lg, Jl_si, Jl_lg,
                                                                              t_si, t_lg, l_si, l_lg, jointx_lg,
                                                                              rcm_current_si, rcm_current_lg,
                                                                              eye.eyeball_t, eye.eyeball_radius, rcm_init_si,
                                                                              rcm_init_lg, td_eye)
            
            eyeball_jacobian_r = (np.hstack([Jr_rd_si, np.zeros([4, jointx_lg])]))

            if parameter.end_effector_rotation:
            # Quadratic coefficients of the decision variables
                A1 = parameter.alpha * eyeball_jacobian_t.T @ eyeball_jacobian_t
                A2 = (1 - parameter.alpha) * eyeball_jacobian_r.T @ eyeball_jacobian_r
                H1 = parameter.beta * (A1 + A2)                                                   # instrument
                A3 = np.vstack([np.zeros([4, jointx_comb]),
                                np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
                H2 = (1 - parameter.beta) * A3.T @ A3                                             # light guide
                H3 = parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation
                H = 2 * ((H1 + H2 + H3) + parameter.damping * parameter.B_13)
                # print("eyeball_jacobian_t", eyeball_jacobian_t.shape, eyeball_jacobian_t)
                # print("eyeball_jacobian_r", eyeball_jacobian_r.shape, eyeball_jacobian_r)
                # print("A1", A1.shape, A1)
                # print("A2", A2.shape, A2)
                # # print("H1", H1.shape, H1)
                # print("A3", A3.shape, A3)
                # # print("H2", H2.shape, H2)
                # # print("H3", H3.shape, H3)
                # print("H", H.shape, H)

                # Linear coefficients of the decision variables
                A4 = parameter.alpha * eyeball_jacobian_t.T @ e_si_t.T
                A5 = (1 - parameter.alpha) * eyeball_jacobian_r.T @ e_si_r.T
                c1 = parameter.beta * parameter.n * (A4 + A5)  # instrument
                A6 = np.vstack([np.zeros([jointx_si, 1]),
                                Jt_lg.T @ e_lg_t.T])
                c2 = (1 - parameter.beta) * parameter.n * A6   # light guide
                c = 2 * (c1 + c2)
                # print("e_si_t", e_si_t.shape, e_si_t)
                # print("e_si_r", e_si_r.shape, e_si_r)
                # # print("A4", A4.shape, A4)
                # # print("A5", A5.shape, A5)
                # print("c1", c1.shape, c1)
                # # print("A6", A6.shape, A6)
                # print("c2", c2.shape, c2)
                # print("c", c.shape, c)
            else:
                H1 = parameter.beta * eyeball_jacobian_t.T @ eyeball_jacobian_t                   # instrument
                A1 = np.vstack([np.zeros([4, jointx_comb]),                             
                                np.hstack([np.zeros([4, jointx_si]), Jt_lg])])
                H2 = (1 - parameter.beta) * A1.T @ A1                                             # light guide
                H3 = parameter.eyeball_damping * eye_rotation_jacobian.T @ eye_rotation_jacobian  # orbital manipulation
                H = 2 * ((H1 + H2 + H3) + parameter.damping * parameter.B_13)

                c1 = parameter.beta * parameter.n * (eyeball_jacobian_t.T @ e_si_t.T)
                A2 = np.vstack([np.zeros([jointx_si, 1]),     # instrument
                                Jt_lg.T @ e_lg_t.T])
                c2 = (1 - parameter.beta) * parameter.n * A2  # light guide
                c = 2 * (c1 + c2)

            if parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(jointx_comb)

            delta_thetas = qp_solver.solve_quadratic_program(H, c, W, w, np.zeros([1, jointx_comb]), np.zeros([1, 1]))
            # print(9)

            theta_si = theta_si + delta_thetas[:jointx_si] * parameter.tau
            theta_lg = theta_lg + delta_thetas[jointx_si:jointx_comb] * parameter.tau
            theta_si.reshape([jointx_si, 1])
            theta_lg.reshape([jointx_lg, 1])
            # print(10)

            # Set joint position
            robot_si_interface.send_target_joint_positions(theta_si)
            robot_lg_interface.send_target_joint_positions(theta_lg)
            # input("[" + rospy.get_name() + "]:: Tested theta_lg target joint_positions. Press enter to continue...\n")

            # # Control the end effector
            # if functions.is_physical_robot():
            #     forceps.forceps_manipulation(haptic_pose_si, pub_forceps_si_closure)

            ##################################
            # Logging
            ##################################
            data_logger.log("error", np.array([np.linalg.norm(vec4(td_si - t_si))]))

            l_eye = normalize(r_o_e * k_ * conj(r_o_e))
            tilt_angle = np.rad2deg(math.acos(np.dot(vec4(l_eye), vec4(k_))))
            data_logger.log("tilt_angle", np.array([tilt_angle]))

            init_turn = vec4(rcm_init_si - eye.eyeball_t)
            init_turn_xy = normalize(init_turn[1] * i_ + init_turn[2] * j_)
            current_turn = vec4(rcm_current_si - eye.eyeball_t)
            current_turn_xy = normalize(current_turn[1] * i_ + current_turn[2] * j_)
            sign = vec4(current_turn_xy)[2] - vec4(init_turn_xy)[2]
            if sign == 0:
                sign = 0
            else:
                sign = sign / abs(sign)
            turning_angle = sign * np.rad2deg(math.acos(np.dot(vec4(init_turn_xy), vec4(current_turn_xy))))
            data_logger.log("turning_angle", np.array([turning_angle]))

            dot_eye_rotation = np.linalg.norm(eye_rotation_jacobian @ delta_thetas)
            data_logger.log("dot_eye_rotation", np.array([dot_eye_rotation]))

            store_data = np.hstack(
                [theta_si.T, theta_lg.T, delta_thetas, vec8(x_si), vec8(x_lg), vec8(td_si),
                 vec4(rcm_current_si), vec4(rcm_current_lg), vec4(r_o_e)])
            store.send_store_data("kinematics", store_data)

            if parameter.enable_sleep:
                r.sleep()

            data_logger.log("hz_tele", float(1/(time.time()-start)))

            if parameter.print_time:
                print("[" + rospy.get_name() + "]:: " + str(int(1/(time.time()-start)))+" Hz")

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    tele_operation()
