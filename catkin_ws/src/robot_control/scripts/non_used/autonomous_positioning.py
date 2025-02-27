#!/usr/bin/python3
# Import Relevant files from dqrobotics and sas
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from sas_robot_driver import RobotDriverInterface
from sas_datalogger import DataloggerInterface
from robot_loader import Robot

# Original
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
import kinematics.kinematics_functions as kine_func
from tools import functions
from eyeball import eyeball
from image_processing import predict_interface
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from interfaces import store_interface
from interfaces import contact_reporter_interface
from interfaces import positioning_point_interface
from eyesurgery_controllers import EyesurgeryControllers
from autonomous_positioning_controllers import AutonomousPositioningControllers
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics

# Some libraries for calculations
import numpy as np
import time
import math

# ROS
import rospy

def autonomous_positioning():
    """
    This function performs autonomous positioning of the surgical instrument's tip to the target points on the eyeball.
    Planar positioning, overlap prevention, and vertical positioning are implemented with the functions included in the controller file.
    """
    try:
        rospy.init_node('autonomous_positioning', disable_signals=True)

        # Create VrepInterface object
        vi = DQ_VrepInterface()
        vi.disconnect_all
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = physical_parameters.EyesurgerySetup()
        sim_setup = physical_parameters.SimulationSetup
        parameter = control_parameters.Parameters()
        eye_parameter = eyeball_parameter.EyeballParameters
        store = store_interface.StoreInterface()
        data_logger = DataloggerInterface(1000)
        controller = EyesurgeryControllers()
        positioning_controller = AutonomousPositioningControllers()

        contact_reporter = contact_reporter_interface.ContactReporterInterface()

        # Define robots: denso_robot_light : left hand, denso_robot_instrument with attachment: right hand
        robot_si = Robot(setup.robot_parameter_path_instrument).kinematics()
        robot_lg = Robot(setup.robot_parameter_path_light).kinematics()
        time.sleep(0.05)

        # Prepare an object to store some variables so that we can make figures later
        if not functions.is_physical_robot():
            rospy.set_param("px_mm_converter", parameter.converter_per_mm_vrep)

        # Prepare an interface to get distances from the predict node
        predict = predict_interface.PredictInterface()
        target_points = positioning_point_interface.PositioningPointInterface(parameter.converter_per_mm)

        # Start autonomous positioning
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Start autonomous positioning using physical robots. OK?\n")
        else:
            input("[" + rospy.get_name() + "]:: Start autonomous positioning in simulation.\n")
        
        # Set robot effector
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq)

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting for the RobotDriverInterfaces to be enabled.")
        robot_si_interface = RobotDriverInterface("/arm2")
        robot_lg_interface = RobotDriverInterface("/arm1")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        input("[" + rospy.get_name() + "]:: Enabled. Press enter to continue...")

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
        # Get rcm positions from ROS parameters
        rcm_si_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_si_rcm"))
        rcm_lg_dq = 1 + 0.5 * E_ * DQ(rospy.get_param("/robot_lg_rcm"))
        vi.set_object_pose(sim_setup.rcm_si_vrep_name, rcm_si_dq)
        vi.set_object_pose(sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
        print("[" + rospy.get_name() + "]:: Calculated rcm positions from the tip positions!")
        time.sleep(.5)

        # Calculate the eyeball position based on the rcm positions and create an Eyeball object
        eyeball_dq = kine_func.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius, parameter.port_angle)
        eye = eyeball.Eyeball(eyeball_dq, rcm_si_dq, rcm_lg_dq, eye_parameter)
        # Store eyeball parameters
        # eyeball_variables = np.vstack(
        #     [vec4(eye.rcm_si_t).reshape([4, 1]), vec4(eye.rcm_lg_t).reshape([4, 1]), vec4(eye.ws_t).reshape([4, 1]),
        #      vec4(eye.eyeball_t).reshape([4, 1])])
        # store.send_store_data("eyeball_variables", eyeball_variables)
        time.sleep(.5)

        vi.set_object_pose(sim_setup.eyeball_vrep_name, eyeball_dq)
        vi.set_object_translation(sim_setup.workspace_vrep_name, eye.ws_t)

        # Move robots to the initial positions
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to get the target positions...\n")
            controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, [- 0.004 * i_ - 0.0015 * j_ + 0.001 * k_],
                                                                  eye.rcm_lg_t, eye.eyeball_t,
                                                                  sim_setup.lg_vrep_name, vi)
            target_points.unlock_desired_point()
            input("Are the targets correct??")
            target_points.lock_desired_point()
        
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to move the tool-tips to the start positions...\n")
        else:
            print("[" + rospy.get_name() + "]:: Move the tool-tips to the start positions...\n")

        controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, parameter.td_init_set_si,
                                                              eye.rcm_si_t, eye.eyeball_t,
                                                              sim_setup.si_vrep_name, vi)

        controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, parameter.td_init_set_lg,
                                                              eye.rcm_lg_t, eye.eyeball_t,
                                                              sim_setup.lg_vrep_name, vi)

        if not EyeVFI.constraints_are_satisfied(robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye, parameter):
            print("--------------------------------------------------")
            input("[" + rospy.get_name() + "]:: Positioning was quit.")
            print("--------------------------------------------------")
            controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface,
                                                                  [t_lg_inserted - eye.eyeball_t],
                                                                  eye.rcm_lg_t, eye.eyeball_t,
                                                                  sim_setup.lg_vrep_name, vi)
            controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface,
                                                                  [t_si_inserted - eye.eyeball_t],
                                                                  eye.rcm_si_t, eye.eyeball_t,
                                                                  sim_setup.si_vrep_name, vi)
            exit()

        rospy.set_param("/roi_param_publish", True)

        theta_si = robot_si_interface.get_joint_positions()
        theta_lg = robot_lg_interface.get_joint_positions()
        theta_si_init = theta_si
        theta_lg_init = theta_lg

        # Get initial poses and translations after the instruments are inserted into an eyeball model
        x_si = robot_si.fkm(theta_si)
        x_lg = robot_lg.fkm(theta_lg)
        t_si = translation(x_si)
        t_lg = translation(x_lg)

        vi.set_object_pose(sim_setup.si_vrep_name, x_si)
        vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

        r_si = rotation(x_si)
        r_lg = rotation(x_lg)
        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)
        axis = k_
        l_si = normalize(r_si * axis * conj(r_si))
        l_lg = normalize(r_lg * axis * conj(r_lg))
        t_rcm_si, t_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                        eye.eyeball_t, eye.eyeball_radius)
        D_rcm_init = np.linalg.norm(vec4(t_rcm_si - t_rcm_lg)) ** 2

        rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
        rotation_c_n = k_
        rotation_c_plane_2 = rotation_c_n + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, rotation_c_n)
        rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

        [constrained_plane_list_si, constrained_plane_list_lg] = kine_func.get_constrained_plane_list(eye.eyeball_t,
                                                                                                      setup.celling_height,
                                                                                                      setup.floor_height)

        # target_points.get_workspace(t_si)
        # vi.set_object_translation("eyeground_point_1", DQ(target_points.target_point_positions[0, :]))
        # vi.set_object_translation("eyeground_point_2", DQ(target_points.target_point_positions[1, :]))
        # vi.set_object_translation("eyeground_point_3", DQ(target_points.target_point_positions[2, :]))
        # vi.set_object_translation("eyeground_point_4", DQ(target_points.target_point_positions[3, :]))
        # vi.set_object_translation("eyeground_point_5", DQ(target_points.target_point_positions[4, :]))

        # Prepare lists to store data for each positioning step
        positioning_point_list_xy = []
        positioning_radius = 3.5  # mm
        data_logger.log("sim_positioning_radius", positioning_radius)

        i = positioning_radius

        while i >= -1 * positioning_radius:
            j = 0
            while (i ** 2 + j ** 2) <= positioning_radius ** 2:
                if j == 0:
                    positioning_point_list_xy = positioning_point_list_xy + [i*10**(-3) * i_ + j*10**(-3) * j_]
                    data_logger.log("positioning_point_xy", vec4(i * i_ + j * j_))
                else:
                    positioning_point_list_xy = positioning_point_list_xy + [i*10**(-3) * i_ + j*10**(-3) * j_]
                    data_logger.log("positioning_point_xy", vec4(i * i_ + j * j_))
                    positioning_point_list_xy = positioning_point_list_xy + [i*10**(-3) * i_ - j*10**(-3) * j_]
                    data_logger.log("positioning_point_xy", vec4(i * i_ - j * j_))
                j = j + 1
            i = i - 1

        num_of_positioning = len(positioning_point_list_xy)
        data_logger.log("sim_positioning_num", float(num_of_positioning))
        overlap_prevention_i_list = np.array(np.zeros(num_of_positioning))
        planar_i_list = np.array(np.zeros(num_of_positioning))
        planar_error_list = np.array(np.zeros(num_of_positioning))

        # The initial translation of the surgical instrument.
        t_start_si = eye.eyeball_t + parameter.td_init_set_si[-1]

        # Start positioning the instrument's tip to each point in the positioning list
        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Push Enter to start autonomous positioning...\n")
        else:
            print("[" + rospy.get_name() + "]:: Starting autonomous positioning...\n")

        theta_rotation_max = 15
        data_logger.log("sim_theta_rotation_max", float(theta_rotation_max))
        theta_safe_eye_plane_max = 15
        data_logger.log("sim_theta_safe_eye_plane_max", float(theta_safe_eye_plane_max))
        increment = 1
        data_logger.log("increment", float(increment))

        rotation_angle_i = 1

        ##############################
        # Control Loop
        ##############################
        while rotation_angle_i < theta_rotation_max+1:
            plane_angle_i = 1
            while plane_angle_i < theta_safe_eye_plane_max+1:
                print(str((rotation_angle_i/increment + 1) * (plane_angle_i/increment + 1)) + "/" + str((theta_rotation_max/increment+1) * (theta_safe_eye_plane_max/increment+1)))
                print(plane_angle_i)
                print(rotation_angle_i)

                theta_safe_eye_plane = parameter.theta_safe_eye_plane + 15 - plane_angle_i

                normal1 = normalize(math.cos(np.deg2rad(theta_safe_eye_plane)) * i_ + math.sin(
                    np.deg2rad(theta_safe_eye_plane)) * j_)
                normal2 = normalize(-1 * math.cos(np.deg2rad(theta_safe_eye_plane)) * i_ + math.sin(
                    np.deg2rad(theta_safe_eye_plane)) * j_)
                rotation_c_plane_unified_1 = normal1 + E_ * dot(eye.eyeball_t, normal1)
                rotation_c_plane_unified_2 = normal2 + E_ * dot(eye.eyeball_t, normal2)
                rotation_c_plane_unified_list = [rotation_c_plane_unified_1, rotation_c_plane_unified_2]

                positioning = 0
                overlap_num = 0
                overlap_prevention_i_list = np.array(np.zeros(num_of_positioning))
                axis = k_
                while positioning < num_of_positioning:
                    rospy.set_param("next_point", [positioning])
                    target_points.publish_current_step(0)
                    print("---------------- Shadow-based Autonomous Positioning --------------------------")
                    print(str(positioning) + "/" + str(num_of_positioning))
                    time.sleep(1)
                    # target_points.lock_desired_point()
                    print("Target is set!")
                    time.sleep(.5)

                    # Current joint positions and instruments poses
                    theta_si = robot_si_interface.get_joint_positions()
                    theta_lg = robot_lg_interface.get_joint_positions()
                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)
                    t_si = translation(x_si)
                    t_start = t_si

                    target_pixel = vec4(positioning_point_list_xy[positioning])
                    print(target_pixel)
                    td_error = DQ(target_pixel)
                    print("Positioning target: " + str(td_error) + "\n")
                    # data_logger.log("target", vec4(td_error))

                    # Start planar positioning
                    target_points.publish_current_step(1)
                    print("[" + rospy.get_name() + "]:: Start planar positioning...")
                    td_si = t_start + td_error
                    total_iteration = kine_func.get_p2p_iteration(t_start, td_si, parameter.tau, parameter.si_velocity_planar)
                    vi.set_object_translation("tool_tip_d2", td_si)
                    time.sleep(0.1)

                    planar_error = (np.linalg.norm(vec4(td_error))*10**3)*parameter.converter_per_mm
                    # print(planar_error)

                    theta_rotation_angle = rotation_angle_i

                    planar_iteration, theta_si, theta_lg, norm_delta_theta = positioning_controller.planar_positioning_controller(robot_si, robot_lg, robot_si_interface, robot_lg_interface,
                                                                                            theta_si, theta_lg,
                                                                                            t_start, total_iteration, td_error, planar_error, target_pixel,
                                                                                            eye, rcm_init_si, rcm_init_lg, D_rcm_init,
                                                                                            constrained_plane_list_si, constrained_plane_list_lg,
                                                                                            rotation_c_plane_list, rotation_c_plane_unified_list,
                                                                                            predict, store, data_logger, target_points, vi, theta_rotation_angle)

                    data_logger.log("norm_delta_theta", float(norm_delta_theta))

                    robot_si_interface.send_target_joint_positions(theta_si)
                    robot_lg_interface.send_target_joint_positions(theta_lg)

                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)
                    t_si = translation(x_si)
                    t_lg = translation(x_lg)
                    r_si = rotation(x_si)
                    r_lg = rotation(x_lg)
                    l_si = normalize(r_si * axis * conj(r_si))
                    l_lg = normalize(r_lg * axis * conj(r_lg))

                    rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                                eye.eyeball_t,
                                                                                                eye.eyeball_radius)
                    r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                               rcm_current_si, rcm_current_lg)

                    vi.set_object_rotation(sim_setup.eyeball_vrep_name, r_o_e)
                    vi.set_object_translation(sim_setup.rcm_si_vrep_name, rcm_current_si)
                    vi.set_object_translation(sim_setup.rcm_lg_vrep_name, rcm_current_lg)

                    shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
                    vi.set_object_pose(sim_setup.si_vrep_name, x_si)
                    vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)
                    vi.set_object_pose(sim_setup.shadow_vrep_name, shadow_tip_dq)

                    # Show how accurate the planar positioning was performed
                    # theta_si = robot_si_interface.get_joint_positions()
                    # theta_lg = robot_lg_interface.get_joint_positions()
                    x_si = robot_si.fkm(theta_si)
                    x_lg = robot_lg.fkm(theta_lg)
                    # kine_func.show_gap(x_si, td_si)
                    planar_i_list[positioning] = planar_iteration
                    # planar_error_list[positioning] = planar_error
                    # print(planar_i_list)
                    # print(planar_error_list)
                    # time.sleep(1)

                    t_si = translation(x_si)
                    t_lg = translation(x_lg)
                    r_si = rotation(x_si)
                    r_lg = rotation(x_lg)
                    axis = k_
                    l_si = normalize(r_si * axis * conj(r_si))
                    l_lg = normalize(r_lg * axis * conj(r_lg))

                    # shaft_distance = predict.shaft_distance
                    rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                                eye.eyeball_t,
                                                                                                eye.eyeball_radius)
                    r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                               rcm_current_si, rcm_current_lg)
                    shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg, r_o_e)
                    shaft_distance = (eye.get_shaft_shadow_tip_distance(shadow_tip_dq, x_si)*10**3)*parameter.converter_per_mm
                    threshold_overlap_prevention = parameter.threshold_overlap_prevention_pixel
                    threshold_vertical_positioning = parameter.threshold_vertical_positioning_pixel
                    converter_per_mm = parameter.converter_per_mm
                    store.send_store_data("thresholds",
                                          np.vstack([threshold_overlap_prevention, threshold_vertical_positioning]))
                    time.sleep(1)
                    print("[" + rospy.get_name() + "]:: The thresholds were set for the robot. OK?\n")

                    # Start overlap prevention
                    t_current_si = t_si
                    print("[" + rospy.get_name() + "]:: Start overlap prevention...")
                    print(shaft_distance)
                    fail_position_list, i = positioning_controller.overlap_prevention_controller(robot_si, robot_lg, robot_si_interface, robot_lg_interface,
                                                                                                 theta_si, theta_lg,
                                                                                                 rcm_init_si, rcm_init_lg,
                                                                                                 threshold_overlap_prevention, t_current_si, target_pixel, eye, shaft_distance,
                                                                                                 constrained_plane_list_si, constrained_plane_list_lg,
                                                                                                 predict, store, data_logger, target_points, vi)

                    # Show the number of the iterations this step needed
                    overlap_prevention_i_list[positioning] = i
                    data_logger.log("overlap_iteration", float(i))
                    if i > 0:
                        print("The number of iterations for overlap prevention step was " + str(
                            format(overlap_prevention_i_list[positioning], '.0f')) + ".")
                        print(overlap_prevention_i_list)
                        overlap_num = overlap_num + 1
                        print(overlap_num)

                    positioning = positioning + 1

                    robot_si_interface.send_target_joint_positions(theta_si_init)
                    robot_lg_interface.send_target_joint_positions(theta_lg_init)
                    time.sleep(1)

                plane_angle_i = plane_angle_i + increment
                data_logger.log("sim_overlap_num", float(overlap_num))

            rotation_angle_i = rotation_angle_i + increment

        vi.disconnect_all()

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
        print("please check the error message above.")

    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    autonomous_positioning()
