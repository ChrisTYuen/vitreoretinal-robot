#!/usr/bin/python3
import rospy

# Original
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
import kinematics.kinematics_functions as kine_func
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from eyesurgery_controllers import EyesurgeryControllers
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics

# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from dqrobotics.interfaces.json11 import DQ_JsonReader

from sas_robot_driver import RobotDriverInterface

# Some libraries for calculations
import numpy as np

# For calculating the sampling time
import time
import sys
sys.path.append('/home/yuki/git/ykoyama2017/catkin_ws_unified/devel_release/lib/python3/dist-packages')
sys.path.insert(1, '/home/nml/git/ykoyama2017/catkin_ws_unified/src/sas/sas_datalogger/src/sas_datalogger/__init__.py')


def debug_orbital_manipulation():
    try:
        rospy.init_node("vitreoretinal_kinematics", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start kinematics...")

        vi = DQ_VrepInterface()
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = physical_parameters.EyesurgerySetup()
        sim_setup = physical_parameters.SimulationSetup
        parameter = control_parameters.Parameters()
        eye_parameter = eyeball_parameter.EyeballParameters
        controller = EyesurgeryControllers()

        # Define robots: denso_robot_light : left hand, denso_robot_instrument: right hand
        reader = DQ_JsonReader()
        robot_si = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_instrument)
        robot_lg = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_light)

        print("[" + rospy.get_name() + "]:: Debug of orbital manipulation.\n")

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
        robot_si_interface = RobotDriverInterface("/arm2/joints/")
        robot_lg_interface = RobotDriverInterface("/arm1/joints/")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        input("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

        # Set robot base
        robot_si.set_reference_frame(robot_si_interface.get_reference_frame())
        time.sleep(0.02)
        robot_lg.set_reference_frame(robot_si_interface.get_reference_frame()*setup.robot_lg_base_rel)
        time.sleep(0.02)

        # Set robot effector
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq)

        theta_si, theta_lg = controller.set_manipulators_initial_thetas(robot_si, robot_lg, vi,
                                                                        robot_si_interface, robot_lg_interface)
        time.sleep(.5)

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
        rcm_si_dq = x_si_inserted * (1 - 0.5 * E_ * setup.insertion_distance * k_)
        rcm_lg_dq = x_lg_inserted * (1 - 0.5 * E_ * setup.insertion_distance * k_)
        vi.set_object_pose(sim_setup.rcm_si_vrep_name, rcm_si_dq)
        vi.set_object_pose(sim_setup.rcm_lg_vrep_name, rcm_lg_dq)
        print("[" + rospy.get_name() + "]:: Calculated rcm positions from the tip positions!")
        time.sleep(.5)

        eyeball_dq = kine_func.get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, parameter.eyeball_radius,
                                                             parameter.port_angle)
        eye = eyeball.Eyeball(eyeball_dq, rcm_si_dq, rcm_lg_dq, eye_parameter)
        time.sleep(.5)
        vi.set_object_pose(sim_setup.eyeball_vrep_name, eyeball_dq)
        vi.set_object_translation(sim_setup.workspace_vrep_name, eye.ws_t)

        input("[" + rospy.get_name() + "]:: Push Enter to move the tool-tips to the start positions...\n")

        controller.translation_controller_with_rcm_constraint(robot_si, robot_si_interface, parameter.td_init_set_si,
                                                              eye.rcm_si_t, eye.eyeball_t,
                                                              sim_setup.si_vrep_name, vi)

        controller.translation_controller_with_rcm_constraint(robot_lg, robot_lg_interface, parameter.td_init_set_lg,
                                                              eye.rcm_lg_t, eye.eyeball_t,
                                                              sim_setup.lg_vrep_name, vi)

        if not EyeVFI.constraints_are_satisfied(robot_si, robot_lg, robot_si_interface, robot_lg_interface, eye,
                                                parameter):
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

        theta_si = robot_si_interface.get_joint_positions()
        theta_si = np.array([theta_si]).reshape([6, 1])
        theta_lg = robot_lg_interface.get_joint_positions()
        theta_lg = np.array([theta_lg]).reshape([6, 1])

        x_si = robot_si.fkm(theta_si)
        t_si = translation(x_si)
        t_si_init = t_si

        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)

        td_eye = eye.eyeball_radius*normalize(-1*j_)

        ##############################
        # Control Loop
        ##############################
        i = 0
        total_iteration = (np.linalg.norm(vec4(td_eye+eye.eyeball_t-t_si_init))/0.001)/parameter.tau
        time.sleep(1)
        r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius,
                                                   rcm_init_si, rcm_init_lg, rcm_init_si, rcm_init_lg)
        td = eye.eyeball_t + r_o_e * td_eye * conj(r_o_e)
        vi.set_object_translation("xd2", td)

        input("start loop")

        r = rospy.Rate(parameter.fps)
        while i < total_iteration:
            i = i + 1

            # Get the pose of the current tooltip pose
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)
            vi.set_object_pose(sim_setup.si_vrep_name, x_si)
            vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

            current_rcm_si, current_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, r_si, r_lg,
                                                                                        eye.eyeball_t, eye.eyeball_radius)

            vi.set_object_translation("rcm1_current", current_rcm_si)
            vi.set_object_translation("rcm2_current", current_rcm_lg)

            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       current_rcm_si, current_rcm_lg)

            vi.set_object_rotation(sim_setup.eyeball_vrep_name, r_o_e)

            td_si = eye.eyeball_t + r_o_e*(t_si_init-eye.eyeball_t)*conj(r_o_e) + DQ(vec4(r_o_e*(td_eye+eye.eyeball_t-t_si_init)*conj(r_o_e)) / total_iteration * i)
            vi.set_object_translation("xd1", td_si)
            td = eye.eyeball_t + r_o_e * td_eye * conj(r_o_e)
            vi.set_object_translation("xd2", td)

            vi.set_object_translation("x3", eye.eyeball_t + r_o_e*(rcm_init_si-eye.eyeball_t)*conj(r_o_e))
            vi.set_object_translation("x4", eye.eyeball_t + r_o_e*(rcm_init_lg-eye.eyeball_t)*conj(r_o_e))

            # Get Jacobians related to the current tooltip poses
            J_si = robot_si.pose_jacobian(theta_si)
            Jt_si = robot_si.translation_jacobian(J_si, x_si)
            Jr_si = DQ_Kinematics.rotation_jacobian(J_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)
            Jr_lg = DQ_Kinematics.rotation_jacobian(J_lg)

            d_si, d_lg, J_d_si, J_d_lg = om_kinematics.get_current_inserted_length_and_jacobian(Jt_si, Jt_lg, Jr_si, Jr_lg,
                                                                                                t_si, t_lg, r_si, r_lg,
                                                                                                eye.eyeball_t, eye.eyeball_radius)
            Jt_trocar_si, Jt_trocar_lg = om_kinematics.get_current_rcm_translation_jacobians(Jt_si, Jt_lg, Jr_si, Jr_lg,
                                                                                             J_d_si, J_d_lg, r_si, r_lg,
                                                                                             d_si, d_lg)

            # Get jacobians and distances for vitreoretinal environment constraints
            J_D_rcm = 2 * vec4(current_rcm_si - current_rcm_lg) @ np.hstack([Jt_trocar_si, -1 * Jt_trocar_lg])

            # Define errors
            e_si = np.array([vec4(t_si - td_si)])

            e_error = np.vstack([e_si.T, 0])

            # Quadratic programming (with the proposed constraints)
            eyeball_jacobian = om_kinematics.get_eyeball_jacobian(Jt_si, Jt_lg, Jr_si, Jr_lg, t_si, t_lg, r_si, r_lg,
                                                                  eye.eyeball_t, eye.eyeball_radius,
                                                                  rcm_init_si, rcm_init_lg, td_eye)

            task_jacobian = np.vstack([np.array([eyeball_jacobian]).reshape(4, 12),
                                       np.array([J_D_rcm]).reshape(1, 12)])

            delta_thetas = -1*parameter.n*np.linalg.pinv(task_jacobian)@e_error

            theta_si = theta_si + delta_thetas[:6] * parameter.tau
            theta_lg = theta_lg + delta_thetas[6:12] * parameter.tau
            theta_si.reshape([6, 1])
            theta_lg.reshape([6, 1])

            robot_si_interface.send_target_joint_positions(theta_si)
            robot_lg_interface.send_target_joint_positions(theta_lg)

            if parameter.enable_sleep:
                r.sleep()

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    debug_orbital_manipulation()
