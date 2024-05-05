#!/usr/bin/python3
# Import Relevant files from dqrobotics and sas
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.solvers import DQ_CPLEXSolver
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.interfaces.json11 import DQ_JsonReader
from sas_robot_driver import RobotDriverInterface
from sas_robot_kinematics import RobotKinematicsProvider
from sas_datalogger import DataloggerInterface

# Original
from kinematics.eyesurgery_VFIs import EyesurgeryVFIs as EyeVFI
import kinematics.kinematics_functions as kine_func
from tools import functions
from eyeball import eyeball
from eyeball.eyeball_parameter import eyeball_parameter
from kinematics.parameters import physical_parameters, control_parameters
from interfaces import store_interface
from eyesurgery_controllers import EyesurgeryControllers
from kinematics.orbital_manipulation_kinematics import OrbitalManipulationKinematics as om_kinematics

# Some libraries for calculations
import numpy as np
import time
import sys
import rospy

sys.path.append('/home/yuki/git/ykoyama2017/catkin_ws_unified/devel_release/lib/python3/dist-packages')
sys.path.insert(1, '/home/nml/git/ykoyama2017/catkin_ws_unified/src/sas/sas_datalogger/src/sas_datalogger/__init__.py')


def tele_operation():
    try:
        rospy.init_node("tele_controller", disable_signals=True)
        print("[" + rospy.get_name() + "]:: Start kinematics...")

        vi = DQ_VrepInterface()
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        setup = physical_parameters.EyesurgerySetup()
        sim_setup = physical_parameters.SimulationSetup
        parameter = control_parameters.Parameters()
        eye_parameter = eyeball_parameter.EyeballParameters
        store = store_interface.StoreInterface()
        data_logger = DataloggerInterface(10)
        controller = EyesurgeryControllers()

        # Define robots: denso_robot_light : left hand, denso_robot_instrument: right hand
        reader = DQ_JsonReader()
        robot_si = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_instrument)
        robot_lg = reader.get_serial_manipulator_denso_from_json(setup.robot_parameter_path_light)

        if functions.is_physical_robot():
            input("[" + rospy.get_name() + "]:: Start tele_operation with physical robots. OK?\n")
        else:
            print("[" + rospy.get_name() + "]:: Start tele_operation in simulation.\n")

        # Define Solver
        if parameter.solver == 0:
            qp_solver = DQ_QuadprogSolver()
        else:
            qp_solver = DQ_CPLEXSolver()

        # Robot driver interface
        print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
        robot_si_interface = RobotDriverInterface("/arm2/joints/")
        robot_lg_interface = RobotDriverInterface("/arm1/joints/")
        while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
            time.sleep(0.01)
        input("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

        robot_si_provider = RobotKinematicsProvider("/arm1_kinematics/")
        robot_lg_provider = RobotKinematicsProvider("/arm2_kinematics/")

        # Set robot base
        robot_si.set_reference_frame(robot_si_interface.get_reference_frame())
        time.sleep(0.02)
        robot_lg.set_reference_frame(robot_si_interface.get_reference_frame()*setup.robot_lg_base_rel)
        time.sleep(0.02)

        # Set robot effector
        robot_si.set_effector(setup.robot_si_effector_dq)
        robot_lg.set_effector(setup.robot_lg_effector_dq)

        if functions.is_physical_robot():
            theta_si = np.array(robot_si_interface.get_joint_positions())
            theta_lg = np.array(robot_lg_interface.get_joint_positions())
        else:
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

        print("[" + rospy.get_name() + "]:: Waiting to RobotKinematicsProvider is enabled...")
        while not robot_lg_provider.is_enabled() or not robot_si_provider.is_enabled():
            robot_si_provider.send_pose(x_si)
            robot_lg_provider.send_pose(x_lg)
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
        rcm_init_si = translation(rcm_si_dq)
        rcm_init_lg = translation(rcm_lg_dq)
        t_rcm_si, t_rcm_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, r_si, r_lg,
                                                                        eye.eyeball_t, eye.eyeball_radius)
        D_rcm_init = np.linalg.norm(vec4(t_rcm_si - t_rcm_lg)) ** 2

        rotation_c_plane_1 = -1 * j_ + E_ * dot(eye.eyeball_t, -1 * j_)
        rotation_c_n = k_
        rotation_c_plane_2 = rotation_c_n + E_ * dot(eye.eyeball_t + eye.eyeball_radius / 2 * k_, rotation_c_n)
        rotation_c_plane_list = [rotation_c_plane_1, rotation_c_plane_2]

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

            if parameter.lg_automation:
                xd_lg = vi.get_object_pose(sim_setup.lg_vrep_name)
            else:
                xd_lg = robot_lg_provider.get_desired_pose()

            vi.set_object_pose(sim_setup.si_xd_vrep_name, xd_si)
            vi.set_object_pose(sim_setup.lg_xd_vrep_name, xd_lg)

            # Get the pose of the current tooltip pose
            x_si = robot_si.fkm(theta_si)
            x_lg = robot_lg.fkm(theta_lg)
            t_si = translation(x_si)
            t_lg = translation(x_lg)
            r_si = rotation(x_si)
            r_lg = rotation(x_lg)

            axis = k_
            l_si = normalize(r_si * axis * conj(r_si))
            l_lg = normalize(r_lg * axis * conj(r_lg))

            robot_si_provider.send_pose(x_si)
            robot_lg_provider.send_pose(x_lg)

            shadow_tip_dq = eye.get_shadow_tip_position(x_si, x_lg)
            vi.set_object_pose(sim_setup.shadow_vrep_name, shadow_tip_dq)

            rcm_current_si, rcm_current_lg = om_kinematics.get_current_rcm_translations(t_si, t_lg, l_si, l_lg,
                                                                                        eye.eyeball_t, eye.eyeball_radius)
            r_o_e = om_kinematics.get_eyeball_rotation(eye.eyeball_t, eye.eyeball_radius, rcm_init_si, rcm_init_lg,
                                                       rcm_current_si, rcm_current_lg)

            if parameter.orbital_manipulation:
                vi.set_object_rotation(sim_setup.eyeball_vrep_name, r_o_e)
                vi.set_object_translation(sim_setup.rcm_si_vrep_name, rcm_current_si)
                vi.set_object_translation(sim_setup.rcm_lg_vrep_name, rcm_current_lg)

            # Get Jacobians related to the current tooltip poses
            J_si = robot_si.pose_jacobian(theta_si)
            Jt_si = robot_si.translation_jacobian(J_si, x_si)
            J_lg = robot_lg.pose_jacobian(theta_lg)
            Jt_lg = robot_lg.translation_jacobian(J_lg, x_lg)

            # Define errors
            e_si = np.array([vec4(t_si - translation(xd_si))])
            e_lg = np.array([vec4(t_lg - translation(xd_lg))])
            if parameter.print_error:
                print(np.linalg.norm(e_si))

            # Quadratic programming (without the proposed constraints)
            W_vitreo, w_vitreo = EyeVFI.get_vitreoretinal_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.rcm_si_t,
                                                               eye.rcm_lg_t, eye.eyeball_t, parameter,
                                                               constrained_plane_list_si, constrained_plane_list_lg)
            W_conical, w_conical = EyeVFI.get_conical_VFIs(robot_si, robot_lg, theta_si, theta_lg, eye.ws_t, parameter)
            W_om, w_om = om_kinematics.get_orbital_manipulation_VFIs_icra2023(robot_si, robot_lg, theta_si, theta_lg,
                                                                              eye.eyeball_t, eye.eyeball_radius,
                                                                              parameter, D_rcm_init, rotation_c_plane_list)
            W = np.vstack([W_vitreo,
                           W_conical,
                           W_om])
            w = np.vstack([w_vitreo,
                           w_conical,
                           w_om])

            # Quadratic programming (with the proposed constraints)
            A = np.vstack([np.hstack([Jt_si, np.zeros([4, 6])]),
                           np.hstack([np.zeros([4, 6]), Jt_lg])])
            H = A.T @ parameter.B_8 @ A
            e = np.vstack([e_si.T,
                           e_lg.T])
            c = 2 * parameter.n * (A.T @ parameter.B_8 @ e)

            if parameter.solver == 0:
                w = w.reshape(w.shape[0])
                c = c.reshape(12)

            delta_thetas = qp_solver.solve_quadratic_program(2 * (H + parameter.damping * parameter.B_12),
                                                             c, W, w, np.zeros([1, 12]), np.zeros([1, 1]))

            theta_si = theta_si + delta_thetas[:6] * parameter.tau
            theta_lg = theta_lg + delta_thetas[6:12] * parameter.tau
            theta_si.reshape([6, 1])
            theta_lg.reshape([6, 1])

            # Set joint position
            robot_si_interface.send_target_joint_positions(theta_si)
            robot_lg_interface.send_target_joint_positions(theta_lg)

            # Logging
            store_data = np.hstack(
                [theta_si.T, theta_lg.T, delta_thetas, vec8(x_si), vec8(x_lg), vec8(xd_si), vec8(xd_lg),
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
