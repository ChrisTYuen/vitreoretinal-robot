#!/usr/bin/python3
# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface

# import VS050RobotDH
from sas_robot_driver import RobotDriverInterface

# Import original files
from kinematics.parameters import physical_parameters
from eyesurgery_controllers import EyesurgeryControllers
from tools import functions
from tools.robot_loader import Robot

# For calculating the sampling time
import time
import traceback

# ROS
import rospy

try:
    """
    This script sets the RCM positions for the instruments. If the script is run with the physical robots, the robots
    should be manually moved to the initial positions before running the script. Otherwise, the script will be run through
    simulation and the robots will be moved to the initial positions automatically.
    """
    rospy.init_node("set_rcm_positions", disable_signals=True)

    # Create VrepInterface object
    vi = DQ_VrepInterface()
    vi.connect("127.0.0.1", 19996, 100, 10)
    vi.start_simulation()

    # Get experimental configuration and control parameters
    setup = physical_parameters.EyesurgerySetup()
    sim_setup = physical_parameters.SimulationSetup()
    controller = EyesurgeryControllers()

    # Define robots: denso_robot_light : left hand, denso_robot_instrument with attachment: right hand
    robot_si = Robot(setup.robot_parameter_path_instrument).kinematics()
    robot_lg = Robot(setup.robot_parameter_path_light).kinematics()
    time.sleep(0.50)

    if functions.is_physical_robot():
        input("[" + rospy.get_name() + "]:: Start rcm_set with physical robots. OK? Press enter to continue...\n")
    else:
        input("[" + rospy.get_name() + "]:: Start rcm_set in simulation. Press enter to continue...\n")
    
    # Set the end_effector positions
    robot_si.set_effector(setup.robot_si_effector_dq)
    robot_lg.set_effector(setup.robot_lg_effector_dq)

    # Create robot arm object
    print("[" + rospy.get_name() + "]:: Waiting to connect to v-rep robot...")
    robot_si_interface = RobotDriverInterface("/arm2")
    robot_lg_interface = RobotDriverInterface("/arm1")
    while not robot_lg_interface.is_enabled() or not robot_si_interface.is_enabled():
        time.sleep(0.01)
    print("[" + rospy.get_name() + "]:: Connected to v-rep robot.")

    # Set the reference frame
    robot_si.set_reference_frame(vi.get_object_pose("VS050_reference#2"))
    time.sleep(0.02)
    robot_lg.set_reference_frame(vi.get_object_pose("VS050_reference#2")*setup.robot_lg_base_rel)
    time.sleep(0.02)

    # Set the manipulators to the initial pose
    theta_si, theta_lg = controller.set_manipulators_initial_thetas(robot_si, robot_lg, vi,
                                                                    robot_si_interface, robot_lg_interface)
    
    time.sleep(.5)

    x_si = robot_si.fkm(theta_si)
    x_lg = robot_lg.fkm(theta_lg)
    t_si_inserted = translation(x_si)
    t_lg_inserted = translation(x_lg)
    r_si_inserted = rotation(x_si)
    r_lg_inserted = rotation(x_lg)

    rospy.set_param("t_si_inserted", vec4(t_si_inserted).tolist())
    rospy.set_param("t_lg_inserted", vec4(t_lg_inserted).tolist())
    rospy.set_param("r_si_inserted", vec4(r_si_inserted).tolist())
    rospy.set_param("r_lg_inserted", vec4(r_lg_inserted).tolist())

    vi.set_object_pose(sim_setup.si_vrep_name, x_si)
    vi.set_object_pose(sim_setup.lg_vrep_name, x_lg)

    # Calculate rcm positions from the tool-tip positions
    rcm_si_t = translation(x_si * (1 - 0.5 * E_ * setup.insertion_distance * k_))
    rcm_lg_t = translation(x_lg * (1 - 0.5 * E_ * setup.insertion_distance * k_))

    rospy.set_param("robot_si_rcm", vec4(rcm_si_t).tolist())
    rospy.set_param("robot_lg_rcm", vec4(rcm_lg_t).tolist())

    print("RCM set is done!!")

except Exception as exp:
    print("[" + rospy.get_name() + "]:: {}.".format(exp))
    traceback.print_exc()

except KeyboardInterrupt:
    print("Keyboard Interrupt!!")

