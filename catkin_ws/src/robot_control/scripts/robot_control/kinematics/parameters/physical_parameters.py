# Import dqrobotics for calculation of dual quaternion algebra
from dqrobotics import *

# Import other dependencies
import math
import numpy as np


class EyesurgerySetup:
    """
        This class contains parameters related to the physical robots and experimental setup.
    """
    # Relative pose of the two robots
    robot_lg_base_rel = DQ([0.4979824368, 0.0015391332, -0.0006935076, 0.8671846466,-1.85128689620542*10**(-5), -0.0001568641, 0.3505309199, 0.0002911987])
    robot_lg_base_rel_rot = P(robot_lg_base_rel)
    robot_lg_base_rel_trans = 2*D(robot_lg_base_rel)*conj(robot_lg_base_rel_rot)
    robot_lg_base_rel_trans = robot_lg_base_rel_trans - 0.0005 * j_ - 0. * i_ - 0.0014 * k_
    robot_lg_base_rel = robot_lg_base_rel_rot + 0.5 * E_ * robot_lg_base_rel_trans * robot_lg_base_rel_rot
    robot_lg_base_rel = normalize(robot_lg_base_rel)

    # Poses of the end effectors
    effector_t_si = DQ([1, 0, 0, 0])  # DQ([0, 0.13785, -0.00478, 0.03468]) # DQ([0, 0.15764571991111298, -0.005962599174867305, 0.037203466919140026])
    effector_r_si = DQ([1, 0, 0, 0])  # DQ([math.cos(math.pi/4), 0, math.sin(math.pi/4), 0])*DQ([math.cos(math.pi/2), 0, 0, math.sin(math.pi/2)])
    robot_si_effector_dq = DQ([1]) # effector_r_si + 0.5 * E_ * effector_t_si * effector_r_si

    effector_t_lg = DQ([1, 0, 0, 0])  # DQ([0, 0.12538286262512585, -0.0019151152274889775, 0.03939227466717819])
    effector_r_lg = DQ([1, 0, 0, 0])  # DQ([math.cos(math.pi/4), 0, math.sin(math.pi/4), 0])*DQ([math.cos(math.pi/2), 0, 0, math.sin(math.pi/2)])
    robot_lg_effector_dq = DQ([1])  # effector_r_lg + 0.5 * E_ * effector_t_lg * effector_r_lg

    # Insertion distance of the instruments
    insertion_distance = 0.005

    # Experimental Environment
    celling_height = 0.48
    floor_height = 0.15

    # Calibration DH parameters
    ###### Calibrated Denso parameters #####
    robot_parameter_path_instrument = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/robots/denso_vs050_denso_11U483_instrument_calibrated.json"
    robot_parameter_path_light = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/robots/denso_vs050_denso_11U473_light_calibrated.json"
    ###### Factory Denso parameters #####
    # robot_parameter_path_instrument = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/robots/denso_vs050_denso_11U483_instrument.json"
    # robot_parameter_path_light = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/robots/denso_vs050_denso_11U473_light.json"
    ###### Generic Standard DH parameters #####
    # robot_parameter_path_instrument = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/robots/densoDH_robot2_instrument.json"
    # robot_parameter_path_light = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/robots/densoDH_robot1_light.json"

    sas_operator_side_reciever_path = "/home/yuki/git/ctyuen2022/catkin_ws/src/robot_control/cfg/sas_operator_side_receiver.yaml"


class SimulationSetup:
    """
    This class contains the parameters used only for simulation.
    """
    # Initial joint positions for simulation
    theta_init_fa = [0 / 180]
    theta_init_si = np.array(np.append([100 / 180, 10 / 180, 100 / 180, -10 / 180, -60 / 180, 10 / 180] , theta_init_fa)) * math.pi
    theta_init_lg = np.array([-100 / 180, 10 / 180, 100 / 180, 10 / 180, -60 / 180, -10 / 180]) * math.pi

    # Eyeball position for simulation
    eyeball_position = 1 + 0.5 * E_ * (0.0249*i_ + 0.3127*j_ + 1.0502*k_)

    # The angle used to calculate the rcm positions in simulation
    port_radius = 0.008

    # objects name
    si_vrep_name = 'instrument_tip'
    lg_vrep_name = 'light_tip'
    si_xd_vrep_name = 'x1'
    lg_xd_vrep_name = 'x2'
    td_1_vrep_name = 'tool_tip_d1'
    td_2_vrep_name = 'tool_tip_d2'
    shadow_vrep_name = 'shadow_tip'
    eyeball_vrep_name = 'Eyeball'
    workspace_vrep_name = 'workspace'
    eye_rotation_vrep_name = 'eyeball_rotation'
    rcm_si_vrep_name = 'x3'
    rcm_lg_vrep_name = 'x4'
    initialize_velocity = 10
    initialize_velocity_sim = 100


