from dqrobotics import *
import math
import numpy as np
from tools import functions

class Parameters:
    """
    This class contains parameters for control.
    """
    ####### simulation or not #######
    is_open_space = False

    ####### Forceps control #######
    end_effector_rotation = True
    forceps_control = True
    forceps_two_buttons = False  # else one button

    ####### Light guide and Orbital Manipulation automation #######
    lg_automation = True
    orbital_manipulation = False
    orbital_manipulation_all_direction = False
    om_version_icra_ver = False

    ####### Eyeball model #########
    real_ILM_model = True  # Production ILM model, if false, detechable R&D model

    ####### For debug #######
    print_time = False
    print_error = True
    print_datalog_time = False
    enable_sleep = True
    
    ####### Constraints (True=1, False = 0) #######
    if lg_automation:
        illumination_cone = 1
        shadow_cone = 0
        tip = 1
    else:
        illumination_cone = 0
        shadow_cone = 0
        tip = 0

    if orbital_manipulation:
        om_trocar_si = 1
        om_trocar_lg = 1
        om_D_rcm = 1
        om_rot_plane = 1
        om_rot = 1
        rcm_si = 0
        rcm_lg = 0
        trocar_si = 0
        trocar_lg = 0
        collision = 1
        eyeball = 1
        forceps = 0
    else:
        om_trocar_si = 0
        om_trocar_lg = 0
        om_D_rcm = 0
        om_rot_plane = 0
        om_rot = 0
        rcm_si = 1
        rcm_lg = 1
        trocar_si = 1
        trocar_lg = 1
        collision = 1
        eyeball = 1
        forceps = 0

    view = 0
    plane = 0

    if orbital_manipulation:
        theta_safe_eye_plane = 51  # 51
        eyeball_damping = 0.001  # 0.001  # Higher value = less rotation
        if orbital_manipulation_all_direction:
            theta_safe_eye_rotation = 11  # 11
        else:
            theta_safe_eye_rotation = 0

    else:
        theta_safe_eye_rotation = 15
        theta_safe_eye_plane = 45
        eyeball_damping = 1

    ####### Solver (DQ_QuadprogSolver=0, DQ_CPLEXSolver=1) #######
    solver = 0
    
    ####### Positioning points for autonomous positioning #######
    # P1: [0, 0., 0.003, 0.], P2: [0, 0.003, 0., 0.], P3: [0, 0., 0., 0.], P4: [0, -0.003, 0., 0.],
    # P5: [0, 0., -0.003, 0.], P6: [0, -0.001, -0.0028, 0]
    # positioning_list = [[0, -0.001, -0.0028, 0.]]
    positioning_radius = 2.0  # mm
    pr = positioning_radius * 10 ** (-3)
    positioning_list = [[0, 0., pr, 0.], [0, -pr, 0., 0.], [0, 0., 0., 0.], [0, pr, 0., 0.0], [0, 0., -pr, 0.0]]

    ####### Points used to move the surgical instrument to the initial point #######
    if real_ILM_model:
        # td_init_set_si = [0.001 * k_, - 0.003 * k_]
        td_init_set_si = [0.001 * k_, - 0.003 * k_]
    else:
        # td_init_set_si = [0. * k_, - 0.004 * k_]
        td_init_set_si = [0. * k_, 0.0015 * j_ - 0.005 * k_]


    ####### Points used to move the light guide to the initial point #######
    if real_ILM_model:
        # td_init_set_lg = [0.001 * k_, - 0.005 * i_ - 0.005 * j_ + 0.0025 * k_]
        td_init_set_lg = [0.001 * k_, - 0.003 * i_ - 0.002 * j_ + 0.0025 * k_]
    else:
        td_init_set_lg = [0. * k_, - 0.0032 * i_ - 0.002 * j_ + 0.005 * k_]
        # td_init_set_lg = [0. * k_, - 0.003 * i_ - 0.002 * j_ + 0.0005 * k_]


    ####### Points used for trajectory control with om #######
    om_trajectory_point_list = [normalize(j_ - 2* k_), normalize(i_ -2* k_), normalize(-j_ -2* k_), normalize(-i_ -2* k_)]

    ####### Joint Limits #######
    theta_limit_plu = np.array([[110 / 180, 90 / 180, 135 / 180, 45 / 180, 5 / 180, 90 / 180, 180 / 180]]) * math.pi
    theta_limit_minu = np.array([[-110 / 180, 10 / 180, 45 / 180, -45 / 180, -90 / 180, -90 / 180, -180 / 180]]) * math.pi

    ####### Forceps Control #######
    open_voltage = 3.2
    close_voltage = 1.8
    max_closing = 1.05
    max_opening = 0.675

    ####### Eyeball and trocar parameters #######
    eyeball_radius = 0.012
    port_angle = 30 / 180 * math.pi
    eyeground_radius = 0.005
    if real_ILM_model:
        eyeground_plus_height = 0.003
    else:
        eyeground_plus_height = 0.002 

    radius_bias = 0.  # 0.0015
    eyeground_radius_init = eyeground_radius
    ws_radius = eyeground_radius_init - radius_bias

    trocar_outer_radius = 0.00025
    trocar_inner_radius = 0.0002

    ####### Axis #######
    axis = k_

    ####### Parameters for initialization #######
    setup_velocity = 5
    n_initialize = 140
    damping_initialize = 0.001
    control_priority = 0.999

   ####### Velocities (mm/s) #######
    si_velocity_setup = 5
    si_velocity_calibration_check = 10
    if functions.is_physical_robot():
        si_velocity_planar = 0.2  # 0.2         # Planar positioning velocity
        si_velocity_planar2 = 0.1  # 0.1        # Vertical positioning velocity
        si_velocity_vertical = 0.1  # 0.1       # Vertical positioning total iteration calculation
        td_init_target_lg = [ - 0.003 * i_ - 0.003 * j_ + 0.005 * k_]  # Lower i = right, Lower j = down
        si_velocity_vertical_additional = 0.10 #0.15   # Additional vertical positioning total iteration calculation
    else:  # Simulation
        si_velocity_planar = 2  # 0.2         
        si_velocity_planar2 = 1  # 0.1       
        si_velocity_vertical = 1  # 0.1       
        td_init_target_lg = [- 0.003 * i_ - 0.003 * j_ + 0.001 * k_]  
        si_velocity_vertical_additional = 0.5  #0.15   

    ####### Autonomous Positioning Thresholds #######
    threshold_overlap_prevention_vrep = 0.001
    threshold_vertical_positioning_vrep = 0.0003  # 0.001
    pixel_threshold = True                       # If false, use distance (capture) threshold
    threshold_planar_positioning_pixel = 20
    threshold_overlap_prevention_pixel = 150  # 100
    threshold_vertical_positioning_pixel = 15
    threshold_overlap_prevention_capture = 0.001
    threshold_vertical_positioning_capture = 0.0025  # 0.0003
    counter_window_tip_dis_length = 45
    tip_dis_count = 40
    converter_per_mm_vrep = 1 / (threshold_vertical_positioning_vrep * 1000)
    converter_per_mm = 417.38  # 254.5
    additional_positioning_margin = 0.00013   # 0.00013  # 0.002

    tan_gamma = 0.88
    tan_bias = 0.70  # 0.63  # Adjust the max cone angle of light guide, thetaC_2,safe
    gamma = math.atan(tan_gamma - tan_bias)

    ####### Control parameters #######
    fps = 250
    tau = 1/fps
    n = 7
    damping = 0.0001  # 0.001

    ####### autonomous positioning #######
    n_planar = 140
    n_vertical = 140
    damping_setup = 0.001
    damping_planar = 0.0001 # 0.003
    damping_overlap_instrument = 0.01
    damping_overlap_light = 0.0001
    damping_vertical_instrument = 0.005
    damping_vertical_light = 0.005
    D_velocity = 0.001  # m/s

    tan_gamma = 0.88
    tan_bias = 0.63  # 0.63  # Adjust the max cone angle of light guide, thetaC_2,safe
    gamma = math.atan(tan_gamma - tan_bias)

    rotation_c_theta_icra = math.pi / 4
    # theta_safe_eye_rotation = math.pi / 15

    nd_rcm = 0.01
    nd_trocar = 1.
    nd_eyeball = 1.
    nd_collision = 1.
    nd_view = 1.
    nd_limit = 1.
    nd_plane = 1.
    nd_cone = 1.
    nd_illumination = 1.
    nd_tip = 1.
    nd_rotation_constraint = 1.
    nd_forceps = 1.

    d_safe_rcm = 0.0005
    d_safe_rcm_om = 0.0005
    d_safe_eyeball = eyeball_radius - 0.001
    d_safe_collision = 0.001
    d_safe_view = 0.01
    d_safe_trocar = 0.005
    d_safe_tip = 0.008
    a_safe_forceps = math.pi / 180 

    D_safe_rcm = d_safe_rcm ** 2
    D_safe_rcm_om = d_safe_rcm_om ** 2
    D_safe_eyeball = d_safe_eyeball ** 2
    D_safe_collision = d_safe_collision ** 2
    D_safe_view = d_safe_view ** 2
    D_safe_trocar = d_safe_trocar ** 2
    D_safe_tip = d_safe_tip ** 2
    A_safe_forceps = a_safe_forceps

    ####### Constrained optimization parameters #######
    alpha = 0.999995
    beta = 0.99  # 0.99
    # Dampening matrix 
    B_8 = np.vstack([np.hstack([beta * np.eye(4), np.zeros([4, 4])]),
                     np.hstack([np.zeros([4, 4]), (1 - beta) * np.eye(4)])])
    B_13 = np.vstack([np.hstack([beta * np.eye(7), np.zeros([7, 6])]),
                      np.hstack([np.zeros([6, 7]), (1 - beta) * np.eye(6)])])
    # Forceps joint prioritization
    B_13[6,6] = 0.01
