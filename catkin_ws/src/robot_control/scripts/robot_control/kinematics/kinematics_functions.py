# Import Relevant files from dqrobotics
from dqrobotics import *

# Import other dependencies
import math
import numpy as np
from scipy import interpolate

"""
This file contains some functions that help you to conduct vitreoretinal experiments smoothly. 
"""


def get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, eyeball_radius, port_angle):
    """
    get_eyeball_position_from_rcm(rcm_si_dq, rcm_lg_dq, eyeball_radius, port_angle) returns the dual quaternions
    "eyeball_dq" that represents the pose of the eyeball calculated using the poses of the rcm points "rcm_si_dq"
    and "rcm_lg_dq".
    """
    # Calculate the translation of the eyeball using the rcm positions and "eyeball_radius"
    rcm_si_t = translation(rcm_si_dq)
    rcm_lg_t = translation(rcm_lg_dq)

    mid_vec = (vec4(rcm_si_t) + vec4(rcm_lg_t)) * 0.5
    midp_t = DQ(mid_vec)
    rcm_radius = np.linalg.norm(vec4(midp_t - rcm_si_t)) / math.cos(port_angle)

    rcm_height = math.sqrt(eyeball_radius**2 - rcm_radius**2)
    rcm_circle_t = midp_t + (math.cos(math.pi/4)+k_*math.sin(math.pi/4)) * (rcm_si_t-midp_t)\
                                                                         * conj(math.cos(math.pi/4) + k_*math.sin(math.pi/4))\
                                                                         * math.tan(port_angle)
    eyeball_t = rcm_circle_t - rcm_height * k_


    # Get the pose of the eyeball letting the rotation be equal to 1
    eyeball_r = 1.
    eyeball_dq = eyeball_r + 0.5 * E_ * eyeball_t * eyeball_r
    return eyeball_dq


def get_initial_pose_in_eyeball(eyeball_dq, eyeball_radius, insertion_distance, rcm_dq):
    td = translation(rcm_dq)\
         + DQ(vec4(translation(eyeball_dq) - translation(rcm_dq)) / eyeball_radius * insertion_distance)

    insertion_direction = vec4(normalize(translation(eyeball_dq) - translation(rcm_dq)))
    angle1 = math.acos(insertion_direction[1] / math.sqrt(insertion_direction[1] ** 2
                                                          + insertion_direction[2] ** 2))
    angle2 = math.acos(-1 * insertion_direction[3] / math.sqrt(insertion_direction[1] ** 2
                                                               + insertion_direction[2] ** 2
                                                               + insertion_direction[3] ** 2))
    rd = (math.sin(angle1 / 2) - k_ * math.cos(angle1 / 2)) * (math.cos(-1 * math.pi / 4 - angle2 / 2)
                                                               + j_ * math.sin(-1 * math.pi / 4 - angle2 / 2))
    return td, rd


def spline(check_points, iterations, deg):
    """
        spline(check_points,iterations,deg) returns the 3 dimensional spline that is defined based on "check_point",
        "iterations", and "deg".
        """
    # See also https://docs.scipy.org/doc/scipy/tutorial/interpolate.html
    tck, u = interpolate.splprep(check_points, k=deg, s=0)
    u = np.linspace(0, 1, num=iterations + 1, endpoint=True)
    generated_spline = interpolate.splev(u, tck)
    return generated_spline[0], generated_spline[1], generated_spline[2]


def get_constrained_plane_list(eyeball_t, celling_height, floor_height):
    """
    get_constrained_plane_list(eyeball_t, celling_height, floor_height) returns the lists of the dual quaternions
    "constrained_plane_list_1" and "constrained_plane_list_2" used to enforce the constraints for safety.
    See also Vitreoretinal Task Constraints described in VIII of Koyama et al. (2022)
    """
    # Calculate the planes placed between the robots to avoid the collision of the two robots
    pi_middle_2 = i_ + E_ * dot(eyeball_t, i_)
    pi_middle_1 = -1*i_ + E_ * dot(eyeball_t, -1*i_)

    # Calculate the planes that correspond to the celling and the floor of the cage
    # pi_celling = k_ + E_ * (celling_height + dot(eyeball_t, k_))
    # pi_floor = -1*k_ + E_ * dot(floor_height*k_, -1*k_)

    # Store them in arrays
    # constrained_plane_list_1 = [pi_middle_1, pi_celling, pi_floor]
    # constrained_plane_list_2 = [pi_middle_2, pi_celling, pi_floor]
    constrained_plane_list_1 = [pi_middle_1]
    constrained_plane_list_2 = [pi_middle_2]
    return [constrained_plane_list_1, constrained_plane_list_2]


def get_total_iteration(tau, check_points, velocity):
    """
    get_total_iteration(tau, check_points, si_velocity) returns "total_iteration", that is the number of the iterations
    needed to move the tip to the desired translations at a constant velocity "velocity".
    """
    trajectory_length = 0
    check_points_x = check_points[0]
    check_points_y = check_points[1]

    # Calculate the total length of the trajectory that connects each point
    for check_point in range(len(check_points_x) - 1):
        trajectory_length = trajectory_length + math.sqrt((check_points_x[check_point + 1] - check_points_x[check_point])**2 + (check_points_y[check_point + 1] - check_points_y[check_point])**2)

    # Calculate the total iteration
    total_iteration = (trajectory_length * 10 ** 3 / velocity) // tau

    # Calculate the time needed for the instrument to go along the trajectory
    experiment_time = total_iteration*tau
    print("[eyesurgery_kinematics_functions.py]:: Experiment time is " + str(format(experiment_time, '.4f')) + "[s]\n")
    return total_iteration


def get_p2p_iteration(current_t, desired_t, tau, velocity):
    """
    get_positioning_total_iteration(tau, length, si_velocity) returns "total_iteration", that is the number of the
    iterations needed for the instrument to move the distance of "length" at a constant velocity "velocity".
    """
    # Get the total iteration
    trajectory_length = np.linalg.norm(vec4(desired_t-current_t))
    total_iteration = (trajectory_length * 10 ** 3 / velocity) // tau

    # Calculate the time needed for the instrument to go along the trajectory
    experiment_time = total_iteration * tau
    print("[eyesurgery_kinematics_functions.py]:: Duration is " + str(format(experiment_time, '.1f')) + "[s]")
    return total_iteration


def show_gap(x1, td_1):
    """
    show_gap(x1, td_1) shows the error distance between the desired translation and the translation after moving.
    """
    # Get the translation after the process
    t_final = translation(x1)

    # Calculate the error
    gap = np.linalg.norm(vec4(t_final - td_1))
    print("[eyesurgery_kinematics_functions.py]:: Completed! The gap to the desired point is " + str(format(gap * 10 ** 3, '.4f')) + " mm.")


def get_rcm_positions(port_angle, port_radius, eyeball_radius, eyeball_center_t):
    """
    get_rcm_positions(port_angle, port_radius, eyeball_radius, eyeball_center_t) returns the ideal poses of the rcm
    points, "rcm_si_dq" and "rcm_lg_dq", calculated from the pose of the eyeball.
    """
    # Calculate the x, y, and z components
    rcm_x_component = port_radius * math.cos(port_angle)
    rcm_y_component = port_radius * math.sin(port_angle)
    rcm_z_component = math.sqrt(eyeball_radius**2 - (rcm_x_component**2 + rcm_y_component**2))

    # Calculate the desired translations of the rcm points
    rcm_si_t = rcm_x_component*i_ - rcm_y_component*j_ + rcm_z_component*k_ + eyeball_center_t
    rcm_lg_t = -1 * rcm_x_component*i_ - rcm_y_component*j_ + rcm_z_component*k_ + eyeball_center_t

    # Calculate the desired poses of the rcm points letting the rotations be equal to 1
    rcm_si_dq = 1 + 0.5*E_*rcm_si_t
    rcm_lg_dq = 1 + 0.5*E_*rcm_lg_t
    return rcm_si_dq, rcm_lg_dq


def closest_invariant_rotation_error(r, rd):
    """
    closest_invariant_rotation_error(r, rd) returns the closest invariant rotation error "er".
    For more details, see Marinho, M. M., Adorno, B. V., Harada, K, Deie, K., Deguet, A., Kazanzides, P.,
    Taylor, R. H., and Mitsuishi, M. (2019).
    A Unified Framework for the Teleoperation of Surgical Robots in Constrained Workspaces.
    2013 IEEE International Conference on Robotics and Automation (ICRA)
    """
    er_plus_norm = np.linalg.norm(vec4(conj(r) * rd - 1))
    er_minus_norm = np.linalg.norm(vec4(conj(r) * rd + 1))

    # Section IV-A of Marinho et al. (2019)
    if er_plus_norm < er_minus_norm:
        er = conj(r) * rd - 1
    else:
        er = conj(r) * rd + 1
    return er















