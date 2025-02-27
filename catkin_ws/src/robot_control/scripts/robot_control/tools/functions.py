import rospy

"""
This file checks if the robot is physically used in real life or through simulation.
"""


def is_physical_robot():
    """
    """
    robot_1_status = rospy.get_param('/arm1/use_real_robot')
    robot_2_status = rospy.get_param('/arm2/use_real_robot')
    return robot_1_status and robot_2_status


