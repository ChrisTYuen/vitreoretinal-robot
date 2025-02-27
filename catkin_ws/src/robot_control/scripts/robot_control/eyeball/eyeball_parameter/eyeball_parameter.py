import math
from kinematics.parameters.control_parameters import Parameters

class EyeballParameters:
    """
    This class contains parameters for eyeball object.
    """
    # Eyeball and trocar parameters
    eyeball_radius = 0.012
    port_angle = 30 / 180 * math.pi
    eyeground_radius = 0.005
    eyeground_plus_height = Parameters.eyeground_plus_height  # 0.002 for detachable R&D model

    radius_bias = 0.  # 0.0015
    eyeground_radius_init = eyeground_radius
    ws_radius = eyeground_radius_init - radius_bias

    trocar_outer_radius = Parameters.trocar_outer_radius
    trocar_inner_radius = Parameters.trocar_inner_radius
