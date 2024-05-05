import math


class EyeballParameters:
    """
    This class contains parameters for eyeball object.
    """
    # Eyeball and trocar parameters
    eyeball_radius = 0.012
    port_angle = 30 / 180 * math.pi
    eyeground_radius = 0.005
    eyeground_plus_height = 0.003  # 0.002 for detachable R&D model

    radius_bias = 0.  # 0.0015
    eyeground_radius_init = eyeground_radius
    ws_radius = eyeground_radius_init - radius_bias

    trocar_outer_radius = 0.00025
    trocar_inner_radius = 0.0002


