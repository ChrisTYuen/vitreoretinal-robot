#!/usr/bin/python3
from orbital_manipulation_kinematics import OrbitalManipulationKinematics as eye_kinematics
from parameters import physical_parameters, control_parameters

# Import Relevant files from dqrobotics
from dqrobotics import *
from dqrobotics.interfaces.vrep import DQ_VrepInterface

# Some libraries for calculations
import math
import time
import rospy


def eyeball_manipulation_test():
    try:
        rospy.init_node("eyeball_rotation_test", disable_signals=False)

        vi = DQ_VrepInterface()
        vi.connect("127.0.0.1", 19996, 100, 10)
        vi.start_simulation()

        # Get experimental parameter and control parameters
        sim_setup = physical_parameters.SimulationSetup()
        parameter = control_parameters.Parameters()

        eyeball_center = translation(sim_setup.eyeball_position)
        eyeball_radius = parameter.eyeball_radius
        vi.set_object_translation("rcm1_init", eyeball_center)
        time.sleep(.1)
        vi.set_object_translation("rcm2_init", eyeball_center)
        time.sleep(.1)
        vi.set_object_translation("rcm1_target", eyeball_center)
        time.sleep(.1)
        vi.set_object_translation("rcm2_target", eyeball_center)
        time.sleep(.1)
        vi.set_object_translation("rcm1_current", eyeball_center)
        time.sleep(.1)
        vi.set_object_translation("rcm2_current", eyeball_center)
        time.sleep(.1)
        vi.set_object_translation("rcm2_mediate", eyeball_center)

        init_si = normalize(j_)
        theta = 1 * math.pi / 16
        axis_init = normalize(k_-i_)

        target_si = normalize(k_)
        axis_target = normalize(-j_-i_)

        init_lg = (math.cos(theta/2) + axis_init * math.sin(theta/2)) * init_si * conj(math.cos(theta/2) + axis_init * math.sin(theta/2))
        target_lg = (math.cos(theta/2) + axis_target * math.sin(theta/2)) * target_si * conj(math.cos(theta/2) + axis_target * math.sin(theta/2))

        rcm_si_init = eyeball_center + eyeball_radius * init_si
        rcm_lg_init = eyeball_center + eyeball_radius * init_lg

        rcm_si_target = eyeball_center + eyeball_radius * target_si
        rcm_lg_target = eyeball_center + eyeball_radius * target_lg

        vi.set_object_translation(sim_setup.eyeball_vrep_name, eyeball_center)
        vi.set_object_translation("rcm1_init", rcm_si_init)
        vi.set_object_translation("rcm2_init", rcm_lg_init)

        input("Set objects to the initial points!\n")

        vi.set_object_translation("rcm1_target", rcm_si_target)
        vi.set_object_translation("rcm2_target", rcm_lg_target)

        input("Set objects to the target points!\n")

        r_o_e = eye_kinematics.get_eyeball_rotation(eyeball_center, eyeball_radius, rcm_si_init, rcm_lg_init,
                                                    rcm_si_target, rcm_lg_target)

        input("Calculated the eyeball rotation!\n")

        rcm_si_current = eyeball_center + eyeball_radius * r_o_e * init_si * conj(r_o_e)
        rcm_lg_current = eyeball_center + eyeball_radius * r_o_e * init_lg * conj(r_o_e)

        vi.set_object_translation("rcm1_current", rcm_si_current)
        vi.set_object_translation("rcm2_current", rcm_lg_current)

        print("Set the RCM points to the rotated points! Check if it's correct!\n")

        time.sleep(1)

    except Exception as exp:
        print("[" + rospy.get_name() + "]:: {}.".format(exp))
    except KeyboardInterrupt:
        print("Keyboard Interrupt!!")


if __name__ == '__main__':
    eyeball_manipulation_test()
