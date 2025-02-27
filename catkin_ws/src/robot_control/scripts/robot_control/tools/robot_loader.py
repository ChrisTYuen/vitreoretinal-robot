import os
import logging

# logger init
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

from dqrobotics.interfaces.json11 import DQ_JsonReader

import json


# Class definition of VS050 DENSO robot for pathology

class Robot:
    """
    This class is used to load the robot parameters from the json file.
    """
    def __init__(self, json_path=None):
        # Standard of VS050

        if json_path is None or not os.path.isfile(json_path):
            raise ValueError("robot.json not specified")

        try:
            with open(json_path) as j_f:
                jdata = json.load(j_f)

        except Exception as e:
            raise ValueError("DH loading file read error")

        reader = DQ_JsonReader()

        if jdata['robot_type'] == "DQ_SerialManipulatorDH":
            self.robot = reader.get_serial_manipulator_dh_from_json(json_path)
        elif jdata['robot_type'] == "DQ_SerialManipulatorDenso":
            self.robot = reader.get_serial_manipulator_denso_from_json(json_path)
        else:
            raise ValueError("json parameter type definition error: " + str(type))

    # @staticmethod
    def kinematics(self):
        return self.robot
