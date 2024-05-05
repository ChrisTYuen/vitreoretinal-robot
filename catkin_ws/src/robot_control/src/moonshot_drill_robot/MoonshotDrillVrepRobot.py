from dqrobotics import DQ_VrepRobot
import numpy as np

class MoonshotDrillVrepRobot:
    def __init__(self, robot_name, vrep_interface):
        self.vrep_interface = vrep_interface
        self.robot_name = robot_name
        self.robot_name_ = None
        self.vrep_interface_ = None
        self.joint_names_ = None
        self.splited_name = self.robot_name_.split('#')
        self.robot_label = self.splited_name[0]

        if self.robot_label.compare("CVR038") != 0:
            raise Exception("Expected CVR038")

        self.robot_index("")
        if self.splited_name.size() > 1:
            self.robot_index = self.splited_name[1]

        for i in range(7):
            self.current_joint_name = self.robot_label + "_joint" + str(i) + "#" + self.robot_index
            self.joint_names_ = np.append(self.joint_names_, self.current_joint_name)

        self.reference_frame_name_ = self.robot_label + "_reference" + self.robot_index

    def send_q_to_vrep(self, q):
        self.vrep_interface_.set_joint_positions(self.joint_names_, q)

    def get_q_from_vrep(self):
        return self.vrep_interface_.get_joint_positions(self.joint_names_)

    def get_reference_frame(self):
        return self.vrep_interface_.get_object_pose(self.reference_frame_name_)
