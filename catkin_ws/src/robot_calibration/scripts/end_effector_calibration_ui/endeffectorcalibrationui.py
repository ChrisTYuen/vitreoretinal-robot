# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys
import yaml
import json

from PyQt5 import uic, QtGui
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QFile

import dqrobotics as dql
# from dqrobotics.robot_modeling import DQ_Kinematics
import numpy as np

import rospy


def combn2(keys):
    index1=[]
    index2=[]
    for ind, key in enumerate(keys):
        for j in range(ind+1, len(keys)):
            index1.append(keys[ind])
            index2.append(keys[j])

    return index1, index2

def read_yaml_file(file_path):
    with open(file_path, 'r') as yaml_file:
        data = yaml.load(yaml_file, Loader=yaml.SafeLoader)
    return data

def write_yaml_file(file_path, data):
    with open(file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=None)

def read_json_file(file_path):
    with open(file_path, 'r') as json_file:
        data = json.load(json_file)
    return data

def write_json_file(file_path, data):
    with open(file_path, 'w') as json_file:
        json.dump(data, json_file, indent=2)

class EndEffectorCalibrationUI(QWidget):
    def __init__(self, robot1, robot2, rosConfig, calibrationConfig):
        super(EndEffectorCalibrationUI, self).__init__()

        #save robot handle
        self._robot1 = robot1['robot1']
        self._robot2 = robot2['robot2']
        self._robot1_interface = robot1['robot1_interface']
        self._robot2_interface = robot2['robot2_interface']
        self._rosConfig = rosConfig
        self._calibrationConfig = calibrationConfig

        # list of data points
        self._calibration_data = {
        'arm1': {
            'joint_list': {},
            'pose_list': {},
            'point_count': 0
            },
        'arm2': {
            'joint_list': {},
            'pose_list': {},
            'point_count': 0
            }
        }

        self.load_ui()


    def load_ui(self):
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        uic.loadUi(path, self)  # Load the .ui file
        
        self.arm1Button.toggled.connect(self.arm1_btn_toggled_)
        self.arm2Button.toggled.connect(self.arm2_btn_toggled_)
        self.addButton.clicked.connect(self.add_btn_pressed_)
        self.removeButton.clicked.connect(self.remove_btn_pressed_)
        self.calculateButton.clicked.connect(self.calculate_btn_pressed_)
        self.poseListWidget.currentItemChanged.connect(self.list_select_changed_)
        self.updateCalibrationFileButton.clicked.connect(self.update_calibration_file_)
    
    
    def get_current_calibration_data(self):
        if self.arm1Button.isChecked():
            return self._calibration_data['arm1']
        elif self.arm2Button.isChecked():
            return self._calibration_data['arm2']
        else:
            raise ValueError("No Arm selected")


    def update_calibration_list(self):
        current_data = self.get_current_calibration_data()
        calibration_joint_list = current_data['joint_list']
        calibration_pose_list = current_data['pose_list']
        calibration_point_count = current_data['point_count']

        if self.arm1Button.isChecked():
            new_calibration_joint_list = rospy.get_param(self._calibrationConfig['arm1_calibration_data_namespace'] + "calibration_joint", {})
            new_calibration_pose_list = rospy.get_param(self._calibrationConfig['arm1_calibration_data_namespace'] + "calibration_pose", {})
        elif self.arm2Button.isChecked():
            new_calibration_joint_list = rospy.get_param(self._calibrationConfig['arm2_calibration_data_namespace'] + "calibration_joint", {})
            new_calibration_pose_list = rospy.get_param(self._calibrationConfig['arm2_calibration_data_namespace'] + "calibration_pose", {})

        for ind, (key, val) in enumerate(calibration_joint_list.items()):
            try:
                new_point_count = int(key)+1
                if new_point_count > calibration_point_count:
                    calibration_point_count = new_point_count
            except ValueError:
                continue
            calibration_joint_list[key] = val
            calibration_pose_list[key] = dql.DQ(new_calibration_joint_list[key]).normalize()
            rospy.loginfo("[" + self._rosConfig['name'] + "]::got historical data "+str(calibration_pose_list[key]))
            self.poseListWidget.insertItem(ind, key)

        current_data['point_count']= calibration_point_count


    def arm1_btn_toggled_(self, checked):
        if checked:
            self._robot = self._robot1
            self._robot_interface = self._robot1_interface
            self.end_effector_info_save_path = self._calibrationConfig['arm1_end_effector_info_save_path']
            self.calibration_data_namespace = self._calibrationConfig['arm1_calibration_data_namespace']
            self.factory_calibration_save_path = self._calibrationConfig['arm1_parameter_file_path']
            self.updated_calibration_save_path = self._calibrationConfig['arm1_parameter_save_path_calibrated']

            self.update_calibration_list()


    def arm2_btn_toggled_(self, checked):
        if checked:
            self._robot = self._robot2
            self._robot_interface = self._robot2_interface
            self.end_effector_info_save_path = self._calibrationConfig['arm2_end_effector_info_save_path']
            self.calibration_data_namespace = self._calibrationConfig['arm2_calibration_data_namespace']
            self.factory_calibration_save_path = self._calibrationConfig['arm2_parameter_file_path']
            self.updated_calibration_save_path = self._calibrationConfig['arm2_parameter_save_path_calibrated']

            self.update_calibration_list()


    def update_ros_param_(self):
        current_data = self.get_current_calibration_data()
        calibration_joint_list = current_data['joint_list']
        calibration_pose_list = current_data['pose_list']
        rospy.set_param(self.calibration_data_namespace + "calibration_joint", [[float(val) for val in sublist] for sublist in calibration_joint_list.values()])
        for key, pose in calibration_pose_list.items():
            rospy.set_param(self.calibration_data_namespace + "calibration_pose/" + key, [float(val) for val in pose.vec8().tolist()])


    def set_max_entry_count_(self):
        current_data = self.get_current_calibration_data()
        calibration_joint_list = current_data['joint_list']
        calibration_point_count = 0
        for key in calibration_joint_list:
            if int(key) >= calibration_point_count:
                calibration_point_count = int(key) + 1

        current_data['point_count'] = calibration_point_count


    def add_btn_pressed_(self):
        current_data = self.get_current_calibration_data()
        calibration_joint_list = current_data['joint_list']
        calibration_pose_list = current_data['pose_list']
        calibration_point_count = current_data['point_count']

        joints_entry = self._robot_interface.get_joint_positions()
        if self._calibrationConfig['use_tool_pose']:
            rospy.loginfo("[" + self._rosConfig['name'] + "]::logged with tool pose")
        else:
            pose_entry = self._robot.fkm(joints_entry)

        calibration_joint_list[str(calibration_point_count)] = joints_entry
        calibration_pose_list[str(calibration_point_count)] = pose_entry
        calibration_point_count += 1
        current_data['point_count'] = calibration_point_count

        self.poseListWidget.insertItem(0, str(calibration_point_count-1))

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Added datapoint: (joint) "
                                                                +str(joints_entry)
                                                                +" (pose) "+str(pose_entry))
        self.update_ros_param_()


    def remove_btn_pressed_(self):
        current_data = self.get_current_calibration_data()
        calibration_joint_list = current_data['joint_list']
        calibration_pose_list = current_data['pose_list']

        selected = self.poseListWidget.selectedItems()
        if not selected:
            print("No selected item")
            return
        for item in selected:
            key = int(item.text())
            calibration_joint_list.pop(item.text(), None)
            calibration_pose_list.pop(item.text(), None)
            itemIndex = self.poseListWidget.row(item)
            self.poseListWidget.takeItem(itemIndex)
        
        current_data['point_count'] = len(calibration_joint_list)

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Removed datapoint")

        self.set_max_entry_count_()
        self.update_ros_param_()


    def list_select_changed_(self, current, previous):
        current_data = self.get_current_calibration_data()
        calibration_joint_list = current_data['joint_list']
        if current is None:
            for ind in range(6):
                label_h = getattr(self, "jointDisplay_" + str(ind + 1))
                label_h.setText("N/A")
            return
        key = current.text()
        for ind, joint_val in enumerate(calibration_joint_list[key]):
            label_h = getattr(self, "jointDisplay_"+str(ind+1))
            label_h.setText("{:.5f}".format(joint_val))


    def calculate_btn_pressed_(self):
        current_data = self.get_current_calibration_data()
        calibration_pose_list = current_data['pose_list']

        # n = len(calibration_pose_list.keys())

        list_of_rotations = {}
        list_of_translations = {}
        for key, pose in calibration_pose_list.items():
            list_of_rotations[key] = dql.rotation(pose)
            list_of_translations[key] = dql.translation(pose)

        index1, index2 = combn2(list(calibration_pose_list))
        ncomb = len(index1)

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Number of possible 2x2 combinations = "+str(ncomb))

        A_ext = np.zeros((ncomb*4, 4))
        B_ext = np.zeros((ncomb*4))

        for i in range(ncomb):
            a = index1[i]
            b = index2[i]

            td = list_of_translations[a] - list_of_translations[b];
            B = np.array([0, td.q[1], td.q[2], td.q[3]])
            B_ext[i*4:i*4+4] = B

            ra = list_of_rotations[a]
            rb = list_of_rotations[b]
            A = dql.hamiplus4(ra) @ dql.haminus4(dql.conj(ra)) - dql.hamiplus4(rb) @ dql.haminus4(dql.conj(rb))
            A_ext[i*4:i*4+4, :] = A

        d = -np.linalg.pinv(A_ext) @ B_ext
        rospy.loginfo("[" + self._rosConfig['name'] + "]::Distance vector found as = " + str(d))

        calibration_resultdq = dql.DQ([1, 0, 0, 0, 0, d[1], d[2], d[3]])
        rospy.set_param(self.calibration_data_namespace + "calibration/result", calibration_resultdq.q.tolist())
        data={
            "translation": d[1:].tolist(),
            "dq_change": calibration_resultdq.q.tolist(),
        }
        write_yaml_file(self.end_effector_info_save_path, data)


    def update_calibration_file_(self):

        calibration_resultdq = read_yaml_file(self.end_effector_info_save_path)
        factory_calibration_data = read_json_file(self.factory_calibration_save_path)
        updated_calibration_data = read_json_file(self.updated_calibration_save_path)
    
        if self.arm1Button.isChecked():
            # Update arm1 calibration parameter
            # Update translation
            updated_calibration_data['a'][5] = factory_calibration_data['a'][5] + calibration_resultdq['translation'][0]
            updated_calibration_data['b'][5] = factory_calibration_data['b'][5] + calibration_resultdq['translation'][1]
            updated_calibration_data['d'][5] = factory_calibration_data['d'][5] + calibration_resultdq['translation'][2]

            # Update rotation
            updated_calibration_data['alpha'][5] = 180
            updated_calibration_data['beta'][5] = 90

            write_json_file(self.updated_calibration_save_path, updated_calibration_data)

            rospy.loginfo("[" + self._rosConfig['name'] + "]::Updated Arm1 Light Guide Calibration File")

        elif self.arm2Button.isChecked():
            # Update arm2 calibration parameter
            # Update translation    
            updated_calibration_data['b'][5] = factory_calibration_data['b'][5] + calibration_resultdq['translation'][1]
            updated_calibration_data['d'][5] = factory_calibration_data['d'][5] + calibration_resultdq['translation'][2]
            updated_calibration_data['d'][6] = calibration_resultdq['translation'][0]

            # Update rotation
            updated_calibration_data['alpha'][5] = 180
            updated_calibration_data['beta'][5] = 90

            write_json_file(self.updated_calibration_save_path, updated_calibration_data)

            rospy.loginfo("[" + self._rosConfig['name'] + "]::Updated Arm2 Instrument Calibration File")

        else:
            rospy.loginfo("[" + self._rosConfig['name'] + "]::No Arm Selected")    


if __name__ == "__main__":
    app = QApplication([])
    widget = EndEffectorCalibrationUI()
    widget.show()
    sys.exit(app.exec_())
