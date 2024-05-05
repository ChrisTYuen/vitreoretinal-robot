# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys
import yaml

from PyQt5 import uic, QtGui
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import QFile

import dqrobotics as dql
# from dqrobotics.robot_modeling import DQ_Kinematics
import numpy as np
from itertools import combinations

import rospy


def comb3(keys):
    index1=[]
    index2=[]
    index3=[]

    for comb in combinations(keys, 3):
        index1.append(comb[0])
        index2.append(comb[1])
        index3.append(comb[2])

    return index1, index2, index3


class TableCalibrationUI(QWidget):
    def __init__(self, robot, rosConfig, calibrationConfig):
        super(TableCalibrationUI, self).__init__()

        #save robot handle
        self._robot = robot['robot']
        self._robot_interface = robot['robot_interface']
        self._rosConfig = rosConfig
        self._calibrationConfig = calibrationConfig


        # list of data points
        self._calibration_point_list={}
        self._calibration_point_count=0

        self.load_ui()

    def load_ui(self):
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        uic.loadUi(path, self)  # Load the .ui file

        self.addButton.clicked.connect(self.add_btn_pressed_)
        self.removeButton.clicked.connect(self.remove_btn_pressed_)
        self.calculateButton.clicked.connect(self.calculate_btn_pressed_)
        self.poseListWidget.currentItemChanged.connect(self.list_select_changed_)

        calibration_point_list = rospy.get_param(self._rosConfig['cal_data_ns'] + "calibration", {})


        for ind, (key, val) in enumerate(calibration_point_list.items()):
            try:
                self._calibration_point_count = int(key)+1
            except ValueError:
                continue
            self._calibration_point_list[key] = val
            self.poseListWidget.insertItem(ind, key)

    def update_ros_param_(self):
        rospy.set_param(self._rosConfig['cal_data_ns'] + "calibration", self._calibration_point_list)

    def set_max_entry_count_(self):
        calibration_point_count = 0
        for key in self._calibration_point_list:
            if int(key) >= calibration_point_count:
                calibration_point_count = int(key) + 1

        self._calibration_point_count = calibration_point_count

    def add_btn_pressed_(self):
        joints_entry = self._robot_interface.get_joint_positions()
        output_dq = self._robot.fkm(joints_entry)
        output_t= dql.translation(output_dq).vec3()


        self._calibration_point_list[str(self._calibration_point_count)] = output_t.tolist()
        self._calibration_point_count += 1
        self.poseListWidget.insertItem(0, str(self._calibration_point_count-1))

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Added datapoint: "+str(output_t.tolist()))
        self.update_ros_param_()

    def remove_btn_pressed_(self):

        selected = self.poseListWidget.selectedItems()
        if not selected:
            return
        for item in selected:
            self._calibration_point_list.pop(item.text(), None)
            itemIndex = self.poseListWidget.row(item)
            self.poseListWidget.takeItem(itemIndex)

        self.set_max_entry_count_()
        self.update_ros_param_()

    def list_select_changed_(self, current, previous):
        key = current.text()
        for ind, name in enumerate(['x', 'y', 'z']):
            label_h = getattr(self, "pointDisplay_"+name)
            label_h.setText("{:.5f}".format(self._calibration_point_list[key][ind]))


    def calculate_btn_pressed_(self):

        list_of_pts = {}

        for key, point in self._calibration_point_list.items():
            list_of_pts[key] = dql.DQ(point)

        n_pts = len(list(list_of_pts.keys()))

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Number of datapoints = "+str(n_pts))

        index1, index2, index3 = comb3(list(list_of_pts.keys()))

        ncomb = len(index1)

        rospy.loginfo("[" + self._rosConfig['name'] + "]::Number of possible 3x3 combinations = "+str(ncomb))

        list_of_normal = []
        list_of_distance = []
        for ind in range(ncomb):
            a = list_of_pts[index1[ind]]
            b = list_of_pts[index2[ind]]
            c = list_of_pts[index3[ind]]

            n = dql.normalize(dql.cross(b-a, b-c))
            if dql.norm(n).q[0]<=0:
                rospy.loginfo("[" + self._rosConfig['name'] +
                            "]::un-computable plane exist for point combination = "+
                            str([index1[ind], index2[ind], index3[ind]]))
                ncomb -= 1
                continue
            #check normal direction
            cos_angle = dql.dot(self._calibrationConfig['desired_table_normal'], n).q[0]
            d = dql.dot(n, b)
            if cos_angle<0:
                list_of_normal.append(-n)
                list_of_distance.append(-d)
            else:
                list_of_normal.append(n)
                list_of_distance.append(d)

        rospy.loginfo("[" + self._rosConfig['name'] +
                    "]::list of normal vector = "+
                    str(list_of_normal))
        rospy.loginfo("[" + self._rosConfig['name'] +
                    "]::list of distance = "+
                    str(list_of_distance))
        #average normal
        n_sum = dql.DQ([0])
        d_sum = dql.DQ([0])
        for ind in range(ncomb):
            n_sum = n_sum + list_of_normal[ind]
            d_sum = d_sum + list_of_distance[ind]
        n_pi = n_sum.normalize()
        d_pi = d_sum * (1.0/ncomb) + self._calibrationConfig['table_offset']
        desk_plane = n_pi + dql.DQ.E*d_pi


        rospy.loginfo("[" + self._rosConfig['name'] +
                    "]::table normal vector = "+
                    str(n_pi))
        rospy.loginfo("[" + self._rosConfig['name'] +
                    "]::table distance = "+
                    str(d_pi))

        rospy.loginfo("[" + self._rosConfig['name'] + "]::plane found as = " + str(desk_plane))

        rospy.set_param(self._rosConfig['cal_data_ns'] + "calibration/result", desk_plane.q.tolist())
        data={
            "normal": n_pi.q[1:4].tolist(),
            "distance": d_pi.q[0].tolist(),
            "desk_plane": desk_plane.q.tolist(),
        }
        with open(self._calibrationConfig['table_info_save_path'], 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=None)





if __name__ == "__main__":
    app = QApplication([])
    widget = EndEffectorCalibrationUI()
    widget.show()
    sys.exit(app.exec_())
