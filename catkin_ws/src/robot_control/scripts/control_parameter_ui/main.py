#!/usr/bin/python3
uiName = "UIMaster"

import sys
import os
import argparse

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, QByteArray, QLocale, qChecksum, QDataStream, QIODevice, QBuffer, QTime, QEvent
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication, QWidget, QGraphicsScene
from PyQt5.QtNetwork import QUdpSocket, QHostAddress
from PyQt5.QtGui import QDoubleValidator

from scipy.spatial.transform import Rotation as R
import ctypes

import rospy

from functools import partial

# DEFAULT_REMOTE_IP="10.8.154.141"
DEFAULT_REMOTE_IP = "10.8.154.108"
DEFAULT_RECVFROM_PORT = 2223
DEFAULT_SENDTO_PORT = 2222

DEFAULT_MOTION_SCALE = 1
DEFAULT_LOOP_RATE = 30

ROTATION_SCALING = 1000
WHEEL_SCALING = 12000
PLANE_SCALING = 20000

X_ELEMENTS = ['zt_left', 'yt_left', 'xt_left',
              'zr_left', 'yr_left', 'xr_left',
              'yt_right', 'zt_right', 'xt_right',
              'yr_right', 'zr_right', 'xr_right']


class ControlGraphicsScene(QGraphicsScene):
    signalMouseState = pyqtSignal(float, float, float, str)

    def __init__(self, parent):
        super(ControlGraphicsScene, self).__init__(parent)

    def mouseReleaseEvent(self, QGraphicsSceneMouseEvent):
        pos = QGraphicsSceneMouseEvent.lastScenePos()
        self.signalMouseState.emit(pos.x(), pos.y(), 0, 'release')

    def mouseMoveEvent(self, QGraphicsSceneMouseEvent):
        pos = QGraphicsSceneMouseEvent.lastScenePos()
        self.signalMouseState.emit(pos.x(), pos.y(), 0, 'move')

    def mousePressEvent(self, QGraphicsSceneMouseEvent):
        pos = QGraphicsSceneMouseEvent.lastScenePos()
        self.signalMouseState.emit(pos.x(), pos.y(), 0, 'pressed')

    def wheelEvent(self, QGraphicsSceneWheelEvent):
        delta = QGraphicsSceneWheelEvent.delta()
        self.signalMouseState.emit(0, 0, float(delta), 'wheel')


class outDataStruct(ctypes.Structure):
    _fields_ = [
        # ('clutch', ctypes.c_int32),('motionScale', ctypes.c_float),
        ('motionScale', ctypes.c_float), ('clutch', ctypes.c_int32),
        ('xt_left', ctypes.c_float), ('yt_left', ctypes.c_float), ('zt_left', ctypes.c_float),
        ('wqr_left', ctypes.c_float), ('xqr_left', ctypes.c_float), ('yqr_left', ctypes.c_float),
        ('zqr_left', ctypes.c_float),
        ('xt_left_v', ctypes.c_float), ('yt_left_v', ctypes.c_float), ('zt_left_v', ctypes.c_float),
        ('xr_left_v', ctypes.c_float), ('yr_left_v', ctypes.c_float), ('zr_left_v', ctypes.c_float),
        ('btnLeft', ctypes.c_float),
        ('gripperLeft', ctypes.c_float),
        ('xt_right', ctypes.c_float), ('yt_right', ctypes.c_float), ('zt_right', ctypes.c_float),
        ('wqr_right', ctypes.c_float), ('xqr_right', ctypes.c_float), ('yqr_right', ctypes.c_float),
        ('zqr_right', ctypes.c_float),
        ('xt_right_v', ctypes.c_float), ('yt_right_v', ctypes.c_float), ('zt_right_v', ctypes.c_float),
        ('xr_right_v', ctypes.c_float), ('yr_right_v', ctypes.c_float), ('zr_right_v', ctypes.c_float),
        ('btnRight', ctypes.c_float),
        ('gripperRight', ctypes.c_float),
        # ('crc', ctypes.c_uint16),
    ]


def zxy2quat(yaw, pitch, roll):
    r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    q_w = r.as_quat()
    return [q_w[3], q_w[0], q_w[1], q_w[2]];


class ControllerUI(QWidget):
    def __init__(self, args_in):
        super(ControllerUI, self).__init__()
        self.updateRate = None
        self.udpRecvSocket = None
        self.doubleValidator = None
        self.udpSendSocket = None
        self.init_args = args_in
        self.load_ui()

        # setup loopTimer
        self.loopTimer = QTimer(self)
        self.loopTimer.setSingleShot(False)

        # setup communication obj
        self.remoteConnected = False
        self.udpSendQAddress = None
        self.setup_socket()

        self.loopDtTimer = QTime()
        self.loopDtTimer.start()

        # graph control memory
        self.control_active = None
        self.control_x_init = 0
        self.control_y_init = 0

    def load_ui(self):
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        uic.loadUi(path, self)  # Load the .ui file

        self.setFixedSize(970, 470)
        self.xd_raw_control_state_var = {
            key: 0 for key in X_ELEMENTS
        }

        self.control_state_var = {
            key: 0 for key in X_ELEMENTS
        }
        self.control_state_var.update({
            key + "_v": 0 for key in X_ELEMENTS
        })
        self.control_state_var.update({
            "gripperLeft": 0,
            "btnLeft": False,
            "gripperRight": 0,
            "btnRight": False,
            "clutch": False,
            'motionScale': self.init_args.motion_scale
        })

        # connect signal and misc
        self.xrGraph_left.setInteractive(True)
        self.xrGraph_left_scene = ControlGraphicsScene(self)
        self.xrGraph_left_scene.signalMouseState.connect(partial(self.graph_control_cb_, 'r_left'))
        self.xrGraph_left.setScene(self.xrGraph_left_scene)

        self.xtGraph_left.setInteractive(True)
        self.xtGraph_left_scene = ControlGraphicsScene(self)
        self.xtGraph_left_scene.signalMouseState.connect(partial(self.graph_control_cb_, 't_left'))
        self.xtGraph_left.setScene(self.xtGraph_left_scene)

        self.xrGraph_right.setInteractive(True)
        self.xrGraph_right_scene = ControlGraphicsScene(self)
        self.xrGraph_right_scene.signalMouseState.connect(partial(self.graph_control_cb_, 'r_right'))
        self.xrGraph_right.setScene(self.xrGraph_right_scene)

        self.xtGraph_right.setInteractive(True)
        self.xtGraph_right_scene = ControlGraphicsScene(self)
        self.xtGraph_right_scene.signalMouseState.connect(partial(self.graph_control_cb_, 't_right'))
        self.xtGraph_right.setScene(self.xtGraph_right_scene)

        self.gripperSlider_right.valueChanged.connect(partial(self.gripper_value_changed_,
                                                              self.gripperSlider_right,
                                                              'gripperRight'))
        self.manipulatorBtn_right.clicked.connect(partial(self.control_btn_changed_,
                                                          self.manipulatorBtn_right,
                                                          'btnRight'))
        self.gripperSlider_left.valueChanged.connect(partial(self.gripper_value_changed_,
                                                             self.gripperSlider_left,
                                                             'gripperLeft'))
        self.manipulatorBtn_left.clicked.connect(partial(self.control_btn_changed_,
                                                         self.manipulatorBtn_left,
                                                         'btnLeft'))
        self.clutchBtn.clicked.connect(partial(self.control_btn_changed_,
                                               self.clutchBtn,
                                               'clutch'))
        self.doubleValidator = QDoubleValidator(self)
        locale = QLocale("en")
        self.doubleValidator.setLocale(locale)
        self.motionScaleEdit.setText(str(self.init_args.motion_scale))
        self.motionScaleEdit.setValidator(self.doubleValidator)
        self.motionScaleEdit.returnPressed.connect(self.motion_scale_changed_)

        self.remoteConnecBtn.clicked.connect(self.remote_connect_btn_clicked_)

        self.zeroXdBtn.clicked.connect(self.zero_all_xd_comonents_)

        # set default remote addr
        self.RemoteAddrEdit.setText(self.init_args.host)

    def setup_socket(self):
        self.udpRecvSocket = QUdpSocket(self)
        if self.udpRecvSocket.bind(QHostAddress.LocalHost, self.init_args.local_port):
            print(uiName + "::Listing on UDP Port: " + str(self.init_args.local_port) + " setup success")
        else:
            print(uiName + "::UDP Recive Port: " + str(self.init_args.local_port) + " setup failled")
            sys.exit()
        self.udpRecvSocket.readyRead.connect(self.process_recv_udp_data_)

        self.udpSendSocket = QUdpSocket(self)

    # control
    #####################################################################################################################
    def zero_wheel_(self):
        for source in ['t_left', 'r_left', 't_right', 'r_right']:
            self.xd_raw_control_state_var['z' + source] = 0

    def graph_control_cb_(self, source, x, y, delta, cmd_type):
        # print("from:", source, "got position", x, y, delta, cmd_type)
        # 'zt_left', 'yt_left', 'xt_left',
        # 'zr_left', 'yr_left', 'xr_left',
        # 'yt_right', 'zt_right', 'xt_right',
        # 'yr_right', 'zr_right', 'xr_right'
        # recalculate value and map direction
        if cmd_type == 'pressed':
            self.control_active = source
            self.control_x_init = x
            self.control_y_init = y
            delta_x = 0
            delta_y = 0
            delta_z = 0
        if cmd_type == 'release':
            self.control_active = None
            delta_x = 0
            delta_y = 0
            delta_z = 0
        if cmd_type == 'move':
            delta_x = (x - self.control_x_init) / PLANE_SCALING
            delta_y = -(y - self.control_y_init) / PLANE_SCALING
            # self.control_x_init = x
            # self.control_y_init = y
            delta_z = 0
            if self.control_active != source:
                self.control_active = None
                delta_x = 0
                delta_y = 0
                delta_z = 0
        if cmd_type == 'wheel':
            delta_z = delta / WHEEL_SCALING
            delta_x = 0
            delta_y = 0

        if source in ['r_left', 'r_right']:
            mlt = ROTATION_SCALING
        else:
            mlt = 1

        self.xd_raw_control_state_var['x' + source] = delta_x * mlt
        self.xd_raw_control_state_var['y' + source] = delta_y * mlt
        self.xd_raw_control_state_var['z' + source] = delta_z * mlt

    #####################################################################################################################

    def zero_all_xd_comonents_(self):
        if self.control_state_var['clutch']:
            print(uiName + "::Cannot zero when clutch is pressed")
        for key in self.xd_raw_control_state_var:
            self.xd_raw_control_state_var[key] = 0
            self.control_state_var[key] = 0
            self.control_state_var[key + "_v"] = 0

    def control_btn_changed_(self, obj, key):
        if obj.isChecked():
            self.control_state_var[key] = True
        else:
            self.control_state_var[key] = False

    def gripper_value_changed_(self, obj, key):
        self.control_state_var[key] = obj.value() / 100.0

    def remote_connect_btn_clicked_(self):

        if self.remoteConnecBtn.isChecked():
            send_ip = self.RemoteAddrEdit.text()
            self.udpSendQAddress = QHostAddress(send_ip)
            if not self.udpSendQAddress.isNull():
                print(uiName + "::Sending on UDP " + send_ip + " Port: " + str(
                    self.init_args.remote_port) + " setup success")
                self.remoteConnected = True
                self.RemoteConnectedStatus.setCheckState(True)
            else:
                print(uiName + "::UDP Send " + send_ip + " Port: " + str(self.init_args.remote_port) + " setup failled")
                self.udpSendQAddress = None
                self.remoteConnecBtn.toggle()
                self.remoteConnected = False
                self.RemoteConnectedStatus.setCheckState(False)
                return
        else:
            print(uiName + "::Reset Remote Address")
            self.remoteConnected = False
            self.udpSendQAddress = None
            self.RemoteConnectedStatus.setCheckState(False)

    def motion_scale_changed_(self):

        self.control_state_var['motionScale'] = float(self.motionScaleEdit.text())

    def process_recv_udp_data_(self):
        while self.udpRecvSocket.hasPendingDatagrams():
            datagram, host, port = self.udpRecvSocket.readDatagram(self.udpRecvSocket.pendingDatagramSize())
            # print("Packet Recv")
            # print(datagram)

    def start_loop_callback(self, update_rate):

        self.updateRate = update_rate
        self.loopTimer.setInterval(int(1000 / update_rate))  # in milliseconds, so 5000 = 5 seconds
        self.loopTimer.timeout.connect(self.timer_callback_)
        self.loopTimer.start()

