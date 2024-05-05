#!/usr/bin/python3

# Imports
import rospy
import serial
from serial.tools import list_ports

import time

from std_msgs.msg import Bool


def node_manager():
    name = config['name']

    # get serial port information
    if config['serial_port'] is None:
        devices_list = list_ports.comports()
        port = None
        for device in devices_list:
            if device.vid == config['device_vid'] and device.pid == config['device_pid']:
                port = device.device
                rospy.loginfo("[" + name + "]::Serial device with " + '{0:0{1}X}'.format(config['device_vid'], 4)
                              + ":" + '{0:0{1}X}'.format(config['device_pid'], 4) +
                              " found at: " + port)
        if port is None:
            rospy.logerr("[" + name + "]::No valid serial port was found")
            raise RuntimeError("No valid serial port information was defined")
    else:
        port = config['serial_port']

    # Initialize serial connection
    ser = serial.Serial(port, baudrate=config['baud_rate'], bytesize=serial.EIGHTBITS,
                        stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, rtscts=False, dsrdtr=False)

    pub_topic_h = rospy.Publisher(config['pub_topic'], Bool, queue_size=1)

    # r = rospy.Rate(config['rate'])  # 60hz
    try:
        ser.flush()
        while not rospy.is_shutdown():

            # if sigw_h.get_pedal_timeout():
            #     if sigw_h.get_pedal():
            #         if sigw_h.get_auto_control() is None or sigw_h.get_auto_control():
            #             status = 'g'
            #         else:
            #             status = 'b'
            #     else:
            #         status = 'y'
            # else:
            #     status = 'r'

            try:
                in_byte = ser.readline()
                try:
                    ret = str(in_byte,'utf-8').strip()
                except Exception:
                    ret = 'N'
                # rospy.logwarn("[" + name + "]::"+ret)
                pub_topic_h.publish(Bool(data=ret=='C'))

                # ret = ser.write(bytes(status, 'utf-8'))

            except serial.SerialTimeoutException as e:
                rospy.logerr("[" + name + "]::Serial Port error")
                rospy.logerr("[" + name + "]::" + str(e))
                break

            # r.sleep()
    except KeyboardInterrupt:
        rospy.logwarn("[" + name + "]::Exit on KeyboardInterrupt")
    except Exception as e:
        rospy.logwarn("[" + name + "]::Exit on un-handled error"+str(e))

    ser.close()


if __name__ == '__main__':
    rospy.init_node('contact_reporter_node', anonymous=True, disable_signals=True)

    name = rospy.get_name()
    params = rospy.get_param(name)

    config = {}

    try:
        config['device_pid'] = params.get("serial_device_pid", None)
        config['device_vid'] = params.get("serial_device_vid", None)
        if config['device_pid'] is None or config['device_vid'] is None:
            rospy.logwarn("[" + name + "]::Falling back to using serial path")
            config['serial_port'] = params['serial_port']
        else:
            config['serial_port'] = None

        config['baud_rate'] = params['baud_rate']

        # config['rate'] = params['rate']

        # define signal topics
        config['pub_topic'] = params['pub_topic']

        rospy.loginfo("[" + name + "]::Node parameter Done")
    except KeyError:
        rospy.logerr("[" + name + "]::Node parameter error")
        rospy.signal_shutdown("[" + name + "]::Initialization end on parameter error")
        exit()

    ns = rospy.get_namespace()

    config.update({"ns": ns, "name": name})

    node_manager()
