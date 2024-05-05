#pragma once
/*
# ################################################################
# All rights reserved. This file is confidential.
#   Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
# ################################################################
*/

#include <memory>

#include <QSerialPort>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <sas_operator_side_receiver/sas_operator_side_receiver_interface.h>

namespace moonshot
{

struct ArduinoReceiverConfiguration
{
    std::string serial_port;
    double sampling_time_sec;
    std::string master_manipulator_label;
};

class ArduinoReceiver
{
private:
    ros::NodeHandle& node_handle_;
    QSerialPort serial_;
    const ArduinoReceiverConfiguration cfg;
    sas::OperatorSideReceiverInterface operator_side_receiver_interface_;
    std::atomic_bool* kill_this_node_;

    ros::Subscriber activate_subscriber_;
    bool activate_;
    void _activate_callback(const std_msgs::Bool::ConstPtr& msg);
public:
    ArduinoReceiver(ros::NodeHandle& node_handle,
                    const ArduinoReceiverConfiguration& cfg,
                    std::atomic_bool* kill_this_node);
    ~ArduinoReceiver();

    void loop();
};

}
