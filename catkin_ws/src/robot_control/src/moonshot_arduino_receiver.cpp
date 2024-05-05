#include "moonshot_arduino_receiver.h"
#include <sas_clock/sas_clock.h>

namespace moonshot
{

ArduinoReceiver::ArduinoReceiver(ros::NodeHandle& node_handle,
                                 const ArduinoReceiverConfiguration &cfg, std::atomic_bool *kill_this_node):
    node_handle_(node_handle),
    cfg(cfg),
    operator_side_receiver_interface_(node_handle),
    kill_this_node_(kill_this_node)
{
    serial_.setPortName(QString::fromStdString(cfg.serial_port));
    serial_.setBaudRate(QSerialPort::Baud115200);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    operator_side_receiver_interface_.add_manipulator_manager(cfg.master_manipulator_label);
}

void ArduinoReceiver::loop()
{
    //Initialize serial port
    if(!serial_.open(QIODevice::ReadWrite))
    {
        throw std::runtime_error(ros::this_node::getName() + cfg.serial_port + ".");
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName() << "::" << "Connected to Arduino at " << cfg.serial_port << ".");
    }

    //Initialize clock loop
    sas::Clock clock_(cfg.sampling_time_sec*sas::NSEC_TO_SEC_D);
    clock_.init();

    auto manipulator_manager = operator_side_receiver_interface_.get_manipulator_manager_ptr(cfg.master_manipulator_label);

    while(not *kill_this_node_)
    {
        if(manipulator_manager->is_enabled())
        {
            if(manipulator_manager->is_clutch_pressed() and
                    manipulator_manager->get_button() == 1.0)
            {
                QString command = QString("1\n");
                QByteArray x = command.toLocal8Bit();
                serial_.write(x);
                serial_.waitForBytesWritten(10);
            }
            else
            {
                QString command = QString("0\n");
                QByteArray x = command.toLocal8Bit();
                serial_.write(x);
                serial_.waitForBytesWritten(10);
            }
        }
        ros::spinOnce();
        clock_.update_and_sleep();
        ros::spinOnce();
    }
}

ArduinoReceiver::~ArduinoReceiver()
{
    serial_.close();
}

}


