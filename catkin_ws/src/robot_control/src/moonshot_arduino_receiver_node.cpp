#include "moonshot_arduino_receiver.h"
#include <ros/ros.h>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "moonshot_arduino_receiver_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodehandle;

    try {
        moonshot::ArduinoReceiverConfiguration cfg;
        if(!nodehandle.getParam(ros::this_node::getName()+"/serial_port",cfg.serial_port))
        {
            throw std::runtime_error(ros::this_node::getName() + "::Error loading serial_port");
        }

        if(!nodehandle.getParam(ros::this_node::getName()+"/sampling_time_sec",cfg.sampling_time_sec))
        {
            throw std::runtime_error(ros::this_node::getName() + "::Error loading sampling_time_sec");
        }

        if(!nodehandle.getParam(ros::this_node::getName()+"/master_manipulator_label",cfg.master_manipulator_label))
        {
            throw std::runtime_error(ros::this_node::getName() + "::Error loading master_manipulator_label");
        }

        moonshot::ArduinoReceiver arduino_receiver(nodehandle, cfg, &kill_this_process);
        arduino_receiver.loop();
    }  catch (const std::exception& e) {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Error::" << e.what());
    }

    return 0;
}
