/*
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
*/

#include "moonshot_drill_robot.h"

#include <exception>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <sas_conversions/eigen3_std_conversions.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

/*********************************************
 * GLOBAL SCOPE FUNCTIONS (INCLUDING MAIN)
 * *******************************************/
MoonshotDrillRobotConfiguration get_configuration_from_ros_parameter_server()
{
    ros::NodeHandle nodehandle;

    MoonshotDrillRobotConfiguration configuration;

    ROS_INFO_STREAM("Trying to load Denso Communication Node parameters for node " << ros::this_node::getName());

    if(!nodehandle.getParam(ros::this_node::getName()+"/use_real_robot",configuration.use_real_robot)){throw std::runtime_error(ros::this_node::getName() + "Error loading use_real_robot");}

    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_readonly",configuration.vrep_readonly)){throw std::runtime_error(ros::this_node::getName() + "Error loading vrep_readonly");}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_send_initial_posture",configuration.vrep_send_initial_posture)){throw std::runtime_error(ros::this_node::getName() + "Error loading vrep_send_initial_posture");}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_robot_name",configuration.vrep_robot_name)){throw std::runtime_error(ros::this_node::getName() + "Error loading vrep_robot_name");}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_ip",configuration.vrep_ip)){throw std::runtime_error(ros::this_node::getName() + "Error loading vrep_ip");}
    if(!nodehandle.getParam(ros::this_node::getName()+"/vrep_port",configuration.vrep_port)){throw std::runtime_error(ros::this_node::getName() + "Error loading vrep_port");}

    if(!nodehandle.getParam(ros::this_node::getName()+"/robot_parameter_file_path",configuration.robot_parameter_file_path)){throw std::runtime_error(ros::this_node::getName() + "Error loading robot_parameter_file_path");}

    if(!nodehandle.getParam(ros::this_node::getName()+"/thread_sampling_time_nsec",configuration.thread_sampling_time_nsec)){throw std::runtime_error(ros::this_node::getName() + "Error loading thread_sampling_time_nsec");}

    if(!nodehandle.getParam(ros::this_node::getName()+"/robot_ip_address",configuration.robot_driver_denso_configuration.ip_address)){throw std::runtime_error(ros::this_node::getName() + "Error loading robot_ip_address");}
    if(!nodehandle.getParam(ros::this_node::getName()+"/robot_port",configuration.robot_driver_denso_configuration.port)){throw std::runtime_error(ros::this_node::getName() + "Error loading robot_port");}
    if(!nodehandle.getParam(ros::this_node::getName()+"/robot_speed",configuration.robot_driver_denso_configuration.speed)){throw std::runtime_error(ros::this_node::getName() + "Error loading robot_speed");}

    ROS_INFO_STREAM(ros::this_node::getName() + "::Parameter load OK.");
    return configuration;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "arm1", ros::init_options::NoSigintHandler);
    ros::NodeHandle nodehandle_publisher;
    ros::NodeHandle nodehandle_subscriber;

    try
    {
        MoonshotDrillRobotConfiguration configuration = get_configuration_from_ros_parameter_server();
        MoonshotDrillRobot moonshot_drill_robot(nodehandle_publisher, nodehandle_subscriber, configuration, &kill_this_process);
        moonshot_drill_robot.control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }

    return 0;
}
