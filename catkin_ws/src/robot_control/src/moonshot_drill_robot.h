/*
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
*/
#pragma once

#include <memory>
#include <atomic>
#include <vector>

//DQ Robotics
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

//ROS related
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <sas_robot_driver_denso/sas_robot_driver_denso.h>
#include <sas_robot_driver/sas_robot_driver_provider.h>
#include <sas_clock/sas_clock.h>

#include "MoonshotDrillVrepRobot.h"

struct MoonshotDrillRobotConfiguration
{
    bool use_real_robot;

    bool vrep_readonly;
    bool vrep_send_initial_posture;
    std::string vrep_robot_name;
    std::string vrep_ip;
    int vrep_port;

    std::string robot_parameter_file_path;

    int thread_sampling_time_nsec;

    sas::RobotDriverDensoConfiguration robot_driver_denso_configuration;
};

class MoonshotDrillRobot
{
private:
    //Initialized as default
    ros::CallbackQueue publisher_callback_queue_;
    ros::CallbackQueue subscriber_callback_queue_;

    //Initialized in the member initializer list
    MoonshotDrillRobotConfiguration configuration_;
    std::atomic_bool* kill_this_node_;
    DQ_VrepInterface vrep_interface_;
    sas::RobotDriverDenso robot_driver_denso_;
    sas::Clock clock_;

    //unique_ptr because cannot be initialized in the member initializer list
    std::unique_ptr<sas::RobotDriverProvider> robot_driver_provider_joint_dofs_;
    std::unique_ptr<MoonshotDrillVrepRobot> vrep_robot_;
    // std::unique_ptr<DQ_SerialManipulatorDH> robot_;
    std::unique_ptr<DQ_SerialManipulatorDenso> robot_;

    bool _should_shutdown() const;

    void _initialize();

    void _set_target_joint_positions();
    void _send_joint_positions();
    void _send_joint_limits();

public:
    MoonshotDrillRobot(const MoonshotDrillRobot&)=delete;
    MoonshotDrillRobot()=delete;

    MoonshotDrillRobot(ros::NodeHandle& nodehandle_publisher,
              ros::NodeHandle& nodehandle_subscriber,
              const MoonshotDrillRobotConfiguration& configuration,
              std::atomic_bool* kill_this_node);
    ~MoonshotDrillRobot();

    int control_loop();
    bool is_enabled() const;
};
