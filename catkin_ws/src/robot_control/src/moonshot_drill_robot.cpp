/*
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################
*/
#include "moonshot_drill_robot.h"
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

MoonshotDrillRobot::MoonshotDrillRobot(ros::NodeHandle &nodehandle_publisher, ros::NodeHandle &nodehandle_subscriber, const MoonshotDrillRobotConfiguration &configuration, std::atomic_bool *kill_this_node):
    configuration_(configuration),
    kill_this_node_(kill_this_node),
    vrep_interface_(kill_this_node),
    robot_driver_denso_(configuration.robot_driver_denso_configuration,kill_this_node),
    clock_(configuration.thread_sampling_time_nsec)
{
    //The callback queue has to be specified before using the nodehandles.
    nodehandle_publisher.setCallbackQueue(&publisher_callback_queue_);
    nodehandle_subscriber.setCallbackQueue(&subscriber_callback_queue_);

//    robot_kinematics_provider_.reset(new sas::RobotKinematicsProvider(nodehandle_publisher, nodehandle_subscriber, configuration.sas_robot_kinematics_prefix));
    //All members that use those nodehandles are unique_ptr because we can only call the constructor after the callback queues are set.
    robot_driver_provider_joint_dofs_.reset(new sas::RobotDriverProvider(nodehandle_publisher, nodehandle_subscriber, ros::this_node::getName()+"/joints/"));
}

/**
 * @brief Using the desired joint values obtained from ROS, send them to the robot, the forceps, and CoppeliaSim.
 */
void MoonshotDrillRobot::_set_target_joint_positions()
{
    if(is_enabled())
    {
        //Robot
        VectorXd desired_joint_positions_vectorxd = robot_driver_provider_joint_dofs_->get_target_joint_positions();

        if(configuration_.use_real_robot)
        {
            //Robot
            robot_driver_denso_.set_target_joint_positions(desired_joint_positions_vectorxd);

        }
        //If not using real robot, send to CoppeliaSim
        else
        {
            if(not configuration_.vrep_readonly)
            {
                vrep_robot_->send_q_to_vrep(desired_joint_positions_vectorxd);
            }
        }
    }
}

/**
 * @brief Send the current joint values obtained from the robot and the forceps (or CoppeliaSim) to ROS.
 */
void MoonshotDrillRobot::_send_joint_positions()
{
    VectorXd joint_positions_vectorxd(6);
    if(configuration_.use_real_robot)
    {
        //Robot and forceps rotation
        joint_positions_vectorxd << robot_driver_denso_.get_joint_positions();

        //If using real robot, update CoppeliaSim with the current joint values.
        if(not configuration_.vrep_readonly)
        {
            vrep_robot_->send_q_to_vrep(joint_positions_vectorxd);
        }
    }
    else
    {
        joint_positions_vectorxd = vrep_robot_->get_q_from_vrep();
    }
    robot_driver_provider_joint_dofs_->send_joint_positions(joint_positions_vectorxd);
}

void MoonshotDrillRobot::_send_joint_limits()
{
    robot_driver_provider_joint_dofs_->send_joint_limits(std::tuple<VectorXd, VectorXd>(robot_->get_lower_q_limit(), robot_->get_upper_q_limit()));
}

int MoonshotDrillRobot::control_loop()
{
    try{
        _initialize();
        ROS_INFO_STREAM(ros::this_node::getName() + ":: initialized");
        while(! _should_shutdown())
        {
            clock_.update_and_sleep();

            subscriber_callback_queue_.callAvailable();
            _set_target_joint_positions();

            _send_joint_positions();
            _send_joint_limits();
//            ROS_INFO_STREAM(ros::this_node::getName() + ":: send reference frame");
            publisher_callback_queue_.callAvailable();
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Error or exception caught::" << e.what());
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Unexpected error or exception caught");
    }

    return 0;
}

bool MoonshotDrillRobot::is_enabled() const
{
    return robot_driver_provider_joint_dofs_->is_enabled();
}

bool MoonshotDrillRobot::_should_shutdown() const
{
    return (*kill_this_node_);
}

void MoonshotDrillRobot::_initialize()
{
    ROS_INFO_STREAM(ros::this_node::getName() << "::Getting robot information from: " << configuration_.robot_parameter_file_path);
    //This is a unique_ptr because of the very delayed constructor.
    // robot_ = std::make_unique<DQ_SerialManipulatorDH>(DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(configuration_.robot_parameter_file_path));
    robot_ = std::make_unique<DQ_SerialManipulatorDenso>(DQ_JsonReader::get_from_json<DQ_SerialManipulatorDenso>(configuration_.robot_parameter_file_path));
    ROS_INFO_STREAM(ros::this_node::getName() << "::Obtained robot information with DoF =" << std::to_string(robot_->get_dim_configuration_space()));

    ROS_INFO_STREAM(ros::this_node::getName() << "::Connecting to VREP...");
    if(!vrep_interface_.connect(configuration_.vrep_ip,
                                configuration_.vrep_port,
                                100,
                                10))
    {
//        throw std::runtime_error(ros::this_node::getName()+"::Unable to connect to VREP.");
        throw std::runtime_error(ros::this_node::getName() + "::Unable to connect to VREP ip" +
                                 configuration_.vrep_ip +
                                 " port " +
                                 std::to_string(configuration_.vrep_port));

    }
    vrep_robot_.reset(new MoonshotDrillVrepRobot(configuration_.vrep_robot_name, &vrep_interface_));
    ROS_INFO_STREAM(ros::this_node::getName() << "::Connected to VREP.");

    clock_.init();

    if(configuration_.use_real_robot)
    {
        ROS_INFO_STREAM(ros::this_node::getName() << "::Waiting to connect with robot (over bCap)...");
        robot_driver_denso_.connect();
        robot_driver_denso_.initialize();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Connected to robot.");
    }
}

MoonshotDrillRobot::~MoonshotDrillRobot()
{
    robot_driver_denso_.deinitialize();
    robot_driver_denso_.disconnect();
    vrep_interface_.disconnect();
}
