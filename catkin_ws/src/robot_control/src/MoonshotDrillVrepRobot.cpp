/**
(C) Copyright 2020 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
*/
#include "MoonshotDrillVrepRobot.h"

namespace DQ_robotics
{

MoonshotDrillVrepRobot::MoonshotDrillVrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface): DQ_VrepRobot(robot_name, vrep_interface)
{
    std::vector<std::string> splited_name = _strsplit(robot_name_,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string("VS050")) != 0)
    {
        std::runtime_error("Expected VS050");
    }

    std::string robot_index("");
    if(splited_name.size() > 1)
        robot_index = splited_name[1];

    for(int i=1;i<7;i++)
    {
        std::string current_joint_name = robot_label + std::string("_joint") + std::to_string(i) + std::string("#") + robot_index;
        joint_names_.push_back(current_joint_name);
    }

    reference_frame_name_ = (robot_label + std::string("_reference#") + robot_index);
}


void MoonshotDrillVrepRobot::send_q_to_vrep(const VectorXd &q)
{
    vrep_interface_->set_joint_positions(joint_names_,q);
}

VectorXd MoonshotDrillVrepRobot::get_q_from_vrep()
{
    return vrep_interface_->get_joint_positions(joint_names_);
}

DQ MoonshotDrillVrepRobot::get_reference_frame() const
{
    return vrep_interface_->get_object_pose(reference_frame_name_);
}

void MoonshotDrillVrepRobot::set_configuration_space_positions(const VectorXd& q)
{
    configuration_space_positions_ = q;
}

VectorXd MoonshotDrillVrepRobot::get_configuration_space_positions()
{
    return configuration_space_positions_;
}

}
