// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define MAXLINE 4096

#include "ros2_control_frcobot/frcobot_system.hpp"
#include <math.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_frcobot
{
    hardware_interface::CallbackReturn FrCobotSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        memset(&serverSendAddr, 0, sizeof(serverSendAddr));
        serverSendAddr.sin_family = AF_INET;
        serverSendAddr.sin_addr.s_addr = inet_addr(info_.hardware_parameters["server_address"].c_str());
        serverSendAddr.sin_port = htons(std::stoi(info_.hardware_parameters["server_host"]));
        sendaddr_length = sizeof(serverSendAddr);

        if ((confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("FrCobotSystemHardware"), "socket() error.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (connect(confd, (struct sockaddr *)&serverSendAddr, sizeof(serverSendAddr)) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("FrCobotSystemHardware"), "connect() error.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("FrCobotSystemHardware"), "Connected to server. FrRobotHWInterface Ready.");

        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // FrCobotSystem has exactly three states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("FrCobotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("FrCobotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 3)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("FrCobotSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("FrCobotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("FrCobotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("FrCobotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        recieveJointData("GetActualJointPosRadian", position_count_, hw_commands_); // Setting current position commands to current robot position.

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> FrCobotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> FrCobotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn FrCobotSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        // set some default values
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_efforts_[i] = 0;
                hw_commands_[i] = 0;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("FrCobotSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FrCobotSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        RCLCPP_INFO(rclcpp::get_logger("FrCobotSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type FrCobotSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        hardware_interface::return_type status;

        status = recieveJointData("GetActualJointPosRadian", position_count_, hw_positions_);
        status = recieveJointData("GetActualJointSpeedsRadian", velocity_count_, hw_velocities_);
        status = recieveJointData("GetJointTorques", effort_count_, hw_efforts_);

        return status;
    }

    hardware_interface::return_type FrCobotSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {

        // enforceLimits(period);

        sprintf(send_buf, "ServoJ(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)", hw_commands_[0] * 180 / M_PI, hw_commands_[1] * 180 / M_PI, hw_commands_[2] * 180 / M_PI, hw_commands_[3] * 180 / M_PI, hw_commands_[4] * 180 / M_PI, hw_commands_[5] * 180 / M_PI, 0.0, 0.0, 0.002, 0.002, 0.0);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("FrCobotSystemHardware"), "ServoJ Command: " << send_buf);
        len = strlen(send_buf);
        sprintf(sendCmdLine, "/f/bIII%dIII376III%dIII%sIII/b/f", command_count_, len, send_buf);
        command_count_++;

        int send_length = 0;
        send_length = send(confd, sendCmdLine, sizeof(sendCmdLine), 0);
        if (send_length < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("FrCobotSystemHardware"), "send() error");
            return hardware_interface::return_type::ERROR;
        }
        int recv_length = 0;
        recvLine[MAXLINE] = '\0';
        recv_length = recv(confd, recvLine, sizeof(recvLine), 0);
        if (recv_length <= 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("FrCobotSystemHardware"), "recv() error");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type FrCobotSystemHardware::recieveJointData(std::string command, uint16_t &count, std::vector<double> &data)
    {

        int send_length_sta = 0;

        len = strlen(command.c_str());
        sprintf(sendStaLine, "/f/bIII%dIII377III%dIII%s()III/b/f", count, len + 2, command.c_str());
        count++;
        send_length_sta = send(confd, sendStaLine, sizeof(sendStaLine), 0);
        if (send_length_sta < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FrCobotSystemHardware"), "sendto() error.");
            return hardware_interface::return_type::ERROR;
        }

        int recv_length = 0;
        recvLine[MAXLINE] = '\0';
        recv_length = recv(confd, recvLine, sizeof(recvLine), 0);
        if (recv_length <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FrCobotSystemHardware"), "recv() error.");
            return hardware_interface::return_type::ERROR;
        }

        int pos = 0;
        int jointsDataLen = 0;
        std::string tempJoints = recvLine;
        for (int i = 0; i < 3; i++)
        {
            pos = tempJoints.find("III") + 3;
            tempJoints = tempJoints.substr(pos);
        }
        pos = tempJoints.find("III");
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("FrCobotSystemHardware"), "Test: " << "/f/bIII" << count << "III377III" << len + 2 << "III" << command << "()III/b/f");
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("FrCobotSystemHardware"), "Joints: " << tempJoints);
        try
        {
            jointsDataLen = stoi(tempJoints.substr(0, pos));
            tempJoints = tempJoints.substr(pos + 3, jointsDataLen);
            for (int i = 0; i < 6; i++)
            {
                pos = tempJoints.find(",");
                data[i] = stof(tempJoints.substr(0, pos));
                tempJoints = tempJoints.substr(pos + 1);
            }
            return hardware_interface::return_type::OK;
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("FrCobotSystemHardware"), "Return data malformed. Should be list of six joint values, is '" << tempJoints << "'");
            return hardware_interface::return_type::ERROR;
        }
    }

    // void FrCobotSystemHardware::enforceLimits(rclcpp::Duration &period)
    // {
    //     // ----------------------------------------------------
    //     // ----------------------------------------------------
    //     // ----------------------------------------------------
    //     //
    //     // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    //     // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    //     // DEPENDING ON YOUR CONTROL METHOD
    //     //
    //     // EXAMPLES:
    //     //
    //     // Saturation Limits ---------------------------
    //     //
    //     // Enforces position and velocity
    //     pos_jnt_sat_interface_.enforceLimits(period);
    //     //
    //     // Enforces velocity and acceleration limits
    //     // vel_jnt_sat_interface_.enforceLimits(period);
    //     //
    //     // Enforces position, velocity, and effort
    //     // eff_jnt_sat_interface_.enforceLimits(period);

    //     // Soft limits ---------------------------------
    //     //
    //     // pos_jnt_soft_limits_.enforceLimits(period);
    //     // vel_jnt_soft_limits_.enforceLimits(period);
    //     // eff_jnt_soft_limits_.enforceLimits(period);
    //     //
    //     // ----------------------------------------------------
    //     // ----------------------------------------------------
    //     // ----------------------------------------------------
    // }

} // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    ros2_control_frcobot::FrCobotSystemHardware, hardware_interface::SystemInterface)