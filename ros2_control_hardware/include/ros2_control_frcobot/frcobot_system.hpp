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

#ifndef ROS2_CONTROL_FRCOBOT__FRCOBOT_SYSTEM_HPP
#define ROS2_CONTROL_FRCOBOT__FRCOBOT_SYSTEM_HPP

#define MAXLINE 4096

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ros2_control_frcobot/visibility_control.h"

#include <sys/socket.h>
#include <arpa/inet.h>

namespace ros2_control_frcobot
{
    class FrCobotSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FrCobotSystemHardware);

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        virtual hardware_interface::return_type recieveJointData(std::string command, std::vector<double> &data);

        // virtual void enforceLimits(rclcpp::Duration &period);

    private:
        int confd;
        int len;
        int flag = 1;
        socklen_t sendaddr_length;
        char recvLine[MAXLINE];
        char sendCmdLine[MAXLINE];
        char sendStaLine[MAXLINE];
        char recv_buf[MAXLINE];
        char send_buf[MAXLINE];
        struct sockaddr_in serverSendAddr;

        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;

        // uint16_t command_count_ = 123;
        // uint16_t position_count_ = 123;
        // uint16_t velocity_count_ = 123;
        // uint16_t effort_count_ = 123;
    };

} // namespace ros2_control_frcobot

#endif // ROS2_CONTROL_FRCOBOT__FRCOBOT_SYSTEM_HPP