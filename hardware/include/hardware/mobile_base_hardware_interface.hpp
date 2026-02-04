// Copyright 2021 ros2_control Development Team
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

#ifndef HARDWARE__MOBILE_BASE_HARDWARE_INTERFACE_HPP_
#define HARDWARE__MOBILE_BASE_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

// My  imports
#include "arduino_comms.h"
#include "config.h"
#include "pid.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "wheel.h"

using hardware_interface::return_type;

namespace hardware {

class MobileBaseHardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MobileBaseHardwareInterface)

    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
        override;

    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type
    write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_cmd_publisher_;
    PIDController pid_linear_;
    PIDController pid_angular_;
    Config cfg_;
    Wheel l_wheel_;
    Wheel r_wheel_;
    ArduinoComms arduino_;
    std::chrono::time_point<std::chrono::system_clock> time_;
};

} // namespace hardware

#endif // HARDWARE__MOBILE_BASE_HARDWARE_INTERFACE_HPP_
