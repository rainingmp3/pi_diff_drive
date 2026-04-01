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

#ifndef ROBOT__HARDWARE_INTERFACE_HPP_
#define ROBOT__HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

// My  imports
#include "arduino_comms.h"
#include "config.h"
#include "custom_msgs/msg/pid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid.h"
#include "sensor_msgs/msg/Imu.msg"
#include "setpoint_following.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "wheel.h"

using hardware_interface::return_type;

namespace hardware {

class RobotHardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RobotHardwareInterface)

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

    void publishTwist(float velocity, float angular_velocity);
    void commandVelocity();
    void createPublishersAndSubscribers();
    void importConfigVariables();
    void publishDebugVariables();
    float left_cmd, right_cmd; // for debug

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_publisher_;
    rclcpp::Publisher<sensor_msg::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_cmd_publisher_;
    rclcpp::Publisher<custom_msgs::msg::Pid>::SharedPtr
        pid_linear_debug_publisher_;
    rclcpp::Publisher<custom_msgs::msg::Pid>::SharedPtr
        pid_angular_debug_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        subscription_goal;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    PIDController pid_linear_;
    PIDController pid_angular_;
    Config cfg_;
    Wheel l_wheel_;
    Wheel r_wheel_;
    ArduinoComms arduino_;
    std::chrono::time_point<std::chrono::system_clock> time_;
    hardware::RobotGoalPosition goal_;
    hardware::RobotTelemetry telemetry_;
};

} // namespace hardware

#endif // ROBOT__HARDWARE_INTERFACE_HPP_
