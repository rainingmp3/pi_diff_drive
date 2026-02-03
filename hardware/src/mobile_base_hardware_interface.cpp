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

#include "hardware/mobile_base_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace hardware {

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // My specific changes

    cfg_.left_wheel_name = (info_.hardware_parameters["left_wheel_name"]);
    cfg_.right_wheel_name = (info_.hardware_parameters["right_wheel_name"]);
    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.port = (info_.hardware_parameters["port"]);
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
    cfg_.enc_counts_per_rev =
        std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);

    // Wheels setup
    l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

    // Arduino communication setup
    arduino_.setup(cfg_.port, cfg_.baud_rate, cfg_.timeout);
    if (!arduino_.connected()) {
        RCLCPP_INFO(get_logger(), "No connection to start with");
    } else {

        RCLCPP_INFO(get_logger(), "Finished Configuration");
    };

    // End of my specific changes

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // DiffBotSystem has exactly two states and one command interface on
        // each joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' has %zu state interface. 2 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(
                get_logger(),
                "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name !=
            hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(get_logger(),
                         "Joint '%s' have '%s' as second state interface. '%s' "
                         "expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    debug_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
        "/debug/hardware_interface", 10);

    left_cmd_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
        "/debug/left_cmd", 10);
    right_cmd_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
        "/debug/right_cmd", 10);
    if (!debug_publisher_) {
        RCLCPP_INFO(get_logger(), "Couldn't establish debug publisher");
    }
    // reset values always when configuring hardware
    for (const auto &[name, descr] : joint_state_interfaces_) {
        set_state(name, 0.0);
    }
    for (const auto &[name, descr] : joint_command_interfaces_) {
        set_command(name, 0.0);
    }
    RCLCPP_INFO(get_logger(), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    // command and state should be equal when starting
    for (const auto &[name, descr] : joint_command_interfaces_) {
        set_command(name, get_state(name));
    }

    // NOTE: So, I am not strictly following the controller lifecycle,
    arduino_.sendEmptyMsg();
    // arduino.setPidValues(9,7,0,100);
    // arduino_.setPidValues(14, 7, 0, 1);

    arduino_.setPidValues(cfg_.pid_p, cfg_.pid_i, cfg_.pid_d, cfg_.pid_o);
    set_state(cfg_.left_wheel_name + "/position", 0.0);
    set_state(cfg_.right_wheel_name + "/position", 0.0);
    set_state(cfg_.left_wheel_name + "/velocity", 0.0);
    set_state(cfg_.right_wheel_name + "/velocity", 0.0);

    // RCLCPP_INFO(get_logger(), "!! PID values: %s", pids_log.str().c_str());
    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Successfully deactivated!(NOTHING)");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MobileBaseHardwareInterface::read(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration &period) {
    // Calculate time delta
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    // NOTE: I am gonna comment out this check as excesssive
    // if (!arduino_.connected()) {
    //   return return_type::ERROR;
    // }

#ifdef DEBUG
    std::string read_log = arduino_.readSerial();
    RCLCPP_WARN(get_logger(), "read: %s", read_log.c_str());
#endif // DEBUG

    // NOTE: why end with //DEBUG, what is the philosophy behind it?
    arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

    double pos_prev = l_wheel_.pos;
    l_wheel_.pos = l_wheel_.calcEncAngle();
    l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = r_wheel_.pos;
    r_wheel_.pos = r_wheel_.calcEncAngle();
    r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

    // Tying state variables to controller_manger
    set_state(cfg_.left_wheel_name + "/" + "position", l_wheel_.pos);
    set_state(cfg_.left_wheel_name + "/" + "velocity", l_wheel_.vel);
    set_state(cfg_.right_wheel_name + "/" + "position", r_wheel_.pos);
    set_state(cfg_.right_wheel_name + "/" + "velocity", r_wheel_.vel);

    // RCLCPP_INFO(get_logger(), "left vel %f; right vel %f", l_wheel_.vel,
    //             r_wheel_.vel);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type hardware::MobileBaseHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    l_wheel_.cmd = get_command(cfg_.left_wheel_name + "/" + "velocity");

    r_wheel_.cmd = get_command(cfg_.right_wheel_name + "/" + "velocity");

    float left_cmd_pwm =
        l_wheel_.cmd / (l_wheel_.rads_per_count) / cfg_.loop_rate;

    float right_cmd_pwm =
        r_wheel_.cmd / (r_wheel_.rads_per_count) / cfg_.loop_rate;

    arduino_.setMotorValues(left_cmd_pwm, right_cmd_pwm);

    // Publish debug info
    // auto message = std_msgs::msg::String();
    // message.data = "Skibidi dop..";
    // debug_publisher_->publish(message);

    // Publish left pwm info
    auto left_cmd_log = std_msgs::msg::Int32();
    left_cmd_log.data = left_cmd_pwm;
    left_cmd_publisher_->publish(left_cmd_log);

    // Publish right pwm info
    auto right_cmd_log = std_msgs::msg::Int32();
    right_cmd_log.data = right_cmd_pwm;
    right_cmd_publisher_->publish(right_cmd_log);

    // RCLCPP_INFO(get_logger(), "left cmd %f; loop_rate %f, rads per c %f",
    //             l_wheel_.cmd, cfg_.loop_rate, l_wheel_.rads_per_count);
    return hardware_interface::return_type::OK;
}
} // namespace hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hardware::MobileBaseHardwareInterface,
                       hardware_interface::SystemInterface)
