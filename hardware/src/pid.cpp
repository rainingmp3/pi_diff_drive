#include "hardware/pid.h"

void PIDController::setupPID(float kp, float ki, float kd, float time_step,
                             float max_input, float max_windup) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    time_step_ = time_step;
    max_input_ = max_input;
    max_windup_ = max_windup;
};

float PIDController::computeControl(float setpoint, float current_state) {
    error = setpoint - current_state;
    derivative_error = (current_state - prev_state_) / time_step_;
    integral_error_ += error * time_step_;
    integral_error_ = std::clamp(integral_error_, -max_windup_, max_windup_);

    control_input =
        derivative_error * kd_ + integral_error_ * ki_ + error * kp_;
    control_input = std::clamp(control_input, -max_input_, max_input_);
    prev_error_ = error;
    prev_state_ = current_state;
    // TODO: from within ros cpp publish those
    RCLCPP_INFO(logger_, "error: %f, kp:%f, max_input:%f, control_input:%f",
                error, kp_, max_input_, control_input);
    return control_input;
}
