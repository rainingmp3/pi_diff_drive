#include "hardware/pid.h"
#include <algorithm>

PIDController::PIDController() {}

void PIDController::setupPID(float kp, float ki, float kd, float max_input,
                             float max_windup) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_input_ = max_input;
    max_windup_ = max_windup;
}

float PIDController::computeControl(float setpoint, float current_state,
                                    float time_step) {
    float error = setpoint - current_state;
    float derivative_error = (error - prev_error_) / time_step;
    integral_error_ += error * time_step;
    integral_error_ = std::clamp(integral_error_, -max_windup_, max_windup_);

    float control_input =
        derivative_error * kd_ + integral_error_ * ki_ + error * kp_;
    control_input = std::clamp(control_input, -max_input_, max_input_);
    prev_error_ = error;
    return control_input;
}
