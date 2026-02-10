#ifndef HARDWARE__PID_H_
#define HARDWARE__PID_H_

#include <algorithm>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

class PIDController {
  public:
    PIDController() : logger_(rclcpp::get_logger("pid_logs")) {};
    float computeControl(float setpoint, float current_state);
    void setupPID(float kp, float ki, float kd, float time_step,
                  float max_input, float max_windup);

    float kp_ = 0;
    float ki_ = 0;
    float kd_ = 0;
    float time_step_ = 0;
    float prev_state_ = 0;
    float prev_error_ = 0;
    float integral_error_ = 0;
    float max_windup_ = 0;
    float max_input_ = 0;

  private:
    rclcpp::Logger logger_;
};
#endif // !HARDWARE__PID_H_
