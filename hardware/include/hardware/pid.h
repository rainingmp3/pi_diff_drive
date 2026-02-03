#pragma once

class PIDController {
  public:
    PIDController();
    void setupPID(float kp, float ki, float kd, float max_input,
                  float max_windup);
    float computeControl(float setpoint, float current_state, float time_step);

  private:
    float kp_, ki_, kd_;
    float max_windup_, max_input_;
    float integral_error_{0.0f}, prev_error_{0.0f};
};
