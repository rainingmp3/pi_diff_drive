#ifndef HARDWARE__CONFIG_H_
#define HARDWARE__CONFIG_H_

#include <string>

struct Config {
    std::string left_wheel_name = "left_wheel";
    std::string right_wheel_name = "right_wheel";
    float loop_rate = 30.0;
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 57600;
    int timeout = 1000;
    int enc_counts_per_rev = 20;
    float pid_p = 30;
    float pid_i = 20;
    float pid_d = 0;
    float pid_o = 100;
    float linear_pid_p = 1;
    float linear_pid_i = 0;
    float linear_pid_d = 0;
    float linear_pid_max_input = 1;
    float linear_pid_max_windup = 1;
    float angular_pid_p = 1;
    float angular_pid_i = 0;
    float angular_pid_d = 0;
    float angular_pid_max_input = 1;
    float angular_pid_max_windup = 1;
};

#endif // HARDWARE__CONFIG_H_
