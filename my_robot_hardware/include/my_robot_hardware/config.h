#ifndef MY_ROBOT_HARDWARE__CONFIG_H_
#define MY_ROBOT_HARDWARE__CONFIG_H_

#include <string>

struct Config {
  std::string left_wheel_name = "left_wheel";
  std::string right_wheel_name = "right_wheel";
  float loop_rate = 30.0;
  std::string port = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;
  int enc_counts_per_rev = 20;
};

#endif // MY_ROBOT_HARDWARE__CONFIG_H_
