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
  int pid_p = 30;
  int pid_i = 20;
  int pid_d = 0;
  int pid_o = 100;
};

#endif // HARDWARE__CONFIG_H_
