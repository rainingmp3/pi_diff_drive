#ifndef MY_ROBOT_HARDWARE__ARDUINO_COMMS_H_
#define MY_ROBOT_HARDWARE__ARDUINO_COMMS_H_

#include <cstring>
#include <serial/serial.h>

class ArduinoComms {

public:
  ArduinoComms() {}

  std::string serial_message_;
  ArduinoComms(const std::string &serial_device, int32_t baud_rate,
               int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate,
                     serial::Timeout::simpleTimeout(timeout_ms)) {}

  void setup(const std::string &serial_device, int32_t baud_rate,
             int32_t timeout_ms);
  std::string sendEmptyMsg();
  std::string readEncoderValues(int &val_1, int &val_2);
  std::stringstream setMotorValues(double val_1, double val_2);
  std::stringstream setPidValues(float k_p, float k_d, float k_i, float k_o);
  std::string readSerial();
  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send,
                      bool print_output = false);

private:
  serial::Serial serial_conn_; ///< Underlying serial connection
};
#endif // MY_ROBOT_HARDWARE__ARDUINO_COMMS_H_
