#ifndef HARDWARE__ARDUINO_COMMS_H_
#define HARDWARE__ARDUINO_COMMS_H_

#include <cstdlib>
#include <cstring>
#include <rclcpp/logging.hpp>
#include <serial/serial.h>

class ArduinoComms {

  public:
    ArduinoComms() : logger_(rclcpp::get_logger("arduino_logs")) {};
    ArduinoComms(const std::string &serial_device, int32_t baud_rate,
                 int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate,
                       serial::Timeout::simpleTimeout(timeout_ms)),
          logger_(rclcpp::get_logger("arduino_logs")) {}

    void setup(const std::string &serial_device, int32_t baud_rate,
               int32_t timeout_ms);
    std::string sendEmptyMsg();
    void readEncoderValues(int &val_1, int &val_2);
    void setMotorValues(double val_1, double val_2);
    void setPidValues(float k_p, float k_d, float k_i, float k_o);
    std::string readSerial();
    bool connected() const { return serial_conn_.isOpen(); }

    std::string sendMsg(const std::string &msg_to_send,
                        bool print_output = false);

  private:
    serial::Serial serial_conn_;
    rclcpp::Logger logger_;
};
#endif // HARDWARE__ARDUINO_COMMS_H_
