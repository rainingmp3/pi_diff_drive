#include "hardware/arduino_comms.h"

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate,
                         int32_t timeout_ms) {
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout
                                 // takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate,
    // serial::Timeout::simpleTimeout(timeout_ms));
}

std::string ArduinoComms::sendEmptyMsg() {
    std::string response = sendMsg("\r");
    return response;
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2) {
    std::string response = sendMsg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}
void ArduinoComms::readImuValues(int imu_vector[]) {
    std::string response = sendMsg("i\r");
    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    int init_index = 0;
    int values[6]; // 3xaccel 3xgyro

    for (int i = 0; i < 6; i++) {
        std::string token =
            response.substr(init_index, del_pos); // get a value's string
        float value = atof(token.c_str);          // str->float
        values[i] = value;                        // assign value to the array

        init_index = del_pos + 1;
        del_pos = response.find(delimeter, init_index)
    }
}

std::string ArduinoComms::readSerial() {
    if (serial_conn_.available()) {

        std::string serial_message = serial_conn_.readline();
        return serial_message;
    } else
        return "Serial is empty!";
}

void ArduinoComms::setMotorValues(double val_1, double val_2) {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o) {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send,
                                  bool print_output) {
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output) {
        // RCLCPP_INFO_STREAM(get_logger(),"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(get_logger(),"Received: " << response);
    }

    return response;
}
