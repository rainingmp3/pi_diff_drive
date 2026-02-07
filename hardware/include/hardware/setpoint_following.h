#ifndef SETPOINT_FOLLOWING__H
#define SETPOINT_FOLLOWING__H
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

const float PI = 3.14159265359;
namespace hardware {

typedef struct {
    float goal_position_x;
    float goal_position_y;
    float goal_position_z;
    float goal_yaw;

    float goal_qx;
    float goal_qy;
    float goal_qz;
    float goal_qw;
} RobotGoalPosition;

typedef struct {

    float position_x;
    float position_y;
    float position_z;

    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
    float orientation_yaw;
} RobotTelemetry;

inline float wrapAngle(float angle_to_wrap) {
    float wrapped_angle = angle_to_wrap;
    if (angle_to_wrap > PI / 2)
        return wrapped_angle = angle_to_wrap - PI;
    else if (angle_to_wrap < -PI / 2)
        return wrapped_angle = angle_to_wrap + PI;
    else
        return wrapped_angle;
}

inline float quatToYaw(float qx, float qy, float qz, float qw) {
    float yaw = atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)); // TODO: learn the derivation pls
    return yaw;
}

} // namespace hardware
#endif // !SETPOINT_FOLLOWING__H
