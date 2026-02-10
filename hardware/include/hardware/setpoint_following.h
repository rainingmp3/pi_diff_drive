#ifndef SETPOINT_FOLLOWING__H
#define SETPOINT_FOLLOWING__H
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

const float PI = 3.14159265359;
namespace hardware {

typedef struct {
    float goal_position_x = 0;
    float goal_position_y = 0;
    float goal_position_z = 0;
    float goal_yaw = 0;

    float goal_qx = 0;
    float goal_qy = 0;
    float goal_qz = 0;
    float goal_qw = 0;
} RobotGoalPosition;

typedef struct {
    float position_x = 0;
    float position_y = 0;
    float position_z = 0;

    float orientation_x = 0;
    float orientation_y = 0;
    float orientation_z = 0;
    float orientation_w = 0;
    float orientation_yaw = 0;
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
