//************************************
//       ToFGeometry.cpp
//************************************
// Shared ToF sensor geometry implementation

#include "ToFGeometry.h"
#include <math.h>

#define PI_F 3.14159265358979f

void ToFGeometry::getSensorWorldFrame(
    const RobotPose &robot_pose,
    const ToFExtrinsics &tof_extrinsics,
    float &sensor_x_m,
    float &sensor_y_m,
    float &sensor_yaw_rad
) {
    // Robot pose is in mm and rad
    float robot_x_m = robot_pose.x_mm / 1000.0f;
    float robot_y_m = robot_pose.y_mm / 1000.0f;
    float robot_theta = robot_pose.theta_rad;
    
    // Sensor offset is relative to robot center, in robot frame
    float offset_x_m = tof_extrinsics.x_offset_m;  // forward
    float offset_y_m = tof_extrinsics.y_offset_m;  // left
    
    // Rotate sensor offset to world frame
    float cos_th = cosf(robot_theta);
    float sin_th = sinf(robot_theta);
    
    float offset_x_world = offset_x_m * cos_th - offset_y_m * sin_th;
    float offset_y_world = offset_x_m * sin_th + offset_y_m * cos_th;
    
    // Sensor position in world frame
    sensor_x_m = robot_x_m + offset_x_world;
    sensor_y_m = robot_y_m + offset_y_world;
    
    // Sensor beam yaw in world frame
    sensor_yaw_rad = robot_theta + tof_extrinsics.yaw_offset_rad;
    
    // Normalize to [-pi, pi]
    while (sensor_yaw_rad > PI_F) sensor_yaw_rad -= 2.0f * PI_F;
    while (sensor_yaw_rad <= -PI_F) sensor_yaw_rad += 2.0f * PI_F;
}

void ToFGeometry::getToFHitPosition(
    float sensor_x_m,
    float sensor_y_m,
    float sensor_yaw_rad,
    float range_m,
    float &hit_x_m,
    float &hit_y_m
) {
    // Simple ray calculation
    hit_x_m = sensor_x_m + range_m * cosf(sensor_yaw_rad);
    hit_y_m = sensor_y_m + range_m * sinf(sensor_yaw_rad);
}
