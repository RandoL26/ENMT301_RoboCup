#include "RoboSLAM.h"
#include "telemetry.h"
#include <Arduino.h>
#include <math.h>

RoboSLAM::RoboSLAM(MappingNav &nav, MotorControl &motors, OpticalFlow &flow)
    : m_nav(nav), m_motors(motors), m_flow(flow) {}


    
void RoboSLAM::begin() {
  prev_left_pulses = m_motors.getLeftEncoderPulses();
  prev_right_pulses = m_motors.getRightEncoderPulses();
  prev_flow_x_mm = m_flow.getTotalXmm();
  prev_flow_y_mm = m_flow.getTotalYmm();
  IMU_Data imu = read_imu();
  prev_imu_heading_deg = imu.euler_h;
}

void RoboSLAM::processScan(LD06 &ld, const IMU_Data &imu) {
  // 1) Compute odometry deltas
  int32_t left = m_motors.getLeftEncoderPulses();
  int32_t right = m_motors.getRightEncoderPulses();
  int32_t dl = left - prev_left_pulses;
  int32_t dr = right - prev_right_pulses;
  prev_left_pulses = left;
  prev_right_pulses = right;

  float left_m = ((float)dl) / pulses_per_meter;
  float right_m = ((float)dr) / pulses_per_meter;
  float encoder_dx = 0.5f * (left_m + right_m);
  float encoder_dy = 0.0f;
  float encoder_dtheta = (right_m - left_m) / wheel_base_m; // rad

  // 2) Optical flow deltas (mm -> m)
  float fx = m_flow.getTotalXmm();
  float fy = m_flow.getTotalYmm();
  float dfx = (fx - prev_flow_x_mm) / 1000.0f;
  float dfy = (fy - prev_flow_y_mm) / 1000.0f;
  prev_flow_x_mm = fx;
  prev_flow_y_mm = fy;

  // 3) IMU gyro delta (degrees -> rad)
  float heading_now = imu.euler_h;
  float dheading = heading_now - prev_imu_heading_deg;
  // Normalize to [-180,180]
  while (dheading > 180.0f) dheading -= 360.0f;
  while (dheading <= -180.0f) dheading += 360.0f;
  prev_imu_heading_deg = heading_now;
  float imu_dtheta_rad = dheading * (M_PI / 180.0f);

  // 4) Feed MappingNav pose input
  MappingNav::PoseUpdateInput pui;
  pui.encoder_dx_m = encoder_dx;
  pui.encoder_dy_m = encoder_dy;
  pui.encoder_dtheta_rad = encoder_dtheta;
  pui.flow_dx_m = dfx;
  pui.flow_dy_m = dfy;
  pui.imu_gyro_dtheta_rad = imu_dtheta_rad;

  m_nav.updatePose(pui);

  // LD06 mounting position relative to robot centre.
  // +X = forward, +Y = left.
  // Measure these values on the actual robot.
  //
  // Example:
  // 0.10f = LiDAR is 10 cm forward of robot centre.
  // 0.00f = LiDAR is centred left/right.
  const float LIDAR_OFFSET_X_M = 0.10f;
  const float LIDAR_OFFSET_Y_M = -0.04f;

  // Rotation of the LiDAR relative to robot forward.
  // 0 = LiDAR forward direction matches robot forward.
  const float LIDAR_YAW_OFFSET_RAD = 0.0f;

  // 5) Convert LD06 points into SensorRay array
  uint16_t n = ld.getNbPointsInScan();

  MappingNav::Pose2D pose = m_nav.getPose();

  const float c = cosf(pose.theta_rad);
  const float s = sinf(pose.theta_rad);

  const float lidar_x =
      pose.x_m + LIDAR_OFFSET_X_M * c - LIDAR_OFFSET_Y_M * s;

  const float lidar_y =
      pose.y_m + LIDAR_OFFSET_X_M * s + LIDAR_OFFSET_Y_M * c;

  // Temporarily use the LiDAR position for ray mapping.
  m_nav.setPose(lidar_x, lidar_y, pose.theta_rad);

  const uint16_t MAX_RAYS = 1200;
  static MappingNav::SensorRay rays[MAX_RAYS];

  // LiDAR filtering limits.
  // Tune these after testing on the real arena.
 

  if (n > MAX_RAYS) n = MAX_RAYS;

  uint16_t valid_rays = 0;

  for (uint16_t i = 0; i < n; ++i) {
    const auto *p = ld.getPoints(i);
    if (!p) continue;

    // Reject obviously invalid LiDAR measurements.
    if (p->distance == 0) {
      // Keep zero-distance readings as no-hit rays.
    } else if (p->distance == 0xFFFF) {
      // Invalid / saturated sensor value.
      continue;
    }

    const float distance_m =
        ((float)p->distance) / 1000.0f;

    // LD06 distance == 0 means no valid obstacle return.
    // Keep this as a no-hit ray so the free-space ray casting
    // can still work.
    // A zero-distance reading is treated as invalid.
    // Do NOT interpret it as "free space to max range".
    if (p->distance == 0) {
        continue;
    }

    // Reject measurements that are too close or too far away.
    if (distance_m < MappingNav::LIDAR_MIN_RANGE_M ||
        distance_m > MappingNav::LIDAR_MAX_RANGE_M) {
      continue;
    }

    MappingNav::SensorRay &r = rays[valid_rays++];

    r.valid = true;
    r.has_hit = true;
    r.kind = MappingNav::SENSOR_LIDAR;

    const float ang_rad =
        (p->angle) * (M_PI / 180.0f);

    r.angle_offset_rad =
        ang_rad + LIDAR_YAW_OFFSET_RAD;

    r.distance_m = distance_m;
    r.max_range_m = MappingNav::LIDAR_MAX_RANGE_M;
  }

  // Only send the filtered rays to the map.
  // Only update the map when we have valid rays.
  if (valid_rays > 0) {
    m_nav.updateGridFromSensors(rays, valid_rays);
  }

  // Restore robot-centre pose.
  m_nav.setPose(pose.x_m, pose.y_m, pose.theta_rad);

  // 7) Boundary correction + replan
  m_nav.applyBoundaryCorrection(
        rays,
        valid_rays,
        0.25f,
        0.12f,
        LIDAR_OFFSET_X_M,
        LIDAR_OFFSET_Y_M);
  const float robot_radius_m =
    sqrtf(
        (MappingNav::ROBOT_LENGTH_M * 0.5f) *
        (MappingNav::ROBOT_LENGTH_M * 0.5f) +
        (MappingNav::ROBOT_WIDTH_M * 0.5f) *
        (MappingNav::ROBOT_WIDTH_M * 0.5f)
    ) +
    MappingNav::ROBOT_SAFETY_MARGIN_M;

  m_nav.replanPath(robot_radius_m);

  // Send localisation diagnostics.
  // This lets the PC visualiser compare the individual
  // sensor contributions with the final MappingNav pose.
  telemetry_send_localisation_debug(
      encoder_dx,
      encoder_dtheta,
      dfx,
      dfy,
      imu_dtheta_rad,
      heading_now,
      m_nav.getPose()
  );

}

