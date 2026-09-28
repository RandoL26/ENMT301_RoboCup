#include "RoboSLAM.h"
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

  // 5) Convert LD06 points into SensorRay array
  uint16_t n = ld.getNbPointsInScan();
  const uint16_t MAX_RAYS = 1200;
  static MappingNav::SensorRay rays[MAX_RAYS];
  if (n > MAX_RAYS) n = MAX_RAYS;

  // The LD06 getPoints() returns a pointer to DataPoint in LD06 implementation.
  // We'll read using the getPoints(uint16_t) accessor defined in ld06.h
  for (uint16_t i = 0; i < n; ++i) {
    const auto *p = ld.getPoints(i);
    MappingNav::SensorRay &r = rays[i];
    r.valid = true;
    r.has_hit = (p->distance > 0);
    r.kind = MappingNav::SENSOR_TOF;
    // LD06 angle is in degrees; assume it's angle relative to robot forward
    float ang_rad = (p->angle) * (M_PI / 180.0f);
    r.angle_offset_rad = ang_rad;
    r.distance_m = ((float)p->distance) / 1000.0f;
    r.max_range_m = 12.0f; // sensible default
  }

  // 6) Update map
  m_nav.updateGridFromSensors(rays, n);

  // 7) Optional boundary correction + replan
  m_nav.applyBoundaryCorrection(rays, n, 0.25f, 0.12f);
  m_nav.replanPath(0.12f);
}
