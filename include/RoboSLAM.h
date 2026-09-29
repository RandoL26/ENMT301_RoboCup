#ifndef ROBOSLAM_H_
#define ROBOSLAM_H_

#include "MappingNav.h"
#include "ld06.h"
#include "motor_control.h"
#include "imu_sensor.h"
#include "odometry_config.h"

class RoboSLAM {
public:
  RoboSLAM(MappingNav &nav, MotorControl &motors);
  void begin();
  // Call from the 10 ms prediction task. Encoder calibration remains disabled
  // until encoder counts/metre and effective track width have been measured.
  void updatePrediction(const IMU_Data &imu, uint32_t now_ms);
  // Process a completed scan: match to the existing map, then map at corrected pose.
  void processScan(LD06 &ld);

private:
  MappingNav &m_nav;
  MotorControl &m_motors;
  int32_t prev_left_pulses = 0;
  int32_t prev_right_pulses = 0;
  uint32_t last_update_ms = 0;
  float last_left_delta_m = 0.0f;
  float last_right_delta_m = 0.0f;
  float last_encoder_dtheta_rad = 0.0f;
  float last_imu_dtheta_rad = 0.0f;
  float last_match_score = 0.0f;
  bool last_match_accepted = false;

  // Set these after physical calibration. Zero disables the corresponding
  // encoder estimate; no guessed scale/track width is used by default.
  static constexpr float IMU_HEADING_WEIGHT = 1.0f;
  static constexpr float LIDAR_OFFSET_X_M = 0.10f;
  static constexpr float LIDAR_OFFSET_Y_M = -0.04f;
  static constexpr float LIDAR_YAW_OFFSET_RAD = 0.0f;
  static constexpr uint16_t LIDAR_MATCH_MIN_POINTS = 24;
  static constexpr float LIDAR_MATCH_MIN_SCORE = 300.0f;
  static constexpr float LIDAR_MATCH_MIN_IMPROVEMENT = 80.0f;
  static constexpr float LIDAR_MATCH_AMBIGUITY_MARGIN = 40.0f;
  static constexpr float LIDAR_CORRECTION_MAX_M = 0.05f;
  static constexpr float LIDAR_CORRECTION_MAX_RAD = 0.08726646f;

  bool matchScan(LD06 &ld, float &dx, float &dy, float &dtheta,
                 float &score);
  float scorePose(LD06 &ld, const MappingNav::Pose2D &pose,
                  uint16_t stride, uint16_t &tested);
};

#endif
