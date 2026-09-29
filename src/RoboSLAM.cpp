#include "RoboSLAM.h"
#include "telemetry.h"
#include <Arduino.h>
#include <math.h>

namespace {
const float PI_F = 3.14159265358979323846f;
float wrapRad(float angle) {
  while (angle >= PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}
}

RoboSLAM::RoboSLAM(MappingNav &nav, MotorControl &motors)
    : m_nav(nav), m_motors(motors) {}

void RoboSLAM::begin() {
  prev_left_pulses = m_motors.getLeftEncoderPulses();
  prev_right_pulses = m_motors.getRightEncoderPulses();
  last_update_ms = millis();
  last_left_delta_m = last_right_delta_m = 0.0f;
  last_encoder_dtheta_rad = last_imu_dtheta_rad = 0.0f;
  last_match_score = 0.0f;
  last_match_accepted = false;
}

void RoboSLAM::updatePrediction(const IMU_Data &imu, uint32_t now_ms) {
  const int32_t left = m_motors.getLeftEncoderPulses();
  const int32_t right = m_motors.getRightEncoderPulses();
  // Unsigned subtraction gives a wrap-safe delta for the signed 32-bit counters.
  const int32_t dl = (int32_t)((uint32_t)left - (uint32_t)prev_left_pulses);
  const int32_t dr = (int32_t)((uint32_t)right - (uint32_t)prev_right_pulses);
  prev_left_pulses = left;
  prev_right_pulses = right;

  const uint32_t elapsed_ms = now_ms - last_update_ms;
  last_update_ms = now_ms;
  // Ignore implausibly long gaps after pauses/resets rather than applying a jump.
  const bool timing_valid = elapsed_ms > 0 && elapsed_ms <= 250;
  const bool encoder_translation_valid = timing_valid && OdometryConfig::ENCODER_COUNTS_PER_M > 0.0f;
  const bool encoder_heading_valid = encoder_translation_valid && OdometryConfig::ENCODER_TRACK_M > 0.0f;

  last_left_delta_m = encoder_translation_valid
      ? OdometryConfig::LEFT_ENCODER_SIGN * (float)dl / OdometryConfig::ENCODER_COUNTS_PER_M : 0.0f;
  last_right_delta_m = encoder_translation_valid
      ? OdometryConfig::RIGHT_ENCODER_SIGN * (float)dr / OdometryConfig::ENCODER_COUNTS_PER_M : 0.0f;
  last_encoder_dtheta_rad = encoder_heading_valid
      ? (last_right_delta_m - last_left_delta_m) / OdometryConfig::ENCODER_TRACK_M : 0.0f;

  // Sensor read success flag and bounded timing prevent integrating stale/invalid gyro data.
  const bool imu_valid = timing_valid && imu.gyro_valid &&
                         fabsf(imu.gyro_z) < 20.0f;
  last_imu_dtheta_rad = imu_valid
      ? OdometryConfig::IMU_YAW_SIGN * imu.gyro_z * ((float)elapsed_ms / 1000.0f) : 0.0f;

  MappingNav::PoseUpdateInput input;
  input.encoder_left_delta_m = last_left_delta_m;
  input.encoder_right_delta_m = last_right_delta_m;
  input.encoder_dtheta_rad = last_encoder_dtheta_rad;
  input.imu_gyro_dtheta_rad = last_imu_dtheta_rad;
  input.encoder_translation_valid = encoder_translation_valid;
  input.encoder_heading_valid = encoder_heading_valid;
  input.imu_heading_valid = imu_valid;
  m_nav.updatePose(input, IMU_HEADING_WEIGHT);
}

float RoboSLAM::scorePose(LD06 &ld, const MappingNav::Pose2D &pose,
                          uint16_t stride, uint16_t &tested) {
  tested = 0;
  uint16_t hits = 0;
  const uint16_t count = ld.getNbPointsInScan();
  const float c = cosf(pose.theta_rad), s = sinf(pose.theta_rad);
  const float lx = pose.x_m + LIDAR_OFFSET_X_M * c - LIDAR_OFFSET_Y_M * s;
  const float ly = pose.y_m + LIDAR_OFFSET_X_M * s + LIDAR_OFFSET_Y_M * c;
  for (uint16_t i = 0; i < count; i = (uint16_t)(i + stride)) {
    const DataPoint *p = ld.getPoints(i);
    if (!p || p->distance < 350 || p->distance > 4500) continue;
    const float a = pose.theta_rad + LIDAR_YAW_OFFSET_RAD +
                    p->angle * (PI_F / 180.0f);
    const float r = (float)p->distance * 0.001f;
    ++tested;
    if (m_nav.isOccupiedWorld(lx + r * cosf(a), ly + r * sinf(a))) ++hits;
  }
  return tested ? (1000.0f * (float)hits / (float)tested) : 0.0f;
}

bool RoboSLAM::matchScan(LD06 &ld, float &dx, float &dy, float &dtheta,
                         float &score) {
  const MappingNav::Pose2D origin = m_nav.getPose();
  MappingNav::Pose2D best = origin;
  uint16_t tested = 0, bestTested = 0;
  float baseline = scorePose(ld, origin, 8, tested);
  score = baseline;
  if (tested < LIDAR_MATCH_MIN_POINTS) return false;

  float bestScore = baseline;
  // Coarse bounded search: +/- 10 cm and +/- 10 degrees at 5 cm/5 degree steps.
  for (int xi=-2; xi<=2; ++xi) for (int yi=-2; yi<=2; ++yi)
    for (int ai=-2; ai<=2; ++ai) {
      MappingNav::Pose2D candidate = origin;
      candidate.x_m += xi * 0.05f;
      candidate.y_m += yi * 0.05f;
      candidate.theta_rad = wrapRad(origin.theta_rad + ai * (5.0f * PI_F / 180.0f));
      uint16_t n = 0;
      const float candidateScore = scorePose(ld, candidate, 8, n);
      if (n >= LIDAR_MATCH_MIN_POINTS && candidateScore > bestScore) {
        best = candidate;
        bestScore = candidateScore;
        bestTested = n;
      }
    }

  // Fine local refinement around the coarse winner (3x3x3 candidates).
  const MappingNav::Pose2D coarseBest = best;
  for (int xi=-1; xi<=1; ++xi) for (int yi=-1; yi<=1; ++yi)
    for (int ai=-1; ai<=1; ++ai) {
      MappingNav::Pose2D candidate = coarseBest;
      candidate.x_m += xi * 0.025f;
      candidate.y_m += yi * 0.025f;
      candidate.theta_rad = wrapRad(coarseBest.theta_rad + ai * (2.5f * PI_F / 180.0f));
      uint16_t n = 0;
      const float candidateScore = scorePose(ld, candidate, 8, n);
      if (n >= LIDAR_MATCH_MIN_POINTS && candidateScore > bestScore) {
        best = candidate;
        bestScore = candidateScore;
        bestTested = n;
      }
    }

  // Reject repeated/ambiguous geometry when a separated coarse candidate is
  // nearly as good as the winning pose (scores are occupied returns / tested).
  float runnerUp = 0.0f;
  for (int xi=-2; xi<=2; ++xi) for (int yi=-2; yi<=2; ++yi)
    for (int ai=-2; ai<=2; ++ai) {
      MappingNav::Pose2D candidate = origin;
      candidate.x_m += xi * 0.05f;
      candidate.y_m += yi * 0.05f;
      candidate.theta_rad = wrapRad(origin.theta_rad + ai * (5.0f * PI_F / 180.0f));
      const float sepX = candidate.x_m - best.x_m;
      const float sepY = candidate.y_m - best.y_m;
      const float sepTheta = wrapRad(candidate.theta_rad - best.theta_rad);
      if (sqrtf(sepX*sepX + sepY*sepY) < 0.075f &&
          fabsf(sepTheta) < 7.5f * PI_F / 180.0f) continue;
      uint16_t n = 0;
      const float alternative = scorePose(ld, candidate, 8, n);
      if (n >= LIDAR_MATCH_MIN_POINTS && alternative > runnerUp) runnerUp = alternative;
    }

  dx = best.x_m - origin.x_m;
  dy = best.y_m - origin.y_m;
  dtheta = wrapRad(best.theta_rad - origin.theta_rad);
  score = bestScore;
  const float positionCorrection = sqrtf(dx * dx + dy * dy);
  const bool meaningfulImprovement = bestScore >= baseline + LIDAR_MATCH_MIN_IMPROVEMENT;
  const bool unambiguous = bestScore >= runnerUp + LIDAR_MATCH_AMBIGUITY_MARGIN;
  const bool safeCorrection = positionCorrection <= LIDAR_CORRECTION_MAX_M &&
                              fabsf(dtheta) <= LIDAR_CORRECTION_MAX_RAD;
  return bestTested >= LIDAR_MATCH_MIN_POINTS && bestScore >= LIDAR_MATCH_MIN_SCORE &&
         meaningfulImprovement && unambiguous && safeCorrection;
}

void RoboSLAM::processScan(LD06 &ld) {
  const uint16_t n = ld.getNbPointsInScan();
  float corrX = 0.0f, corrY = 0.0f, corrTheta = 0.0f, score = 0.0f;
  last_match_accepted = matchScan(ld, corrX, corrY, corrTheta, score);
  last_match_score = score;

  if (last_match_accepted) {
    m_nav.applyPoseCorrection(corrX, corrY, corrTheta);
  }

  // Convert the scan to rays once. Map from the final corrected pose, with the
  // same verified-in-code lidar extrinsics used by the matcher.
  static MappingNav::SensorRay rays[LD06_MAX_PTS_SCAN];
  uint16_t rayCount = 0;
  const uint16_t limit = n < LD06_MAX_PTS_SCAN ? n : LD06_MAX_PTS_SCAN;
  for (uint16_t i=0; i<limit; ++i) {
    DataPoint *p = ld.getPoints(i);
    if (!p || p->distance < 50 || p->distance > 8000) continue;
    MappingNav::SensorRay &r = rays[rayCount++];
    r.valid = true;
    r.has_hit = true;
    r.kind = MappingNav::SENSOR_LIDAR;
    r.angle_offset_rad = LIDAR_YAW_OFFSET_RAD + p->angle * (PI_F / 180.0f);
    r.distance_m = (float)p->distance * 0.001f;
    r.max_range_m = MappingNav::LIDAR_MAX_RANGE_M;
  }

  // Rejected/ambiguous scans do not alter pose. The outer-wall boundary
  // correction remains available in MappingNav but is not mixed into this
  // scan update, so the acceptance diagnostic matches actual pose behavior.
  const MappingNav::Pose2D pose = m_nav.getPose();
  const float c = cosf(pose.theta_rad), s = sinf(pose.theta_rad);
  const float lidarX = pose.x_m + LIDAR_OFFSET_X_M*c - LIDAR_OFFSET_Y_M*s;
  const float lidarY = pose.y_m + LIDAR_OFFSET_X_M*s + LIDAR_OFFSET_Y_M*c;
  m_nav.setPose(lidarX, lidarY, pose.theta_rad);
  if (rayCount) m_nav.updateGridFromSensors(rays, rayCount);
  m_nav.setPose(pose.x_m, pose.y_m, pose.theta_rad);

  const float robot_radius_m = sqrtf(0.25f*MappingNav::ROBOT_LENGTH_M*MappingNav::ROBOT_LENGTH_M +
                                    0.25f*MappingNav::ROBOT_WIDTH_M*MappingNav::ROBOT_WIDTH_M) +
                               MappingNav::ROBOT_SAFETY_MARGIN_M;
  m_nav.replanPath(robot_radius_m);
  const MappingNav::Pose2D finalPose = m_nav.getPose();
  const float left = last_left_delta_m, right = last_right_delta_m;
  telemetry_send_localisation_debug(left, right, last_encoder_dtheta_rad,
                                    last_imu_dtheta_rad, last_match_score,
                                    last_match_accepted ? 1.0f : 0.0f, finalPose,
                                    last_match_accepted ? corrX : 0.0f,
                                    last_match_accepted ? corrY : 0.0f,
                                    last_match_accepted ? corrTheta : 0.0f,
                                    m_motors.getLeftEncoderPulses(),
                                    m_motors.getRightEncoderPulses());
}
