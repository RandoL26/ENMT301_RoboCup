//************************************
//       Localisation.cpp
//************************************
// Sensor fusion localisation implementation

#include "Localisation.h"
#include <Arduino.h>
#include <math.h>
#include "ld06.h"
#include "imu_sensor.h"
#include "optical_flow.h"
#include "MappingNav.h"

// Constants
#define PI_F 3.14159265358979f
#define TWO_PI_F (2.0f * PI_F)

Localisation::Localisation()
  : m_prev_imu_heading_deg(0.0f),
    m_last_update_ms(0) {
  
  // Initialize pose at origin
  m_pose.x_mm = 0.0f;
  m_pose.y_mm = 0.0f;
  m_pose.theta_rad = 0.0f;
  m_pose.covariance_x = 1000.0f;   // High initial uncertainty
  m_pose.covariance_y = 1000.0f;
  m_pose.covariance_theta = 0.1f;  // rad^2
  
  // Initialize sensors as invalid
  m_sensors.imu_valid = false;
  m_sensors.flow_valid = false;
  m_sensors.imu_timestamp_ms = 0;
  m_sensors.flow_total_x_mm = 0.0f;
  m_sensors.flow_total_y_mm = 0.0f;
  m_sensors.flow_prev_x_mm = 0.0f;
  m_sensors.flow_prev_y_mm = 0.0f;
  
  // Default configuration (can be tuned)
  m_config.encoder_pulses_per_m = 1000.0f;
  m_config.wheel_base_mm = 200.0f;
  m_config.flow_mm_per_count = 0.05f;
  
  // Default sensor offsets (measure these for your robot!)
  m_config.lidar_x_offset_mm = 0.0f;
  m_config.lidar_y_offset_mm = 0.0f;
  m_config.lidar_yaw_offset_rad = 0.0f;
  
  m_config.flow_x_offset_mm = 0.0f;
  m_config.flow_y_offset_mm = 0.0f;
  m_config.flow_yaw_offset_rad = 0.0f;
  
  m_config.imu_yaw_offset_rad = 0.0f;
  
  // LiDAR correction defaults
  m_config.lidar_correction_max_mm = 50.0f;    // Max 50 mm position correction per update
  m_config.lidar_correction_max_rad = 0.1f;    // Max ~5.7 deg angle correction
  m_config.lidar_min_valid_points = 50;        // Need at least 50 valid points
  m_config.lidar_min_match_score = 300;        // Match score must be >=300/1000
  
  // Clear diagnostics
  memset(&m_diags, 0, sizeof(m_diags));
}

void Localisation::begin() {
  m_last_update_ms = millis();
  m_diags.frame_count = 0;
  m_diags.lidar_correction_count = 0;
}

void Localisation::updateIMU(const IMU_Data &imu) {
  m_sensors.imu = imu;
  m_sensors.imu_valid = true;
  m_sensors.imu_timestamp_ms = millis();
  m_diags.imu_yaw_rad = (imu.euler_h - 180.0f) * (PI_F / 180.0f);  // Convert degrees to rad
}

void Localisation::updateOpticalFlow(const OpticalFlow &flow) {
  m_sensors.flow_total_x_mm = flow.getTotalXmm();
  m_sensors.flow_total_y_mm = flow.getTotalYmm();
  m_sensors.flow_valid = true;
}

void Localisation::updateLiDAR(LD06 &lidar, const MappingNav &nav) {
  // LiDAR is read during updateFromSensors() 
  // This is just a placeholder to match the API
}

void Localisation::updateFromSensors() {
  uint32_t now = millis();
  float dt_sec = 0.0f;
  
  if (m_last_update_ms > 0) {
    dt_sec = (float)(now - m_last_update_ms) / 1000.0f;
    // Clamp dt to reasonable range to handle timing hiccups
    if (dt_sec < 0.0f) dt_sec = 0.0f;
    if (dt_sec > 1.0f) dt_sec = 1.0f;
  }
  m_last_update_ms = now;
  
  // Step 1: Prediction (IMU gyro + optical flow)
  predict(dt_sec);
  
  // Step 2: LiDAR-based correction will be done externally
  // (too expensive to do every cycle; called separately)
  
  m_diags.frame_count++;
}

void Localisation::correctFromLiDAR(LD06 &lidar, const MappingNav &nav) {
  // Perform LiDAR-based correction
  correctWithLiDAR(lidar, nav);
}

void Localisation::predict(float dt_sec) {
  if (dt_sec <= 0.0f) return;
  
  // IMU provides heading rate
  float imu_gyro_z = 0.0f;
  if (m_sensors.imu_valid) {
    // Convert gyro from rad/s to rad/update
    imu_gyro_z = m_sensors.imu.gyro_z;
    m_diags.imu_gyro_z = imu_gyro_z;
  }
  
  // Optical flow provides velocity estimate in robot frame
  float flow_dx_mm = 0.0f;
  float flow_dy_mm = 0.0f;
  if (m_sensors.flow_valid) {
    flow_dx_mm = (m_sensors.flow_total_x_mm - m_sensors.flow_prev_x_mm);
    flow_dy_mm = (m_sensors.flow_total_y_mm - m_sensors.flow_prev_y_mm);
    
    m_sensors.flow_prev_x_mm = m_sensors.flow_total_x_mm;
    m_sensors.flow_prev_y_mm = m_sensors.flow_total_y_mm;
    
    m_diags.flow_dx_mm = flow_dx_mm;
    m_diags.flow_dy_mm = flow_dy_mm;
  }
  
  // Update heading from IMU gyro
  // Gyro rate * dt gives heading change
  float dtheta = imu_gyro_z * dt_sec;
  m_pose.theta_rad += dtheta;
  m_pose.theta_rad = wrapAngleRad(m_pose.theta_rad);
  
  // Transform optical flow from robot frame to world frame
  // Assume flow is along robot +X (forward) and perpendicular +Y (left)
  float cos_th = cosf(m_pose.theta_rad);
  float sin_th = sinf(m_pose.theta_rad);
  
  // Rotate robot-frame displacement to world frame
  float dx_world = flow_dx_mm * cos_th - flow_dy_mm * sin_th;
  float dy_world = flow_dx_mm * sin_th + flow_dy_mm * cos_th;
  
  m_pose.x_mm += dx_world;
  m_pose.y_mm += dy_world;
  
  // Increase uncertainty with time (dead reckoning drift)
  // Position uncertainty grows with distance traveled
  float dist_traveled = sqrtf(dx_world*dx_world + dy_world*dy_world);
  m_pose.covariance_x += 0.1f * dist_traveled + 0.01f * (dt_sec * dt_sec);
  m_pose.covariance_y += 0.1f * dist_traveled + 0.01f * (dt_sec * dt_sec);
  m_pose.covariance_theta += 0.001f * (dt_sec * dt_sec);
}

void Localisation::correctWithLiDAR(LD06 &lidar, const MappingNav &nav) {
  // Perform scan-to-map matching
  float corr_x = 0.0f, corr_y = 0.0f, corr_theta = 0.0f;
  uint16_t match_score = scanToMapMatch(lidar, nav, m_pose, corr_x, corr_y, corr_theta);
  
  m_diags.lidar_match_score = match_score;
  m_diags.lidar_corr_x = corr_x;
  m_diags.lidar_corr_y = corr_y;
  m_diags.lidar_corr_theta = corr_theta;
  
  // Apply correction only if score is high enough
  if (match_score >= m_config.lidar_min_match_score) {
    // Limit magnitude of corrections
    if (fabsf(corr_x) > m_config.lidar_correction_max_mm) {
      corr_x = (corr_x > 0.0f) ? m_config.lidar_correction_max_mm : -m_config.lidar_correction_max_mm;
    }
    if (fabsf(corr_y) > m_config.lidar_correction_max_mm) {
      corr_y = (corr_y > 0.0f) ? m_config.lidar_correction_max_mm : -m_config.lidar_correction_max_mm;
    }
    if (fabsf(corr_theta) > m_config.lidar_correction_max_rad) {
      corr_theta = (corr_theta > 0.0f) ? m_config.lidar_correction_max_rad : -m_config.lidar_correction_max_rad;
    }
    
    // Apply correction
    m_pose.x_mm += corr_x;
    m_pose.y_mm += corr_y;
    m_pose.theta_rad += corr_theta;
    m_pose.theta_rad = wrapAngleRad(m_pose.theta_rad);
    
    // Reduce uncertainty after successful LiDAR correction
    m_pose.covariance_x *= 0.5f;
    m_pose.covariance_y *= 0.5f;
    m_pose.covariance_theta *= 0.5f;
    
    m_diags.lidar_correction_accepted = 1;
    m_diags.lidar_correction_count++;
  } else {
    m_diags.lidar_correction_accepted = 0;
  }
}

uint16_t Localisation::scanToMapMatch(LD06 &lidar,
                                       const MappingNav &nav,
                                       const RobotPose &predicted,
                                       float &corr_x_mm,
                                       float &corr_y_mm,
                                       float &corr_theta_rad) {
  corr_x_mm = 0.0f;
  corr_y_mm = 0.0f;
  corr_theta_rad = 0.0f;
  
  uint16_t n_pts = lidar.getNbPointsInScan();
  
  // Reject scan if too few points
  if (n_pts < m_config.lidar_min_valid_points) {
    return 0;  // No match
  }
  
  // Lightweight scan-to-map correlation:
  // Count how many LiDAR points align with occupied cells in the occupancy grid.
  // This is a simple occupancy-grid-based matcher (not full scan-to-map ICP).
  
  uint16_t match_count = 0;
  uint16_t valid_point_count = 0;
  
  // Iterate over LiDAR scan points and check if they align with occupancy grid
  for (uint16_t i = 0; i < n_pts; ++i) {
    const DataPoint *p = lidar.getPoints(i);
    if (!p || p->distance == 0 || p->distance > 12000) continue;  // Skip invalid points
    
    valid_point_count++;
    
    // LiDAR angle in robot frame (degrees -> radians)
    float ang_rad = p->angle * (PI_F / 180.0f);
    
    // Global angle in world frame
    float global_ang = predicted.theta_rad + ang_rad;
    
    // Endpoint of ray in world coordinates (meters)
    float dist_m = ((float)p->distance) / 1000.0f;
    float end_x_m = predicted.x_mm / 1000.0f + dist_m * cosf(global_ang);
    float end_y_m = predicted.y_mm / 1000.0f + dist_m * sinf(global_ang);
    
    // Clamp to arena bounds
    end_x_m = (end_x_m < 0.0f) ? 0.0f : (end_x_m > MappingNav::ARENA_WIDTH_M ? MappingNav::ARENA_WIDTH_M : end_x_m);
    end_y_m = (end_y_m < 0.0f) ? 0.0f : (end_y_m > MappingNav::ARENA_HEIGHT_M ? MappingNav::ARENA_HEIGHT_M : end_y_m);
    
    // Convert to cell coordinates
    uint16_t cell_x = (uint16_t)(end_x_m / MappingNav::CELL_SIZE_M);
    uint16_t cell_y = (uint16_t)(end_y_m / MappingNav::CELL_SIZE_M);
    
    // Bounds check
    if (cell_x >= MappingNav::GRID_WIDTH || cell_y >= MappingNav::GRID_HEIGHT) continue;
    
    // Check occupancy state at this cell (would need to add getter to MappingNav)
    // For now, we do a simple heuristic: count valid points as a match
    // A better implementation would check the actual grid occupancy
    match_count++;
  }
  
  // If we have very few valid points, reject the match
  if (valid_point_count < m_config.lidar_min_valid_points) {
    return 0;
  }
  
  // Match score: percentage of valid points that align
  uint16_t score = (uint16_t)((match_count * 1000) / valid_point_count);
  
  // For now, return conservative zero correction
  // A real scan matcher would perform iterative closest point (ICP) or similar
  // This stub preserves fused pose but allows correction framework to be added later
  corr_x_mm = 0.0f;
  corr_y_mm = 0.0f;
  corr_theta_rad = 0.0f;
  
  return score;
}

RobotPose Localisation::getPose() const {
  return m_pose;
}

void Localisation::resetPose(float x_mm, float y_mm, float theta_rad) {
  m_pose.x_mm = x_mm;
  m_pose.y_mm = y_mm;
  m_pose.theta_rad = wrapAngleRad(theta_rad);
  
  // Reset uncertainty after manual reset
  m_pose.covariance_x = 100.0f;   // Reasonably confident after reset
  m_pose.covariance_y = 100.0f;
  m_pose.covariance_theta = 0.01f;
  
  // Reset sensor buffers
  m_sensors.flow_prev_x_mm = m_sensors.flow_total_x_mm;
  m_sensors.flow_prev_y_mm = m_sensors.flow_total_y_mm;
  m_prev_imu_heading_deg = m_sensors.imu.euler_h;
  
  m_last_update_ms = millis();
}

LocalisationDiags Localisation::getDiags() const {
  return m_diags;
}

void Localisation::setEncoderPulsesPerMeter(float pulses_per_m) {
  if (pulses_per_m > 0.0f) {
    m_config.encoder_pulses_per_m = pulses_per_m;
  }
}

void Localisation::setWheelBaseMillimeters(float wb_mm) {
  if (wb_mm > 0.0f) {
    m_config.wheel_base_mm = wb_mm;
  }
}

void Localisation::setOpticalFlowMMPerCount(float mm_per_count) {
  if (mm_per_count > 0.0f) {
    m_config.flow_mm_per_count = mm_per_count;
  }
}

void Localisation::setLidarOffsetMM(float x_mm, float y_mm) {
  m_config.lidar_x_offset_mm = x_mm;
  m_config.lidar_y_offset_mm = y_mm;
}

void Localisation::setLidarYawOffsetRad(float offset_rad) {
  m_config.lidar_yaw_offset_rad = offset_rad;
}

void Localisation::setFlowOffsetMM(float x_mm, float y_mm) {
  m_config.flow_x_offset_mm = x_mm;
  m_config.flow_y_offset_mm = y_mm;
}

void Localisation::setFlowYawOffsetRad(float offset_rad) {
  m_config.flow_yaw_offset_rad = offset_rad;
}

void Localisation::setIMUYawOffsetRad(float offset_rad) {
  m_config.imu_yaw_offset_rad = offset_rad;
}

void Localisation::setLidarCorrectionMaxMM(float max_mm) {
  if (max_mm > 0.0f) {
    m_config.lidar_correction_max_mm = max_mm;
  }
}

void Localisation::setLidarCorrectionMaxRad(float max_rad) {
  if (max_rad > 0.0f) {
    m_config.lidar_correction_max_rad = max_rad;
  }
}

void Localisation::setLidarMinValidPoints(uint16_t min_pts) {
  m_config.lidar_min_valid_points = min_pts;
}

void Localisation::setLidarMinMatchScore(uint16_t min_score) {
  m_config.lidar_min_match_score = min_score;
}

float Localisation::wrapAngleRad(float angle) {
  while (angle > PI_F) angle -= TWO_PI_F;
  while (angle <= -PI_F) angle += TWO_PI_F;
  return angle;
}

float Localisation::wrapAngleDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}
