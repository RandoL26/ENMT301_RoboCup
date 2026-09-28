//************************************
//       Localisation.h
//************************************
// Sensor fusion localisation module
// Combines LD06 LiDAR, BNO055 IMU, and PMW3901 optical flow
// to produce fused 2D robot pose (x_mm, y_mm, theta_rad)

#ifndef LOCALISATION_H_
#define LOCALISATION_H_

#include <stdint.h>
#include "imu_sensor.h"

// Forward declarations
class LD06;
class OpticalFlow;
class MappingNav;

// Fused robot pose with uncertainty estimates
struct RobotPose {
  float x_mm;           // Position X in mm (world frame)
  float y_mm;           // Position Y in mm (world frame)
  float theta_rad;      // Heading in radians [-pi, pi]

  float covariance_x;   // Uncertainty in X (mm^2)
  float covariance_y;   // Uncertainty in Y (mm^2)
  float covariance_theta; // Uncertainty in theta (rad^2)
};

// Localisation diagnostics (for debugging and telemetry)
struct LocalisationDiags {
  float flow_dx_mm;     // Last optical flow X displacement
  float flow_dy_mm;     // Last optical flow Y displacement
  float imu_yaw_rad;    // Current IMU yaw/heading in radians
  float imu_gyro_z;     // Last IMU gyro Z reading (rad/s)
  
  float lidar_corr_x;   // Last LiDAR correction in X (mm)
  float lidar_corr_y;   // Last LiDAR correction in Y (mm)
  float lidar_corr_theta; // Last LiDAR correction in theta (rad)
  
  uint16_t lidar_match_score; // Quality score of last LiDAR match (0-1000)
  uint8_t  lidar_correction_accepted; // 1 if last LiDAR correction accepted, 0 if rejected
  
  uint32_t frame_count; // Total update frames processed
  uint16_t lidar_correction_count; // Number of LiDAR corrections applied
};

class Localisation {
public:
  Localisation();
  
  // Initialize localisation system
  void begin();
  
  // Called periodically from sensor reading tasks
  // These accumulate sensor data; fusion happens in updateFromSensors()
  void updateIMU(const IMU_Data &imu);
  void updateOpticalFlow(const OpticalFlow &flow);
  void updateLiDAR(LD06 &lidar, const MappingNav &nav);
  
  // Main fusion update - call once per localisation cycle (~10-100 Hz)
  // Performs prediction (IMU+flow) and LiDAR correction
  void updateFromSensors();
  
  // LiDAR-based pose correction (call when scan is ready, ~5-10 Hz)
  void correctFromLiDAR(LD06 &lidar, const MappingNav &nav);
  
  // Get current fused pose
  RobotPose getPose() const;
  
  // Reset pose to known state (e.g., at start or after manual reset)
  void resetPose(float x_mm, float y_mm, float theta_rad);
  
  // Get diagnostics (for telemetry)
  LocalisationDiags getDiags() const;
  
  // Configuration methods
  void setEncoderPulsesPerMeter(float pulses_per_m);
  void setWheelBaseMillimeters(float wb_mm);
  void setOpticalFlowMMPerCount(float mm_per_count);
  
  // Calibration: sensor offsets from robot center
  void setLidarOffsetMM(float x_mm, float y_mm);
  void setLidarYawOffsetRad(float offset_rad);
  
  void setFlowOffsetMM(float x_mm, float y_mm);
  void setFlowYawOffsetRad(float offset_rad);
  
  void setIMUYawOffsetRad(float offset_rad);
  
  // LiDAR correction tuning
  void setLidarCorrectionMaxMM(float max_mm);        // Max position correction per update
  void setLidarCorrectionMaxRad(float max_rad);      // Max angle correction per update
  void setLidarMinValidPoints(uint16_t min_pts);     // Min points to attempt correction
  void setLidarMinMatchScore(uint16_t min_score);    // Min match score to accept correction
  
private:
  // Prediction state (dead reckoning)
  RobotPose m_pose;
  
  // Sensor data buffer (holds latest reading)
  struct {
    IMU_Data imu;
    bool imu_valid;
    uint32_t imu_timestamp_ms;
    
    float flow_total_x_mm;
    float flow_total_y_mm;
    float flow_prev_x_mm;
    float flow_prev_y_mm;
    bool flow_valid;
    
    // LiDAR is read via LD06 interface
  } m_sensors;
  
  // Previous IMU heading for delta calculation
  float m_prev_imu_heading_deg;
  
  // Timing
  uint32_t m_last_update_ms;
  
  // Configuration
  struct {
    float encoder_pulses_per_m;
    float wheel_base_mm;
    float flow_mm_per_count;
    
    float lidar_x_offset_mm;
    float lidar_y_offset_mm;
    float lidar_yaw_offset_rad;
    
    float flow_x_offset_mm;
    float flow_y_offset_mm;
    float flow_yaw_offset_rad;
    
    float imu_yaw_offset_rad;
    
    float lidar_correction_max_mm;
    float lidar_correction_max_rad;
    uint16_t lidar_min_valid_points;
    uint16_t lidar_min_match_score;
  } m_config;
  
  // Diagnostics
  LocalisationDiags m_diags;
  
  // Helper methods
  float wrapAngleRad(float angle);
  float wrapAngleDeg(float angle);
  
  // Prediction step: integrate IMU gyro for heading, optical flow for position
  void predict(float dt_sec);
  
  // LiDAR correction: scan-to-map matching
  void correctWithLiDAR(LD06 &lidar, const MappingNav &nav);
  
  // Scan-to-map matcher: returns match score (0-1000) and correction (dx, dy, dtheta)
  uint16_t scanToMapMatch(LD06 &lidar,
                          const MappingNav &nav,
                          const RobotPose &predicted,
                          float &corr_x_mm,
                          float &corr_y_mm,
                          float &corr_theta_rad);
};

#endif // LOCALISATION_H_
