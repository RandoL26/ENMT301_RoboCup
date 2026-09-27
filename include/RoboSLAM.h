// Lightweight on-board SLAM-like integrator that feeds MappingNav
#ifndef ROBOSLAM_H_
#define ROBOSLAM_H_

#include "MappingNav.h"
#include "ld06.h"
#include "motor_control.h"
#include "optical_flow.h"
#include "imu_sensor.h"
#include "BNO055_support.h"

class RoboSLAM {
public:
  RoboSLAM(MappingNav &nav, MotorControl &motors, OpticalFlow &flow);
  void begin();
  // Called when a full LD06 scan is available
  void processScan(LD06 &ld, const IMU_Data &imu);

  // Configuration
  void setEncoderPulsesPerMeter(float p) { pulses_per_meter = p; }
  void setWheelBaseMeters(float wb) { wheel_base_m = wb; }

private:
  MappingNav &m_nav;
  MotorControl &m_motors;
  OpticalFlow &m_flow;

  int32_t prev_left_pulses = 0;
  int32_t prev_right_pulses = 0;
  float prev_flow_x_mm = 0.0f;
  float prev_flow_y_mm = 0.0f;
  float prev_imu_heading_deg = 0.0f;

  // Tunables (sane defaults, please adjust for your robot)
  float pulses_per_meter = 1000.0f; // encoder pulses per linear metre
  float wheel_base_m = 0.20f;       // distance between wheels
};

#endif // ROBOSLAM_H_
