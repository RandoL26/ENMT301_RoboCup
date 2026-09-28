#ifndef SEEK_WEIGHT_H
#define SEEK_WEIGHT_H

#include <Arduino.h>
#include "target_detector.h"
#include "dc_motor.h"        // was "motors.h" (placeholder). DCMotor is the real motor class.
#include "ld06.h"
#include "optical_flow.h"
#include "MappingNav.h"
#include "imu_sensor.h"

//************************************
//   Seek Weight state - public API
//************************************
// Typical use in your main file:
//
//   SeekHardware hw = { &leftMotor, &rightMotor, &flow, &lidar, &detector, &nav };
//   seek_weight_init(hw, SEEK_START_BOTTOM_LEFT);      // once, after every begin()
//   ...
//   loop() {
//     SeekStatus s = seek_weight_update();              // every loop, never blocks
//     if (s == SEEK_TARGET_FOUND) { /* switch to Pick Up, read seek_weight_get_target_report() */ }
//   }

// What seek_weight_update() tells the caller.
enum SeekStatus : uint8_t {
  SEEK_RUNNING = 0,     // still searching -> keep calling seek_weight_update()
  SEEK_TARGET_FOUND,    // motors stopped; details in seek_weight_get_target_report()
  SEEK_STUCK,           // stall or planner failure; motors stopped; see seek_weight_get_debug().stuck_reason
  SEEK_AREA_COMPLETE    // every lane driven and nothing found; motors stopped
};

// Which bottom corner the robot starts in (it always starts facing the top of the arena).
enum SeekStartCorner : uint8_t {
  SEEK_START_BOTTOM_LEFT = 0,
  SEEK_START_BOTTOM_RIGHT = 1
};

// Objects owned by your main file. Seek Weight only borrows them.
// All of them must already be begin()-ed / initialised before seek_weight_init().
struct SeekHardware {
  DCMotor *leftMotor;
  DCMotor *rightMotor;
  OpticalFlow *flow;
  LD06 *lidar;
  TieredTargetDetector *detector;
  MappingNav *map;
};

// Why the state stopped with SEEK_STUCK.
enum SeekStuckReason : uint8_t {
  SEEK_STUCK_NONE = 0,
  SEEK_STUCK_STALL = 1,        // motors commanded but no motion seen (flow + gyro) for a while
  SEEK_STUCK_PLAN_FAILED = 2   // MappingNav::replanPath() returned false
};

// Snapshot handed to Pick Up when the target is confirmed.
struct SeekTargetReport {
  bool valid;
  MappingNav::Pose2D pose;   // drive-axle centre pose (arena metres, theta CCW from +x) when confirmed
  bool steering_active;      // detector only saw the target with one sensor of a tier
  float steering;            // detector steering value: -1 (full left) .. 0 .. +1 (full right)
  uint16_t near_avg_mm;      // detector near-tier average distance
  uint16_t far_avg_mm;       // detector far-tier average distance
  uint32_t time_ms;          // millis() at confirmation
};

// Everything worth looking at while debugging (printed as teleplot lines by the state).
struct SeekDebug {
  const char *sub_state;
  uint16_t goal_index, goal_count;
  uint16_t goal_x_cell, goal_y_cell;
  float x_m, y_m, theta_deg;         // fused pose
  float heading_err_deg;             // bearing to look-ahead point minus heading (+ = needs CCW turn)
  float look_x_m, look_y_m;          // point being steered towards
  float gyro_yaw_deg;                // accumulated gyro yaw (sign-corrected)
  float gyro_bias_dps;               // drift measured while standing still at the start
  float bno_heading_deg;             // BNO055 fused heading, shown for comparison only
  float enc_dist_mm;                 // this cycle, from encoders (0 if not calibrated)
  float flow_fwd_mm, flow_left_mm;   // this cycle, from optical flow (after rotation compensation)
  int32_t enc_left, enc_right;       // raw encoder counts (after sign)
  float cmd_left, cmd_right;         // motor commands after slew limiting, before motor sign
  TierDetectionResult detect;        // latest detector result
  uint8_t target_streak;             // consecutive cycles the detector said Target
  uint16_t obstacle_seen;            // cycles the detector said Obstacle (not acted on yet)
  uint16_t imu_glitches;             // IMU heading jumps rejected
  uint16_t replans;                  // number of times a path was planned
  uint32_t plan_us;                  // time the last replanPath() took
  uint32_t cycle_us;                 // time the last control cycle took
  SeekStuckReason stuck_reason;
};

// Set everything up and place the robot at its start pose. Call once.
void seek_weight_init(const SeekHardware &hardware, SeekStartCorner corner);

// Call every loop(). Cheap when it is not time for a control cycle yet.
SeekStatus seek_weight_update();

// Hard stop both motors right now (does not change the status).
void seek_weight_stop();

SeekStatus seek_weight_get_status();
const SeekTargetReport &seek_weight_get_target_report();
MappingNav::Pose2D seek_weight_get_base_pose();   // start pose, for a Return_to_base function
const SeekDebug &seek_weight_get_debug();
void seek_weight_set_debug_print(bool enable);    // teleplot + state-change log on Serial

#endif