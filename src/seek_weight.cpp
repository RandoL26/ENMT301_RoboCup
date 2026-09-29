#include "seek_weight.h"
#include "odometry_config.h"
#include <math.h>

/*************************************************************************
 * Seek Weight state - Roam the field using LiDAR, Target detector, Optical Flow
 * Sensor and IMU.
 * to search for weights. This state will be the state the robot is mostly
 * in. 
 * Upon starting, an origin point will be made setting the Optical Flow
 * Sensor to (0, 0) so a function like Return_to_base can be called which
 * guides the robot back to the origin point (Base).
 * The robot will be moving forward initially whilst mapping the arena using
 * LiDAR and Optical Flow. The arena will be structured as grid blocks within 
 * the 2.4m x 4.9m boundary. Whilst this is happening the Target Detector 
 * will interrupt this seek state and swtich to Pick up Sequence. 
 * The robot will search this grid one by one taking note and mapping out the
 * arena using the LiDAR and Optical Flow Sensor. Movement will be check by comparing
 * Optical Flow sensor and IMU meaning that a projection of future desired position
 * can be calculated. Controls for acceleration, speed, and position should be
 * implemented.
 * 
 * Set speeds will be implemented eg. Search Speed, Pick up Speed, Turn Speed
 * to maximise scan accuracy whislt maintaining robot speed and smoothness.
 * 
 * Note: The sensors are placed off centre on the robot which could be a problem
 * movement wise. The robot should note its dimensions by defining a center point
 * in the robot and defining the position of the sensors. This will help with 
 * arena navigation as the robot will constantly compare its dimensions and bearing
 * with the arena avoid collision with obstacles. 
 *************************************************************************/

/*************************************************************************
 * HOW THIS FILE IS ORGANISED (read top to bottom, each section is small)
 *
 *   1. Settings ............ every number you might need to measure or tune
 *   2. Internal state ...... the variables the state remembers between calls
 *   3. Small helpers ....... angle wrap, clamp, slew limiter, cell <-> metres
 *   4. Lane plan ........... which grid cells to visit, in which order
 *   5. Sensors -> pose ..... wheel encoders + IMU -> MappingNav pose
 *   6. Detector check ...... "did the target detector see a weight?"
 *   7. Sub-states .......... CALIBRATE -> PLAN -> DRIVE <-> TURN -> BRAKE -> PLAN ...
 *   8. Motor output ........ slew limiting + motor signs
 *   9. Debug output ........ teleplot lines + state change log
 *  10. Public functions .... what the rest of the program calls
 *
 * One control cycle (every CONTROL_PERIOD_MS) does, in this order:
 *   read sensors -> update pose -> stall check -> detector check ->
 *   run the current sub-state (sets wanted wheel speeds) -> write motors -> debug
 *
 * Frames used in THIS file (same as MappingNav):
 *   arena x = across the 2.4 m width, arena y = along the 4.9 m length,
 *   origin = bottom-left corner, theta = 0 along +x, positive = counter-clockwise.
 *   Robot body: +x forward, +y left. Pose is the DRIVE-AXLE centre.
 *   Distances are metres here (MappingNav's units), the detector uses mm.
 *
 * NOT DONE ON PURPOSE (on hold): feeding LiDAR into the map. Until that exists
 * the map has no obstacles, so an "Obstacle" from the detector is only counted
 * (dbg.obstacle_seen), it does not change the robot's motion yet.
 *************************************************************************/


// ============================================================================
// 1. SETTINGS
//    Search for "MEASURE ME" - those are numbers I could not know.
// ============================================================================

// 1 = follow paths from MappingNav (D* Lite). 0 = skip the planner and steer
// straight at the goal cell (useful for bench-testing pose + motors first).
#define SEEK_USE_PLANNER 1

// ---- Robot geometry (pose reference = drive-axle centre) ------------------
static const float ROBOT_LENGTH_M   = 0.370f;
static const float ROBOT_WIDTH_M    = 0.210f;
static const float AXLE_TO_REAR_M   = 0.185f;  // MEASURE ME (0.185 = axle exactly at mid-length)
static const float START_REAR_GAP_M = 0.020f;  // MEASURE ME: gap between robot's rear and bottom wall at start

// ---- Motors and encoders --------------------------------------------------
// Sign: -1 if a POSITIVE command makes that side drive BACKWARDS.
static const int8_t LEFT_MOTOR_SIGN  = +1;     // MEASURE ME
static const int8_t RIGHT_MOTOR_SIGN = +1;     // MEASURE ME
// Sign: -1 if that encoder's count FALLS when that side drives forward.
static const int8_t LEFT_ENC_SIGN    = OdometryConfig::LEFT_ENCODER_SIGN;
static const int8_t RIGHT_ENC_SIGN   = OdometryConfig::RIGHT_ENCODER_SIGN;
// Push the robot forward exactly 1 m by hand and read the count. 0 = "not measured":
// encoder translation remains disabled until calibration is entered.
static const float ENCODER_COUNTS_PER_M = OdometryConfig::ENCODER_COUNTS_PER_M;
// Only used for encoder heading, which is weighted out by default (see HEADING_IMU_WEIGHT).
static const float ENCODER_TRACK_M      = OdometryConfig::ENCODER_TRACK_M;

// ---- Optical flow sensor --------------------------------------------------
// mm per count lives on the OpticalFlow object: flow.setScaleMMPerCount(...).
// Push the robot 1 m by hand and compare with the debug value flow_fwd_mm (summed).
// ---- IMU ------------------------------------------------------------------
// +1 if turning counter-clockwise (seen from above) makes imu.gyro_z INCREASE.
static const int8_t IMU_YAW_SIGN = OdometryConfig::IMU_YAW_SIGN;
// A single 20 ms cycle cannot turn this far. If it "does", the IMU read failed -> ignore it.
static const float  MAX_DTHETA_RAD = 0.5f;
// After init the robot must stand STILL for this long. The gyro angle that builds up meanwhile
// is pure bias (drift), which is then subtracted for the rest of the run.
static const uint32_t IMU_BIAS_CAL_MS = 1500;

// Heading: gyro only by default; calibrated wheel geometry can provide fallback.
static const float HEADING_IMU_WEIGHT = 1.0f;

// ---- Search pattern (lanes) -----------------------------------------------
// Lanes run along the 4.9 m length. Robot turns on the spot at each lane end, so the
// axle must stay further from a wall than the swept radius (~0.213 m for 370x210 mm)
// plus clearance. 5 cells = 0.275 m to the cell centre.  Strip NOT scanned next to each wall = 0.275 - 0.105 = ~0.17 m.
static const uint16_t EDGE_MARGIN_CELLS = 5;
// Largest sideways step between lanes, in 5 cm cells. 4 cells = 0.20 m, which is
// less than the detector's ~0.233 m swath (210 mm + 23 mm at the 700 mm cutoff), so lanes overlap.
static const uint16_t LANE_MAX_SPACING_CELLS = 4;

// ---- Speeds (percent of full scale, -100..100) - STARTING GUESSES, tune on the floor ----
static const float SEARCH_SPEED       = 40.0f;  // straight driving
static const float APPROACH_MIN_SPEED = 20.0f;  // slowest speed while creeping up to a goal
static const float TURN_SPEED         = 30.0f;  // fastest turn on the spot
static const float MIN_TURN_SPEED     = 15.0f;  // slowest turn (must beat motor dead-band)
static const float SPEED_SLEW_PER_CYCLE = 4.0f; // max change of a wheel command per cycle (acceleration limit)

// ---- Steering ---------------------------------------------------------------
static const float KP_HEADING      = 1.2f;   // steering trim (speed units) per degree of heading error while driving
static const float MAX_STEER_TRIM  = 25.0f;  // largest trim
static const float TURN_ENTER_DEG  = 30.0f;  // driving -> turning on the spot when error is bigger than this
static const float TURN_EXIT_DEG   = 4.0f;   // turning -> driving once error is smaller than this
static const float KP_TURN         = 0.6f;   // turn speed = KP_TURN * error(deg), limited to [MIN_TURN_SPEED, TURN_SPEED]

// ---- Following the path ---------------------------------------------------
static const float    GOAL_TOL_M     = 0.06f;  // "arrived" when the axle is this close to the goal cell centre
static const float    SLOWDOWN_DIST_M = 0.20f; // start slowing this far from the goal
static const uint16_t LOOKAHEAD_CELLS = 4;     // steer at the path cell this many cells ahead (0.20 m)
static const uint16_t MAX_PATH_CELLS  = 128;   // longest path copied from MappingNav (an empty-arena lane is ~90)

// ---- Timing -----------------------------------------------------------------
static const uint32_t CONTROL_PERIOD_MS = 20;   // 50 Hz control cycle
static const uint32_t DEBUG_PERIOD_MS   = 100;  // debug print rate
static const uint8_t  TARGET_CONFIRM_CYCLES = 3; // detector must say Target this many cycles in a row

// ---- Stall detection ------------------------------------------------------
static const float    STALL_MIN_CMD      = 15.0f;   // only check for stall while a wheel command is at least this big
static const uint32_t STALL_TIMEOUT_MS   = 2000;    // no motion for this long while commanded -> STUCK
static const int      STALL_MIN_FLOW_COUNTS = 1;    // flow counts per cycle that count as "moving"
static const float    STALL_MIN_TURN_DEG = 0.10f;   // gyro degrees per cycle that count as "moving"


// ============================================================================
// 2. INTERNAL STATE
// ============================================================================

// The little state machine inside Seek Weight.
enum SubState : uint8_t {
  SS_CALIBRATE,     // standing still for IMU_BIAS_CAL_MS, measuring gyro bias
  SS_PLAN,          // pick the next goal cell, ask MappingNav for a path
  SS_TURN,          // rotate on the spot until facing the look-ahead point
  SS_DRIVE,         // drive along the path, steering with a heading trim
  SS_BRAKE,         // reached a goal: ramp wheels to zero before turning
  SS_TARGET_FOUND,  // terminal
  SS_STUCK,         // terminal
  SS_DONE           // terminal: all lanes driven
};

static const char *subStateName(SubState s) {
  switch (s) {
    case SS_CALIBRATE:    return "CALIBRATE";
    case SS_PLAN:         return "PLAN";
    case SS_TURN:         return "TURN";
    case SS_DRIVE:        return "DRIVE";
    case SS_BRAKE:        return "BRAKE";
    case SS_TARGET_FOUND: return "TARGET_FOUND";
    case SS_STUCK:        return "STUCK";
    case SS_DONE:         return "DONE";
  }
  return "?";
}

typedef MappingNav::Pose2D Pose2D;

static SeekHardware hw = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
static bool initialised = false;
static bool debugPrintOn = true;

static SeekStartCorner startCorner = SEEK_START_BOTTOM_LEFT;
static SubState sub = SS_PLAN;
static SeekStatus status = SEEK_STUCK;   // "STUCK" until init has run
static SeekTargetReport targetReport;
static SeekDebug dbg;
static Pose2D basePose = {0.0f, 0.0f, 0.0f};

// Goal bookkeeping
static uint16_t goalIdx = 0;             // which goal of the lane plan we are on
static uint16_t goalX = 0, goalY = 0;    // current goal, in grid cells

// Path copied from MappingNav when we plan
static MappingNav::CellCoord pathBuf[MAX_PATH_CELLS];
static uint16_t pathLen = 0;
static uint16_t pathIdx = 0;             // nearest path cell ahead of the robot

// Wheel commands: "tgt" = what the sub-state wants, "cmd" = what we actually send (slew limited)
static float tgtLeft = 0.0f, tgtRight = 0.0f;
static float cmdLeft = 0.0f, cmdRight = 0.0f;

// Sensor bookkeeping (previous values, so we can take differences)
static int32_t prevEncL = 0, prevEncR = 0;
static float prevYawDeg = 0.0f;
static float gyroBiasDegPerS = 0.0f;   // measured in SS_CALIBRATE
static uint32_t calStartMs = 0;

// Timers and counters
static uint32_t lastCycleMs = 0, lastMotionMs = 0, lastDebugMs = 0;
static uint8_t targetStreak = 0;


// ============================================================================
// 3. SMALL HELPERS
// ============================================================================

static float wrapPi(float a) {
  while (a > (float)PI)  a -= 2.0f * (float)PI;
  while (a < -(float)PI) a += 2.0f * (float)PI;
  return a;
}

static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// Move 'current' towards 'wanted' by at most 'maxStep'. This is the acceleration limit.
static float slewToward(float current, float wanted, float maxStep) {
  return current + clampf(wanted - current, -maxStep, maxStep);
}

static float cellCentreM(uint16_t cell) {
  return ((float)cell + 0.5f) * MappingNav::CELL_SIZE_M;
}

// Smallest circle around the axle that contains the whole robot. Used as the planner's
// inflation radius, and it is also the clearance needed to turn on the spot.
static float robotRadiusM() {
  float front = ROBOT_LENGTH_M - AXLE_TO_REAR_M;
  float longer = (front > AXLE_TO_REAR_M) ? front : AXLE_TO_REAR_M;   // axle -> furthest end
  return sqrtf(longer * longer + (ROBOT_WIDTH_M * 0.5f) * (ROBOT_WIDTH_M * 0.5f));
}


// ============================================================================
// 4. LANE PLAN
//
//   Lanes run up and down the 4.9 m length. Goals alternate:
//     goal 0 = drive up lane 0          goal 1 = shift sideways to lane 1 (at the top)
//     goal 2 = drive down lane 1        goal 3 = shift sideways to lane 2 (at the bottom)
//     ... and so on until the last lane.
//   Lane columns are spread evenly between the two edge margins so no gap is
//   bigger than LANE_MAX_SPACING_CELLS. When starting bottom-right the columns
//   are mirrored, so lane 0 is always the lane the robot starts on.
// ============================================================================

static uint16_t laneFirstCol() { return EDGE_MARGIN_CELLS; }
static uint16_t laneLastCol()  { return (uint16_t)(MappingNav::GRID_WIDTH - 1 - EDGE_MARGIN_CELLS); }
static uint16_t rowLow()       { return EDGE_MARGIN_CELLS; }
static uint16_t rowHigh()      { return (uint16_t)(MappingNav::GRID_HEIGHT - 1 - EDGE_MARGIN_CELLS); }

static uint16_t laneCount() {
  uint16_t span = laneLastCol() - laneFirstCol();
  return (uint16_t)((span + LANE_MAX_SPACING_CELLS - 1) / LANE_MAX_SPACING_CELLS + 1);
}

static uint16_t goalCount() {
  return (uint16_t)(2 * laneCount() - 1);
}

// Grid column of a lane (0 = the lane the robot starts on).
static uint16_t laneCol(uint16_t lane) {
  uint16_t n = laneCount();
  uint16_t first = laneFirstCol();
  uint16_t span = laneLastCol() - first;
  uint16_t col = first + (uint16_t)(((uint32_t)lane * span + (n - 1) / 2) / (n - 1));  // rounded
  if (startCorner == SEEK_START_BOTTOM_RIGHT) {
    col = (uint16_t)(MappingNav::GRID_WIDTH - 1 - col);
  }
  return col;
}

// Convert goal number -> grid cell.
static void goalCell(uint16_t g, uint16_t &cx, uint16_t &cy) {
  uint16_t lane = g / 2;
  bool laneGoesUp = ((lane % 2) == 0);
  uint16_t endRow = laneGoesUp ? rowHigh() : rowLow();
  if ((g % 2) == 0) {
    cx = laneCol(lane);        // even goal: drive along this lane to its end
  } else {
    cx = laneCol(lane + 1);    // odd goal: step across to the next lane, same end
  }
  cy = endRow;
}


// ============================================================================
// 5. SENSORS -> POSE
// ============================================================================

// How the robot moved during ONE control cycle, in the robot body frame.
struct StepMotion {
  float enc_dx_m;
  float enc_left_delta_m;
  float enc_right_delta_m;
  float enc_dtheta_rad;
  float gyro_dtheta_rad;   // counter-clockwise positive
  bool imu_valid;
  int   flow_counts_abs;   // |dx|+|dy| raw counts (used for stall detection)
};

static StepMotion readMotion(float dtS) {
  StepMotion m = {0, 0, 0, 0, 0, false, 0};

  // --- IMU: heading change ------------------------------------------------
  // read_imu() returns instantaneous angular velocity in rad/s.
  IMU_Data imu = read_imu();
  m.imu_valid = imu.gyro_valid;
  float dYaw = imu.gyro_valid ? (float)IMU_YAW_SIGN * imu.gyro_z * dtS : 0.0f;
  if (!imu.gyro_valid || fabsf(dYaw) > MAX_DTHETA_RAD) {
    dbg.imu_glitches++;
    dYaw = 0.0f;
  }
  m.gyro_dtheta_rad = dYaw;
  dbg.gyro_yaw_deg += dYaw * (float)RAD_TO_DEG;
  dbg.bno_heading_deg = imu.euler_h;   // comparison only: BNO055 heading grows clockwise, gyro_z grows CCW

  // --- Encoders: forward distance and (weak) heading ---------------------
  int32_t encL = (int32_t)LEFT_ENC_SIGN  * hw.leftMotor->getEncoderPulses();
  int32_t encR = (int32_t)RIGHT_ENC_SIGN * hw.rightMotor->getEncoderPulses();
  dbg.enc_left = encL;
  dbg.enc_right = encR;
  if (ENCODER_COUNTS_PER_M > 0.0f) {
    float dL = (float)(encL - prevEncL) / ENCODER_COUNTS_PER_M;   // metres this cycle
    float dR = (float)(encR - prevEncR) / ENCODER_COUNTS_PER_M;
    m.enc_left_delta_m = dL;
    m.enc_right_delta_m = dR;
    m.enc_dx_m = 0.5f * (dL + dR);
    if (ENCODER_TRACK_M > 0.0f) m.enc_dtheta_rad = (dR - dL) / ENCODER_TRACK_M;
  }
  prevEncL = encL;
  prevEncR = encR;
  dbg.enc_dist_mm = m.enc_dx_m * 1000.0f;

  // --- Optical flow: forward / sideways distance --------------------------
  int16_t fx = 0, fy = 0;
  if (hw.flow->read(fx, fy)) {
    m.flow_counts_abs = (fx < 0 ? -fx : fx) + (fy < 0 ? -fy : fy);
    float mmPerCount = hw.flow->getScaleMMPerCount();
  }

  return m;
}

static void updatePose(const StepMotion &m) {
  MappingNav::PoseUpdateInput in;
  in.encoder_left_delta_m = m.enc_left_delta_m;
  in.encoder_right_delta_m = m.enc_right_delta_m;
  in.encoder_dtheta_rad = m.enc_dtheta_rad;
  in.imu_gyro_dtheta_rad = m.gyro_dtheta_rad;
  in.encoder_translation_valid = ENCODER_COUNTS_PER_M > 0.0f;
  in.encoder_heading_valid = ENCODER_COUNTS_PER_M > 0.0f && ENCODER_TRACK_M > 0.0f;
  in.imu_heading_valid = m.imu_valid;
  hw.map->updatePose(in, HEADING_IMU_WEIGHT);

  Pose2D p = hw.map->getPose();
  dbg.x_m = p.x_m;
  dbg.y_m = p.y_m;
  dbg.theta_deg = p.theta_rad * (float)RAD_TO_DEG;
}


// ============================================================================
// 6. DETECTOR CHECK
// ============================================================================

// Runs the tiered target detector. Returns true once a Target has been seen
// TARGET_CONFIRM_CYCLES cycles in a row (and fills targetReport).
static bool checkDetector(const Pose2D &pose) {
  // hw.lidar->readScan() was already called at the top of seek_weight_update().
  bool fresh = hw.detector->update();
  TierDetectionResult r = fresh ? hw.detector->getDetectionResult() : TierDetectionResult::None;
  dbg.detect = r;

  if (r == TierDetectionResult::Target) {
    if (targetStreak < 255) targetStreak++;
  } else {
    targetStreak = 0;
  }
  if (r == TierDetectionResult::Obstacle && dbg.obstacle_seen < 65535) dbg.obstacle_seen++;
  dbg.target_streak = targetStreak;

  if (targetStreak < TARGET_CONFIRM_CYCLES) return false;

  const TieredTargetDetector::DetectionDebug &d = hw.detector->getDebug();
  targetReport.valid = true;
  targetReport.pose = pose;
  targetReport.steering_active = d.steeringActive;
  targetReport.steering = d.steeringValue;
  targetReport.near_avg_mm = d.nearAverage;
  targetReport.far_avg_mm = d.farAverage;
  targetReport.time_ms = millis();
  return true;
}


// ============================================================================
// 7. SUB-STATES
// ============================================================================

static void hardStop() {
  hw.leftMotor->stop();
  hw.rightMotor->stop();
  cmdLeft = cmdRight = tgtLeft = tgtRight = 0.0f;
}

static void enterSub(SubState next) {
  if (next == sub) return;
  if (debugPrintOn) {
    Serial.print("[seek] ");
    Serial.print(subStateName(sub));
    Serial.print(" -> ");
    Serial.print(subStateName(next));
    Serial.print("  goal ");
    Serial.print(goalIdx + 1);
    Serial.print("/");
    Serial.print(goalCount());
    Serial.print(" cell (");
    Serial.print(goalX);
    Serial.print(",");
    Serial.print(goalY);
    Serial.println(")");
  }
  sub = next;
}

// Finish with a terminal status: stop the robot and remember why.
static void finish(SeekStatus s, SubState terminalSub) {
  hardStop();
  status = s;
  enterSub(terminalSub);
}

// Distance from the robot to path cell i.
static float distToPathCell(uint16_t i, const Pose2D &p) {
  return hypotf(cellCentreM(pathBuf[i].x) - p.x_m, cellCentreM(pathBuf[i].y) - p.y_m);
}

// Where to steer: a path cell LOOKAHEAD_CELLS ahead of the robot (or the goal itself
// if there is no path / we are near the end of it).
static void lookAheadPoint(const Pose2D &p, float goalXm, float goalYm, float &tx, float &ty) {
  if (pathLen == 0) {
    tx = goalXm;
    ty = goalYm;
    return;
  }
  // Move pathIdx forward while the next cell is at least as close as the current one.
  while ((uint16_t)(pathIdx + 1) < pathLen &&
         distToPathCell(pathIdx + 1, p) <= distToPathCell(pathIdx, p)) {
    pathIdx++;
  }
  uint16_t i = (uint16_t)(pathIdx + LOOKAHEAD_CELLS);
  if (i >= pathLen) i = (uint16_t)(pathLen - 1);
  tx = cellCentreM(pathBuf[i].x);
  ty = cellCentreM(pathBuf[i].y);
}

// Heading error (deg) to the look-ahead point; also fills the debug fields.
// Returns the distance to the goal in metres.
static float steeringError(const Pose2D &p, float &errDeg) {
  float gx = cellCentreM(goalX);
  float gy = cellCentreM(goalY);
  float tx, ty;
  lookAheadPoint(p, gx, gy, tx, ty);
  float bearing = atan2f(ty - p.y_m, tx - p.x_m);
  errDeg = wrapPi(bearing - p.theta_rad) * (float)RAD_TO_DEG;
  dbg.look_x_m = tx;
  dbg.look_y_m = ty;
  dbg.heading_err_deg = errDeg;
  return hypotf(gx - p.x_m, gy - p.y_m);
}

// ---- SS_CALIBRATE: robot stands still, measure the gyro's drift --------------
// The gyro angle is only ever integrated, so a small constant bias grows into a big heading
// error over a few minutes. Standing still, the angle that accumulates IS the bias.
static void runCalibrateGyro(uint32_t nowMs) {
  IMU_Data imu = read_imu();
  uint32_t elapsed = nowMs - calStartMs;
  if (elapsed < IMU_BIAS_CAL_MS) return;

  float yawDeg = (float)IMU_YAW_SIGN * imu.gyro_z;            // accumulated while stationary
  gyroBiasDegPerS = yawDeg / ((float)elapsed * 0.001f);
  prevYawDeg = yawDeg;

  // Forget any encoder / flow movement seen while calibrating (there should be none).
  prevEncL = (int32_t)LEFT_ENC_SIGN  * hw.leftMotor->getEncoderPulses();
  prevEncR = (int32_t)RIGHT_ENC_SIGN * hw.rightMotor->getEncoderPulses();
  int16_t dx = 0, dy = 0;
  hw.flow->read(dx, dy);
  hw.flow->resetTotals();
  lastMotionMs = nowMs;

  dbg.gyro_bias_dps = gyroBiasDegPerS;
  if (debugPrintOn) {
    Serial.print("[seek] gyro bias measured: ");
    Serial.print(gyroBiasDegPerS, 3);
    Serial.println(" deg/s (subtracted from now on)");
  }
  enterSub(SS_PLAN);
}

// ---- SS_PLAN: choose the goal, get a path -----------------------------------
static void runPlan() {
  goalCell(goalIdx, goalX, goalY);
  dbg.goal_index = goalIdx;
  dbg.goal_x_cell = goalX;
  dbg.goal_y_cell = goalY;
  pathLen = 0;
  pathIdx = 0;

#if SEEK_USE_PLANNER
  bool ok = hw.map->setGoalCell(goalX, goalY);
  if (ok) {
    uint32_t t0 = micros();
    ok = hw.map->replanPath(robotRadiusM());
    dbg.plan_us = micros() - t0;
  }
  if (!ok) {
    dbg.stuck_reason = SEEK_STUCK_PLAN_FAILED;
    if (debugPrintOn) {
      Serial.println("[seek] planner gave no path. If this happens on the very first plan, check MappingNav::setGoalCell()");
      Serial.println("[seek] (it must call plannerResetAll() BEFORE storing m_goal_idx / m_goal_set).");
    }
    finish(SEEK_STUCK, SS_STUCK);
    return;
  }
  pathLen = hw.map->getPathCells(pathBuf, MAX_PATH_CELLS);
#endif

  dbg.replans++;
  enterSub(SS_DRIVE);
}

// ---- SS_TURN: rotate on the spot -------------------------------------------
static void runTurn(const Pose2D &p) {
  float errDeg;
  steeringError(p, errDeg);

  if (fabsf(errDeg) < TURN_EXIT_DEG) {
    tgtLeft = tgtRight = 0.0f;
    enterSub(SS_DRIVE);
    return;
  }
  float speed = clampf(KP_TURN * fabsf(errDeg), MIN_TURN_SPEED, TURN_SPEED);
  float dir = (errDeg > 0.0f) ? 1.0f : -1.0f;   // + error = goal is counter-clockwise of us
  tgtLeft  = -dir * speed;
  tgtRight =  dir * speed;
}

// ---- SS_DRIVE: drive along the path -----------------------------------------
static void runDrive(const Pose2D &p) {
  float errDeg;
  float distGoal = steeringError(p, errDeg);

  if (distGoal < GOAL_TOL_M) {          // arrived
    tgtLeft = tgtRight = 0.0f;
    enterSub(SS_BRAKE);
    return;
  }

  // The copied path was cut short (longer than MAX_PATH_CELLS) and we reached its end: plan again from here.
  if (pathLen >= MAX_PATH_CELLS && pathIdx >= (uint16_t)(pathLen - 1)) {
    tgtLeft = tgtRight = 0.0f;
    enterSub(SS_PLAN);
    return;
  }

  if (fabsf(errDeg) > TURN_ENTER_DEG) {
    enterSub(SS_TURN);
    runTurn(p);
    return;
  }

  float forward = SEARCH_SPEED;
  if (distGoal < SLOWDOWN_DIST_M) {     // slow down smoothly near the goal
    forward = fmaxf(APPROACH_MIN_SPEED, SEARCH_SPEED * distGoal / SLOWDOWN_DIST_M);
  }
  float trim = clampf(KP_HEADING * errDeg, -MAX_STEER_TRIM, MAX_STEER_TRIM);
  tgtLeft  = forward - trim;            // + error = need to turn CCW = right side faster
  tgtRight = forward + trim;
}

// ---- SS_BRAKE: stop fully before turning ------------------------------------
static void runBrake() {
  tgtLeft = tgtRight = 0.0f;
  if (fabsf(cmdLeft) < 1.0f && fabsf(cmdRight) < 1.0f) {
    goalIdx++;
    if (goalIdx >= goalCount()) {
      finish(SEEK_AREA_COMPLETE, SS_DONE);
    } else {
      enterSub(SS_PLAN);
    }
  }
}


// ============================================================================
// 8. MOTOR OUTPUT
// ============================================================================

static void writeMotors() {
  cmdLeft  = slewToward(cmdLeft,  tgtLeft,  SPEED_SLEW_PER_CYCLE);
  cmdRight = slewToward(cmdRight, tgtRight, SPEED_SLEW_PER_CYCLE);
  dbg.cmd_left = cmdLeft;
  dbg.cmd_right = cmdRight;

  int16_t l = (int16_t)lroundf(clampf(cmdLeft  * (float)LEFT_MOTOR_SIGN,  -100.0f, 100.0f));
  int16_t r = (int16_t)lroundf(clampf(cmdRight * (float)RIGHT_MOTOR_SIGN, -100.0f, 100.0f));
  hw.leftMotor->setSpeed(l);
  hw.rightMotor->setSpeed(r);
}


// ============================================================================
// 9. DEBUG OUTPUT  (teleplot format: ">name:value" - see https://teleplot.fr/)
// ============================================================================

static void plot(const char *name, float v, uint8_t digits = 3) {
  Serial.print(">");
  Serial.print(name);
  Serial.print(":");
  Serial.println(v, digits);
}

static void printDebug() {
  Serial.print(">pose_xy:");
  Serial.print(dbg.x_m, 3);
  Serial.print(":");
  Serial.print(dbg.y_m, 3);
  Serial.println("|xy");
  plot("x_m", dbg.x_m);
  plot("y_m", dbg.y_m);
  plot("theta_deg", dbg.theta_deg, 1);
  plot("heading_err_deg", dbg.heading_err_deg, 1);
  plot("gyro_yaw_deg", dbg.gyro_yaw_deg, 1);
  plot("gyro_bias_dps", dbg.gyro_bias_dps, 3);
  plot("bno_heading_deg", dbg.bno_heading_deg, 1);
  plot("enc_dist_mm", dbg.enc_dist_mm, 2);
  plot("flow_fwd_mm", dbg.flow_fwd_mm, 2);
  plot("flow_left_mm", dbg.flow_left_mm, 2);
  plot("cmd_left", dbg.cmd_left, 1);
  plot("cmd_right", dbg.cmd_right, 1);
  plot("goal_index", (float)dbg.goal_index, 0);
  plot("detect", (float)(uint8_t)dbg.detect, 0);      // 0 None, 1 Target, 2 Obstacle, 3 Indeterminate
  plot("target_streak", (float)dbg.target_streak, 0);
  plot("imu_glitches", (float)dbg.imu_glitches, 0);
  plot("plan_us", (float)dbg.plan_us, 0);
  plot("cycle_us", (float)dbg.cycle_us, 0);
}


// ============================================================================
// 10. PUBLIC FUNCTIONS
// ============================================================================

void seek_weight_init(const SeekHardware &hardware, SeekStartCorner corner) {
  hw = hardware;
  initialised = false;
  status = SEEK_STUCK;
  if (!hw.leftMotor || !hw.rightMotor || !hw.flow || !hw.lidar || !hw.detector || !hw.map) {
    Serial.println("[seek] init failed: a SeekHardware pointer is null");
    return;
  }

  startCorner = corner;
  memset(&dbg, 0, sizeof(dbg));
  memset(&targetReport, 0, sizeof(targetReport));

  // Warn about numbers that have not been measured yet.
  if (ENCODER_COUNTS_PER_M <= 0.0f) Serial.println("[seek] Encoder scale not calibrated - encoder translation disabled");
  if (AXLE_TO_REAR_M <= 0.0f)       Serial.println("[seek] WARNING: AXLE_TO_REAR_M invalid");

  // Start pose: on lane 0, facing the top of the arena (+y, i.e. 90 degrees).
  float startX = cellCentreM(laneCol(0));
  float startY = AXLE_TO_REAR_M + START_REAR_GAP_M;
  hw.map->reset();
  hw.map->setPose(startX, startY, (float)HALF_PI);
  basePose = hw.map->getPose();

  // Zero every sensor reference at this pose.
  calibrate_gyroscope();                 // gyro angle := 0 here (so heading = start heading + gyro)
                                         // (this only zeroes the angle; the drift is measured in SS_CALIBRATE)
  prevYawDeg = 0.0f;
  hw.leftMotor->resetEncoderPulses();
  hw.rightMotor->resetEncoderPulses();
  prevEncL = prevEncR = 0;
  int16_t dx = 0, dy = 0;
  hw.flow->read(dx, dy);                 // flush counts that piled up before now
  hw.flow->resetTotals();

  // Reset the state machine.
  hardStop();
  goalIdx = 0;
  pathLen = pathIdx = 0;
  targetStreak = 0;
  sub = SS_CALIBRATE;
  gyroBiasDegPerS = 0.0f;
  calStartMs = millis();
  lastCycleMs = millis();
  lastMotionMs = lastCycleMs;
  lastDebugMs = lastCycleMs;
  status = SEEK_RUNNING;
  initialised = true;

  if (debugPrintOn) {
    Serial.print("[seek] init: ");
    Serial.print(laneCount());
    Serial.print(" lanes, ");
    Serial.print(goalCount());
    Serial.print(" goals, start (");
    Serial.print(startX, 3);
    Serial.print(", ");
    Serial.print(startY, 3);
    Serial.print(") m, robot radius ");
    Serial.print(robotRadiusM(), 3);
    Serial.println(" m");
  }
}

SeekStatus seek_weight_update() {
  if (!initialised) return status;

  // The LiDAR must be serviced as often as possible (its serial buffer fills quickly),
  // so this is NOT inside the 20 ms gate below.
  hw.lidar->readScan();

  if (status != SEEK_RUNNING) return status;   // a terminal state: motors are already stopped

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastCycleMs) < CONTROL_PERIOD_MS) return status;
  float dtS = (float)(uint32_t)(nowMs - lastCycleMs) * 0.001f;
  lastCycleMs = nowMs;
  uint32_t startUs = micros();

  // 0. First job after init: stand still and measure the gyro bias.
  if (sub == SS_CALIBRATE) {
    runCalibrateGyro(nowMs);
    dbg.sub_state = subStateName(sub);
    return status;
  }

  // 1. Where are we now?
  StepMotion motion = readMotion(dtS);
  updatePose(motion);
  Pose2D pose = hw.map->getPose();

  // 2. Stall check: wheels commanded but nothing moved (flow and gyro both quiet).
  bool moved = (motion.flow_counts_abs >= STALL_MIN_FLOW_COUNTS) ||
               (fabsf(motion.gyro_dtheta_rad * (float)RAD_TO_DEG) >= STALL_MIN_TURN_DEG);
  bool commanded = (fabsf(cmdLeft) >= STALL_MIN_CMD) || (fabsf(cmdRight) >= STALL_MIN_CMD);
  if (moved || !commanded) lastMotionMs = nowMs;
  if (commanded && (uint32_t)(nowMs - lastMotionMs) > STALL_TIMEOUT_MS) {
    dbg.stuck_reason = SEEK_STUCK_STALL;
    finish(SEEK_STUCK, SS_STUCK);
  }

  // 3. Did the detector find a weight? (Interrupts whatever we were doing.)
  if (status == SEEK_RUNNING && checkDetector(pose)) {
    finish(SEEK_TARGET_FOUND, SS_TARGET_FOUND);
  }

  // 4. Run the current sub-state. It only sets the wanted wheel speeds (tgtLeft / tgtRight).
  if (status == SEEK_RUNNING) {
    switch (sub) {
      case SS_PLAN:  runPlan();       break;
      case SS_DRIVE: runDrive(pose);  break;
      case SS_TURN:  runTurn(pose);   break;
      case SS_BRAKE: runBrake();      break;
      default: break;                 // terminal states never get here
    }
  }

  // 5. Send speeds to the motors (slew limited), unless a terminal state already stopped them.
  if (status == SEEK_RUNNING) writeMotors();

  // 6. Debug.
  dbg.sub_state = subStateName(sub);
  dbg.goal_count = goalCount();
  dbg.cycle_us = micros() - startUs;
  if (debugPrintOn && (uint32_t)(nowMs - lastDebugMs) >= DEBUG_PERIOD_MS) {
    lastDebugMs = nowMs;
    printDebug();
  }
  return status;
}

void seek_weight_stop() {
  if (initialised) hardStop();
}

SeekStatus seek_weight_get_status() {
  return status;
}

const SeekTargetReport &seek_weight_get_target_report() {
  return targetReport;
}

Pose2D seek_weight_get_base_pose() {
  return basePose;
}

const SeekDebug &seek_weight_get_debug() {
  return dbg;
}

void seek_weight_set_debug_print(bool enable) {
  debugPrintOn = enable;
}
