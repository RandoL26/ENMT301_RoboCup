//************************************
//       POIDetector.h
//************************************
// POI (Point of Interest / Weight) detection via ToF sensor fusion
// with LiDAR mapping and occupancy grid

#ifndef POI_DETECTOR_H
#define POI_DETECTOR_H

#include <stdint.h>
#include <Arduino.h>

// Forward declarations
class MappingNav;
class TOFSensorArray;

//====================================
// CONFIGURATION CALIBRATION SECTION
//====================================

// VL53L1X ToF Sensor Extrinsics (MUST BE MEASURED FOR YOUR ROBOT!)
// These define the physical mounting position and orientation of the ToF sensor
// relative to the robot center. Units are meters and radians.
//
// x_offset_m: Forward/backward offset from robot center (+ is forward)
// y_offset_m: Left/right offset from robot center (+ is left)
// yaw_offset_rad: Rotation angle of sensor relative to robot heading (+ is CCW)

struct ToFExtrinsics {
    float x_offset_m = 0.0f;      // CALIBRATE: measure robot center to ToF sensor
    float y_offset_m = 0.0f;      // CALIBRATE: lateral offset
    float yaw_offset_rad = 0.0f;  // CALIBRATE: sensor rotation
};

// POI detection thresholds and parameters
namespace POIConfig {
    // Minimum range difference (m) between ToF and expected LiDAR
    // to trigger a candidate. Larger = more conservative.
    // Example: 0.1m = 100mm difference required
    static constexpr float POI_MIN_RANGE_DIFF_M = 0.1f;  // CALIBRATE
    
    // Number of consistent readings required to confirm a POI
    static constexpr uint16_t POI_CONFIRM_COUNT = 4;     // CALIBRATE
    
    // Time window (ms) for accumulating readings toward confirmation
    static constexpr uint32_t POI_CONFIRM_TIME_MS = 2000;  // CALIBRATE
    
    // Position tolerance (m): if detected position moves more than this
    // between readings, candidate is rejected (not spatially consistent)
    static constexpr float POI_POSITION_TOLERANCE_M = 0.15f;  // CALIBRATE
    
    // Maximum valid ToF range (m)
    static constexpr float POI_MAX_RANGE_M = 2.0f;        // VL53L1X typical max
    
    // Minimum number of LiDAR points to form a valid expected-range estimate
    static constexpr uint8_t POI_MIN_LIDAR_POINTS = 5;
    
    // If expected LiDAR range is uncertain (unknown cells, etc),
    // don't use as strong POI evidence
    static constexpr float POI_UNKNOWN_THRESHOLD_M = 0.5f;
    
    // Spatial merge radius (m): if two confirmed POIs are within this distance,
    // treat them as the same object
    static constexpr float POI_MERGE_RADIUS_M = 0.3f;
};

//====================================
// POI DETECTION STATE MACHINE
//====================================

enum POIState {
    POI_STATE_NONE,           // No candidate
    POI_STATE_CANDIDATE,      // Anomaly detected, awaiting confirmation
    POI_STATE_CONFIRMED,      // Multiple consistent readings, POI accepted
    POI_STATE_REJECTED        // Failed confirmation, waiting to reset
};

enum ExpectedRangeStatus {
    EXPECTED_RANGE_VALID,     // High confidence estimate from map
    EXPECTED_RANGE_UNKNOWN,   // Map cells in beam path are unknown
    EXPECTED_RANGE_BLOCKED,   // LiDAR endpoint beyond ToF range
    EXPECTED_RANGE_INVALID    // Ray outside grid or other error
};

//====================================
// DATA STRUCTURES
//====================================

struct POICandidate {
    // Current state
    POIState state = POI_STATE_NONE;
    
    // World position (meters) of detected object
    float x_m = 0.0f;
    float y_m = 0.0f;
    
    // Sensor readings
    float tof_range_m = 0.0f;           // Raw ToF measurement
    float expected_lidar_range_m = 0.0f; // LiDAR/map predicted range
    ExpectedRangeStatus lidar_status = EXPECTED_RANGE_INVALID;
    float range_difference_m = 0.0f;    // tof - expected
    
    // Confirmation tracking
    uint16_t confirmation_count = 0;    // Readings supporting this candidate
    uint32_t first_seen_ms = 0;        // Timestamp of first anomaly
    uint32_t last_seen_ms = 0;         // Timestamp of most recent reading
    
    // Confidence metric [0, 1000]
    float confidence = 0.0f;
    
    // Rejection tracking (if state == REJECTED)
    uint32_t rejection_time_ms = 0;
    uint8_t rejection_count = 0;  // How many times this region has failed
};

struct ConfirmedPOI {
    bool active = false;
    float x_m = 0.0f;
    float y_m = 0.0f;
    float confidence = 0.0f;     // [0, 1000]
    uint32_t timestamp_ms = 0;
    uint16_t confirmation_count = 0;
};

struct POIDetectorDiags {
    // ToF readings
    uint16_t tof_reading_count = 0;
    float tof_last_range_m = 0.0f;
    bool tof_last_valid = false;
    
    // LiDAR expected range
    float expected_range_m = 0.0f;
    ExpectedRangeStatus expected_status = EXPECTED_RANGE_INVALID;
    uint8_t expected_lidar_points = 0;
    
    // POI state
    POIState candidate_state = POI_STATE_NONE;
    float candidate_confidence = 0.0f;
    uint16_t candidate_count = 0;
    float candidate_x_m = 0.0f;
    float candidate_y_m = 0.0f;
    
    // Statistics
    uint16_t confirmed_poi_count = 0;
    uint16_t rejected_candidate_count = 0;
    uint16_t invalid_tof_reading_count = 0;
};

//====================================
// POI DETECTOR CLASS
//====================================

class POIDetector {
public:
    // Constructor
    POIDetector();
    
    // Initialize detector (call from robot_init)
    void begin(const ToFExtrinsics &extrinsics);
    
    // Main update: process latest ToF reading
    // Call this periodically (e.g., 10-20 Hz) when new ToF data is available
    // Requires:
    // - mappingNav: authoritative fused robot pose and occupancy map
    // - mappingNav: occupancy grid and map knowledge
    // - tofDistance_m: latest ToF measurement (0 = invalid/no return)
    void update(const MappingNav &mappingNav,
                float tofDistance_m);
    
    // Query current POI candidate state
    POICandidate getCandidateState() const;
    
    // Get list of confirmed POIs
    // Returns number of POIs populated into output array
    uint16_t getConfirmedPOIs(ConfirmedPOI *poi_array, uint16_t max_pois) const;
    
    // Get diagnostics for telemetry/debug
    POIDetectorDiags getDiags() const;
    
    // Manually set sensor extrinsics (for calibration/testing)
    void setExtrinsics(const ToFExtrinsics &ext);
    
    // Reset all state (start fresh)
    void reset();
    
    // Check if we have any confirmed POIs
    uint16_t getConfirmedPOICount() const;

private:
    // Configuration
    ToFExtrinsics m_extrinsics;
    
    // Current candidate
    POICandidate m_candidate;
    
    // Confirmed POIs (fixed array to avoid heap on Teensy)
    static constexpr uint8_t MAX_CONFIRMED_POIS = 10;
    ConfirmedPOI m_confirmed_pois[MAX_CONFIRMED_POIS];
    uint8_t m_confirmed_poi_count = 0;
    
    // Diagnostics
    POIDetectorDiags m_diags;
    
    // Internal state
    uint32_t m_last_update_ms = 0;
    
    // Helper methods
    
    // Calculate ToF sensor position in world frame given robot pose
    void calculateToFWorldPosition(const MappingNav &nav,
                                   float &out_x_m, float &out_y_m,
                                   float &out_yaw_rad) const;
    
    // Calculate expected LiDAR range along ToF beam direction
    // Returns expected range, status, and point count used
    void calculateExpectedLiDARRange(const MappingNav &nav,
                                     float sensor_x_m, float sensor_y_m,
                                     float sensor_yaw_rad,
                                     float &out_expected_range_m,
                                     ExpectedRangeStatus &out_status,
                                     uint8_t &out_point_count);
    
    // Calculate world position where ToF beam hit
    void calculateToFHitPosition(float sensor_x_m, float sensor_y_m,
                                 float sensor_yaw_rad, float tof_range_m,
                                 float &out_x_m, float &out_y_m) const;
    
    // Process anomaly: update candidate state based on latest reading
    void processAnomaly(const MappingNav &nav, float tof_range_m,
                        float expected_range_m, ExpectedRangeStatus status);
    
    // Try to confirm candidate if criteria met
    void checkConfirmation();
    
    // Try to merge with existing confirmed POI
    bool tryMergePOI(float x_m, float y_m, float confidence);
    
    // Check if new reading is spatially consistent with candidate
    bool isPositionConsistent(float new_x_m, float new_y_m) const;
};

#endif // POI_DETECTOR_H
