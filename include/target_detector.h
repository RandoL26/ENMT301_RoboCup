#ifndef TIER_TARGET_DETECTOR_H
#define TIER_TARGET_DETECTOR_H

#include <Arduino.h>
#include "tof_sensor_array.h"
#include "ld06.h"

enum class TierDetectionResult {
    None,
    Target,
    Obstacle,
    Indeterminate
};

enum class TargetTrackingState {
    NONE,
    FAR_CANDIDATE,
    APPROACHING,
    NEAR_CONFIRMING,
    TARGET_CONFIRMED,
    MOVING_OBJECT
};

// ---------------------------------------------------------------------------
// Shared coordinate frame used by both the TOF sensors and the lidar.
//
// Robot coordinate convention:
//   - origin (0,0) : midpoint of the robot
//   - +X            : robot's RIGHT
//   - -X            : robot's LEFT
//   - +Y            : toward the REAR of the robot
//   - -Y            : toward the FRONT of the robot
//
// All dimensions are in mm.
//
// TOF sensor positions are measured relative to this origin.
// ---------------------------------------------------------------------------
struct GridPoint {
    float x = 0.0f;
    float y = 0.0f;
    bool valid = false;

    float distanceFromOrigin() const {
        return valid ? sqrtf(x * x + y * y) : -1.0f;
    }
    float bearingFromOriginDeg() const {
    // Robot frame:
    //   0°   = front (-Y)
    //   +90° = right (+X)
    //   -90° = left (-X)
    return valid ? atan2f(x, -y) * 180.0f / PI : 0.0f;
    }
};

struct SensorMount {
    float xMm = 0.0f;
    float yMm = 0.0f;
    float boresightDeg = 0.0f;
};

struct TieredTargetDetectorConfig {
    // --- Tier geometry validation (raw sensor-to-crossing distance, same
    //     role these fields had before -- used by pairMatch()) ---
    uint16_t nearIntersectMm = 200;
    uint16_t farIntersectMm = 400;
    uint16_t targetHeightMm = 70;
    uint16_t targetHeightToleranceMm = 20;
    uint16_t toleratedWidthMm = 50;
    uint16_t intersectionToleranceMm = 75;

    // --- TOF <-> lidar comparison (replaces the old top-sensor fields) ---
    uint16_t maxDetectionRangeMm = 700;    // hard cap: ignore TOF readings beyond this
    uint16_t lidarMatchToleranceMm = 60;   // |tofDist - lidarDist| <= this => same surface => Obstacle
    float lidarBearingToleranceDeg = 1.0f;

    // --- Separate LiDAR obstacle ---
    // A target candidate requires the LiDAR to see a different physical
    // object from the ToF POI. The separate object must contain at least
    // this many nearby LiDAR returns.
    uint16_t lidarObstacleMinMm = 80;
    uint16_t lidarObstacleMaxMm = 700;
    uint8_t minimumSeparateLidarPoints = 3;
    uint16_t separateObstacleDistanceMm = 100;
    uint16_t lidarClusterRadiusMm = 80;

    // Only the separate-obstacle test uses this gate. Keeping it separate
    // from useAngleGate means the existing ToF <-> LiDAR same-object test
    // does not change behaviour unexpectedly.
    bool useSeparateObstacleAngleGate = true;
    float separateObstacleAngleMinDeg = -90.0f;
    float separateObstacleAngleMaxDeg = 90.0f; // matching window for the lidar lookup at a given bearing
                                            // (LD06 does 4500 samples/sec over 360 deg, so this can
                                            // stay tight -- widen it if real-world returns are sparse)

    // --- Skeleton only: angular field-of-view gate for the lidar lookup.
    //     Not required for the 600mm cap to work; wire in useAngleGate=true
    //     once you've picked a cone (e.g. to ignore lidar returns from
    //     behind the robot). See findNearestLidarAtBearing() in the .cpp. ---
    float detectionAngleMinDeg = -90.0f;
    float detectionAngleMaxDeg = 90.0f;
    bool useAngleGate = false;

    // --- Target confidence ---
    uint8_t farDetectionConfidence = 20;
    uint8_t nearDetectionConfidence = 35;
    uint8_t positionConsistencyConfidence = 15;
    uint8_t lidarConfidence = 10;

    uint8_t targetConfirmThreshold = 70;
    uint8_t candidateExpireThreshold = 20;

    uint16_t candidateTimeoutMs = 500;
    uint16_t positionToleranceMm = 60;

    uint8_t farCandidateThreshold = 20;
    uint8_t approachingThreshold = 40;
    uint8_t nearConfirmThreshold = 70;

    uint16_t approachingDistanceToleranceMm = 80;
    uint16_t nearConfirmationDistanceMm = 275;
    uint16_t movingObjectThresholdMm = 100;

};

class TieredTargetDetector {
public:
    TieredTargetDetector(TOFSensorArray& sensorArray,
                         LD06& sensorLidar,
                         uint8_t nearAIndex,
                         uint8_t nearBIndex,
                         uint8_t farAIndex,
                         uint8_t farBIndex);

    void setConfig(const TieredTargetDetectorConfig& config);
    void setExpectedIntersections(uint16_t nearMm, uint16_t farMm);

    // Override the physical layout defaults computed in the constructor.
    void setSensorMounts(const SensorMount& nearA, const SensorMount& nearB,
                          const SensorMount& farA, const SensorMount& farB);

    // Override the lidar's offset from the shared origin (also set once,
    // automatically, in the constructor -- call this again if you re-measure).
    void setLidarOffset(float xMm, float yMm, float angleDeg);

    // Pull fresh readings from the TOF array + the lidar's last completed
    // scan and re-run detection. Call lidar.readScan() yourself first (same
    // as before) -- this no longer takes a topDistance parameter.
    bool update();

    TierDetectionResult getDetectionResult() const;
    bool isTargetConfirmed() const;
    bool isObstacleDetected() const;
    bool hasValidData() const;
    const char* resultToString(TierDetectionResult result) const;
    void debugPrint() const;
    void debugPrintGrid(Stream& serialport) const; // teleplot-style xy dump, see ld06.cpp printScanTeleplot
    void debugPrintLidarFrontTest() const;

    uint8_t getTargetConfidence() const;
    bool isTargetLatched() const;
    bool isMovingObjectDetected() const;

    TargetTrackingState getTrackingState() const;
    const char* trackingStateToString(TargetTrackingState state) const;

    struct DetectionDebug {
        uint16_t nearA;
        uint16_t nearB;
        uint16_t farA;
        uint16_t farB;
        uint16_t nearAverage;
        uint16_t farAverage;
        uint16_t objectDistance;
        bool nearPairMatch;
        bool farPairMatch;
        bool nearLidarConfirmsObstacle;
        bool farLidarConfirmsObstacle;
        GridPoint nearAPoint;
        GridPoint nearBPoint;
        GridPoint farAPoint;
        GridPoint farBPoint;
        bool steeringActive;
        float steeringValue; // -1 (full left) .. 0 (centered) .. +1 (full right)
        uint8_t targetConfidence;
        bool targetLatched;
        bool movingObjectDetected;
    };

    const DetectionDebug& getDebug() const;

private:
    TOFSensorArray& tofArray;
    LD06& lidar;
    uint8_t sensorNearA;
    uint8_t sensorNearB;
    uint8_t sensorFarA;
    uint8_t sensorFarB;
    TieredTargetDetectorConfig config;
    TierDetectionResult lastResult;
    DetectionDebug debug;

    SensorMount mountNearA;
    SensorMount mountNearB;
    SensorMount mountFarA;
    SensorMount mountFarB;

    static constexpr uint16_t INVALID_DISTANCE = 0xFFFF;

    bool isDistanceValid(uint16_t mm) const;
    bool pairMatch(uint16_t aMm, uint16_t bMm, uint16_t expectedMm) const;
    uint16_t averageDistance(uint16_t aMm, uint16_t bMm) const;

    bool withinDetectionEnvelope(uint16_t rangeMm) const;
    GridPoint tofPointFromRange(const SensorMount& mount, uint16_t rangeMm) const;
    GridPoint midpoint(const GridPoint& a, const GridPoint& b) const;

    GridPoint lidarPointToRobotGrid(const DataPoint& point) const;

    bool isObstacleAgainstLidar(const GridPoint& tofPoint) const;
    bool hasSeparateLidarObstacle(const GridPoint& tofPoint) const;
    bool findNearestLidarAtBearing(float bearingDeg, float toleranceDeg, float& outDistanceMm) const;
    float computeSteering(const GridPoint& hit, const SensorMount& mountA, const SensorMount& mountB) const;

    uint8_t targetConfidence;
    bool targetLatched;
    bool movingObjectDetected;

    TargetTrackingState trackingState;

    GridPoint lastCandidatePoint;
    uint32_t lastCandidateUpdateMs;
};

#endif // TIER_TARGET_DETECTOR_H