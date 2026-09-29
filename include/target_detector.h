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

    // --- Tier geometry validation ---
    // These are the expected raw sensor-to-crossing distances used
    // by pairMatch().
    uint16_t nearIntersectMm = 200;
    uint16_t farIntersectMm = 400;

    uint16_t targetHeightMm = 70;
    uint16_t targetHeightToleranceMm = 20;

    uint16_t toleratedWidthMm = 50;
    uint16_t intersectionToleranceMm = 75;

    // --- TOF <-> LiDAR comparison ---
    uint16_t maxDetectionRangeMm = 700;

    // If the LiDAR return and ToF POI are at approximately the same
    // range, treat them as the same physical object.
    uint16_t lidarMatchToleranceMm = 60;

    float lidarBearingToleranceDeg = 1.0f;

    // --- Separate LiDAR obstacle ---
    //
    // A separate LiDAR cluster provides additional evidence that the
    // ToF POI is a target rather than the same object seen by LiDAR.
    //
    // These are deliberately conservative starting values for testing.
    uint16_t lidarObstacleMinMm = 80;
    uint16_t lidarObstacleMaxMm = 700;

    // Require at least 3 LiDAR points to form a separate obstacle.
    uint8_t minimumSeparateLidarPoints = 3;

    // The LiDAR cluster must be at least this far from the ToF POI.
    uint16_t separateObstacleDistanceMm = 100;

    // Maximum distance between LiDAR points for them to belong
    // to the same obstacle cluster.
    uint16_t lidarClusterRadiusMm = 80;

    // --- LiDAR angular field-of-view gate ---
    //
    // This remains available for the existing LiDAR lookup.
    // We are NOT using an additional angle gate for the separate
    // obstacle detection yet.
    float detectionAngleMinDeg = -90.0f;
    float detectionAngleMaxDeg = 90.0f;
    bool useAngleGate = false;

    // -----------------------------------------------------------------------
    // Target confidence
    //
    // Confidence is bounded from 0..100.
    //
    // ToF is the primary source of target evidence.
    // Temporal consistency strengthens the candidate.
    // A separate LiDAR obstacle adds evidence.
    // No LiDAR return gives NO penalty.
    // Same-object LiDAR is contradictory evidence.
    // Large unexpected movement is strongly contradictory.
    // -----------------------------------------------------------------------

    // ToF evidence
    uint8_t farDetectionConfidence = 10;
    uint8_t nearDetectionConfidence = 20;

    // Temporal evidence
    uint8_t positionConsistencyConfidence = 5;

    // Separate-LiDAR evidence
    uint8_t lidarConfidence = 10;

    // Confirmation thresholds
    uint8_t targetConfirmThreshold = 70;
    uint8_t candidateExpireThreshold = 20;

    // Confidence decay applied during each valid candidate update.
    uint8_t confidenceDecay = 2;

    // Penalty when LiDAR identifies the same physical object
    // as the ToF POI.
    uint8_t sameObjectPenalty = 30;

    // Penalty when the candidate moves unexpectedly.
    uint8_t movingObjectPenalty = 0;

    // State thresholds
    uint16_t candidateTimeoutMs = 500;
    uint16_t positionToleranceMm = 60;

    uint8_t farCandidateThreshold = 20;
    uint8_t approachingThreshold = 40;
    uint8_t nearConfirmThreshold = 70;

    // Distance/state behaviour
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

    // Override the LiDAR offset from the shared origin.
    void setLidarOffset(float xMm, float yMm, float angleDeg);

    // Pull fresh readings from the TOF array + the LiDAR's last completed
    // scan and re-run detection. Call lidar.readScan() yourself first.
    bool update();

    TierDetectionResult getDetectionResult() const;
    bool isTargetConfirmed() const;
    bool isObstacleDetected() const;
    bool hasValidData() const;

    const char* resultToString(TierDetectionResult result) const;

    void debugPrint() const;
    void debugPrintGrid(Stream& serialport) const;
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

        // LiDAR confirms that the ToF POI is the same physical object.
        bool nearLidarConfirmsObstacle;
        bool farLidarConfirmsObstacle;

        // LiDAR sees a separate physical obstacle from the ToF POI.
        bool separateLidarObstacle;

        GridPoint nearAPoint;
        GridPoint nearBPoint;
        GridPoint farAPoint;
        GridPoint farBPoint;

        bool steeringActive;
        float steeringValue; // -1 = full left, 0 = centered, +1 = full right

        uint8_t targetConfidence;
        bool targetLatched;
        bool movingObjectDetected;

        bool positionConsistent;
        bool farNearConsistent;
        bool largeMovement;
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

    bool pairMatch(uint16_t aMm,
                   uint16_t bMm,
                   uint16_t expectedMm) const;

    uint16_t averageDistance(uint16_t aMm,
                              uint16_t bMm) const;

    bool withinDetectionEnvelope(uint16_t rangeMm) const;

    GridPoint tofPointFromRange(const SensorMount& mount,
                                uint16_t rangeMm) const;

    GridPoint midpoint(const GridPoint& a,
                       const GridPoint& b) const;

    GridPoint lidarPointToRobotGrid(const DataPoint& point) const;

    // Returns true when LiDAR sees the same physical object
    // as the supplied ToF POI.
    bool isObstacleAgainstLidar(const GridPoint& tofPoint) const;

    // Returns true when LiDAR sees a separate cluster of at least
    // minimumSeparateLidarPoints points that is sufficiently far
    // from the supplied ToF POI.
    bool hasSeparateLidarObstacle(const GridPoint& tofPoint) const;

    bool findNearestLidarAtBearing(float bearingDeg,
                                   float toleranceDeg,
                                   float& outDistanceMm) const;

    float computeSteering(const GridPoint& hit,
                          const SensorMount& mountA,
                          const SensorMount& mountB) const;

    uint8_t targetConfidence;

    bool targetLatched;
    bool movingObjectDetected;

    TargetTrackingState trackingState;

    GridPoint lastCandidatePoint;
    uint32_t lastCandidateUpdateMs;
    bool lastCandidateWasFar;
};

#endif // TIER_TARGET_DETECTOR_H