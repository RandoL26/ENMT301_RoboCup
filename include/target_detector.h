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

// ---------------------------------------------------------------------------
// Shared coordinate frame used by both the TOF sensors and the lidar.
//
// Convention (chosen to match the LD06's own coordinate system exactly, so
// its DataPoint.x/.y can be compared directly against a TOF hit point with
// no extra conversion):
//   - origin (0,0) : a fixed reference point on the robot chassis (see the
//                     defaults in target_detector.cpp for where this project
//                     currently puts it)
//   - +X            : straight ahead of the robot
//   - +Y            : to the robot's RIGHT
//   - angles (deg)  : measured CLOCKWISE from +X ("straight ahead")
// This matches the LD06 datasheet's coordinate system definition (left-handed,
// rotation center = origin, angle increases clockwise from the sensor's
// forward mark), so lidar points need no sign-flip to line up with TOF points.
// ---------------------------------------------------------------------------
struct GridPoint {
    float x = 0.0f;
    float y = 0.0f;
    bool valid = false;

    float distanceFromOrigin() const {
        return valid ? sqrtf(x * x + y * y) : -1.0f;
    }
    float bearingFromOriginDeg() const {
        return valid ? atan2f(y, x) * 180.0f / PI : 0.0f;
    }
};

// Fixed mounting definition for one TOF sensor: where it sits on the robot
// (relative to the shared origin above) and which way its boresight points.
// boresightDeg uses the same clockwise-from-+X convention as GridPoint.
struct SensorMount {
    float xMm = 0.0f;
    float yMm = 0.0f;
    float boresightDeg = 0.0f;

    // Build a mount at (mountXMm, mountYMm) whose boresight is aimed to
    // converge with the opposite sensor of its tier on the robot's
    // centerline (y = 0), given the RAW straight-line sensor-to-crossing
    // distance (i.e. what the TOF sensor itself would read at the design
    // crossing point -- this is the same quantity nearIntersectMm /
    // farIntersectMm represent below).
    static SensorMount aimedAtCenterlineCrossing(float mountXMm, float mountYMm, float rawIntersectMm) {
        SensorMount m;
        m.xMm = mountXMm;
        m.yMm = mountYMm;
        float underSqrt = rawIntersectMm * rawIntersectMm - mountYMm * mountYMm;
        float forwardLegMm = underSqrt > 0.0f ? sqrtf(underSqrt) : 0.0f;
        m.boresightDeg = atan2f(-mountYMm, forwardLegMm) * 180.0f / PI;
        return m;
    }
};

struct TieredTargetDetectorConfig {
    // --- Tier geometry validation (raw sensor-to-crossing distance, same
    //     role these fields had before -- used by pairMatch()) ---
    uint16_t nearIntersectMm = 100;
    uint16_t farIntersectMm = 175;
    uint16_t targetHeightMm = 70;
    uint16_t targetHeightToleranceMm = 20;
    uint16_t toleratedWidthMm = 50;
    uint16_t intersectionToleranceMm = 75;

    // --- TOF <-> lidar comparison (replaces the old top-sensor fields) ---
    uint16_t maxDetectionRangeMm = 700;    // hard cap: ignore TOF readings beyond this
    uint16_t lidarMatchToleranceMm = 60;   // |tofDist - lidarDist| <= this => same surface => Obstacle
    float lidarBearingToleranceDeg = 1.0f; // matching window for the lidar lookup at a given bearing
                                            // (LD06 does 4500 samples/sec over 360 deg, so this can
                                            // stay tight -- widen it if real-world returns are sparse)

    // --- Skeleton only: angular field-of-view gate for the lidar lookup.
    //     Not required for the 600mm cap to work; wire in useAngleGate=true
    //     once you've picked a cone (e.g. to ignore lidar returns from
    //     behind the robot). See findNearestLidarAtBearing() in the .cpp. ---
    float detectionAngleMinDeg = -90.0f;
    float detectionAngleMaxDeg = 90.0f;
    bool useAngleGate = false;
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
    bool isObstacleAgainstLidar(const GridPoint& tofPoint) const;
    bool findNearestLidarAtBearing(float bearingDeg, float toleranceDeg, float& outDistanceMm) const;
    float computeSteering(const GridPoint& hit, const SensorMount& mountA, const SensorMount& mountB) const;
};

#endif // TIER_TARGET_DETECTOR_H