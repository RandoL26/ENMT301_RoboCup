#ifndef TIER_TARGET_DETECTOR_H
#define TIER_TARGET_DETECTOR_H

#include <Arduino.h>
#include "tof_sensor_array.h"

enum class TierDetectionResult {
    None,
    Target,
    Obstacle,
    Indeterminate
};

struct TieredTargetDetectorConfig {
    uint16_t nearIntersectMm = 300;
    uint16_t farIntersectMm = 500;
    uint16_t targetHeightMm = 70;
    uint16_t targetHeightToleranceMm = 20;
    uint16_t toleratedWidthMm = 50;
    uint16_t intersectionToleranceMm = 50;
    uint16_t topSensorClearanceMm = 80;
    uint16_t topSensorRejectMarginMm = 40;
};

class TieredTargetDetector {
public:
    TieredTargetDetector(TOFSensorArray& sensorArray,
                         uint8_t nearAIndex,
                         uint8_t nearBIndex,
                         uint8_t farAIndex,
                         uint8_t farBIndex);

    void setConfig(const TieredTargetDetectorConfig& config);
    void setExpectedIntersections(uint16_t nearMm, uint16_t farMm);
    void setTopSensorThreshold(uint16_t clearanceMm, uint16_t rejectMarginMm);

    bool update(uint16_t topDistance);
    TierDetectionResult getDetectionResult() const;
    bool isTargetConfirmed() const;
    bool isObstacleDetected() const;
    bool hasValidData() const;
    const char* resultToString(TierDetectionResult result) const;
    void debugPrint() const;

    struct DetectionDebug {
        uint16_t nearA;
        uint16_t nearB;
        uint16_t farA;
        uint16_t farB;
        uint16_t top;
        uint16_t nearAverage;
        uint16_t farAverage;
        uint16_t objectDistance;
        bool nearPairMatch;
        bool farPairMatch;
        bool topIndicatesObstacle;
    };

    const DetectionDebug& getDebug() const;

private:
    TOFSensorArray& tofArray;
    uint8_t sensorNearA;
    uint8_t sensorNearB;
    uint8_t sensorFarA;
    uint8_t sensorFarB;
    TieredTargetDetectorConfig config;
    TierDetectionResult lastResult;
    DetectionDebug debug;

    static constexpr uint16_t INVALID_DISTANCE = 0xFFFF;

    bool isDistanceValid(uint16_t mm) const;
    bool pairMatch(uint16_t aMm, uint16_t bMm, uint16_t expectedMm) const;
    bool topSensorIndicatesObstacle(uint16_t topMm, uint16_t objectMm) const;
    uint16_t averageDistance(uint16_t aMm, uint16_t bMm) const;
};

#endif // TIER_TARGET_DETECTOR_H
