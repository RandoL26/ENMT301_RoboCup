#include "target_detector.h"

TieredTargetDetector::TieredTargetDetector(TOFSensorArray& sensorArray,
                                           uint8_t nearAIndex,
                                           uint8_t nearBIndex,
                                           uint8_t farAIndex,
                                           uint8_t farBIndex)
    : tofArray(sensorArray),
      sensorNearA(nearAIndex),
      sensorNearB(nearBIndex),
      sensorFarA(farAIndex),
      sensorFarB(farBIndex),
      lastResult(TierDetectionResult::None) {
    debug = { INVALID_DISTANCE, INVALID_DISTANCE, INVALID_DISTANCE, INVALID_DISTANCE,
              INVALID_DISTANCE, INVALID_DISTANCE, INVALID_DISTANCE, INVALID_DISTANCE,
              false, false, false };
}

void TieredTargetDetector::setConfig(const TieredTargetDetectorConfig& newConfig) {
    config = newConfig;
}

void TieredTargetDetector::setExpectedIntersections(uint16_t nearMm, uint16_t farMm) {
    config.nearIntersectMm = nearMm;
    config.farIntersectMm = farMm;
}

void TieredTargetDetector::setTopSensorThreshold(uint16_t clearanceMm, uint16_t rejectMarginMm) {
    config.topSensorClearanceMm = clearanceMm;
    config.topSensorRejectMarginMm = rejectMarginMm;
}

bool TieredTargetDetector::update(uint16_t topDistance) {
    if (!tofArray.isInitialized()) {
        lastResult = TierDetectionResult::None;
        return false;
    }

    uint16_t nearA = tofArray.getDistance(sensorNearA);
    uint16_t nearB = tofArray.getDistance(sensorNearB);
    uint16_t farA = tofArray.getDistance(sensorFarA);
    uint16_t farB = tofArray.getDistance(sensorFarB);

    debug.nearA = nearA;
    debug.nearB = nearB;
    debug.farA = farA;
    debug.farB = farB;
    debug.top = topDistance;
    debug.nearAverage = averageDistance(nearA, nearB);
    debug.farAverage = averageDistance(farA, farB);
    debug.objectDistance = INVALID_DISTANCE;
    debug.nearPairMatch = pairMatch(nearA, nearB, config.nearIntersectMm);
    debug.farPairMatch = pairMatch(farA, farB, config.farIntersectMm);
    if (debug.nearPairMatch && debug.farPairMatch) {
        debug.objectDistance = debug.nearAverage < debug.farAverage ? debug.nearAverage : debug.farAverage;
    } else if (debug.nearPairMatch) {
        debug.objectDistance = debug.nearAverage;
    } else if (debug.farPairMatch) {
        debug.objectDistance = debug.farAverage;
    }
    debug.topIndicatesObstacle = topSensorIndicatesObstacle(debug.top, debug.objectDistance);

    if (debug.nearPairMatch && debug.farPairMatch) {
        if (debug.topIndicatesObstacle) {
            lastResult = TierDetectionResult::Obstacle;
        } else {
            lastResult = TierDetectionResult::Target;
        }
    } else if (debug.nearPairMatch || debug.farPairMatch) {
        if (debug.topIndicatesObstacle) {
            lastResult = TierDetectionResult::Obstacle;
        } else {
            lastResult = TierDetectionResult::Indeterminate;
        }
    } else {
        lastResult = TierDetectionResult::None;
    }

    return true;
}

TierDetectionResult TieredTargetDetector::getDetectionResult() const {
    return lastResult;
}

bool TieredTargetDetector::isTargetConfirmed() const {
    return lastResult == TierDetectionResult::Target;
}

bool TieredTargetDetector::isObstacleDetected() const {
    return lastResult == TierDetectionResult::Obstacle;
}

bool TieredTargetDetector::hasValidData() const {
    return isDistanceValid(debug.nearA) && isDistanceValid(debug.nearB) &&
           isDistanceValid(debug.farA) && isDistanceValid(debug.farB);
}

const char* TieredTargetDetector::resultToString(TierDetectionResult result) const {
    switch (result) {
        case TierDetectionResult::None:
            return "None";
        case TierDetectionResult::Target:
            return "Target";
        case TierDetectionResult::Obstacle:
            return "Obstacle";
        case TierDetectionResult::Indeterminate:
            return "Indeterminate";
    }
    return "Unknown";
}

void TieredTargetDetector::debugPrint() const {
    Serial.print("TierDetector nearA:"); Serial.print(debug.nearA == INVALID_DISTANCE ? 0xFFFF : debug.nearA);
    Serial.print(" nearB:"); Serial.print(debug.nearB == INVALID_DISTANCE ? 0xFFFF : debug.nearB);
    Serial.print(" farA:"); Serial.print(debug.farA == INVALID_DISTANCE ? 0xFFFF : debug.farA);
    Serial.print(" farB:"); Serial.print(debug.farB == INVALID_DISTANCE ? 0xFFFF : debug.farB);
    Serial.print(" top:"); Serial.print(debug.top == INVALID_DISTANCE ? 0xFFFF : debug.top);
    Serial.print(" nearAvg:"); Serial.print(debug.nearAverage == INVALID_DISTANCE ? 0xFFFF : debug.nearAverage);
    Serial.print(" farAvg:"); Serial.print(debug.farAverage == INVALID_DISTANCE ? 0xFFFF : debug.farAverage);
    Serial.print(" objDist:"); Serial.print(debug.objectDistance == INVALID_DISTANCE ? 0xFFFF : debug.objectDistance);
    Serial.print(" pairNear:"); Serial.print(debug.nearPairMatch);
    Serial.print(" pairFar:"); Serial.print(debug.farPairMatch);
    Serial.print(" topObs:"); Serial.print(debug.topIndicatesObstacle);
    Serial.print(" result:"); Serial.println(resultToString(lastResult));
}

const TieredTargetDetector::DetectionDebug& TieredTargetDetector::getDebug() const {
    return debug;
}

bool TieredTargetDetector::isDistanceValid(uint16_t mm) const {
    return mm != INVALID_DISTANCE && mm != 0;
}

bool TieredTargetDetector::pairMatch(uint16_t aMm, uint16_t bMm, uint16_t expectedMm) const {
    if (!isDistanceValid(aMm) || !isDistanceValid(bMm)) {
        return false;
    }

    int32_t avg = (int32_t(aMm) + int32_t(bMm)) / 2;
    return (avg >= int32_t(expectedMm) - config.intersectionToleranceMm) &&
           (avg <= int32_t(expectedMm) + config.intersectionToleranceMm);
}

bool TieredTargetDetector::topSensorIndicatesObstacle(uint16_t topMm, uint16_t objectMm) const {
    if (!isDistanceValid(objectMm)) {
        return false;
    }

    if (!isDistanceValid(topMm)) {
        return false; // timeout or no return above the target height means the detected object is likely below the top sensor plane
    }

    // If the top sensor sees the object at almost the same distance as the tier match,
    // then the object extends above the target plane and should be treated as an obstacle.
    if (topMm <= objectMm + config.topSensorRejectMarginMm) {
        return true;
    }

    // If the top sensor returns a close distance even though the tier sensors see the object,
    // then the object is taller than the 70mm target height and should be rejected.
    if (topMm <= config.topSensorClearanceMm) {
        return true;
    }

    return false;
}

uint16_t TieredTargetDetector::averageDistance(uint16_t aMm, uint16_t bMm) const {
    if (!isDistanceValid(aMm) || !isDistanceValid(bMm)) {
        return INVALID_DISTANCE;
    }
    return uint16_t((uint32_t(aMm) + uint32_t(bMm)) / 2);
}
