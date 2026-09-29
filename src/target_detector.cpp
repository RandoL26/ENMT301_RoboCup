#include "target_detector.h"

namespace {

constexpr float LIDAR_X_MM = 38.0f;
constexpr float LIDAR_Y_MM = -96.0f;

// LD06's existing coordinate frame:
// 0° = +X, 90° = -Y
//
// Robot detector frame:
// 0° = front (-Y), +90° = right (+X)
//
// Therefore the robot-frame angle is:
// robotAngle = LD06 angle - 90°

float angleDiffDeg(float aDeg, float bDeg) {
    float d = fmodf(aDeg - bDeg + 540.0f, 360.0f) - 180.0f;
    return d;
}

} // namespace

TieredTargetDetector::TieredTargetDetector(TOFSensorArray& sensorArray,
                                           LD06& sensorLidar,
                                           uint8_t nearAIndex,
                                           uint8_t nearBIndex,
                                           uint8_t farAIndex,
                                           uint8_t farBIndex)
    : tofArray(sensorArray),
      lidar(sensorLidar),
      sensorNearA(nearAIndex),
      sensorNearB(nearBIndex),
      sensorFarA(farAIndex),
      sensorFarB(farBIndex),
      lastResult(TierDetectionResult::None) {

    mountFarA = SensorMount{-86.0f, -140.0f, 11.20f};
    mountFarB = SensorMount{ 90.0f, -126.0f, 12.95f};

    mountNearA = SensorMount{-40.5f, -110.0f, 14.04f};
    mountNearB = SensorMount{ 60.5f, -102.0f, 14.04f};

    lidar.setOffsetPosition(
    int16_t(LIDAR_X_MM),
    int16_t(LIDAR_Y_MM),
    0.0f);

    debug = DetectionDebug{};

    targetConfidence = 0;
    targetLatched = false;
    movingObjectDetected = false;

    trackingState = TargetTrackingState::NONE;

    lastCandidatePoint = GridPoint{};
    lastCandidateUpdateMs = 0;
}

void TieredTargetDetector::setConfig(const TieredTargetDetectorConfig& newConfig) {
    config = newConfig;
}

void TieredTargetDetector::setExpectedIntersections(uint16_t nearMm, uint16_t farMm) {
    // NOTE: this only updates the pairMatch() validation targets, same as
    // before. It does NOT recompute sensor mount angles -- call
    // setSensorMounts() (or SensorMount::aimedAtCenterlineCrossing() again
    // yourself) if the physical crossing distance actually changed.
    config.nearIntersectMm = nearMm;
    config.farIntersectMm = farMm;
}

void TieredTargetDetector::setSensorMounts(const SensorMount& nearA, const SensorMount& nearB,
                                            const SensorMount& farA, const SensorMount& farB) {
    mountNearA = nearA;
    mountNearB = nearB;
    mountFarA = farA;
    mountFarB = farB;
}

GridPoint TieredTargetDetector::tofPointFromRange(
    const SensorMount& mount,
    uint16_t rangeMm) const {
    // The detector frame uses +X to the robot's right and -Y toward the front.
    const float angleRad = mount.boresightDeg * PI / 180.0f;
    GridPoint point;
    point.x = mount.xMm + rangeMm * sinf(angleRad);
    point.y = mount.yMm - rangeMm * cosf(angleRad);
    point.valid = true;
    return point;
}

void TieredTargetDetector::setLidarOffset(float xMm, float yMm, float angleDeg) {
    lidar.setOffsetPosition(int16_t(xMm), int16_t(yMm), angleDeg);
}

bool TieredTargetDetector::update() {

    if (!tofArray.isInitialized()) {
        lastResult = TierDetectionResult::None;
        trackingState = TargetTrackingState::NONE;
        targetLatched = false;
        return false;
    }

    // ============================================================
    // 1. READ TOF SENSORS
    // ============================================================

    uint16_t nearA = tofArray.getDistance(sensorNearA);
    uint16_t nearB = tofArray.getDistance(sensorNearB);
    uint16_t farA  = tofArray.getDistance(sensorFarA);
    uint16_t farB  = tofArray.getDistance(sensorFarB);

    if (!withinDetectionEnvelope(nearA)) nearA = INVALID_DISTANCE;
    if (!withinDetectionEnvelope(nearB)) nearB = INVALID_DISTANCE;
    if (!withinDetectionEnvelope(farA))  farA  = INVALID_DISTANCE;
    if (!withinDetectionEnvelope(farB))  farB  = INVALID_DISTANCE;

    // ============================================================
    // 2. SAVE DEBUG DATA
    // ============================================================

    debug.nearA = nearA;
    debug.nearB = nearB;
    debug.farA  = farA;
    debug.farB  = farB;

    debug.nearAverage = averageDistance(nearA, nearB);
    debug.farAverage  = averageDistance(farA, farB);

    debug.nearPairMatch =
        pairMatch(nearA, nearB, config.nearIntersectMm);

    debug.farPairMatch =
        pairMatch(farA, farB, config.farIntersectMm);

    // ============================================================
    // 3. CONVERT TO ROBOT COORDINATES
    // ============================================================

    debug.nearAPoint =
        isDistanceValid(nearA)
        ? tofPointFromRange(mountNearA, nearA)
        : GridPoint{};

    debug.nearBPoint =
        isDistanceValid(nearB)
        ? tofPointFromRange(mountNearB, nearB)
        : GridPoint{};

    debug.farAPoint =
        isDistanceValid(farA)
        ? tofPointFromRange(mountFarA, farA)
        : GridPoint{};

    debug.farBPoint =
        isDistanceValid(farB)
        ? tofPointFromRange(mountFarB, farB)
        : GridPoint{};

    GridPoint nearTierPoint =
        midpoint(debug.nearAPoint, debug.nearBPoint);

    GridPoint farTierPoint =
        midpoint(debug.farAPoint, debug.farBPoint);

    // ============================================================
    // 4. LIDAR CORROBORATION
    // ============================================================

    debug.nearLidarConfirmsObstacle =
        debug.nearPairMatch &&
        isObstacleAgainstLidar(nearTierPoint);

    debug.farLidarConfirmsObstacle =
        debug.farPairMatch &&
        isObstacleAgainstLidar(farTierPoint);

    bool anyObstacle =
        debug.nearLidarConfirmsObstacle ||
        debug.farLidarConfirmsObstacle;

    // ============================================================
    // 5. STEERING
    // ============================================================

    debug.steeringActive = false;
    debug.steeringValue = 0.0f;

    if (isDistanceValid(nearA) != isDistanceValid(nearB)) {

        const GridPoint& hit =
            isDistanceValid(nearA)
            ? debug.nearAPoint
            : debug.nearBPoint;

        debug.steeringValue =
            computeSteering(hit, mountNearA, mountNearB);

        debug.steeringActive = true;

    } else if (isDistanceValid(farA) != isDistanceValid(farB)) {

        const GridPoint& hit =
            isDistanceValid(farA)
            ? debug.farAPoint
            : debug.farBPoint;

        debug.steeringValue =
            computeSteering(hit, mountFarA, mountFarB);

        debug.steeringActive = true;
    }

    // ============================================================
    // 6. CURRENT OBJECT DISTANCE
    // ============================================================

    if (debug.nearPairMatch && debug.farPairMatch) {

        debug.objectDistance =
            debug.nearAverage < debug.farAverage
            ? debug.nearAverage
            : debug.farAverage;

    } else if (debug.nearPairMatch) {

        debug.objectDistance = debug.nearAverage;

    } else if (debug.farPairMatch) {

        debug.objectDistance = debug.farAverage;

    } else {

        debug.objectDistance = INVALID_DISTANCE;
    }

    // ============================================================
    // 7. DETERMINE CURRENT CANDIDATE
    // ============================================================

    bool farCandidate = debug.farPairMatch;
    bool nearCandidate = debug.nearPairMatch;

    bool tofCandidate =
        farCandidate || nearCandidate;

    // A ToF pair match alone is NOT enough to start target tracking.
    // The LiDAR must see a separate physical object as well.
    bool separateLidarObstacle = false;

    if (tofCandidate) {
        const GridPoint& tofCandidatePoint =
            nearCandidate ? nearTierPoint : farTierPoint;

        separateLidarObstacle =
            hasSeparateLidarObstacle(tofCandidatePoint);
    }

    bool targetCandidate =
        tofCandidate &&
        separateLidarObstacle &&
        !anyObstacle;

    bool currentCandidate = targetCandidate;

    GridPoint currentCandidatePoint;

    if (targetCandidate) {
        currentCandidatePoint =
            nearCandidate ? nearTierPoint : farTierPoint;
    }

    uint32_t now = millis();

    // ============================================================
    // 8. CHECK TEMPORAL POSITION CONSISTENCY
    // ============================================================

    bool positionConsistent = false;
    bool largeMovement = false;

    if (currentCandidate &&
        currentCandidatePoint.valid &&
        lastCandidatePoint.valid) {

        uint32_t elapsed =
            now - lastCandidateUpdateMs;

        if (elapsed <= config.candidateTimeoutMs) {

            float dx =
                currentCandidatePoint.x -
                lastCandidatePoint.x;

            float dy =
                currentCandidatePoint.y -
                lastCandidatePoint.y;

            float movement =
                sqrtf(dx * dx + dy * dy);

            positionConsistent =
                movement <= float(config.positionToleranceMm);

            largeMovement =
                movement >=
                float(config.movingObjectThresholdMm);
        }
    }

    // ============================================================
    // 9. CHECK FAR -> NEAR GEOMETRY
    // ============================================================

    bool farNearConsistent = false;

    if (farCandidate &&
        nearCandidate &&
        farTierPoint.valid &&
        nearTierPoint.valid) {

        float farDistance =
            farTierPoint.distanceFromOrigin();

        float nearDistance =
            nearTierPoint.distanceFromOrigin();

        float expectedChange =
            float(config.farIntersectMm -
                  config.nearIntersectMm);

        float actualChange =
            farDistance - nearDistance;

        bool distanceConsistent =
            fabsf(actualChange - expectedChange)
            <= float(config.approachingDistanceToleranceMm);

        float farBearing =
            farTierPoint.bearingFromOriginDeg();

        float nearBearing =
            nearTierPoint.bearingFromOriginDeg();

        float bearingDifference =
            fabsf(angleDiffDeg(
                farBearing,
                nearBearing));

        bool bearingConsistent =
            bearingDifference <= 15.0f;

        farNearConsistent =
            distanceConsistent &&
            bearingConsistent;
    }

    // ============================================================
    // 10. MOVING OBJECT DETECTION
    // ============================================================

    if (largeMovement) {
        movingObjectDetected = true;
    }

    // ============================================================
    // 11. SAME-OBJECT OBSTACLE OVERRIDES TARGET TRACKING
    // ============================================================

    if (anyObstacle) {
        targetConfidence = 0;
        targetLatched = false;
        movingObjectDetected = false;
        trackingState = TargetTrackingState::NONE;
        lastCandidatePoint = GridPoint{};
    }

    // ============================================================
    // 12. STATE MACHINE
    // ============================================================

    switch (trackingState) {

        // --------------------------------------------------------
        // NONE
        // --------------------------------------------------------

        case TargetTrackingState::NONE:

            targetLatched = false;
            movingObjectDetected = false;

            if (farCandidate) {

                targetConfidence =
                    config.farDetectionConfidence;

                trackingState =
                    TargetTrackingState::FAR_CANDIDATE;

                lastCandidatePoint =
                    farTierPoint;

                lastCandidateUpdateMs = now;
            }

            else if (nearCandidate) {

                // Seeing something only in the near tier is not
                // enough to immediately call it a target.
                targetConfidence =
                    config.nearDetectionConfidence;

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;

                lastCandidatePoint =
                    nearTierPoint;

                lastCandidateUpdateMs = now;
            }

            break;


        // --------------------------------------------------------
        // FAR CANDIDATE
        // --------------------------------------------------------

        case TargetTrackingState::FAR_CANDIDATE:

            if (!currentCandidate) {

                if ((now - lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    targetConfidence = 0;
                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint = GridPoint{};
                }

                break;
            }

            if (largeMovement) {

                targetConfidence = 0;
                movingObjectDetected = true;

                trackingState =
                    TargetTrackingState::MOVING_OBJECT;

                break;
            }

            if (positionConsistent) {

                targetConfidence +=
                    config.positionConsistencyConfidence;

            } else {

                targetConfidence +=
                    config.farDetectionConfidence;
            }

            if (targetConfidence > 100)
                targetConfidence = 100;

            // If the near pair has now appeared, we have progressed
            // from FAR toward NEAR.
            if (nearCandidate) {

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;
            }

            else if (targetConfidence >=
                     config.approachingThreshold) {

                trackingState =
                    TargetTrackingState::APPROACHING;
            }

            lastCandidatePoint =
                currentCandidatePoint;

            lastCandidateUpdateMs = now;

            break;


        // --------------------------------------------------------
        // APPROACHING
        // --------------------------------------------------------

        case TargetTrackingState::APPROACHING:

            if (!currentCandidate) {

                if ((now - lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    targetConfidence = 0;
                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint = GridPoint{};
                }

                break;
            }

            if (largeMovement) {

                targetConfidence = 0;
                movingObjectDetected = true;

                trackingState =
                    TargetTrackingState::MOVING_OBJECT;

                break;
            }

            if (positionConsistent) {

                targetConfidence +=
                    config.positionConsistencyConfidence;
            }

            if (farNearConsistent) {

                targetConfidence +=
                    config.positionConsistencyConfidence;

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;
            }

            if (nearCandidate) {

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;
            }

            if (targetConfidence > 100)
                targetConfidence = 100;

            lastCandidatePoint =
                currentCandidatePoint;

            lastCandidateUpdateMs = now;

            break;


        // --------------------------------------------------------
        // NEAR CONFIRMING
        // --------------------------------------------------------

        case TargetTrackingState::NEAR_CONFIRMING: {

            if (!nearCandidate) {

                // Don't immediately throw away the candidate.
                // Allow a short ToF dropout.
                if ((now - lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    targetConfidence = 0;
                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint = GridPoint{};
                }

                break;
            }

            if (largeMovement) {

                targetConfidence = 0;
                movingObjectDetected = true;

                trackingState =
                    TargetTrackingState::MOVING_OBJECT;

                break;
            }

            // Near pair is strong evidence.
            targetConfidence +=
                config.nearDetectionConfidence;

            if (positionConsistent) {

                targetConfidence +=
                    config.positionConsistencyConfidence;
            }

            if (farNearConsistent) {

                targetConfidence +=
                    config.positionConsistencyConfidence;
            }

            if (targetConfidence > 100)
                targetConfidence = 100;

            // Require the candidate to actually be in the
            // near region before final confirmation.
            float nearDistance =
                nearTierPoint.distanceFromOrigin();

            bool insideNearRegion =
                nearDistance <=
                float(config.nearConfirmationDistanceMm);

            if (insideNearRegion &&
                targetConfidence >=
                config.nearConfirmThreshold) {

                trackingState =
                    TargetTrackingState::TARGET_CONFIRMED;

                targetLatched = true;
            }

            lastCandidatePoint =
                currentCandidatePoint;

            lastCandidateUpdateMs = now;

            break;
        }


        // --------------------------------------------------------
        // TARGET CONFIRMED
        // --------------------------------------------------------

        case TargetTrackingState::TARGET_CONFIRMED:

            targetLatched = true;

            if (largeMovement) {

                targetLatched = false;
                targetConfidence = 0;
                movingObjectDetected = true;

                trackingState =
                    TargetTrackingState::MOVING_OBJECT;

                break;
            }

            if (currentCandidate) {

                // Keep the target alive while the sensors continue
                // seeing it.
                if (targetConfidence < 100) {
                    targetConfidence += 1;
                }

                lastCandidatePoint =
                    currentCandidatePoint;

                lastCandidateUpdateMs = now;

            } else if ((now - lastCandidateUpdateMs) >
                       config.candidateTimeoutMs) {

                targetLatched = false;
                targetConfidence = 0;

                trackingState =
                    TargetTrackingState::NONE;

                lastCandidatePoint = GridPoint{};
            }

            break;


        // --------------------------------------------------------
        // MOVING OBJECT
        // --------------------------------------------------------

        case TargetTrackingState::MOVING_OBJECT:

            targetLatched = false;
            movingObjectDetected = true;
            targetConfidence = 0;

            // We require the moving object to disappear before
            // looking for another target.
            if (!currentCandidate) {

                if ((now - lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    movingObjectDetected = false;

                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint = GridPoint{};
                }
            }

            break;
    }

    // ============================================================
    // 13. UPDATE RESULT
    // ============================================================

    if (trackingState ==
        TargetTrackingState::TARGET_CONFIRMED) {

        lastResult =
            TierDetectionResult::Target;

    } else if (trackingState ==
               TargetTrackingState::MOVING_OBJECT) {

        lastResult =
            TierDetectionResult::Obstacle;

    } else if (anyObstacle) {

        lastResult =
            TierDetectionResult::Obstacle;

    } else if (currentCandidate) {

        lastResult =
            TierDetectionResult::Indeterminate;

    } else {

        lastResult =
            TierDetectionResult::None;
    }

    // ============================================================
    // 14. DEBUG
    // ============================================================

    debug.targetConfidence = targetConfidence;
    debug.targetLatched = targetLatched;
    debug.movingObjectDetected = movingObjectDetected;

    return true;
}

TierDetectionResult TieredTargetDetector::getDetectionResult() const {
    return lastResult;
}

bool TieredTargetDetector::isTargetConfirmed() const {
    return targetLatched;
}

bool TieredTargetDetector::isMovingObjectDetected() const {
    return movingObjectDetected;
}

TargetTrackingState TieredTargetDetector::getTrackingState() const {
    return trackingState;
}

const char* TieredTargetDetector::trackingStateToString(
    TargetTrackingState state) const {

    switch (state) {

        case TargetTrackingState::NONE:
            return "NONE";

        case TargetTrackingState::FAR_CANDIDATE:
            return "FAR_CANDIDATE";

        case TargetTrackingState::APPROACHING:
            return "APPROACHING";

        case TargetTrackingState::NEAR_CONFIRMING:
            return "NEAR_CONFIRMING";

        case TargetTrackingState::TARGET_CONFIRMED:
            return "TARGET_CONFIRMED";

        case TargetTrackingState::MOVING_OBJECT:
            return "MOVING_OBJECT";
    }

    return "UNKNOWN";
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
    Serial.print(" nearAvg:"); Serial.print(debug.nearAverage == INVALID_DISTANCE ? 0xFFFF : debug.nearAverage);
    Serial.print(" farAvg:"); Serial.print(debug.farAverage == INVALID_DISTANCE ? 0xFFFF : debug.farAverage);
    Serial.print(" objDist:"); Serial.print(debug.objectDistance == INVALID_DISTANCE ? 0xFFFF : debug.objectDistance);
    Serial.print(" pairNear:"); Serial.print(debug.nearPairMatch);
    Serial.print(" pairFar:"); Serial.print(debug.farPairMatch);
    Serial.print(" lidarObsNear:"); Serial.print(debug.nearLidarConfirmsObstacle);
    Serial.print(" lidarObsFar:"); Serial.print(debug.farLidarConfirmsObstacle);
    Serial.print(" steerActive:"); Serial.print(debug.steeringActive);
    Serial.print(" steer:"); Serial.print(debug.steeringValue, 2);
    Serial.print(" confidence:"); Serial.print(debug.targetConfidence);
    Serial.print(" latched:"); Serial.print(debug.targetLatched);
    Serial.print(" moving:"); Serial.print(debug.movingObjectDetected);
    Serial.print(" state:"); Serial.print(trackingStateToString(trackingState));
    Serial.print(" result:"); Serial.println(resultToString(lastResult));
}

void TieredTargetDetector::debugPrintGrid(Stream& serialport) const {
    // teleplot.fr-style dump (see ld06.cpp printScanTeleplot) so the TOF hit
    // points can be overlaid on the same plot as the lidar's own >lidar:xy
    // series. Points marked (0,0)/invalid are omitted.
    serialport.print(F(">tof:"));
    const GridPoint* pts[4] = { &debug.nearAPoint, &debug.nearBPoint, &debug.farAPoint, &debug.farBPoint };
    for (uint8_t i = 0; i < 4; i++) {
        if (!pts[i]->valid) continue;
        serialport.print(pts[i]->x, 1);
        serialport.print(":");
        serialport.print(pts[i]->y, 1);
        serialport.print(";");
    }
    serialport.println(F("|xy"));
}

const TieredTargetDetector::DetectionDebug& TieredTargetDetector::getDebug() const {
    return debug;
}

bool TieredTargetDetector::isDistanceValid(uint16_t mm) const {
    return mm != INVALID_DISTANCE && mm != 0;
}

bool TieredTargetDetector::pairMatch(
    uint16_t aMm,
    uint16_t bMm,
    uint16_t expectedMm) const {

    if (!isDistanceValid(aMm) || !isDistanceValid(bMm)) {
        return false;
    }

    // Both sensors must be reasonably close to each other.
    // This prevents a very different pair of readings from passing
    // just because their average happens to be correct.
    uint16_t pairDifference =
        (aMm > bMm) ? (aMm - bMm) : (bMm - aMm);

    if (pairDifference > config.toleratedWidthMm) {
        return false;
    }

    // The average must still be near the expected intersection.
    int32_t avg = (int32_t(aMm) + int32_t(bMm)) / 2;

    return (avg >= int32_t(expectedMm) -
                     int32_t(config.intersectionToleranceMm)) &&
           (avg <= int32_t(expectedMm) +
                     int32_t(config.intersectionToleranceMm));
}

uint16_t TieredTargetDetector::averageDistance(uint16_t aMm, uint16_t bMm) const {
    if (!isDistanceValid(aMm) || !isDistanceValid(bMm)) {
        return INVALID_DISTANCE;
    }
    return uint16_t((uint32_t(aMm) + uint32_t(bMm)) / 2);
}

bool TieredTargetDetector::withinDetectionEnvelope(uint16_t rangeMm) const {
    if (!isDistanceValid(rangeMm)) return false;
    if (rangeMm > config.maxDetectionRangeMm) return false;
    return true;
}

GridPoint TieredTargetDetector::lidarPointToRobotGrid(
    const DataPoint& point) const {

    GridPoint p;

    // LD06 angle correction.
    //
    // Robot frame:
    //   0°   = FRONT
    //   +90° = RIGHT
    //   -90° = LEFT
    //
    // LiDAR mounting position relative to robot origin:
    //   X = +38 mm
    //   Y = -96 mm

    constexpr float LIDAR_X_MM = 38.0f;
    constexpr float LIDAR_Y_MM = -96.0f;
    constexpr float LIDAR_YAW_OFFSET_DEG = 37.5f;

    float robotAngleDeg = point.angle + LIDAR_YAW_OFFSET_DEG;

    while (robotAngleDeg >= 360.0f) {
        robotAngleDeg -= 360.0f;
    }

    while (robotAngleDeg < 0.0f) {
        robotAngleDeg += 360.0f;
    }

    float angleRad = robotAngleDeg * PI / 180.0f;

    // Point relative to the LiDAR
    float lidarX = float(point.distance) * sinf(angleRad);
    float lidarY = -float(point.distance) * cosf(angleRad);

    // Translate from LiDAR frame to robot-origin frame
    p.x = LIDAR_X_MM + lidarX;
    p.y = LIDAR_Y_MM + lidarY;

    p.valid = true;

    return p;
}

GridPoint TieredTargetDetector::midpoint(const GridPoint& a, const GridPoint& b) const {
    GridPoint m;
    if (a.valid && b.valid) {
        m.x = (a.x + b.x) / 2.0f;
        m.y = (a.y + b.y) / 2.0f;
        m.valid = true;
    } else if (a.valid) {
        m = a;
    } else if (b.valid) {
        m = b;
    }
    return m;
}

bool TieredTargetDetector::isObstacleAgainstLidar(const GridPoint& tofPoint) const {
    if (!tofPoint.valid) return false;

    float tofDist = tofPoint.distanceFromOrigin();
    float tofBearing = tofPoint.bearingFromOriginDeg();

    float lidarDist;
    if (!findNearestLidarAtBearing(tofBearing, config.lidarBearingToleranceDeg, lidarDist)) {
        // No corroborating lidar return at this bearing (timeout, out of
        // lidar range, or momentarily occluded). Can't confirm a tall
        // obstacle from absence of evidence, so don't flag one.
        return false;
    }

    return fabsf(lidarDist - tofDist) <= float(config.lidarMatchToleranceMm);
}

bool TieredTargetDetector::hasSeparateLidarObstacle(const GridPoint& tofPoint) const {

    if (!tofPoint.valid) {
        return false;
    }

    const uint16_t n = lidar.getNbPointsInScan();

    for (uint16_t i = 0; i < n; ++i) {

        DataPoint* anchorPoint = lidar.getPoints(i);

        if (anchorPoint == nullptr ||
            anchorPoint->distance < config.lidarObstacleMinMm ||
            anchorPoint->distance > config.lidarObstacleMaxMm) {
            continue;
        }

        GridPoint anchor = lidarPointToRobotGrid(*anchorPoint);

        if (!anchor.valid) {
            continue;
        }

        if (config.useSeparateObstacleAngleGate) {
            const float bearing = anchor.bearingFromOriginDeg();
            const float lo = config.separateObstacleAngleMinDeg;
            const float hi = config.separateObstacleAngleMaxDeg;

            const bool inside = (lo <= hi)
                ? (bearing >= lo && bearing <= hi)
                : (bearing >= lo || bearing <= hi);

            if (!inside) {
                continue;
            }
        }

        // The anchor itself must be physically separated from the ToF POI.
        const float anchorDx = anchor.x - tofPoint.x;
        const float anchorDy = anchor.y - tofPoint.y;
        const float anchorSeparation =
            sqrtf(anchorDx * anchorDx + anchorDy * anchorDy);

        if (anchorSeparation < float(config.separateObstacleDistanceMm)) {
            continue;
        }

        // Count nearby LiDAR returns around this anchor. The anchor counts
        // as point 1, so we need at least two more returns to form the
        // required 3-point obstacle cluster.
        uint8_t clusterCount = 1;

        for (uint16_t j = 0; j < n; ++j) {

            if (j == i) {
                continue;
            }

            DataPoint* candidatePoint = lidar.getPoints(j);

            if (candidatePoint == nullptr ||
                candidatePoint->distance < config.lidarObstacleMinMm ||
                candidatePoint->distance > config.lidarObstacleMaxMm) {
                continue;
            }

            GridPoint candidate = lidarPointToRobotGrid(*candidatePoint);

            if (!candidate.valid) {
                continue;
            }

            if (config.useSeparateObstacleAngleGate) {
                const float bearing = candidate.bearingFromOriginDeg();
                const float lo = config.separateObstacleAngleMinDeg;
                const float hi = config.separateObstacleAngleMaxDeg;

                const bool inside = (lo <= hi)
                    ? (bearing >= lo && bearing <= hi)
                    : (bearing >= lo || bearing <= hi);

                if (!inside) {
                    continue;
                }
            }

            // Every point in the cluster must also be separated from the
            // ToF POI. This prevents a large object containing the ToF hit
            // from being mistaken for a separate obstacle.
            const float dxToTof = candidate.x - tofPoint.x;
            const float dyToTof = candidate.y - tofPoint.y;
            const float separationFromTof =
                sqrtf(dxToTof * dxToTof + dyToTof * dyToTof);

            if (separationFromTof < float(config.separateObstacleDistanceMm)) {
                continue;
            }

            const float dx = candidate.x - anchor.x;
            const float dy = candidate.y - anchor.y;
            const float clusterDistance =
                sqrtf(dx * dx + dy * dy);

            if (clusterDistance <= float(config.lidarClusterRadiusMm)) {
                ++clusterCount;

                if (clusterCount >= config.minimumSeparateLidarPoints) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool TieredTargetDetector::findNearestLidarAtBearing(float bearingDeg, float toleranceDeg, float& outDistanceMm) const {
    uint16_t n = lidar.getNbPointsInScan();
    float bestDelta = 1.0e6f;
    bool found = false;

    for (uint16_t i = 0; i < n; i++) {
        DataPoint* pt = lidar.getPoints(i);

        GridPoint lidarPoint = lidarPointToRobotGrid(*pt);

        if (!lidarPoint.valid) {
            continue;
        }

        float dist = lidarPoint.distanceFromOrigin();
        float bear = lidarPoint.bearingFromOriginDeg();

        if (config.useAngleGate) {
            float lo = config.detectionAngleMinDeg;
            float hi = config.detectionAngleMaxDeg;
            bool inside = (lo <= hi)
                ? (bear >= lo && bear <= hi)
                : (bear >= lo || bear <= hi);

            if (!inside) continue;
        }

        float delta = fabsf(angleDiffDeg(bear, bearingDeg));

        if (delta <= toleranceDeg && delta < bestDelta) {
            bestDelta = delta;
            outDistanceMm = dist;
            found = true;
        }
    }
    return found;
}

float TieredTargetDetector::computeSteering(
    const GridPoint& hit,
    const SensorMount& mountA,
    const SensorMount& mountB) const {

    if (!hit.valid) {
        return 0.0f;
    }

    // Left/right sensor spacing is along X.
    float halfSpacing =
        fabsf(mountA.xMm - mountB.xMm) / 2.0f;

    if (halfSpacing < 1.0f) {
        return 0.0f;
    }

    // +X = right
    // -X = left
    float norm = hit.x / halfSpacing;

    if (norm > 1.0f) {
        norm = 1.0f;
    }

    if (norm < -1.0f) {
        norm = -1.0f;
    }

    return norm;
}

void TieredTargetDetector::debugPrintLidarFrontTest() const {
    uint16_t n = lidar.getNbPointsInScan();

    Serial.println();
    Serial.println("===== LIDAR RAW FRONT TEST =====");
    Serial.print("Points in scan: ");
    Serial.println(n);

    if (n == 0) {
        Serial.println("No LiDAR points available.");
        Serial.println("================================");
        return;
    }

    // Find the 10 closest LiDAR points in the current scan.
    // We use these to identify the object placed in front of the robot
    // without assuming anything about the LD06 angle convention yet.

    const uint8_t NUM_CLOSEST = 10;

    uint16_t closestDistance[NUM_CLOSEST];
    uint16_t closestIndex[NUM_CLOSEST];

    for (uint8_t i = 0; i < NUM_CLOSEST; i++) {
        closestDistance[i] = 0xFFFF;
        closestIndex[i] = 0xFFFF;
    }

    for (uint16_t i = 0; i < n; i++) {
        DataPoint* pt = lidar.getPoints(i);

        if (pt == nullptr) {
            continue;
        }

        if (pt->distance == 0) {
            continue;
        }

        // Insert this point into the sorted list if it is one
        // of the 10 closest points.
        for (uint8_t j = 0; j < NUM_CLOSEST; j++) {

            if (pt->distance < closestDistance[j]) {

                // Shift existing entries down.
                for (uint8_t k = NUM_CLOSEST - 1; k > j; k--) {
                    closestDistance[k] = closestDistance[k - 1];
                    closestIndex[k] = closestIndex[k - 1];
                }

                closestDistance[j] = pt->distance;
                closestIndex[j] = i;

                break;
            }
        }
    }

    Serial.println("Closest LiDAR points:");

    for (uint8_t i = 0; i < NUM_CLOSEST; i++) {

        if (closestIndex[i] == 0xFFFF) {
            continue;
        }

        DataPoint* pt = lidar.getPoints(closestIndex[i]);

        if (pt == nullptr) {
            continue;
        }

        Serial.print("#");
        Serial.print(i + 1);

        Serial.print("  angle=");
        Serial.print(pt->angle, 2);
        Serial.print(" deg");

        Serial.print("  distance=");
        Serial.print(pt->distance);
        Serial.print(" mm");

        #ifdef LD06_COMPUTE_XY
        Serial.print("  LD06 x=");
        Serial.print(pt->x);

        Serial.print(" mm");

        Serial.print("  LD06 y=");
        Serial.print(pt->y);

        Serial.print(" mm");
        #endif

        GridPoint robotPoint = lidarPointToRobotGrid(*pt);

        Serial.print("  Robot X=");
        Serial.print(robotPoint.x, 1);

        Serial.print(" mm");

        Serial.print("  Robot Y=");
        Serial.print(robotPoint.y, 1);

        Serial.print(" mm");

        Serial.print("  intensity=");
        Serial.println(pt->intensity);
    }

    Serial.println("================================");
}
