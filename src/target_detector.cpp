#include "target_detector.h"

namespace {

constexpr float LIDAR_X_MM = 38.0f;
constexpr float LIDAR_Y_MM = -96.0f;

// LD06 angle correction used by the existing detector.
//
// Robot frame:
//   0°   = FRONT (-Y)
//   +90° = RIGHT (+X)
//   -90° = LEFT (-X)
constexpr float LIDAR_YAW_OFFSET_DEG = 37.5f;

float angleDiffDeg(float aDeg, float bDeg) {
    float d = fmodf(aDeg - bDeg + 540.0f, 360.0f) - 180.0f;
    return d;
}

} // namespace


// ===========================================================================
// CONSTRUCTOR
// ===========================================================================

TieredTargetDetector::TieredTargetDetector(
    TOFSensorArray& sensorArray,
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

    // -----------------------------------------------------------------------
    // TOF SENSOR MOUNTS
    // -----------------------------------------------------------------------

    mountFarA =
        SensorMount{-86.0f, -140.0f, 11.20f};

    mountFarB =
        SensorMount{90.0f, -126.0f, 12.95f};

    mountNearA =
        SensorMount{-40.5f, -110.0f, 14.04f};

    mountNearB =
        SensorMount{60.5f, -102.0f, 14.04f};


    // -----------------------------------------------------------------------
    // LIDAR POSITION
    // -----------------------------------------------------------------------

    lidar.setOffsetPosition(
        int16_t(LIDAR_X_MM),
        int16_t(LIDAR_Y_MM),
        0.0f);


    // -----------------------------------------------------------------------
    // INITIAL STATE
    // -----------------------------------------------------------------------

    debug = DetectionDebug{};

    targetConfidence = 0;
    targetLatched = false;
    movingObjectDetected = false;

    trackingState =
        TargetTrackingState::NONE;

    lastCandidatePoint =
        GridPoint{};

    lastCandidateUpdateMs = 0;

    lastCandidateWasFar = false;
}


// ===========================================================================
// CONFIGURATION
// ===========================================================================

void TieredTargetDetector::setConfig(
    const TieredTargetDetectorConfig& newConfig) {

    config = newConfig;
}


void TieredTargetDetector::setExpectedIntersections(
    uint16_t nearMm,
    uint16_t farMm) {

    config.nearIntersectMm = nearMm;
    config.farIntersectMm = farMm;
}


void TieredTargetDetector::setSensorMounts(
    const SensorMount& nearA,
    const SensorMount& nearB,
    const SensorMount& farA,
    const SensorMount& farB) {

    mountNearA = nearA;
    mountNearB = nearB;
    mountFarA = farA;
    mountFarB = farB;
}


void TieredTargetDetector::setLidarOffset(
    float xMm,
    float yMm,
    float angleDeg) {

    lidar.setOffsetPosition(
        int16_t(xMm),
        int16_t(yMm),
        angleDeg);
}


// ===========================================================================
// TOF RANGE -> ROBOT GRID
// ===========================================================================

GridPoint TieredTargetDetector::tofPointFromRange(
    const SensorMount& mount,
    uint16_t rangeMm) const {

    GridPoint point;

    const float angleRad =
        mount.boresightDeg * PI / 180.0f;

    point.x =
        mount.xMm +
        rangeMm * sinf(angleRad);

    point.y =
        mount.yMm -
        rangeMm * cosf(angleRad);

    point.valid = true;

    return point;
}


// ===========================================================================
// MAIN UPDATE
// ===========================================================================

bool TieredTargetDetector::update() {

    if (!tofArray.isInitialized()) {

        lastResult =
            TierDetectionResult::None;

        trackingState =
            TargetTrackingState::NONE;

        targetLatched = false;
        targetConfidence = 0;
        movingObjectDetected = false;

        return false;
    }


    // =======================================================================
    // 1. READ TOF SENSORS
    // =======================================================================

    uint16_t nearA =
        tofArray.getDistance(sensorNearA);

    uint16_t nearB =
        tofArray.getDistance(sensorNearB);

    uint16_t farA =
        tofArray.getDistance(sensorFarA);

    uint16_t farB =
        tofArray.getDistance(sensorFarB);


    // Apply the configured ToF detection envelope.

    if (!withinDetectionEnvelope(nearA))
        nearA = INVALID_DISTANCE;

    if (!withinDetectionEnvelope(nearB))
        nearB = INVALID_DISTANCE;

    if (!withinDetectionEnvelope(farA))
        farA = INVALID_DISTANCE;

    if (!withinDetectionEnvelope(farB))
        farB = INVALID_DISTANCE;


    // =======================================================================
    // 2. SAVE BASIC DEBUG DATA
    // =======================================================================

    debug.nearA = nearA;
    debug.nearB = nearB;

    debug.farA = farA;
    debug.farB = farB;

    debug.nearAverage =
        averageDistance(nearA, nearB);

    debug.farAverage =
        averageDistance(farA, farB);


    // =======================================================================
    // 3. TOF PAIR MATCHING
    // =======================================================================

    debug.nearPairMatch =
        pairMatch(
            nearA,
            nearB,
            config.nearIntersectMm);

    debug.farPairMatch =
        pairMatch(
            farA,
            farB,
            config.farIntersectMm);


    // =======================================================================
    // 4. CONVERT TO ROBOT COORDINATES
    // =======================================================================

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
        midpoint(
            debug.nearAPoint,
            debug.nearBPoint);

    GridPoint farTierPoint =
        midpoint(
            debug.farAPoint,
            debug.farBPoint);


    // =======================================================================
    // 5. LIDAR SAME-OBJECT CHECK
    // =======================================================================

    debug.nearLidarConfirmsObstacle =
        debug.nearPairMatch &&
        isObstacleAgainstLidar(nearTierPoint);

    debug.farLidarConfirmsObstacle =
        debug.farPairMatch &&
        isObstacleAgainstLidar(farTierPoint);


    bool anyObstacle =
        debug.nearLidarConfirmsObstacle ||
        debug.farLidarConfirmsObstacle;


    // =======================================================================
    // 6. STEERING
    // =======================================================================

    debug.steeringActive = false;
    debug.steeringValue = 0.0f;


    if (isDistanceValid(nearA) !=
        isDistanceValid(nearB)) {

        const GridPoint& hit =
            isDistanceValid(nearA)
            ? debug.nearAPoint
            : debug.nearBPoint;

        debug.steeringValue =
            computeSteering(
                hit,
                mountNearA,
                mountNearB);

        debug.steeringActive = true;

    }

    else if (isDistanceValid(farA) !=
             isDistanceValid(farB)) {

        const GridPoint& hit =
            isDistanceValid(farA)
            ? debug.farAPoint
            : debug.farBPoint;

        debug.steeringValue =
            computeSteering(
                hit,
                mountFarA,
                mountFarB);

        debug.steeringActive = true;
    }


    // =======================================================================
    // 7. CURRENT OBJECT DISTANCE
    // =======================================================================

    if (debug.nearPairMatch &&
        debug.farPairMatch) {

        debug.objectDistance =
            debug.nearAverage < debug.farAverage
            ? debug.nearAverage
            : debug.farAverage;

    }

    else if (debug.nearPairMatch) {

        debug.objectDistance =
            debug.nearAverage;

    }

    else if (debug.farPairMatch) {

        debug.objectDistance =
            debug.farAverage;

    }

    else {

        debug.objectDistance =
            INVALID_DISTANCE;
    }


    // =======================================================================
    // 8. CURRENT TOF CANDIDATE
    // =======================================================================

    bool farCandidate =
        debug.farPairMatch;

    bool nearCandidate =
        debug.nearPairMatch;

    bool tofCandidate =
        farCandidate ||
        nearCandidate;


    // =======================================================================
    // 9. SEPARATE LIDAR OBSTACLE
    //
    // This is NOT required for a ToF candidate.
    //
    // It simply provides additional target evidence.
    // =======================================================================

    bool separateLidarObstacle = false;

    if (tofCandidate) {

        const GridPoint& tofCandidatePoint =
            nearCandidate
            ? nearTierPoint
            : farTierPoint;

        separateLidarObstacle =
            hasSeparateLidarObstacle(
                tofCandidatePoint);
    }

    debug.separateLidarObstacle =
        separateLidarObstacle;


    // =======================================================================
    // 10. CURRENT CANDIDATE
    //
    // IMPORTANT:
    //
    // A valid ToF pair is enough to create a candidate.
    //
    // LiDAR is additional evidence:
    //
    //   Same object     -> obstacle evidence
    //   Separate object -> target evidence
    //   Nothing         -> no additional evidence
    // =======================================================================

    bool currentCandidate =
        tofCandidate;

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

    if (currentCandidate) {

        currentCandidatePoint =
            nearCandidate
            ? nearTierPoint
            : farTierPoint;
    }


    uint32_t now =
        millis();


    // =======================================================================
    // 11. TEMPORAL POSITION CONSISTENCY
    // =======================================================================

    bool positionConsistent = false;
    bool largeMovement = false;


    if (currentCandidate &&
        currentCandidatePoint.valid &&
        lastCandidatePoint.valid) {

        uint32_t elapsed =
            now - lastCandidateUpdateMs;


        if (elapsed <=
            config.candidateTimeoutMs) {

            float dx =
                currentCandidatePoint.x -
                lastCandidatePoint.x;

            float dy =
                currentCandidatePoint.y -
                lastCandidatePoint.y;

            float movement =
                sqrtf(
                    dx * dx +
                    dy * dy);


            positionConsistent =
                movement <=
                float(config.positionToleranceMm);


            largeMovement =
                movement >=
                float(config.movingObjectThresholdMm);
        }
    }


    // =======================================================================
    // 12. FAR -> NEAR GEOMETRY
    // =======================================================================

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
            float(
                config.farIntersectMm -
                config.nearIntersectMm);


        float actualChange =
            farDistance -
            nearDistance;


        bool distanceConsistent =
            fabsf(
                actualChange -
                expectedChange)
            <=
            float(
                config.approachingDistanceToleranceMm);


        float farBearing =
            farTierPoint.bearingFromOriginDeg();

        float nearBearing =
            nearTierPoint.bearingFromOriginDeg();


        float bearingDifference =
            fabsf(
                angleDiffDeg(
                    farBearing,
                    nearBearing));


        bool bearingConsistent =
            bearingDifference <= 15.0f;


        farNearConsistent =
            distanceConsistent &&
            bearingConsistent;
    }


    // =======================================================================
    // 13. MOVEMENT DETECTION
    // =======================================================================

    debug.positionConsistent = positionConsistent;
    debug.farNearConsistent = farNearConsistent;
    debug.largeMovement = largeMovement;

    if (largeMovement) {
        movingObjectDetected = true;
    }


    // =======================================================================
    // 14. CONFIDENCE UPDATE
    //
    // Formula:
    //
    //   new confidence =
    //       old confidence
    //       - natural decay
    //       + ToF evidence
    //       + temporal evidence
    //       + separate LiDAR evidence
    //       - same-object penalty
    //       - movement penalty
    //
    // No LiDAR return gives NO penalty.
    // =======================================================================

    if (currentCandidate) {

        // -------------------------------------------------------------------
        // Natural confidence decay
        // -------------------------------------------------------------------

        if (targetConfidence >=
            config.confidenceDecay) {

            targetConfidence -=
                config.confidenceDecay;

        } else {

            targetConfidence = 0;
        }


        // -------------------------------------------------------------------
        // ToF evidence
        //
        // Near takes priority over far.
        // -------------------------------------------------------------------

        if (nearCandidate) {

            targetConfidence +=
                config.nearDetectionConfidence;

        }

        else if (farCandidate) {

            targetConfidence +=
                config.farDetectionConfidence;
        }


        // -------------------------------------------------------------------
        // Temporal evidence
        // -------------------------------------------------------------------

        if (positionConsistent) {

            targetConfidence +=
                config.positionConsistencyConfidence;
        }


        if (farNearConsistent) {

            targetConfidence +=
                config.positionConsistencyConfidence;
        }


        // -------------------------------------------------------------------
        // Separate LiDAR evidence
        //
        // This is positive target evidence.
        // -------------------------------------------------------------------

        if (separateLidarObstacle &&
            !anyObstacle) {

            targetConfidence +=
                config.lidarConfidence;
        }


        // -------------------------------------------------------------------
        // Same-object LiDAR
        //
        // This is contradictory target evidence.
        // -------------------------------------------------------------------

        if (anyObstacle) {

            if (targetConfidence >=
                config.sameObjectPenalty) {

                targetConfidence -=
                    config.sameObjectPenalty;

            } else {

                targetConfidence = 0;
            }
        }


        // -------------------------------------------------------------------
        // Moving object
        // -------------------------------------------------------------------

        if (largeMovement) {

            if (targetConfidence >=
                config.movingObjectPenalty) {

                targetConfidence -=
                    config.movingObjectPenalty;

            } else {

                targetConfidence = 0;
            }
        }


        // -------------------------------------------------------------------
        // Clamp confidence
        // -------------------------------------------------------------------

        if (targetConfidence > 100) {
            targetConfidence = 100;
        }
    }


    // =======================================================================
    // 15. STATE MACHINE
    // =======================================================================

    switch (trackingState) {


        // ===================================================================
        // NONE
        // ===================================================================

        case TargetTrackingState::NONE:

            targetLatched = false;
            movingObjectDetected = false;


            if (farCandidate) {

                trackingState =
                    TargetTrackingState::FAR_CANDIDATE;

                lastCandidatePoint =
                    farTierPoint;

                lastCandidateUpdateMs =
                    now;

                lastCandidateWasFar = true;
            }


            else if (nearCandidate) {

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;

                lastCandidatePoint =
                    nearTierPoint;

                lastCandidateUpdateMs =
                    now;

                lastCandidateWasFar = false;
            }

            break;


        // ===================================================================
        // FAR CANDIDATE
        // ===================================================================

        case TargetTrackingState::FAR_CANDIDATE:

            if (!currentCandidate) {

                if ((now -
                     lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    targetConfidence = 0;

                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint =
                        GridPoint{};
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


            // Once confidence has reached the approaching threshold,
            // progress to APPROACHING.
            if (targetConfidence >=
                config.approachingThreshold) {

                trackingState =
                    TargetTrackingState::APPROACHING;
            }


            // A near pair immediately moves us toward confirmation.
            if (nearCandidate) {

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;

                lastCandidateWasFar = false;
            }


            lastCandidatePoint =
                currentCandidatePoint;

            lastCandidateUpdateMs =
                now;

            break;


        // ===================================================================
        // APPROACHING
        // ===================================================================

        case TargetTrackingState::APPROACHING:

            if (!currentCandidate) {

                if ((now -
                     lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    targetConfidence = 0;

                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint =
                        GridPoint{};
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


            // Far -> near progression.
            if (farNearConsistent ||
                nearCandidate) {

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;

                lastCandidateWasFar = false;
            }


            lastCandidatePoint =
                currentCandidatePoint;

            lastCandidateUpdateMs =
                now;

            break;


        // ===================================================================
        // NEAR CONFIRMING
        // ===================================================================

        case TargetTrackingState::NEAR_CONFIRMING: {

            if (!nearCandidate) {

                // Allow a short ToF dropout.
                if ((now -
                     lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    targetConfidence = 0;

                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint =
                        GridPoint{};
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


            float nearDistance =
                nearTierPoint.distanceFromOrigin();


            bool insideNearRegion =
                nearDistance <=
                float(
                    config.nearConfirmationDistanceMm);


            // Final confirmation requires:
            //
            //   1. Near ToF pair
            //   2. Correct physical near region
            //   3. Sufficient confidence
            //   4. LiDAR must NOT identify the same object
            //   5. Object must NOT be moving
            //
            if (insideNearRegion &&
                targetConfidence >=
                    config.nearConfirmThreshold &&
                !anyObstacle &&
                !largeMovement) {

                trackingState =
                    TargetTrackingState::TARGET_CONFIRMED;

                targetLatched = true;
            }


            lastCandidatePoint =
                currentCandidatePoint;

            lastCandidateUpdateMs =
                now;

            break;
        }


        // ===================================================================
        // TARGET CONFIRMED
        // ===================================================================

        case TargetTrackingState::TARGET_CONFIRMED:

            targetLatched = true;


            // If the object suddenly moves, leave target-confirmed state.
            if (largeMovement) {

                targetLatched = false;

                targetConfidence = 0;

                movingObjectDetected = true;

                trackingState =
                    TargetTrackingState::MOVING_OBJECT;

                break;
            }


            // If LiDAR directly identifies the ToF POI as an obstacle,
            // do not keep reporting it as a target.
            //
            // We intentionally do not immediately erase all historical
            // confidence here. The confidence system can recover from
            // a single noisy LiDAR observation.
            if (anyObstacle) {

                targetLatched = false;

                trackingState =
                    TargetTrackingState::NEAR_CONFIRMING;

                break;
            }


            if (currentCandidate) {

                lastCandidatePoint =
                    currentCandidatePoint;

                lastCandidateUpdateMs =
                    now;

            }

            else if ((now -
                      lastCandidateUpdateMs) >
                     config.candidateTimeoutMs) {

                targetLatched = false;

                targetConfidence = 0;

                trackingState =
                    TargetTrackingState::NONE;

                lastCandidatePoint =
                    GridPoint{};
            }

            break;


        // ===================================================================
        // MOVING OBJECT
        // ===================================================================

        case TargetTrackingState::MOVING_OBJECT:

            targetLatched = false;

            movingObjectDetected = true;

            targetConfidence = 0;


            // Require the candidate to disappear before looking for
            // another target.
            if (!currentCandidate) {

                if ((now -
                     lastCandidateUpdateMs) >
                    config.candidateTimeoutMs) {

                    movingObjectDetected = false;

                    trackingState =
                        TargetTrackingState::NONE;

                    lastCandidatePoint =
                        GridPoint{};
                }
            }

            else {

                lastCandidatePoint =
                    currentCandidatePoint;

                lastCandidateUpdateMs =
                    now;
            }

            break;
    }


    // =======================================================================
    // 16. FINAL RESULT
    // =======================================================================

    // Same-object LiDAR takes priority over target confirmation for this
    // frame. This means a direct LiDAR/ToF match is reported as an obstacle.
    if (anyObstacle) {

        lastResult =
            TierDetectionResult::Obstacle;
    }

    else if (trackingState ==
             TargetTrackingState::TARGET_CONFIRMED) {

        lastResult =
            TierDetectionResult::Target;
    }

    else if (trackingState ==
             TargetTrackingState::MOVING_OBJECT) {

        lastResult =
            TierDetectionResult::Obstacle;
    }

    else if (currentCandidate) {

        lastResult =
            TierDetectionResult::Indeterminate;
    }

    else {

        lastResult =
            TierDetectionResult::None;
    }


    // =======================================================================
    // 17. DEBUG DATA
    // =======================================================================

    debug.targetConfidence =
        targetConfidence;

    debug.targetLatched =
        targetLatched;

    debug.movingObjectDetected =
        movingObjectDetected;


    return true;
}


// ===========================================================================
// PUBLIC GETTERS
// ===========================================================================

TierDetectionResult
TieredTargetDetector::getDetectionResult() const {

    return lastResult;
}


bool TieredTargetDetector::isTargetConfirmed() const {

    return targetLatched;
}


bool TieredTargetDetector::isObstacleDetected() const {

    return lastResult ==
           TierDetectionResult::Obstacle;
}


bool TieredTargetDetector::hasValidData() const {

    return
        isDistanceValid(debug.nearA) &&
        isDistanceValid(debug.nearB) &&
        isDistanceValid(debug.farA) &&
        isDistanceValid(debug.farB);
}


uint8_t TieredTargetDetector::getTargetConfidence() const {

    return targetConfidence;
}


bool TieredTargetDetector::isTargetLatched() const {

    return targetLatched;
}


bool TieredTargetDetector::isMovingObjectDetected() const {

    return movingObjectDetected;
}


TargetTrackingState
TieredTargetDetector::getTrackingState() const {

    return trackingState;
}


// ===========================================================================
// STRING HELPERS
// ===========================================================================

const char*
TieredTargetDetector::resultToString(
    TierDetectionResult result) const {

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


const char*
TieredTargetDetector::trackingStateToString(
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


// ===========================================================================
// DEBUG PRINT
// ===========================================================================

void TieredTargetDetector::debugPrint() const {

    Serial.println();
    Serial.println(F("========== TARGET DETECTOR =========="));

    // ---------------------------------------------------------------
    // RAW TOF
    // ---------------------------------------------------------------

    Serial.print(F("TOF  NearA="));
    Serial.print(debug.nearA);

    Serial.print(F(" NearB="));
    Serial.print(debug.nearB);

    Serial.print(F(" FarA="));
    Serial.print(debug.farA);

    Serial.print(F(" FarB="));
    Serial.println(debug.farB);


    // ---------------------------------------------------------------
    // TOF GEOMETRY
    // ---------------------------------------------------------------

    Serial.print(F("AVG  Near="));
    Serial.print(debug.nearAverage);

    Serial.print(F(" Far="));
    Serial.print(debug.farAverage);

    Serial.print(F(" Object="));
    Serial.println(debug.objectDistance);


    Serial.print(F("PAIR Near="));
    Serial.print(debug.nearPairMatch);

    Serial.print(F(" Far="));
    Serial.println(debug.farPairMatch);


    // ---------------------------------------------------------------
    // TEMPORAL / MOVEMENT CHECKS
    // ---------------------------------------------------------------

    Serial.print(F("TEMP PositionOK="));
    Serial.print(debug.positionConsistent);

    Serial.print(F(" FarNearOK="));
    Serial.print(debug.farNearConsistent);

    Serial.print(F(" LargeMovement="));
    Serial.println(debug.largeMovement);


    // ---------------------------------------------------------------
    // LIDAR
    // ---------------------------------------------------------------

    Serial.print(F("LIDAR SameNear="));
    Serial.print(debug.nearLidarConfirmsObstacle);

    Serial.print(F(" SameFar="));
    Serial.print(debug.farLidarConfirmsObstacle);

    Serial.print(F(" Separate="));
    Serial.println(debug.separateLidarObstacle);


    // ---------------------------------------------------------------
    // STEERING
    // ---------------------------------------------------------------

    Serial.print(F("STEER Active="));
    Serial.print(debug.steeringActive);

    Serial.print(F(" Value="));
    Serial.println(debug.steeringValue, 2);


    // ---------------------------------------------------------------
    // CONFIDENCE
    // ---------------------------------------------------------------

    Serial.print(F("CONFIDENCE="));
    Serial.print(debug.targetConfidence);

    Serial.print(F(" / 100"));

    Serial.print(F(" Latched="));
    Serial.print(debug.targetLatched);

    Serial.print(F(" Moving="));
    Serial.println(debug.movingObjectDetected);


    // ---------------------------------------------------------------
    // STATE / FINAL RESULT
    // ---------------------------------------------------------------

    Serial.print(F("STATE="));
    Serial.println(
        trackingStateToString(trackingState));

    Serial.print(F("RESULT="));
    Serial.println(
        resultToString(lastResult));


    Serial.println(F("======================================"));
}


// ===========================================================================
// GRID DEBUG
// ===========================================================================

void TieredTargetDetector::debugPrintGrid(
    Stream& serialport) const {

    serialport.print(F(">tof:"));

    const GridPoint* pts[4] = {
        &debug.nearAPoint,
        &debug.nearBPoint,
        &debug.farAPoint,
        &debug.farBPoint
    };


    for (uint8_t i = 0; i < 4; i++) {

        if (!pts[i]->valid)
            continue;

        serialport.print(
            pts[i]->x,
            1);

        serialport.print(":");

        serialport.print(
            pts[i]->y,
            1);

        serialport.print(";");
    }

    serialport.println(F("|xy"));
}


// ===========================================================================
// DEBUG GETTER
// ===========================================================================

const TieredTargetDetector::DetectionDebug&
TieredTargetDetector::getDebug() const {

    return debug;
}


// ===========================================================================
// DISTANCE VALIDATION
// ===========================================================================

bool TieredTargetDetector::isDistanceValid(
    uint16_t mm) const {

    return
        mm != INVALID_DISTANCE &&
        mm != 0;
}


// ===========================================================================
// TOF PAIR MATCH
// ===========================================================================

bool TieredTargetDetector::pairMatch(
    uint16_t aMm,
    uint16_t bMm,
    uint16_t expectedMm) const {

    if (!isDistanceValid(aMm) ||
        !isDistanceValid(bMm)) {

        return false;
    }

    uint16_t pairDifference =
        (aMm > bMm)
        ? (aMm - bMm)
        : (bMm - aMm);

    if (pairDifference >
        config.toleratedWidthMm) {

        return false;
    }

    int32_t avg =
        (int32_t(aMm) +
         int32_t(bMm)) /
        2;

    return
        (avg >=
         int32_t(expectedMm) -
         int32_t(config.intersectionToleranceMm))
        &&
        (avg <=
         int32_t(expectedMm) +
         int32_t(config.intersectionToleranceMm));
}


// ===========================================================================
// AVERAGE DISTANCE
// ===========================================================================

uint16_t TieredTargetDetector::averageDistance(
    uint16_t aMm,
    uint16_t bMm) const {

    if (!isDistanceValid(aMm) ||
        !isDistanceValid(bMm)) {

        return INVALID_DISTANCE;
    }


    return uint16_t(
        (uint32_t(aMm) +
         uint32_t(bMm)) /
        2);
}


// ===========================================================================
// DETECTION ENVELOPE
// ===========================================================================

bool TieredTargetDetector::withinDetectionEnvelope(
    uint16_t rangeMm) const {

    if (!isDistanceValid(rangeMm))
        return false;

    if (rangeMm >
        config.maxDetectionRangeMm)
        return false;

    return true;
}


// ===========================================================================
// TOF POINT MIDPOINT
// ===========================================================================

GridPoint TieredTargetDetector::midpoint(
    const GridPoint& a,
    const GridPoint& b) const {

    GridPoint m;


    if (a.valid && b.valid) {

        m.x =
            (a.x + b.x) /
            2.0f;

        m.y =
            (a.y + b.y) /
            2.0f;

        m.valid = true;
    }

    else if (a.valid) {

        m = a;
    }

    else if (b.valid) {

        m = b;
    }


    return m;
}


// ===========================================================================
// LIDAR POINT -> ROBOT GRID
// ===========================================================================

GridPoint TieredTargetDetector::lidarPointToRobotGrid(
    const DataPoint& point) const {

    GridPoint p;


    // Convert the LD06 angle into the robot coordinate frame.

    float robotAngleDeg =
        point.angle +
        LIDAR_YAW_OFFSET_DEG;


    while (robotAngleDeg >= 360.0f) {
        robotAngleDeg -= 360.0f;
    }

    while (robotAngleDeg < 0.0f) {
        robotAngleDeg += 360.0f;
    }


    float angleRad =
        robotAngleDeg *
        PI /
        180.0f;


    // Position of the LiDAR relative to robot origin.

    float lidarX =
        float(point.distance) *
        sinf(angleRad);

    float lidarY =
        -float(point.distance) *
        cosf(angleRad);


    // Translate to robot-origin coordinates.

    p.x =
        LIDAR_X_MM +
        lidarX;

    p.y =
        LIDAR_Y_MM +
        lidarY;

    p.valid = true;


    return p;
}


// ===========================================================================
// SAME-OBJECT LIDAR CHECK
// ===========================================================================

bool TieredTargetDetector::isObstacleAgainstLidar(
    const GridPoint& tofPoint) const {

    if (!tofPoint.valid)
        return false;


    float tofDist =
        tofPoint.distanceFromOrigin();

    float tofBearing =
        tofPoint.bearingFromOriginDeg();


    float lidarDist;


    if (!findNearestLidarAtBearing(
            tofBearing,
            config.lidarBearingToleranceDeg,
            lidarDist)) {

        // No LiDAR return is NOT evidence that the object is absent.
        return false;
    }


    return
        fabsf(
            lidarDist -
            tofDist)
        <=
        float(
            config.lidarMatchToleranceMm);
}


// ===========================================================================
// SEPARATE LIDAR OBSTACLE
// ===========================================================================
//
// Requirements:
//
//   1. LiDAR point between 80 and 700 mm
//   2. Point is at least 100 mm from ToF POI
//   3. At least 3 LiDAR points form a cluster
//   4. Cluster radius <= 80 mm
//
// No additional angular gate is applied here yet.
//
// ===========================================================================

bool TieredTargetDetector::hasSeparateLidarObstacle(
    const GridPoint& tofPoint) const {

    if (!tofPoint.valid)
        return false;


    const uint16_t n =
        lidar.getNbPointsInScan();


    if (n < config.minimumSeparateLidarPoints)
        return false;


    // -----------------------------------------------------------------------
    // Search for a possible cluster anchor.
    // -----------------------------------------------------------------------

    for (uint16_t i = 0;
         i < n;
         ++i) {

        DataPoint* anchorPoint =
            lidar.getPoints(i);


        if (anchorPoint == nullptr)
            continue;


        if (anchorPoint->distance <
            config.lidarObstacleMinMm) {

            continue;
        }


        if (anchorPoint->distance >
            config.lidarObstacleMaxMm) {

            continue;
        }


        GridPoint anchor =
            lidarPointToRobotGrid(
                *anchorPoint);


        if (!anchor.valid)
            continue;


        // -------------------------------------------------------------------
        // Anchor must be physically separated from the ToF POI.
        // -------------------------------------------------------------------

        float anchorDx =
            anchor.x -
            tofPoint.x;

        float anchorDy =
            anchor.y -
            tofPoint.y;


        float anchorSeparation =
            sqrtf(
                anchorDx * anchorDx +
                anchorDy * anchorDy);


        if (anchorSeparation <
            float(config.separateObstacleDistanceMm)) {

            continue;
        }


        // -------------------------------------------------------------------
        // Count nearby LiDAR points.
        // -------------------------------------------------------------------

        uint8_t clusterCount = 1;


        for (uint16_t j = 0;
             j < n;
             ++j) {

            if (j == i)
                continue;


            DataPoint* candidatePoint =
                lidar.getPoints(j);


            if (candidatePoint == nullptr)
                continue;


            if (candidatePoint->distance <
                config.lidarObstacleMinMm) {

                continue;
            }


            if (candidatePoint->distance >
                config.lidarObstacleMaxMm) {

                continue;
            }


            GridPoint candidate =
                lidarPointToRobotGrid(
                    *candidatePoint);


            if (!candidate.valid)
                continue;


            // ---------------------------------------------------------------
            // Candidate must also be separated from ToF POI.
            // ---------------------------------------------------------------

            float dxToTof =
                candidate.x -
                tofPoint.x;

            float dyToTof =
                candidate.y -
                tofPoint.y;


            float separationFromTof =
                sqrtf(
                    dxToTof * dxToTof +
                    dyToTof * dyToTof);


            if (separationFromTof <
                float(config.separateObstacleDistanceMm)) {

                continue;
            }


            // ---------------------------------------------------------------
            // Candidate must be close to the anchor.
            // ---------------------------------------------------------------

            float dx =
                candidate.x -
                anchor.x;

            float dy =
                candidate.y -
                anchor.y;


            float clusterDistance =
                sqrtf(
                    dx * dx +
                    dy * dy);


            if (clusterDistance <=
                float(config.lidarClusterRadiusMm)) {

                ++clusterCount;


                if (clusterCount >=
                    config.minimumSeparateLidarPoints) {

                    return true;
                }
            }
        }
    }


    return false;
}


// ===========================================================================
// FIND LIDAR POINT AT BEARING
// ===========================================================================

bool TieredTargetDetector::findNearestLidarAtBearing(
    float bearingDeg,
    float toleranceDeg,
    float& outDistanceMm) const {

    uint16_t n =
        lidar.getNbPointsInScan();


    float bestDelta =
        1.0e6f;

    bool found = false;


    for (uint16_t i = 0;
         i < n;
         i++) {

        DataPoint* pt =
            lidar.getPoints(i);


        if (pt == nullptr)
            continue;


        if (pt->distance == 0)
            continue;


        GridPoint lidarPoint =
            lidarPointToRobotGrid(*pt);


        if (!lidarPoint.valid)
            continue;


        float dist =
            lidarPoint.distanceFromOrigin();

        float bear =
            lidarPoint.bearingFromOriginDeg();


        // Existing general LiDAR angle gate.

        if (config.useAngleGate) {

            float lo =
                config.detectionAngleMinDeg;

            float hi =
                config.detectionAngleMaxDeg;


            bool inside =
                (lo <= hi)
                ? (bear >= lo && bear <= hi)
                : (bear >= lo || bear <= hi);


            if (!inside)
                continue;
        }


        float delta =
            fabsf(
                angleDiffDeg(
                    bear,
                    bearingDeg));


        if (delta <= toleranceDeg &&
            delta < bestDelta) {

            bestDelta =
                delta;

            outDistanceMm =
                dist;

            found = true;
        }
    }


    return found;
}


// ===========================================================================
// STEERING
// ===========================================================================

float TieredTargetDetector::computeSteering(
    const GridPoint& hit,
    const SensorMount& mountA,
    const SensorMount& mountB) const {

    if (!hit.valid)
        return 0.0f;


    float halfSpacing =
        fabsf(
            mountA.xMm -
            mountB.xMm) /
        2.0f;


    if (halfSpacing < 1.0f)
        return 0.0f;


    // +X = right
    // -X = left

    float norm =
        hit.x /
        halfSpacing;


    if (norm > 1.0f)
        norm = 1.0f;

    if (norm < -1.0f)
        norm = -1.0f;


    return norm;
}


// ===========================================================================
// LIDAR FRONT TEST
// ===========================================================================

void TieredTargetDetector::debugPrintLidarFrontTest() const {

    uint16_t n =
        lidar.getNbPointsInScan();


    Serial.println();
    Serial.println(
        "===== LIDAR RAW FRONT TEST =====");


    Serial.print(
        "Points in scan: ");

    Serial.println(n);


    if (n == 0) {

        Serial.println(
            "No LiDAR points available.");

        Serial.println(
            "================================");

        return;
    }


    const uint8_t NUM_CLOSEST = 10;


    uint16_t closestDistance[
        NUM_CLOSEST];

    uint16_t closestIndex[
        NUM_CLOSEST];


    for (uint8_t i = 0;
         i < NUM_CLOSEST;
         i++) {

        closestDistance[i] =
            0xFFFF;

        closestIndex[i] =
            0xFFFF;
    }


    for (uint16_t i = 0;
         i < n;
         i++) {

        DataPoint* pt =
            lidar.getPoints(i);


        if (pt == nullptr)
            continue;


        if (pt->distance == 0)
            continue;


        for (uint8_t j = 0;
             j < NUM_CLOSEST;
             j++) {

            if (pt->distance <
                closestDistance[j]) {

                for (uint8_t k =
                         NUM_CLOSEST - 1;
                     k > j;
                     k--) {

                    closestDistance[k] =
                        closestDistance[k - 1];

                    closestIndex[k] =
                        closestIndex[k - 1];
                }


                closestDistance[j] =
                    pt->distance;

                closestIndex[j] =
                    i;

                break;
            }
        }
    }


    Serial.println(
        "Closest LiDAR points:");


    for (uint8_t i = 0;
         i < NUM_CLOSEST;
         i++) {

        if (closestIndex[i] ==
            0xFFFF) {

            continue;
        }


        DataPoint* pt =
            lidar.getPoints(
                closestIndex[i]);


        if (pt == nullptr)
            continue;


        Serial.print("#");
        Serial.print(i + 1);


        Serial.print(
            "  angle=");

        Serial.print(
            pt->angle,
            2);

        Serial.print(
            " deg");


        Serial.print(
            "  distance=");

        Serial.print(
            pt->distance);

        Serial.print(
            " mm");


#ifdef LD06_COMPUTE_XY

        Serial.print(
            "  LD06 x=");

        Serial.print(
            pt->x);

        Serial.print(
            " mm");


        Serial.print(
            "  LD06 y=");

        Serial.print(
            pt->y);

        Serial.print(
            " mm");

#endif


        GridPoint robotPoint =
            lidarPointToRobotGrid(
                *pt);


        Serial.print(
            "  Robot X=");

        Serial.print(
            robotPoint.x,
            1);

        Serial.print(
            " mm");


        Serial.print(
            "  Robot Y=");

        Serial.print(
            robotPoint.y,
            1);

        Serial.print(
            " mm");


        Serial.print(
            "  intensity=");

        Serial.println(
            pt->intensity);
    }


    Serial.println(
        "================================");
}
