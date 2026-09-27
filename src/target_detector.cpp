#include "target_detector.h"

namespace {
// ---------------------------------------------------------------------------
// Physical layout defaults.
//
// Origin (0,0) = midpoint between the two FAR-tier (front) TOF sensors, at
// the front of the robot. +X = straight ahead, +Y = robot's right.
//
// From your message, taken as the RAW sensor-to-crossing distance (same
// quantity pairMatch() has always compared against):
//   - far tier  (front, mounting line = the origin, x = 0 mm): 175 mm
//   - near tier (mounted 30 mm behind the origin):              150 mm
// That puts the far tier's crossing point further from the robot than the
// near tier's once the near tier's 30mm setback is applied (175mm vs
// ~120mm along the centerline) -- so "far"/"near" here follows the net
// forward reach, not which tier is physically mounted further forward.
// Flag this mapping if it's backwards on the real robot.
//
// STILL PLACEHOLDERS (flagged in chat): the left/right half-spacing between
// the two sensors within each tier. I don't have a measured number for this,
// so the values below are guesses picked only to keep the geometry
// self-consistent (they produce ~30 deg toe-in angles, which is physically
// plausible but unverified). Once you have the real spacing, change just
// these two constants -- every mount position/angle below recomputes itself.
constexpr float HALF_SPACING_FAR_MM  = 87.5f;  // PLACEHOLDER -- half the far-tier L/R spacing
constexpr float HALF_SPACING_NEAR_MM = 75.0f;  // PLACEHOLDER -- half the near-tier L/R spacing

constexpr float FAR_TIER_X_MM  = 0.0f;
constexpr float NEAR_TIER_X_MM = -30.0f;

// LD06 datasheet: housing is 38.59 x 38.59 x 33.50 mm (L*W*H), and the
// rotation center (its measurement origin) is the geometric center of that
// footprint for this compact/symmetric design.
constexpr float LD06_HOUSING_SIZE_MM = 38.59f;

// "From the right far-tier sensor, the lidar's top-right corner is 35mm
// left and 35mm back." Right far sensor sits at (FAR_TIER_X_MM,
// +HALF_SPACING_FAR_MM); left = -Y, back = -X in this convention.
constexpr float LIDAR_CORNER_X_MM = FAR_TIER_X_MM - 35.0f;
constexpr float LIDAR_CORNER_Y_MM = HALF_SPACING_FAR_MM - 35.0f;

// ASSUMPTION (flagged in chat): "top right" is taken as the corner nearest
// the front-right of the robot, so its rotation center sits half a housing
// width further back and further left (toward the chassis centerline).
// Verify against the LD06 mechanical drawing / your CAD and flip the signs
// here if it's the wrong corner.
constexpr float LIDAR_OFFSET_X_MM = LIDAR_CORNER_X_MM - (LD06_HOUSING_SIZE_MM / 2.0f);
constexpr float LIDAR_OFFSET_Y_MM = LIDAR_CORNER_Y_MM - (LD06_HOUSING_SIZE_MM / 2.0f);
// ASSUMPTION: lidar's forward mark (its own zero-angle direction) is
// mounted aligned with the robot's forward axis.
constexpr float LIDAR_OFFSET_ANGLE_DEG = 0.0f;

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

    // farA = right sensor, farB = left sensor (mirror the two calls below if
    // that's backwards for your wiring). Same for near.
    mountFarA = SensorMount::aimedAtCenterlineCrossing(FAR_TIER_X_MM, HALF_SPACING_FAR_MM, config.farIntersectMm);
    mountFarB = SensorMount::aimedAtCenterlineCrossing(FAR_TIER_X_MM, -HALF_SPACING_FAR_MM, config.farIntersectMm);
    mountNearA = SensorMount::aimedAtCenterlineCrossing(NEAR_TIER_X_MM, HALF_SPACING_NEAR_MM, config.nearIntersectMm);
    mountNearB = SensorMount::aimedAtCenterlineCrossing(NEAR_TIER_X_MM, -HALF_SPACING_NEAR_MM, config.nearIntersectMm);

    // Configure the LD06's own coordinate transform so DataPoint.x/.y come
    // out already expressed in this same shared frame -- see ld06.cpp
    // computeData(), which folds _xOffset/_yOffset/_angularOffset straight
    // into each point. No separate transform needed on our side for lidar
    // points; we just read pt->x / pt->y directly in findNearestLidarAtBearing().
    lidar.setOffsetPosition(int16_t(LIDAR_OFFSET_X_MM), int16_t(LIDAR_OFFSET_Y_MM), LIDAR_OFFSET_ANGLE_DEG);

    debug = DetectionDebug{};
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

void TieredTargetDetector::setLidarOffset(float xMm, float yMm, float angleDeg) {
    lidar.setOffsetPosition(int16_t(xMm), int16_t(yMm), angleDeg);
}

bool TieredTargetDetector::update() {
    
    if (!tofArray.isInitialized()) {
        lastResult = TierDetectionResult::None;
        return false;
    }

    uint16_t nearA = tofArray.getDistance(sensorNearA);
    uint16_t nearB = tofArray.getDistance(sensorNearB);
    uint16_t farA = tofArray.getDistance(sensorFarA);
    uint16_t farB = tofArray.getDistance(sensorFarB);

    // Detection envelope: hard range cap (angle-gate skeleton lives in
    // findNearestLidarAtBearing() instead, since it needs a bearing to gate).
    if (!withinDetectionEnvelope(nearA)) nearA = INVALID_DISTANCE;
    if (!withinDetectionEnvelope(nearB)) nearB = INVALID_DISTANCE;
    if (!withinDetectionEnvelope(farA)) farA = INVALID_DISTANCE;
    if (!withinDetectionEnvelope(farB)) farB = INVALID_DISTANCE;

    debug.nearA = nearA;
    debug.nearB = nearB;
    debug.farA = farA;
    debug.farB = farB;
    debug.nearAverage = averageDistance(nearA, nearB);
    debug.farAverage = averageDistance(farA, farB);
    debug.nearPairMatch = pairMatch(nearA, nearB, config.nearIntersectMm);
    debug.farPairMatch = pairMatch(farA, farB, config.farIntersectMm);

    // Project every valid raw reading into the shared (x, y) grid.
    debug.nearAPoint = isDistanceValid(nearA) ? tofPointFromRange(mountNearA, nearA) : GridPoint{};
    debug.nearBPoint = isDistanceValid(nearB) ? tofPointFromRange(mountNearB, nearB) : GridPoint{};
    debug.farAPoint = isDistanceValid(farA) ? tofPointFromRange(mountFarA, farA) : GridPoint{};
    debug.farBPoint = isDistanceValid(farB) ? tofPointFromRange(mountFarB, farB) : GridPoint{};

    GridPoint nearTierPoint = midpoint(debug.nearAPoint, debug.nearBPoint);
    GridPoint farTierPoint = midpoint(debug.farAPoint, debug.farBPoint);

    // Obstacle case: the tier's hit point is corroborated by the lidar at
    // (roughly) the same distance along the same bearing -- i.e. the object
    // is tall enough that both the TOF pair and the lidar see the same
    // surface, so it isn't a 70mm target.
    debug.nearLidarConfirmsObstacle = debug.nearPairMatch && isObstacleAgainstLidar(nearTierPoint);
    debug.farLidarConfirmsObstacle = debug.farPairMatch && isObstacleAgainstLidar(farTierPoint);

    // Off-center steering: only one sensor in a tier fired, so the target is
    // to one side of that tier's design crossing point rather than dead
    // center. Near tier takes priority since it's the more immediate case.
    debug.steeringActive = false;
    debug.steeringValue = 0.0f;
    if (isDistanceValid(nearA) != isDistanceValid(nearB)) {
        const GridPoint& hit = isDistanceValid(nearA) ? debug.nearAPoint : debug.nearBPoint;
        debug.steeringValue = computeSteering(hit, mountNearA, mountNearB);
        debug.steeringActive = true;
    } else if (isDistanceValid(farA) != isDistanceValid(farB)) {
        const GridPoint& hit = isDistanceValid(farA) ? debug.farAPoint : debug.farBPoint;
        debug.steeringValue = computeSteering(hit, mountFarA, mountFarB);
        debug.steeringActive = true;
    }

    if (debug.nearPairMatch && debug.farPairMatch) {
        debug.objectDistance = debug.nearAverage < debug.farAverage ? debug.nearAverage : debug.farAverage;
    } else if (debug.nearPairMatch) {
        debug.objectDistance = debug.nearAverage;
    } else if (debug.farPairMatch) {
        debug.objectDistance = debug.farAverage;
    } else {
        debug.objectDistance = INVALID_DISTANCE;
    }

    bool anyObstacle = debug.nearLidarConfirmsObstacle || debug.farLidarConfirmsObstacle;

    if (debug.nearPairMatch && debug.farPairMatch) {
        lastResult = anyObstacle ? TierDetectionResult::Obstacle : TierDetectionResult::Target;
    } else if (debug.nearPairMatch || debug.farPairMatch) {
        lastResult = anyObstacle ? TierDetectionResult::Obstacle : TierDetectionResult::Indeterminate;
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
    Serial.print(" nearAvg:"); Serial.print(debug.nearAverage == INVALID_DISTANCE ? 0xFFFF : debug.nearAverage);
    Serial.print(" farAvg:"); Serial.print(debug.farAverage == INVALID_DISTANCE ? 0xFFFF : debug.farAverage);
    Serial.print(" objDist:"); Serial.print(debug.objectDistance == INVALID_DISTANCE ? 0xFFFF : debug.objectDistance);
    Serial.print(" pairNear:"); Serial.print(debug.nearPairMatch);
    Serial.print(" pairFar:"); Serial.print(debug.farPairMatch);
    Serial.print(" lidarObsNear:"); Serial.print(debug.nearLidarConfirmsObstacle);
    Serial.print(" lidarObsFar:"); Serial.print(debug.farLidarConfirmsObstacle);
    Serial.print(" steerActive:"); Serial.print(debug.steeringActive);
    Serial.print(" steer:"); Serial.print(debug.steeringValue, 2);
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

bool TieredTargetDetector::pairMatch(uint16_t aMm, uint16_t bMm, uint16_t expectedMm) const {
    if (!isDistanceValid(aMm) || !isDistanceValid(bMm)) {
        return false;
    }

    int32_t avg = (int32_t(aMm) + int32_t(bMm)) / 2;
    return (avg >= int32_t(expectedMm) - config.intersectionToleranceMm) &&
           (avg <= int32_t(expectedMm) + config.intersectionToleranceMm);
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

GridPoint TieredTargetDetector::tofPointFromRange(const SensorMount& mount, uint16_t rangeMm) const {
    GridPoint p;
    if (!isDistanceValid(rangeMm)) return p;
    float rad = mount.boresightDeg * (PI / 180.0f);
    p.x = mount.xMm + float(rangeMm) * cosf(rad);
    p.y = mount.yMm + float(rangeMm) * sinf(rad);
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

bool TieredTargetDetector::findNearestLidarAtBearing(float bearingDeg, float toleranceDeg, float& outDistanceMm) const {
    uint16_t n = lidar.getNbPointsInScan();
    float bestDelta = 1.0e6f;
    bool found = false;

    for (uint16_t i = 0; i < n; i++) {
        DataPoint* pt = lidar.getPoints(i);
#ifdef LD06_COMPUTE_XY
        float px = float(pt->x);
        float py = float(pt->y);
#else
        // Falls back to the point's own local range/angle if XY isn't
        // compiled in -- less accurate since it ignores the lidar's
        // mounting offset entirely.
        float rad = pt->angle * (PI / 180.0f);
        float px = float(pt->distance) * cosf(rad);
        float py = float(pt->distance) * sinf(rad);
#endif
        float dist = sqrtf(px * px + py * py);
        float bear = atan2f(py, px) * 180.0f / PI;

        if (config.useAngleGate) {
            // Skeleton: skip lidar returns outside the configured forward
            // cone. TODO: confirm this is the "specified angle of detection"
            // you meant, and tune detectionAngleMinDeg/MaxDeg -- currently
            // defaults to the full front half (-90..+90) and is OFF
            // (useAngleGate=false) until you flip it on.
            float lo = config.detectionAngleMinDeg;
            float hi = config.detectionAngleMaxDeg;
            bool inside = (lo <= hi) ? (bear >= lo && bear <= hi) : (bear >= lo || bear <= hi);
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

float TieredTargetDetector::computeSteering(const GridPoint& hit, const SensorMount& mountA, const SensorMount& mountB) const {
    if (!hit.valid) return 0.0f;
    float halfSpacing = fabsf(mountA.yMm - mountB.yMm) / 2.0f;
    if (halfSpacing < 1.0f) return 0.0f;
    float norm = hit.y / halfSpacing; // +Y = right, matching the shared frame convention
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    return norm;
}