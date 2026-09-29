//************************************
//       POIDetector.cpp
//************************************
// POI (Point of Interest / Weight) detection implementation

#include "POIDetector.h"
#include "Localisation.h"
#include "MappingNav.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f
#define TWO_PI_F (2.0f * PI_F)

// Helper: wrap angle to [-PI, PI]
static float wrapAngleRad(float angle) {
    while (angle > PI_F) angle -= TWO_PI_F;
    while (angle <= -PI_F) angle += TWO_PI_F;
    return angle;
}

// Helper: normalize angle to [0, 2*PI]
// (Currently unused but kept for potential future use)
// static float normalizeAngleCCW(float angle) {
//     while (angle < 0.0f) angle += TWO_PI_F;
//     while (angle >= TWO_PI_F) angle -= TWO_PI_F;
//     return angle;
// }

//====================================
// CONSTRUCTOR
//====================================

POIDetector::POIDetector() {
    m_candidate = POICandidate();
    for (uint8_t i = 0; i < MAX_CONFIRMED_POIS; i++) {
        m_confirmed_pois[i] = ConfirmedPOI();
    }
    m_diags = POIDetectorDiags();
    m_confirmed_poi_count = 0;
}

void POIDetector::begin(const ToFExtrinsics &extrinsics) {
    setExtrinsics(extrinsics);
    m_last_update_ms = millis();
}

void POIDetector::setExtrinsics(const ToFExtrinsics &ext) {
    m_extrinsics = ext;
}

void POIDetector::reset() {
    m_candidate = POICandidate();
    for (uint8_t i = 0; i < MAX_CONFIRMED_POIS; i++) {
        m_confirmed_pois[i] = ConfirmedPOI();
    }
    m_confirmed_poi_count = 0;
}

//====================================
// WORLD POSITION CALCULATION
//====================================

void POIDetector::calculateToFWorldPosition(const Localisation &loc,
                                           float &out_x_m, float &out_y_m,
                                           float &out_yaw_rad) const {
    RobotPose pose = loc.getPose();
    
    // Convert robot pose from mm to m
    float robot_x_m = pose.x_mm / 1000.0f;
    float robot_y_m = pose.y_mm / 1000.0f;
    float robot_theta = pose.theta_rad;
    
    // Transform sensor offset from robot frame to world frame
    // sensor_x_robot = m_extrinsics.x_offset_m (forward)
    // sensor_y_robot = m_extrinsics.y_offset_m (left)
    // Rotation: [cos -sin; sin cos]
    
    float cos_theta = cosf(robot_theta);
    float sin_theta = sinf(robot_theta);
    
    float dx_world = cos_theta * m_extrinsics.x_offset_m
                     - sin_theta * m_extrinsics.y_offset_m;
    float dy_world = sin_theta * m_extrinsics.x_offset_m
                     + cos_theta * m_extrinsics.y_offset_m;
    
    out_x_m = robot_x_m + dx_world;
    out_y_m = robot_y_m + dy_world;
    out_yaw_rad = robot_theta + m_extrinsics.yaw_offset_rad;
    out_yaw_rad = wrapAngleRad(out_yaw_rad);
}

void POIDetector::calculateToFHitPosition(float sensor_x_m, float sensor_y_m,
                                          float sensor_yaw_rad, float tof_range_m,
                                          float &out_x_m, float &out_y_m) const {
    float cos_yaw = cosf(sensor_yaw_rad);
    float sin_yaw = sinf(sensor_yaw_rad);
    
    out_x_m = sensor_x_m + tof_range_m * cos_yaw;
    out_y_m = sensor_y_m + tof_range_m * sin_yaw;
}

//====================================
// EXPECTED LIDAR RANGE CALCULATION
//====================================

void POIDetector::calculateExpectedLiDARRange(const MappingNav &nav,
                                              float sensor_x_m, float sensor_y_m,
                                              float sensor_yaw_rad,
                                              float &out_expected_range_m,
                                              ExpectedRangeStatus &out_status,
                                              uint8_t &out_point_count) {
    out_expected_range_m = 0.0f;
    out_point_count = 0;
    
    // Grid parameters
    const uint16_t grid_w = MappingNav::GRID_WIDTH;
    const uint16_t grid_h = MappingNav::GRID_HEIGHT;
    const float cell_size = MappingNav::CELL_SIZE_M;
    const float arena_w = MappingNav::ARENA_WIDTH_M;
    const float arena_h = MappingNav::ARENA_HEIGHT_M;
    
    // Raycast through occupancy grid along ToF beam
    // Start at sensor origin, step along beam direction
    float cos_yaw = cosf(sensor_yaw_rad);
    float sin_yaw = sinf(sensor_yaw_rad);
    
    // Step size along ray
    const float step_m = 0.05f;  // 5cm steps
    const float max_range = POIConfig::POI_MAX_RANGE_M;
    
    float best_hit_range = max_range;  // No hit found yet
    float unknown_range = -1.0f;       // First unknown cell hit
    float free_count = 0.0f;           // Track free cells (for future use)
    
    for (float r = 0.0f; r <= max_range; r += step_m) {
        float x = sensor_x_m + r * cos_yaw;
        float y = sensor_y_m + r * sin_yaw;
        
        // Boundary check: arena is [0, arena_w] x [0, arena_h]
        if (x < 0.0f || x >= arena_w || y < 0.0f || y >= arena_h) {
            // Ray has exited arena
            if (best_hit_range >= max_range) {
                out_status = EXPECTED_RANGE_BLOCKED;  // Ray exits without hitting occupied cell
            }
            break;
        }
        
        // Convert (x, y) in meters to grid cell index
        uint16_t cell_x = (uint16_t)(x / cell_size);
        uint16_t cell_y = (uint16_t)(y / cell_size);
        
        // Boundary checks (should not happen due to earlier check, but safe)
        if (cell_x >= grid_w || cell_y >= grid_h) continue;
        
        uint16_t cell_idx = cell_y * grid_w + cell_x;
        MappingNav::Occupancy occ = nav.getOccupancy(cell_idx);
        
        if (occ == MappingNav::OCC_OCCUPIED) {
            // Found first occupied cell along beam
            best_hit_range = r;
            out_point_count++;
            break;
        } else if (occ == MappingNav::OCC_UNKNOWN) {
            // Track first unknown region
            if (unknown_range < 0.0f) {
                unknown_range = r;
            }
        } else if (occ == MappingNav::OCC_FREE) {
            free_count++;
        }
    }
    
    // Determine status and expected range
    if (best_hit_range < max_range) {
        // We found an occupied cell; this is our expected range
        out_expected_range_m = best_hit_range;
        out_status = EXPECTED_RANGE_VALID;
    } else if (unknown_range >= 0.0f) {
        // Ray passed through unknown cells; prediction is unreliable
        out_expected_range_m = unknown_range;
        out_status = EXPECTED_RANGE_UNKNOWN;
    } else {
        // Ray is all free space to max range
        out_expected_range_m = max_range;
        out_status = EXPECTED_RANGE_BLOCKED;
    }
    
    m_diags.expected_lidar_points = out_point_count;
}

//====================================
// ANOMALY DETECTION
//====================================

void POIDetector::processAnomaly(const Localisation &loc, float tof_range_m,
                                  float expected_range_m, ExpectedRangeStatus status) {
    uint32_t now_ms = millis();
    
    // Calculate range difference
    // Positive difference = ToF sees something closer than LiDAR/map expected
    // This is the anomaly we're looking for (potential weight/object)
    float range_diff = expected_range_m - tof_range_m;
    
    // If expected range is UNKNOWN or INVALID, don't process as strong anomaly
    if (status == EXPECTED_RANGE_UNKNOWN || status == EXPECTED_RANGE_INVALID) {
        // Could mark as candidate with lower confidence
        if (m_candidate.state == POI_STATE_NONE) {
            // Very conservative: require explicit VALID status for detection
            return;
        }
    }
    
    // Calculate hit position in world frame
    float sensor_x_m, sensor_y_m, sensor_yaw_rad;
    calculateToFWorldPosition(loc, sensor_x_m, sensor_y_m, sensor_yaw_rad);
    
    float hit_x_m, hit_y_m;
    calculateToFHitPosition(sensor_x_m, sensor_y_m, sensor_yaw_rad, tof_range_m,
                            hit_x_m, hit_y_m);
    
    // Check if this looks like an anomaly
    // Only POSITIVE differences are anomalies (ToF closer than expected)
    // Negative or small differences indicate normal obstacles
    if (range_diff < POIConfig::POI_MIN_RANGE_DIFF_M) {
        // No anomaly; ToF either closer or not different enough
        if (m_candidate.state != POI_STATE_NONE) {
            // Candidate exists but this reading doesn't support it
            // Reset or degrade candidate
            m_candidate.state = POI_STATE_NONE;
            m_candidate.confirmation_count = 0;
        }
        return;
    }
    
    // We have a significant positive range difference; treat as potential anomaly
    
    if (m_candidate.state == POI_STATE_NONE) {
        // Start new candidate
        m_candidate.state = POI_STATE_CANDIDATE;
        m_candidate.tof_range_m = tof_range_m;
        m_candidate.expected_lidar_range_m = expected_range_m;
        m_candidate.lidar_status = status;
        m_candidate.range_difference_m = range_diff;
        m_candidate.x_m = hit_x_m;
        m_candidate.y_m = hit_y_m;
        m_candidate.confirmation_count = 1;
        m_candidate.first_seen_ms = now_ms;
        m_candidate.last_seen_ms = now_ms;
        m_candidate.confidence = 100.0f;
    } else if (m_candidate.state == POI_STATE_CANDIDATE) {
        // Existing candidate; check consistency
        if (isPositionConsistent(hit_x_m, hit_y_m)) {
            // Reading is consistent; increment confirmation
            m_candidate.confirmation_count++;
            m_candidate.last_seen_ms = now_ms;
            
            // Update position (weighted average)
            m_candidate.x_m = (m_candidate.x_m * (m_candidate.confirmation_count - 1) + hit_x_m)
                             / m_candidate.confirmation_count;
            m_candidate.y_m = (m_candidate.y_m * (m_candidate.confirmation_count - 1) + hit_y_m)
                             / m_candidate.confirmation_count;
            
            // Increase confidence
            m_candidate.confidence = fminf(999.0f,
                m_candidate.confidence + 150.0f / POIConfig::POI_CONFIRM_COUNT);
            
            checkConfirmation();
        } else {
            // Reading is not spatially consistent; reject candidate
            m_candidate.state = POI_STATE_REJECTED;
            m_candidate.rejection_time_ms = now_ms;
            m_candidate.rejection_count++;
        }
    } else if (m_candidate.state == POI_STATE_REJECTED) {
        // Candidate was rejected; wait before allowing new candidate
        if (now_ms - m_candidate.rejection_time_ms > 5000) {
            // 5 second cooldown; allow new candidate
            m_candidate.state = POI_STATE_NONE;
        }
    }
}

bool POIDetector::isPositionConsistent(float new_x_m, float new_y_m) const {
    if (m_candidate.state == POI_STATE_NONE) return true;
    
    float dx = new_x_m - m_candidate.x_m;
    float dy = new_y_m - m_candidate.y_m;
    float distance = sqrtf(dx * dx + dy * dy);
    
    return distance < POIConfig::POI_POSITION_TOLERANCE_M;
}

void POIDetector::checkConfirmation() {
    uint32_t now_ms = millis();
    
    if (m_candidate.state != POI_STATE_CANDIDATE) return;
    
    // Two confirmation criteria:
    // 1. Enough readings
    // 2. Readings span sufficient time
    
    bool enough_readings = m_candidate.confirmation_count >= POIConfig::POI_CONFIRM_COUNT;
    bool time_span = (now_ms - m_candidate.first_seen_ms) >= POIConfig::POI_CONFIRM_TIME_MS;
    
    if (enough_readings && time_span) {
        // Candidate is confirmed!
        m_candidate.state = POI_STATE_CONFIRMED;
        
        ConfirmedPOI poi;
        poi.active = true;
        poi.x_m = m_candidate.x_m;
        poi.y_m = m_candidate.y_m;
        poi.confidence = m_candidate.confidence;
        poi.timestamp_ms = now_ms;
        poi.confirmation_count = m_candidate.confirmation_count;
        
        // Try to merge with existing POI or add new
        if (!tryMergePOI(poi.x_m, poi.y_m, poi.confidence)) {
            if (m_confirmed_poi_count < MAX_CONFIRMED_POIS) {
                m_confirmed_pois[m_confirmed_poi_count++] = poi;
            }
        }
        
        // Reset for next candidate
        m_candidate.state = POI_STATE_NONE;
        m_candidate.confirmation_count = 0;
    }
}

bool POIDetector::tryMergePOI(float x_m, float y_m, float confidence) {
    for (uint8_t i = 0; i < m_confirmed_poi_count; i++) {
        if (!m_confirmed_pois[i].active) continue;
        
        float dx = x_m - m_confirmed_pois[i].x_m;
        float dy = y_m - m_confirmed_pois[i].y_m;
        float dist = sqrtf(dx * dx + dy * dy);
        
        if (dist < POIConfig::POI_MERGE_RADIUS_M) {
            // Merge with existing POI
            m_confirmed_pois[i].x_m = (m_confirmed_pois[i].x_m * m_confirmed_pois[i].confirmation_count + x_m)
                                     / (m_confirmed_pois[i].confirmation_count + 1);
            m_confirmed_pois[i].y_m = (m_confirmed_pois[i].y_m * m_confirmed_pois[i].confirmation_count + y_m)
                                     / (m_confirmed_pois[i].confirmation_count + 1);
            m_confirmed_pois[i].confidence = fmaxf(m_confirmed_pois[i].confidence, confidence);
            m_confirmed_pois[i].confirmation_count++;
            return true;
        }
    }
    
    return false;
}

//====================================
// PUBLIC API
//====================================

void POIDetector::update(const Localisation &localisation,
                         const MappingNav &mappingNav,
                         float tofDistance_m) {
    uint32_t now_ms = millis();
    m_last_update_ms = now_ms;
    
    m_diags.tof_reading_count++;
    m_diags.tof_last_range_m = tofDistance_m;
    
    // Invalid ToF reading (0 or out of range)
    if (tofDistance_m <= 0.0f || tofDistance_m > POIConfig::POI_MAX_RANGE_M) {
        m_diags.tof_last_valid = false;
        m_diags.invalid_tof_reading_count++;
        return;
    }
    
    m_diags.tof_last_valid = true;
    
    // Calculate ToF sensor world position
    float sensor_x_m, sensor_y_m, sensor_yaw_rad;
    calculateToFWorldPosition(localisation, sensor_x_m, sensor_y_m, sensor_yaw_rad);
    
    // Get expected LiDAR range along beam
    float expected_range_m;
    ExpectedRangeStatus expected_status;
    uint8_t point_count;
    calculateExpectedLiDARRange(mappingNav, sensor_x_m, sensor_y_m, sensor_yaw_rad,
                                expected_range_m, expected_status, point_count);
    
    m_diags.expected_range_m = expected_range_m;
    m_diags.expected_status = expected_status;
    
    // Process anomaly detection
    processAnomaly(localisation, tofDistance_m, expected_range_m, expected_status);
    
    // Update diagnostics
    m_diags.candidate_state = m_candidate.state;
    m_diags.candidate_confidence = m_candidate.confidence;
    m_diags.candidate_count = m_candidate.confirmation_count;
    m_diags.candidate_x_m = m_candidate.x_m;
    m_diags.candidate_y_m = m_candidate.y_m;
    m_diags.confirmed_poi_count = m_confirmed_poi_count;
}

POICandidate POIDetector::getCandidateState() const {
    return m_candidate;
}

uint16_t POIDetector::getConfirmedPOIs(ConfirmedPOI *poi_array, uint16_t max_pois) const {
    uint16_t count = 0;
    for (uint8_t i = 0; i < m_confirmed_poi_count && count < max_pois; i++) {
        if (m_confirmed_pois[i].active) {
            poi_array[count++] = m_confirmed_pois[i];
        }
    }
    return count;
}

POIDetectorDiags POIDetector::getDiags() const {
    return m_diags;
}

uint16_t POIDetector::getConfirmedPOICount() const {
    uint16_t count = 0;
    for (uint8_t i = 0; i < m_confirmed_poi_count; i++) {
        if (m_confirmed_pois[i].active) count++;
    }
    return count;
}
