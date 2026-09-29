//************************************
//       ToFSearchPlanner.cpp
//************************************
// Search target generator implementation

#include "ToFSearchPlanner.h"
#include <math.h>
#include <string.h>
#include <algorithm>

#define PI_F 3.14159265358979f

ToFSearchPlanner::ToFSearchPlanner() {
}

void ToFSearchPlanner::begin() {
    // No initialization needed currently
}

uint16_t ToFSearchPlanner::generateCandidates(const MappingNav::Pose2D &current_pose,
                                              const ToFCoverageMap &coverage_map,
                                              const MappingNav &nav,
                                              SearchCandidate *candidates_out,
                                              uint16_t max_candidates) {
    uint16_t count = 0;
    
    // Generate candidate poses in a grid around the current position
    float current_x = current_pose.x_m;
    float current_y = current_pose.y_m;
    
    const float step = SearchPlannerConfig::CANDIDATE_POSITION_STEP_M;
    const float max_dist = SearchPlannerConfig::MAX_PLANNING_DISTANCE_M;
    const float angle_step = SearchPlannerConfig::CANDIDATE_ANGLE_STEP_RAD;
    
    // Grid search: positions around current location
    for (float dx = -max_dist; dx <= max_dist && count < max_candidates; dx += step) {
        for (float dy = -max_dist; dy <= max_dist && count < max_candidates; dy += step) {
            float x = current_x + dx;
            float y = current_y + dy;
            
            // Skip if outside arena
            if (x < 0.0f || x >= MappingNav::ARENA_WIDTH_M ||
                y < 0.0f || y >= MappingNav::ARENA_HEIGHT_M) {
                continue;
            }
            
            // Check if position is reachable (not occupied)
            if (!isPositionReachable(x, y, nav)) {
                continue;
            }
            
            // Generate multiple angles at this position
            for (float theta = 0.0f; theta < 2 * PI_F && count < max_candidates;
                 theta += angle_step) {
                
                SearchCandidate &cand = candidates_out[count];
                cand.x_m = x;
                cand.y_m = y;
                cand.theta_rad = theta;
                cand.distance_m = distance(current_x, current_y, x, y);
                cand.is_reachable = true;
                
                // Estimate new coverage from this candidate
                cand.new_coverage_percent = estimateNewCoverage(cand, coverage_map, nav);
                cand.is_useful = cand.new_coverage_percent >=
                                SearchPlannerConfig::MIN_NEW_COVERAGE_PERCENT;
                
                // Compute utility score
                if (cand.is_useful) {
                    // Utility = coverage bonus - distance penalty
                    float coverage_bonus = cand.new_coverage_percent *
                                          SearchPlannerConfig::COVERAGE_WEIGHT;
                    if (cand.new_coverage_percent > 10.0f) {
                        coverage_bonus += SearchPlannerConfig::EXPLORATION_BONUS;
                    }
                    float distance_penalty = cand.distance_m *
                                            SearchPlannerConfig::DISTANCE_WEIGHT;
                    cand.utility_score = fmaxf(0.0f, coverage_bonus - distance_penalty);
                } else {
                    cand.utility_score = 0.0f;
                }
                
                count++;
            }
        }
    }
    
    return count;
}

int16_t ToFSearchPlanner::selectBestCandidate(const SearchCandidate *candidates,
                                              uint16_t candidate_count) {
    int16_t best_idx = -1;
    float best_score = 0.0f;
    
    for (uint16_t i = 0; i < candidate_count; i++) {
        if (candidates[i].is_useful && candidates[i].utility_score > best_score) {
            best_score = candidates[i].utility_score;
            best_idx = i;
        }
    }
    
    return best_idx;
}

bool ToFSearchPlanner::isSearchComplete(const ToFCoverageMap &coverage_map,
                                        const MappingNav &nav) {
    // Search is complete if:
    // 1. Very high coverage, OR
    // 2. Remaining uncovered cells are all unreachable or blocked
    
    float coverage_pct = coverage_map.getCoveragePercentage();
    if (coverage_pct > 95.0f) {
        return true;  // Nearly everything explored
    }
    
    // Check if remaining uncovered cells are reachable
    const uint8_t* cov_data = coverage_map.getCoverageData();
    
    for (uint16_t i = 0; i < MappingNav::NUM_CELLS; i++) {
        if (cov_data[i] == 0) {  // Uncovered
            // Convert index to x, y
            uint16_t cell_x = i % MappingNav::GRID_WIDTH;
            uint16_t cell_y = i / MappingNav::GRID_WIDTH;
            
            float x_m = (cell_x + 0.5f) * MappingNav::CELL_SIZE_M;
            float y_m = (cell_y + 0.5f) * MappingNav::CELL_SIZE_M;
            
            // Is this cell reachable?
            if (isPositionReachable(x_m, y_m, nav)) {
                return false;  // Found reachable uncovered cell
            }
        }
    }
    
    return true;  // No reachable uncovered cells remain
}

float ToFSearchPlanner::estimateNewCoverage(const SearchCandidate &candidate,
                                           const ToFCoverageMap &coverage_map,
                                           const MappingNav &nav) {
    // Simple heuristic: estimate percentage of cells that would be covered
    // by a ToF beam from this position
    
    // Assume a 2.0m ToF range and estimate coverage ray
    const float tof_range = 2.0f;
    // Very narrow beam (unused in current implementation but documented for clarity)
    // const float fov_rad = 0.1f;  // ~5.7 degrees
    
    uint16_t new_cells = 0;
    uint16_t total_cells = 0;
    
    // Raycast along candidate's heading
    const float step = 0.05f;
    for (float r = 0.0f; r <= tof_range; r += step) {
        float x = candidate.x_m + r * cosf(candidate.theta_rad);
        float y = candidate.y_m + r * sinf(candidate.theta_rad);
        
        if (x < 0.0f || x >= MappingNav::ARENA_WIDTH_M ||
            y < 0.0f || y >= MappingNav::ARENA_HEIGHT_M) {
            break;
        }
        
        uint16_t cell_x = (uint16_t)(x / MappingNav::CELL_SIZE_M);
        uint16_t cell_y = (uint16_t)(y / MappingNav::CELL_SIZE_M);
        
        if (cell_x >= MappingNav::GRID_WIDTH || cell_y >= MappingNav::GRID_HEIGHT) {
            continue;
        }
        
        uint16_t idx = cell_y * MappingNav::GRID_WIDTH + cell_x;
        total_cells++;
        
        if (!coverage_map.isCellCovered(idx)) {
            new_cells++;
        }
    }
    
    if (total_cells == 0) return 0.0f;
    
    return (100.0f * new_cells) / (float)total_cells;
}

bool ToFSearchPlanner::isPositionReachable(float x_m, float y_m,
                                           const MappingNav &nav) const {
    // Position is reachable if the cell at (x, y) is not OCCUPIED
    
    uint16_t cell_x = (uint16_t)(x_m / MappingNav::CELL_SIZE_M);
    uint16_t cell_y = (uint16_t)(y_m / MappingNav::CELL_SIZE_M);
    
    if (cell_x >= MappingNav::GRID_WIDTH || cell_y >= MappingNav::GRID_HEIGHT) {
        return false;  // Outside arena
    }
    
    uint16_t idx = cell_y * MappingNav::GRID_WIDTH + cell_x;
    MappingNav::Occupancy occ = nav.getOccupancy(idx);
    
    // Position is reachable if it's free or unknown (but not occupied)
    return occ != MappingNav::OCC_OCCUPIED;
}

float ToFSearchPlanner::distance(float x1, float y1, float x2, float y2) const {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}
