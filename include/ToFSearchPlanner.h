//************************************
//       ToFSearchPlanner.h
//************************************
// Generate search target poses to maximize ToF coverage

#ifndef TOF_SEARCH_PLANNER_H
#define TOF_SEARCH_PLANNER_H

#include <stdint.h>
#include "MappingNav.h"
#include "ToFCoverageMap.h"

//====================================
// SEARCH PLANNER CONFIGURATION
//====================================

namespace SearchPlannerConfig {
    // Candidate pose generation: step size and angle resolution
    static constexpr float CANDIDATE_POSITION_STEP_M = 0.2f;  // 20cm spacing
    static constexpr float CANDIDATE_ANGLE_STEP_RAD = 0.785f; // ~45 degrees
    
    // Utility weighting
    static constexpr float COVERAGE_WEIGHT = 1.0f;      // Importance of new coverage
    static constexpr float DISTANCE_WEIGHT = 0.5f;      // Distance penalty
    static constexpr float EXPLORATION_BONUS = 100.0f;  // Bonus for very new areas
    
    // Minimum new coverage needed to consider it "useful"
    static constexpr float MIN_NEW_COVERAGE_PERCENT = 2.0f;
    
    // Maximum planning distance
    static constexpr float MAX_PLANNING_DISTANCE_M = 2.0f;
};

//====================================
// SEARCH TARGET CANDIDATE
//====================================

struct SearchCandidate {
    // Target robot pose
    float x_m = 0.0f;
    float y_m = 0.0f;
    float theta_rad = 0.0f;
    
    // Utility metrics
    float new_coverage_percent = 0.0f;  // Estimated new ToF coverage %
    float distance_m = 0.0f;             // Distance from current pose
    float utility_score = 0.0f;           // Combined score [0, high]
    
    // Quality indicators
    bool is_reachable = false;           // Not blocked by obstacles
    bool is_useful = false;               // Meets minimum coverage threshold
};

//====================================
// SEARCH PLANNER CLASS
//====================================

class ToFSearchPlanner {
public:
    // Constructor
    ToFSearchPlanner();
    
    // Initialize
    void begin();
    
    // Generate search targets based on current state
    // current_pose: robot's current position and heading
    // coverage_map: ToF coverage map
    // nav: mapping and occupancy grid
    // max_candidates: limit output array size
    // Returns: number of valid candidates populated into output array
    uint16_t generateCandidates(const MappingNav::Pose2D &current_pose,
                                const ToFCoverageMap &coverage_map,
                                const MappingNav &nav,
                                SearchCandidate *candidates_out,
                                uint16_t max_candidates);
    
    // Select best candidate from list based on utility
    // Returns: index of best candidate, or -1 if none are useful
    int16_t selectBestCandidate(const SearchCandidate *candidates,
                                uint16_t candidate_count);
    
    // Query: is search complete (no reachable uncovered cells)?
    bool isSearchComplete(const ToFCoverageMap &coverage_map,
                         const MappingNav &nav);
    
    // Estimate new coverage from a candidate pose
    // Uses a simple heuristic based on distance to uncovered cells
    float estimateNewCoverage(const SearchCandidate &candidate,
                             const ToFCoverageMap &coverage_map,
                             const MappingNav &nav);

private:
    // Helper to check if a position is reachable (not in obstacle)
    bool isPositionReachable(float x_m, float y_m, const MappingNav &nav) const;
    
    // Helper to calculate Euclidean distance
    float distance(float x1, float y1, float x2, float y2) const;
};

#endif // TOF_SEARCH_PLANNER_H
