//************************************
//       ToFCoverageMap.h
//************************************
// Track which areas have been examined by ToF sensor beam
// Distinct from LiDAR occupancy map

#ifndef TOF_COVERAGE_MAP_H
#define TOF_COVERAGE_MAP_H

#include <stdint.h>
#include "MappingNav.h"

//====================================
// TOF COVERAGE MAP
//====================================

class ToFCoverageMap {
public:
    // Constructor
    ToFCoverageMap();
    
    // Initialize (call from robot_init)
    void begin();
    
    // Record a ToF beam as "examined"
    // sensor_x_m, sensor_y_m: ToF sensor position in world frame
    // sensor_yaw_rad: ToF beam direction (radians)
    // max_range_m: maximum valid range of this measurement
    void markBeamCovered(float sensor_x_m, float sensor_y_m,
                         float sensor_yaw_rad, float max_range_m);
    
    // Query: has this cell been examined by ToF?
    bool isCellCovered(uint16_t cell_x, uint16_t cell_y) const;
    bool isCellCovered(uint16_t cell_idx) const;
    
    // Get overall coverage percentage [0, 100]
    float getCoveragePercentage() const;
    
    // Count uncovered cells
    uint16_t getUncoveredCellCount() const;
    
    // Reset coverage map
    void reset();
    
    // Get coverage map data (for diagnostics/visualization)
    const uint8_t* getCoverageData() const;

private:
    // Same grid dimensions as MappingNav
    static constexpr uint16_t GRID_WIDTH = MappingNav::GRID_WIDTH;   // 48
    static constexpr uint16_t GRID_HEIGHT = MappingNav::GRID_HEIGHT;  // 98
    static constexpr uint16_t NUM_CELLS = MappingNav::NUM_CELLS;      // 48*98
    static constexpr float CELL_SIZE_M = MappingNav::CELL_SIZE_M;     // 0.05
    
    // Coverage map: 1 byte per cell, 0 = uncovered, 1 = covered by ToF beam
    uint8_t m_coverage[NUM_CELLS];
    
    // Statistics
    uint16_t m_covered_cell_count = 0;
};

#endif // TOF_COVERAGE_MAP_H
