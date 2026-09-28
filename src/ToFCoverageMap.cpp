//************************************
//       ToFCoverageMap.cpp
//************************************
// ToF coverage tracking implementation

#include "ToFCoverageMap.h"
#include <string.h>
#include <math.h>

ToFCoverageMap::ToFCoverageMap() {
    memset(m_coverage, 0, sizeof(m_coverage));
    m_covered_cell_count = 0;
}

void ToFCoverageMap::begin() {
    reset();
}

void ToFCoverageMap::reset() {
    memset(m_coverage, 0, sizeof(m_coverage));
    m_covered_cell_count = 0;
}

void ToFCoverageMap::markBeamCovered(float sensor_x_m, float sensor_y_m,
                                     float sensor_yaw_rad, float max_range_m) {
    // Raycast along beam and mark all cells as covered up to max_range
    const float step_m = CELL_SIZE_M;  // Step per cell roughly
    const float arena_w = MappingNav::ARENA_WIDTH_M;
    const float arena_h = MappingNav::ARENA_HEIGHT_M;
    
    float cos_yaw = cosf(sensor_yaw_rad);
    float sin_yaw = sinf(sensor_yaw_rad);
    
    for (float r = 0.0f; r <= max_range_m; r += step_m) {
        float x = sensor_x_m + r * cos_yaw;
        float y = sensor_y_m + r * sin_yaw;
        
        // Boundary check
        if (x < 0.0f || x >= arena_w || y < 0.0f || y >= arena_h) {
            break;
        }
        
        // Convert to cell index
        uint16_t cell_x = (uint16_t)(x / CELL_SIZE_M);
        uint16_t cell_y = (uint16_t)(y / CELL_SIZE_M);
        
        if (cell_x >= GRID_WIDTH || cell_y >= GRID_HEIGHT) continue;
        
        uint16_t idx = cell_y * GRID_WIDTH + cell_x;
        
        // Mark as covered (only increment count once)
        if (m_coverage[idx] == 0) {
            m_coverage[idx] = 1;
            m_covered_cell_count++;
        }
    }
}

bool ToFCoverageMap::isCellCovered(uint16_t cell_x, uint16_t cell_y) const {
    if (cell_x >= GRID_WIDTH || cell_y >= GRID_HEIGHT) return false;
    uint16_t idx = cell_y * GRID_WIDTH + cell_x;
    return m_coverage[idx] != 0;
}

bool ToFCoverageMap::isCellCovered(uint16_t cell_idx) const {
    if (cell_idx >= NUM_CELLS) return false;
    return m_coverage[cell_idx] != 0;
}

float ToFCoverageMap::getCoveragePercentage() const {
    return (100.0f * m_covered_cell_count) / (float)NUM_CELLS;
}

uint16_t ToFCoverageMap::getUncoveredCellCount() const {
    return NUM_CELLS - m_covered_cell_count;
}

const uint8_t* ToFCoverageMap::getCoverageData() const {
    return m_coverage;
}
