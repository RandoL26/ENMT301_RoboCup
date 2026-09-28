//************************************
//         MappingNav.h
//************************************

#ifndef MAPPING_NAV_H_
#define MAPPING_NAV_H_

#include <stdint.h>

class MappingNav {
public:
  // Arena and grid constants
  static const uint16_t GRID_WIDTH = 48;   // 2.4 m / 0.05 m
  static const uint16_t GRID_HEIGHT = 98;  // 4.9 m / 0.05 m
  static const uint16_t NUM_CELLS = GRID_WIDTH * GRID_HEIGHT;
  static constexpr float CELL_SIZE_M = 0.05f;
  static constexpr float ARENA_WIDTH_M = 2.4f;
  static constexpr float ARENA_HEIGHT_M = 4.9f;

  // Bit-packed cell format (1 byte/cell)
  // bits 0..1: occupancy, bits 2..3: terrain
  enum Occupancy : uint8_t {
    OCC_UNKNOWN = 0,
    OCC_FREE = 1,
    OCC_OCCUPIED = 2
  };

  enum Terrain : uint8_t {
    TERRAIN_FLAT = 0,
    TERRAIN_RAMP = 1,
    TERRAIN_SPEED_BUMP = 2
  };

  struct Pose2D {
    float x_m;
    float y_m;
    float theta_rad;
  };

  struct CellCoord {
    uint16_t x;
    uint16_t y;
  };

  // Per-cycle pose fusion input.
  // Assumption: encoder/flow deltas are in robot body frame for this cycle.
  struct PoseUpdateInput {
    float encoder_dx_m;
    float encoder_dy_m;
    float encoder_dtheta_rad;

    float flow_dx_m;
    float flow_dy_m;

    float imu_gyro_dtheta_rad;
  };

  enum SensorKind : uint8_t {
    SENSOR_TOF = 0,
    SENSOR_ULTRASONIC = 1
  };

  // Generic ray-like range reading.
  // Assumptions:
  // - angle_offset_rad is relative to robot heading (+CCW).
  // - distance_m is along the beam.
  // - has_hit=true means a definite obstacle return at distance_m.
  // - has_hit=false means no return up to max_range_m (free-space only update).
  struct SensorRay {
    bool valid;
    bool has_hit;
    SensorKind kind;

    float angle_offset_rad;
    float distance_m;
    float max_range_m;
  };

  MappingNav();

  void reset();

  void setPose(float x_m, float y_m, float theta_rad);
  Pose2D getPose() const;

  // Complementary fusion of encoder + optical flow + IMU gyro heading delta.
  void updatePose(const PoseUpdateInput &input,
                  float translation_encoder_weight = 0.5f,
                  float heading_imu_weight = 0.8f);

  // Occupancy update by ray-casting each reading.
  void updateGridFromSensors(const SensorRay *rays, uint16_t ray_count);

  // Uses known outer walls to reduce drift when a reading agrees with boundary geometry.
  // correction_gain in [0,1], match_tolerance_m is max |expected-measured| for correction.
  void applyBoundaryCorrection(const SensorRay *rays,
                               uint16_t ray_count,
                               float correction_gain = 0.3f,
                               float match_tolerance_m = 0.12f);

  // Terrain flags are separate from occupancy and remain traversable.
  bool setTerrainAtCell(uint16_t cell_x, uint16_t cell_y, Terrain terrain);

  // Goal selection for planner.
  bool setGoalCell(uint16_t goal_x, uint16_t goal_y);
  bool setGoalWorld(float goal_x_m, float goal_y_m);

  // Inflate occupied cells by robot radius, update D* Lite incrementally, and solve.
  // Returns true if a path to goal exists.
  bool replanPath(float robot_radius_m);

  // Path query helpers.
  bool getNextWaypoint(float &x_m, float &y_m) const;
  uint16_t getPathCells(CellCoord *out_cells, uint16_t max_cells) const;

  // Raw map access (1 byte per cell)
  const uint8_t *getGridData() const;

  // Inflated obstacle map.
  // 1 = blocked after robot-radius inflation
  // 0 = traversable
  const uint8_t *getInflatedGridData() const;

  // Planner status
  bool isGoalSet() const;
  uint16_t getGoalCellIndex() const;
  uint16_t getPathLength() const;
  
  // Cell occupancy query (needed for POI detection and search planning)
  Occupancy getOccupancy(uint16_t idx) const;

private:
  // Cell packing helpers
  static const uint8_t OCC_MASK = 0x03;
  static const uint8_t TERRAIN_MASK = 0x0C;

  // D* Lite constants
  static constexpr float INF = 1.0e9f;
  static constexpr float EPS = 1.0e-4f;

  struct Key {
    float k1;
    float k2;
  };

  // Map storage
  uint8_t m_grid[NUM_CELLS];            // occupancy + terrain (1 byte/cell)
  uint8_t m_inflated_blocked[NUM_CELLS]; // 1 if occupied in config-space

  // Pose
  Pose2D m_pose;

  // D* Lite state
  float m_g[NUM_CELLS];
  float m_rhs[NUM_CELLS];

  // Fixed-size open set (binary heap with at most one entry per node)
  uint16_t m_open_heap[NUM_CELLS];
  uint16_t m_open_pos[NUM_CELLS]; // 0xFFFF if not in heap
  uint16_t m_open_size;

  bool m_goal_set;
  uint16_t m_start_idx;
  uint16_t m_last_start_idx;
  uint16_t m_goal_idx;
  float m_km;

  // Cached extracted path in cells
  CellCoord m_path[NUM_CELLS];
  uint16_t m_path_len;

  // Coordinate helpers
  static float wrapAngle(float a);
  static float absf(float v);
  static float minf(float a, float b);
  static float maxf(float a, float b);

  bool worldToCell(float x_m, float y_m, uint16_t &cx, uint16_t &cy) const;
  void cellToWorldCenter(uint16_t cx, uint16_t cy, float &x_m, float &y_m) const;
  static uint16_t indexOf(uint16_t cx, uint16_t cy);
  static void coordOf(uint16_t idx, uint16_t &cx, uint16_t &cy);
  bool inBoundsCell(int32_t cx, int32_t cy) const;
  bool inBoundsWorld(float x_m, float y_m) const;

  // Cell read/write helpers (occupancy moved to public section above)
  void setOccupancy(uint16_t idx, Occupancy occ);
  Terrain getTerrain(uint16_t idx) const;
  void setTerrain(uint16_t idx, Terrain terrain);

  // Ray update helpers
  void markRay(float start_x, float start_y,
               float end_x, float end_y,
               bool mark_endpoint_occupied);
  void maybeMarkFree(uint16_t idx);

  // Boundary correction helpers
  bool expectedBoundaryDistance(float ray_angle_global, float &out_dist_m) const;

  // Inflation
  void recomputeInflation(float robot_radius_m);

  // D* Lite helpers
  void plannerInitializeIfNeeded();
  void plannerResetAll();
  void updateStartFromPose();

  Key calculateKey(uint16_t s) const;
  float heuristic(uint16_t a, uint16_t b) const;
  bool keyLess(const Key &a, const Key &b) const;
  bool keyEqual(const Key &a, const Key &b) const;

  void openClear();
  bool openContains(uint16_t s) const;
  void openInsertOrUpdate(uint16_t s);
  void openRemove(uint16_t s);
  uint16_t openTop() const;
  Key openTopKey() const;
  uint16_t openPop();
  void openHeapifyUp(uint16_t pos);
  void openHeapifyDown(uint16_t pos);
  void rebuildOpenHeap();

  void forEachNeighbor(uint16_t s, uint16_t *neighbors, uint8_t &count) const;
  float edgeCost(uint16_t from, uint16_t to) const;
  void updateVertex(uint16_t u);
  bool computeShortestPath(uint32_t max_iterations);
  bool extractPath();
};

#endif // MAPPING_NAV_H_
