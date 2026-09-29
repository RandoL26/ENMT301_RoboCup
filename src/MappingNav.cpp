//************************************
//         MappingNav.cpp
//************************************

#include "MappingNav.h"

#include <math.h>
#include <string.h>

// ============================================================
// Robot footprint
// ============================================================

// Overall robot dimensions.
// Change these values if the robot dimensions change.
const float ROBOT_LENGTH_M = 0.370f;
const float ROBOT_WIDTH_M  = 0.210f;

// Additional safety clearance around the robot.
const float ROBOT_SAFETY_MARGIN_M = 0.030f;

MappingNav::MappingNav() {
  reset();
}

void MappingNav::reset() {
  for (uint16_t i = 0; i < NUM_CELLS; ++i) {
    m_grid[i] = (uint8_t)OCC_UNKNOWN;
    m_inflated_blocked[i] = false;
    m_occupancy_score[i] = 0;
  }

  m_pose.x_m = ARENA_WIDTH_M * 0.5f;
  m_pose.y_m = ARENA_HEIGHT_M * 0.5f;
  m_pose.theta_rad = 0.0f;

  plannerResetAll();
}

void MappingNav::setPose(float x_m, float y_m, float theta_rad) {
  m_pose.x_m = minf(maxf(x_m, 0.0f), ARENA_WIDTH_M);
  m_pose.y_m = minf(maxf(y_m, 0.0f), ARENA_HEIGHT_M);
  m_pose.theta_rad = wrapAngle(theta_rad);
}

MappingNav::Pose2D MappingNav::getPose() const {
  return m_pose;
}

void MappingNav::updatePose(const PoseUpdateInput &input,
                            float translation_encoder_weight,
                            float heading_imu_weight) {
  if (translation_encoder_weight < 0.0f) translation_encoder_weight = 0.0f;
  if (translation_encoder_weight > 1.0f) translation_encoder_weight = 1.0f;
  if (heading_imu_weight < 0.0f) heading_imu_weight = 0.0f;
  if (heading_imu_weight > 1.0f) heading_imu_weight = 1.0f;

  const float w_enc = translation_encoder_weight;
  const float w_flow = 1.0f - w_enc;

  const float dx_body = w_enc * input.encoder_dx_m + w_flow * input.flow_dx_m;
  const float dy_body = w_enc * input.encoder_dy_m + w_flow * input.flow_dy_m;

  const float dtheta = (1.0f - heading_imu_weight) * input.encoder_dtheta_rad +
                       heading_imu_weight * input.imu_gyro_dtheta_rad;

  const float theta_mid = m_pose.theta_rad + 0.5f * dtheta;
  const float c = cosf(theta_mid);
  const float s = sinf(theta_mid);

  m_pose.x_m += dx_body * c - dy_body * s;
  m_pose.y_m += dx_body * s + dy_body * c;
  m_pose.theta_rad = wrapAngle(m_pose.theta_rad + dtheta);

  // Keep pose inside arena bounds.
  m_pose.x_m = minf(maxf(m_pose.x_m, 0.0f), ARENA_WIDTH_M);
  m_pose.y_m = minf(maxf(m_pose.y_m, 0.0f), ARENA_HEIGHT_M);
}

void MappingNav::updateGridFromSensors(const SensorRay *rays, uint16_t ray_count) {
  if (!rays) return;

  for (uint16_t i = 0; i < ray_count; ++i) {
    const SensorRay &r = rays[i];
    if (!r.valid) continue;

    const float global_ang = m_pose.theta_rad + r.angle_offset_rad;
    float ray_len = r.has_hit ? r.distance_m : r.max_range_m;
    if (ray_len < 0.0f) ray_len = 0.0f;

    float ex = m_pose.x_m + ray_len * cosf(global_ang);
    float ey = m_pose.y_m + ray_len * sinf(global_ang);

    // Clamp endpoint to arena for robust indexing.
    ex = minf(maxf(ex, 0.0f), ARENA_WIDTH_M);
    ey = minf(maxf(ey, 0.0f), ARENA_HEIGHT_M);

    markRay(m_pose.x_m, m_pose.y_m, ex, ey, r.has_hit);
  }
}

void MappingNav::applyBoundaryCorrection(
    const SensorRay *rays,
    uint16_t ray_count,
    float correction_gain,
    float match_tolerance_m,
    float lidar_offset_x_m,
    float lidar_offset_y_m)
{
    if (!rays || ray_count == 0) {
        return;
    }

    const float c = cosf(m_pose.theta_rad);
    const float s = sinf(m_pose.theta_rad);

    // Calculate the LiDAR position in world coordinates.
    const float lidar_x =
        m_pose.x_m +
        lidar_offset_x_m * c -
        lidar_offset_y_m * s;

    const float lidar_y =
        m_pose.y_m +
        lidar_offset_x_m * s +
        lidar_offset_y_m * c;

    float correction_x = 0.0f;
    float correction_y = 0.0f;
    uint16_t matches = 0;

    for (uint16_t i = 0; i < ray_count; ++i) {
        const SensorRay &ray = rays[i];

        if (!ray.valid || !ray.has_hit) {
            continue;
        }

        const float global_ang =
            m_pose.theta_rad + ray.angle_offset_rad;

        float expected_d = 0.0f;

        if (!expectedBoundaryDistance(
                lidar_x,
                lidar_y,
                global_ang,
                expected_d)) {
            continue;
        }

        const float measured_d = ray.distance_m;
        const float error = expected_d - measured_d;

        if (fabsf(error) > match_tolerance_m) {
            continue;
        }

        // Convert the radial error into a world-frame correction.
        correction_x += error * cosf(global_ang);
        correction_y += error * sinf(global_ang);

        ++matches;
    }

    if (matches == 0) {
        return;
    }

    correction_x /= (float)matches;
    correction_y /= (float)matches;

    m_pose.x_m += correction_gain * correction_x;
    m_pose.y_m += correction_gain * correction_y;

    // Keep pose inside the arena.
    setPose(m_pose.x_m, m_pose.y_m, m_pose.theta_rad);
}

bool MappingNav::setTerrainAtCell(uint16_t cell_x, uint16_t cell_y, Terrain terrain) {
  if (cell_x >= GRID_WIDTH || cell_y >= GRID_HEIGHT) return false;
  const uint16_t idx = indexOf(cell_x, cell_y);
  setTerrain(idx, terrain);
  return true;
}

bool MappingNav::setGoalCell(uint16_t goal_x, uint16_t goal_y) {
  if (goal_x >= GRID_WIDTH || goal_y >= GRID_HEIGHT) {
    return false;
  }

  // Reset planner state first.
  plannerResetAll();

  // Now set the new goal.
  m_goal_idx = indexOf(goal_x, goal_y);
  m_goal_set = true;

  // Initialise D* Lite around the new goal.
  plannerInitializeIfNeeded();

  return true;
}

bool MappingNav::setGoalWorld(float goal_x_m, float goal_y_m) {
  uint16_t gx = 0;
  uint16_t gy = 0;
  if (!worldToCell(goal_x_m, goal_y_m, gx, gy)) return false;
  return setGoalCell(gx, gy);
}

bool MappingNav::replanPath(float robot_radius_m) {
  if (!m_goal_set) return false;

  // Update start cell from current pose (D* Lite incremental behavior).
  updateStartFromPose();

  // Rebuild configuration-space occupancy.
  uint8_t previous[NUM_CELLS];
  memcpy(previous, m_inflated_blocked, sizeof(previous));
  recomputeInflation(robot_radius_m);

  // Incremental updates: for any changed occupancy, update that node and neighbors.
  for (uint16_t idx = 0; idx < NUM_CELLS; ++idx) {
    if (previous[idx] == m_inflated_blocked[idx]) continue;
    updateVertex(idx);

    uint16_t nb[8];
    uint8_t nb_count = 0;
    forEachNeighbor(idx, nb, nb_count);
    for (uint8_t n = 0; n < nb_count; ++n) {
      updateVertex(nb[n]);
    }
  }

  if (!computeShortestPath(NUM_CELLS * 20UL)) {
    m_path_len = 0;
    return false;
  }

  return extractPath();
}

bool MappingNav::getNextWaypoint(float &x_m, float &y_m) const {
  if (m_path_len < 2) return false;

  // m_path[0] is current/start cell. Return the next cell center.
  cellToWorldCenter(m_path[1].x, m_path[1].y, x_m, y_m);
  return true;
}

uint16_t MappingNav::getPathCells(CellCoord *out_cells, uint16_t max_cells) const {
  if (!out_cells || max_cells == 0) return 0;
  const uint16_t n = (m_path_len < max_cells) ? m_path_len : max_cells;
  for (uint16_t i = 0; i < n; ++i) out_cells[i] = m_path[i];
  return n;
}

const uint8_t *MappingNav::getGridData() const {
  return m_grid;
}

const uint8_t *MappingNav::getInflatedGridData() const {
    return m_inflated_blocked;
}

bool MappingNav::isGoalSet() const {
    return m_goal_set;
}

uint16_t MappingNav::getGoalCellIndex() const {
    return m_goal_idx;
}

uint16_t MappingNav::getPathLength() const {
    return m_path_len;
}

float MappingNav::wrapAngle(float a) {
  while (a > 3.14159265359f) a -= 6.28318530718f;
  while (a < -3.14159265359f) a += 6.28318530718f;
  return a;
}

float MappingNav::absf(float v) { return (v < 0.0f) ? -v : v; }
float MappingNav::minf(float a, float b) { return (a < b) ? a : b; }
float MappingNav::maxf(float a, float b) { return (a > b) ? a : b; }

bool MappingNav::worldToCell(float x_m, float y_m, uint16_t &cx, uint16_t &cy) const {
  if (!inBoundsWorld(x_m, y_m)) return false;

  int32_t ix = (int32_t)(x_m / CELL_SIZE_M);
  int32_t iy = (int32_t)(y_m / CELL_SIZE_M);

  if (ix < 0) ix = 0;
  if (iy < 0) iy = 0;
  if (ix >= (int32_t)GRID_WIDTH) ix = (int32_t)GRID_WIDTH - 1;
  if (iy >= (int32_t)GRID_HEIGHT) iy = (int32_t)GRID_HEIGHT - 1;

  cx = (uint16_t)ix;
  cy = (uint16_t)iy;
  return true;
}

void MappingNav::cellToWorldCenter(uint16_t cx, uint16_t cy, float &x_m, float &y_m) const {
  x_m = ((float)cx + 0.5f) * CELL_SIZE_M;
  y_m = ((float)cy + 0.5f) * CELL_SIZE_M;
}

uint16_t MappingNav::indexOf(uint16_t cx, uint16_t cy) {
  return (uint16_t)(cy * GRID_WIDTH + cx);
}

void MappingNav::coordOf(uint16_t idx, uint16_t &cx, uint16_t &cy) {
  cy = (uint16_t)(idx / GRID_WIDTH);
  cx = (uint16_t)(idx - (uint16_t)(cy * GRID_WIDTH));
}

bool MappingNav::inBoundsCell(int32_t cx, int32_t cy) const {
  return (cx >= 0 && cy >= 0 && cx < (int32_t)GRID_WIDTH && cy < (int32_t)GRID_HEIGHT);
}

bool MappingNav::inBoundsWorld(float x_m, float y_m) const {
  return (x_m >= 0.0f && y_m >= 0.0f && x_m <= ARENA_WIDTH_M && y_m <= ARENA_HEIGHT_M);
}

MappingNav::Occupancy MappingNav::getOccupancy(uint16_t idx) const {
  return (Occupancy)(m_grid[idx] & OCC_MASK);
}

void MappingNav::setOccupancy(uint16_t idx, Occupancy occ) {
  m_grid[idx] = (uint8_t)((m_grid[idx] & (uint8_t)(~OCC_MASK)) | ((uint8_t)occ & OCC_MASK));
}

MappingNav::Terrain MappingNav::getTerrain(uint16_t idx) const {
  return (Terrain)((m_grid[idx] & TERRAIN_MASK) >> 2);
}

void MappingNav::setTerrain(uint16_t idx, Terrain terrain) {
  m_grid[idx] = (uint8_t)((m_grid[idx] & (uint8_t)(~TERRAIN_MASK)) |
                          (((uint8_t)terrain & 0x03u) << 2));
}

void MappingNav::markRay(float start_x, float start_y,
                         float end_x, float end_y,
                         bool mark_endpoint_occupied) {
  uint16_t sx = 0, sy = 0, ex = 0, ey = 0;

  // The LiDAR origin must be inside the map.
  if (!worldToCell(start_x, start_y, sx, sy)) return;

  // Map boundaries in metres.
  const float map_max_x =
      (float)GRID_WIDTH * CELL_SIZE_M;
  const float map_max_y =
      (float)GRID_HEIGHT * CELL_SIZE_M;

  float clipped_x = end_x;
  float clipped_y = end_y;

  // Check whether the endpoint is outside the map.
  if (!inBoundsWorld(end_x, end_y)) {
    const float dx = end_x - start_x;
    const float dy = end_y - start_y;

    float t = 1.0f;

    // Vertical map boundaries.
    if (dx > 0.0f && end_x > map_max_x) {
      const float tx = (map_max_x - start_x) / dx;
      if (tx >= 0.0f && tx < t) t = tx;
    } else if (dx < 0.0f && end_x < 0.0f) {
      const float tx = (0.0f - start_x) / dx;
      if (tx >= 0.0f && tx < t) t = tx;
    }

    // Horizontal map boundaries.
    if (dy > 0.0f && end_y > map_max_y) {
      const float ty = (map_max_y - start_y) / dy;
      if (ty >= 0.0f && ty < t) t = ty;
    } else if (dy < 0.0f && end_y < 0.0f) {
      const float ty = (0.0f - start_y) / dy;
      if (ty >= 0.0f && ty < t) t = ty;
    }

    clipped_x = start_x + t * dx;
    clipped_y = start_y + t * dy;

    // Keep the clipped point safely inside the map.
    const float eps = 0.001f;

    if (clipped_x >= map_max_x)
      clipped_x = map_max_x - eps;

    if (clipped_y >= map_max_y)
      clipped_y = map_max_y - eps;

    if (clipped_x < 0.0f)
      clipped_x = 0.0f;

    if (clipped_y < 0.0f)
      clipped_y = 0.0f;
  }

  if (!worldToCell(clipped_x, clipped_y, ex, ey)) return;

  int32_t x0 = (int32_t)sx;
  int32_t y0 = (int32_t)sy;

  const int32_t x1 = (int32_t)ex;
  const int32_t y1 = (int32_t)ey;

  const int32_t dx = abs(x1 - x0);
  const int32_t sx_step = (x0 < x1) ? 1 : -1;

  const int32_t dy = -abs(y1 - y0);
  const int32_t sy_step = (y0 < y1) ? 1 : -1;

  int32_t err = dx + dy;

  const uint16_t origin_idx = indexOf(sx, sy);

  while (true) {
    if (!inBoundsCell(x0, y0)) {
      break;
    }

    const uint16_t idx =
        indexOf((uint16_t)x0, (uint16_t)y0);

    const bool endpoint =
        (x0 == x1 && y0 == y1);

    // Never modify the LiDAR's own cell.
    if (idx == origin_idx) {
      // Nothing to do.
    }
    else if (endpoint) {
      if (mark_endpoint_occupied) {
        // Real LiDAR return: obstacle detected.
        // Close returns receive stronger confidence.
        const float ray_dx = end_x - start_x;
        const float ray_dy = end_y - start_y;
        const float range =
            sqrtf(ray_dx * ray_dx + ray_dy * ray_dy);

        if (range <= LIDAR_CLOSE_RANGE_M) {
          updateOccupancyEvidence(
              idx,
              LIDAR_CLOSE_HIT_SCORE);
        }
        else {
          updateOccupancyEvidence(
              idx,
              LIDAR_FAR_HIT_SCORE);
        }
      }
      else {
        // Ray reached the map boundary with no obstacle.
        maybeMarkFree(idx);
      }

      break;
    }
    else {
      // LiDAR passed through this cell without hitting anything.
      maybeMarkFree(idx);
    }

    const int32_t e2 = 2 * err;

    if (e2 >= dy) {
      err += dy;
      x0 += sx_step;
    }

    if (e2 <= dx) {
      err += dx;
      y0 += sy_step;
    }
  }
}

void MappingNav::maybeMarkFree(uint16_t idx) {
  // A LiDAR ray passing through this cell is evidence
  // that the cell is free.
  updateOccupancyEvidence(idx, -1);
}

void MappingNav::updateOccupancyEvidence(uint16_t idx, int8_t delta) {
  int16_t score = (int16_t)m_occupancy_score[idx] + delta;

  // Saturate the confidence score.
  if (score > 5) score = 5;
  if (score < -5) score = -5;

  m_occupancy_score[idx] = (int8_t)score;

  // Convert confidence into occupancy state.
  if (score >= 2) {
    setOccupancy(idx, OCC_OCCUPIED);
  } else if (score <= -2) {
    setOccupancy(idx, OCC_FREE);
  }
}

bool MappingNav::expectedBoundaryDistance(
    float ray_origin_x,
    float ray_origin_y,
    float ray_angle_global,
    float &out_dist_m) const {

  // Ray:
  //
  // p + t*d
  //
  // where p is the LiDAR position and d is the
  // LiDAR beam direction.

  const float px = ray_origin_x;
  const float py = ray_origin_y;

  const float dx = cosf(ray_angle_global);
  const float dy = sinf(ray_angle_global);

  float best = INF;

  // ----------------------------------------------------------
  // x = 0
  // ----------------------------------------------------------

  if (absf(dx) > 1.0e-6f) {

    float t =
        (0.0f - px) / dx;

    if (t >= 0.0f) {

      float y =
          py + t * dy;

      if (y >= 0.0f &&
          y <= ARENA_HEIGHT_M) {

        best = minf(best, t);
      }
    }

    // x = maximum

    t =
        (ARENA_WIDTH_M - px) / dx;

    if (t >= 0.0f) {

      float y =
          py + t * dy;

      if (y >= 0.0f &&
          y <= ARENA_HEIGHT_M) {

        best = minf(best, t);
      }
    }
  }

  // ----------------------------------------------------------
  // y = 0
  // ----------------------------------------------------------

  if (absf(dy) > 1.0e-6f) {

    float t =
        (0.0f - py) / dy;

    if (t >= 0.0f) {

      float x =
          px + t * dx;

      if (x >= 0.0f &&
          x <= ARENA_WIDTH_M) {

        best = minf(best, t);
      }
    }

    // y = maximum

    t =
        (ARENA_HEIGHT_M - py) / dy;

    if (t >= 0.0f) {

      float x =
          px + t * dx;

      if (x >= 0.0f &&
          x <= ARENA_WIDTH_M) {

        best = minf(best, t);
      }
    }
  }

  if (best >= INF * 0.5f)
    return false;

  out_dist_m = best;

  return true;
}

void MappingNav::recomputeInflation(float robot_radius_m) {
  if (robot_radius_m < 0.0f) robot_radius_m = 0.0f;

  for (uint16_t i = 0; i < NUM_CELLS; ++i) {
    m_inflated_blocked[i] = 0;
  }

  const int32_t r_cells = (int32_t)ceilf(robot_radius_m / CELL_SIZE_M);
  const int32_t r2 = r_cells * r_cells;

  for (uint16_t y = 0; y < GRID_HEIGHT; ++y) {
    for (uint16_t x = 0; x < GRID_WIDTH; ++x) {
      const uint16_t idx = indexOf(x, y);
      if (getOccupancy(idx) != OCC_OCCUPIED) continue;

      for (int32_t dy = -r_cells; dy <= r_cells; ++dy) {
        for (int32_t dx = -r_cells; dx <= r_cells; ++dx) {
          if ((dx * dx + dy * dy) > r2) continue;

          const int32_t nx = (int32_t)x + dx;
          const int32_t ny = (int32_t)y + dy;
          if (!inBoundsCell(nx, ny)) continue;

          m_inflated_blocked[indexOf((uint16_t)nx, (uint16_t)ny)] = 1;
        }
      }
    }
  }
}

void MappingNav::plannerInitializeIfNeeded() {
  if (!m_goal_set) return;

  // Initialize only if goal has not been set in arrays yet.
  if (m_rhs[m_goal_idx] > 0.0f + EPS || m_g[m_goal_idx] < INF * 0.5f) {
    for (uint16_t i = 0; i < NUM_CELLS; ++i) {
      m_g[i] = INF;
      m_rhs[i] = INF;
    }

    m_rhs[m_goal_idx] = 0.0f;
    openClear();
    openInsertOrUpdate(m_goal_idx);
  }
}

void MappingNav::plannerResetAll() {
  for (uint16_t i = 0; i < NUM_CELLS; ++i) {
    m_g[i] = INF;
    m_rhs[i] = INF;
    m_open_pos[i] = 0xFFFFu;
  }

  m_open_size = 0;
  m_goal_set = false;
  m_start_idx = 0;
  m_last_start_idx = 0;
  m_goal_idx = 0;
  m_km = 0.0f;
  m_path_len = 0;
}

void MappingNav::updateStartFromPose() {
  uint16_t sx = 0, sy = 0;
  if (!worldToCell(m_pose.x_m, m_pose.y_m, sx, sy)) {
    sx = 0;
    sy = 0;
  }

  const uint16_t new_start = indexOf(sx, sy);

  if (new_start != m_start_idx) {
    m_last_start_idx = m_start_idx;
    m_start_idx = new_start;
    m_km += heuristic(m_last_start_idx, m_start_idx);
    // Start changed: keys depending on h(start, s) changed.
    rebuildOpenHeap();
  }

  plannerInitializeIfNeeded();
}

MappingNav::Key MappingNav::calculateKey(uint16_t s) const {
  const float v = minf(m_g[s], m_rhs[s]);
  Key k;
  k.k1 = v + heuristic(m_start_idx, s) + m_km;
  k.k2 = v;
  return k;
}

float MappingNav::heuristic(uint16_t a, uint16_t b) const {
  uint16_t ax = 0, ay = 0, bx = 0, by = 0;
  coordOf(a, ax, ay);
  coordOf(b, bx, by);

  const float dx = absf((float)ax - (float)bx);
  const float dy = absf((float)ay - (float)by);
  const float dmin = minf(dx, dy);
  const float dmax = maxf(dx, dy);
  // Octile metric (consistent with 8-connected grid)
  return 1.41421356f * dmin + (dmax - dmin);
}

bool MappingNav::keyLess(const Key &a, const Key &b) const {
  if (a.k1 < b.k1 - EPS) return true;
  if (a.k1 > b.k1 + EPS) return false;
  return (a.k2 < b.k2 - EPS);
}

bool MappingNav::keyEqual(const Key &a, const Key &b) const {
  return (absf(a.k1 - b.k1) <= EPS && absf(a.k2 - b.k2) <= EPS);
}

void MappingNav::openClear() {
  m_open_size = 0;
  for (uint16_t i = 0; i < NUM_CELLS; ++i) {
    m_open_pos[i] = 0xFFFFu;
  }
}

bool MappingNav::openContains(uint16_t s) const {
  return m_open_pos[s] != 0xFFFFu;
}

void MappingNav::openInsertOrUpdate(uint16_t s) {
  if (!openContains(s)) {
    if (m_open_size >= NUM_CELLS) return;
    const uint16_t pos = m_open_size;
    m_open_heap[pos] = s;
    m_open_pos[s] = pos;
    ++m_open_size;
    openHeapifyUp(pos);
  } else {
    const uint16_t pos = m_open_pos[s];
    openHeapifyUp(pos);
    openHeapifyDown(pos);
  }
}

void MappingNav::openRemove(uint16_t s) {
  if (!openContains(s)) return;

  const uint16_t pos = m_open_pos[s];
  const uint16_t last = (uint16_t)(m_open_size - 1);
  m_open_pos[s] = 0xFFFFu;

  if (pos != last) {
    const uint16_t moved = m_open_heap[last];
    m_open_heap[pos] = moved;
    m_open_pos[moved] = pos;
  }

  --m_open_size;
  if (pos < m_open_size) {
    openHeapifyUp(pos);
    openHeapifyDown(pos);
  }
}

uint16_t MappingNav::openTop() const {
  return (m_open_size > 0) ? m_open_heap[0] : 0xFFFFu;
}

MappingNav::Key MappingNav::openTopKey() const {
  if (m_open_size == 0) {
    Key infk = {INF, INF};
    return infk;
  }
  return calculateKey(m_open_heap[0]);
}

uint16_t MappingNav::openPop() {
  if (m_open_size == 0) return 0xFFFFu;
  const uint16_t s = m_open_heap[0];
  openRemove(s);
  return s;
}

void MappingNav::openHeapifyUp(uint16_t pos) {
  while (pos > 0) {
    const uint16_t parent = (uint16_t)((pos - 1) >> 1);
    const uint16_t a = m_open_heap[pos];
    const uint16_t b = m_open_heap[parent];
    if (!keyLess(calculateKey(a), calculateKey(b))) break;

    m_open_heap[pos] = b;
    m_open_heap[parent] = a;
    m_open_pos[a] = parent;
    m_open_pos[b] = pos;
    pos = parent;
  }
}

void MappingNav::openHeapifyDown(uint16_t pos) {
  while (true) {
    const uint16_t left = (uint16_t)(pos * 2 + 1);
    const uint16_t right = (uint16_t)(left + 1);
    uint16_t best = pos;

    if (left < m_open_size &&
        keyLess(calculateKey(m_open_heap[left]), calculateKey(m_open_heap[best]))) {
      best = left;
    }
    if (right < m_open_size &&
        keyLess(calculateKey(m_open_heap[right]), calculateKey(m_open_heap[best]))) {
      best = right;
    }
    if (best == pos) break;

    const uint16_t a = m_open_heap[pos];
    const uint16_t b = m_open_heap[best];
    m_open_heap[pos] = b;
    m_open_heap[best] = a;
    m_open_pos[a] = best;
    m_open_pos[b] = pos;
    pos = best;
  }
}

void MappingNav::rebuildOpenHeap() {
  if (m_open_size == 0) return;
  for (int32_t i = (int32_t)(m_open_size / 2); i >= 0; --i) {
    openHeapifyDown((uint16_t)i);
  }
}

void MappingNav::forEachNeighbor(uint16_t s, uint16_t *neighbors, uint8_t &count) const {
  count = 0;
  uint16_t x = 0, y = 0;
  coordOf(s, x, y);

  for (int32_t dy = -1; dy <= 1; ++dy) {
    for (int32_t dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const int32_t nx = (int32_t)x + dx;
      const int32_t ny = (int32_t)y + dy;
      if (!inBoundsCell(nx, ny)) continue;
      neighbors[count++] = indexOf((uint16_t)nx, (uint16_t)ny);
    }
  }
}

float MappingNav::edgeCost(uint16_t from, uint16_t to) const {
  if (m_inflated_blocked[to]) return INF;

    uint16_t fx = 0, fy = 0, tx = 0, ty = 0;
    coordOf(from, fx, fy);
    coordOf(to, tx, ty);

    const int32_t dx = (int32_t)tx - (int32_t)fx;
    const int32_t dy = (int32_t)ty - (int32_t)fy;

    const bool diag = (dx != 0) && (dy != 0);

    // Prevent diagonal movement through the corner of two blocked cells.
    //
    // Example:
    //
    //   [X] [ ]
    //   [ ] [R]
    //
    // The robot cannot move diagonally from R through the corner
    // if either adjacent cell is blocked.
    if (diag) {
      const uint16_t side_a =
          indexOf((uint16_t)((int32_t)fx + dx), fy);

      const uint16_t side_b =
          indexOf(fx, (uint16_t)((int32_t)fy + dy));

      if (m_inflated_blocked[side_a] ||
          m_inflated_blocked[side_b]) {
        return INF;
      }
    }

    float cost = diag ? 1.41421356f : 1.0f;

  // Unknown is traversable but slightly penalized.
  if (getOccupancy(to) == OCC_UNKNOWN) {
  cost *= UNKNOWN_CELL_COST;
}

  // Terrain remains traversable; optional small penalties can be tuned.
  const Terrain t = getTerrain(to);
  if (t == TERRAIN_RAMP) {
    cost *= RAMP_COST;
  } else if (t == TERRAIN_SPEED_BUMP) {
    cost *= SPEED_BUMP_COST;
  }

  return cost;
}

void MappingNav::updateVertex(uint16_t u) {
  if (!m_goal_set) return;

  if (u != m_goal_idx) {
    float best = INF;
    uint16_t nb[8];
    uint8_t nb_count = 0;
    forEachNeighbor(u, nb, nb_count);
    for (uint8_t i = 0; i < nb_count; ++i) {
      const uint16_t s = nb[i];
      const float c = edgeCost(u, s);
      const float v = c + m_g[s];
      if (v < best) best = v;
    }
    m_rhs[u] = best;
  }

  if (openContains(u)) {
    openRemove(u);
  }
  if (absf(m_g[u] - m_rhs[u]) > EPS) {
    openInsertOrUpdate(u);
  }
}

bool MappingNav::computeShortestPath(uint32_t max_iterations) {
  if (!m_goal_set) return false;

  uint32_t iter = 0;
  while (iter < max_iterations) {
    const Key k_top = openTopKey();
    const Key k_start = calculateKey(m_start_idx);

    const bool cond1 = keyLess(k_top, k_start);
    const bool cond2 = absf(m_rhs[m_start_idx] - m_g[m_start_idx]) > EPS;
    if (!cond1 && !cond2) {
      return (m_g[m_start_idx] < INF * 0.5f);
    }

    const uint16_t u = openPop();
    if (u == 0xFFFFu) break;

    if (m_g[u] > m_rhs[u]) {
      m_g[u] = m_rhs[u];
      uint16_t nb[8];
      uint8_t nb_count = 0;
      forEachNeighbor(u, nb, nb_count);
      for (uint8_t i = 0; i < nb_count; ++i) {
        updateVertex(nb[i]);
      }
    } else {
      m_g[u] = INF;
      updateVertex(u);
      uint16_t nb[8];
      uint8_t nb_count = 0;
      forEachNeighbor(u, nb, nb_count);
      for (uint8_t i = 0; i < nb_count; ++i) {
        updateVertex(nb[i]);
      }
    }

    ++iter;
  }

  return (m_g[m_start_idx] < INF * 0.5f);
}

bool MappingNav::extractPath() {
  m_path_len = 0;
  if (!m_goal_set) return false;
  if (m_g[m_start_idx] >= INF * 0.5f) return false;

  uint16_t current = m_start_idx;
  for (uint16_t step = 0; step < NUM_CELLS; ++step) {
    uint16_t cx = 0, cy = 0;
    coordOf(current, cx, cy);
    m_path[m_path_len].x = cx;
    m_path[m_path_len].y = cy;
    ++m_path_len;

    if (current == m_goal_idx) {
      return true;
    }

    uint16_t nb[8];
    uint8_t nb_count = 0;
    forEachNeighbor(current, nb, nb_count);

    float best = INF;
    uint16_t best_n = 0xFFFFu;
    for (uint8_t i = 0; i < nb_count; ++i) {
      const uint16_t n = nb[i];
      const float c = edgeCost(current, n);
      const float v = c + m_g[n];
      if (v < best) {
        best = v;
        best_n = n;
      }
    }

    if (best_n == 0xFFFFu || best >= INF * 0.5f) {
      m_path_len = 0;
      return false;
    }

    current = best_n;
  }

  m_path_len = 0;
  return false;
}
