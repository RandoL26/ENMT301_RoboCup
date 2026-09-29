#include "telemetry.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "ld06.h"

static Stream *g_serial = nullptr;
static uint16_t g_seq = 0;
// Legacy temporary diagnostic counters for scan telemetry (kept for compatibility)
static uint16_t g_scan_packet_count = 0;
static uint16_t g_scan_point_count = 0;

// LD06 per-scan diagnostics (updated by telemetry_send_downsampled_scan)
uint16_t g_ld_zero_count = 0;
uint16_t g_ld_invalid_count = 0;
uint16_t g_ld_min_distance_mm = 0;
uint16_t g_ld_max_distance_mm = 0;
uint16_t g_ld_large_jump_count = 0;
static float g_prev_scan_dists[LD06_MAX_PTS_SCAN];
static float g_prev_scan_angles[LD06_MAX_PTS_SCAN];
static uint16_t g_prev_scan_nb = 0;

// New diagnostic counters requested by user (non-static so other files can update)
uint32_t scan_ready_count = 0;
uint32_t scan_telemetry_call_count = 0;
uint32_t scan_telemetry_sent_count = 0;
uint16_t last_scan_point_count = 0;

// Angle-matching diagnostics (updated per completed scan)
uint16_t g_ld_matched_count = 0;
uint16_t g_ld_max_delta_mm = 0;
uint16_t g_ld_count_delta_gt_100 = 0;
uint16_t g_ld_count_delta_gt_500 = 0;
uint16_t g_ld_count_delta_gt_1000 = 0;
uint16_t g_ld_angle_of_max_delta_cdeg = 0; // centi-degrees 0..36000
uint16_t g_ld_angle_diff_mean_cdeg = 0;
uint16_t g_ld_angle_diff_std_cdeg = 0;

// Incremental CRC16-CCITT update (poly 0x1021).
static uint16_t crc16_update(uint16_t crc, uint8_t byte) {
  crc ^= ((uint16_t)byte) << 8;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static void write_le_u16(Stream &s, uint16_t v) {
  s.write((uint8_t)(v & 0xFF));
  s.write((uint8_t)((v >> 8) & 0xFF));
}
static void write_le_u32(Stream &s, uint32_t v) {
  s.write((uint8_t)(v & 0xFF));
  s.write((uint8_t)((v >> 8) & 0xFF));
  s.write((uint8_t)((v >> 16) & 0xFF));
  s.write((uint8_t)((v >> 24) & 0xFF));
}
static void write_le_f32(Stream &s, float f) {
  union { float f; uint8_t b[4]; } u;
  u.f = f;
  // little-endian
  s.write(u.b, 4);
}

static void build_header_and_crc(uint8_t version, uint8_t ptype, uint16_t seq, uint16_t payload_len, const uint8_t *payload, uint8_t *out_header_crc, size_t &out_len) {
  // Build the header and calculate CRC incrementally without a temporary
  // heap allocation for each telemetry frame.
  out_header_crc[0] = version;
  out_header_crc[1] = ptype;
  out_header_crc[2] = (uint8_t)(seq & 0xFF);
  out_header_crc[3] = (uint8_t)((seq >> 8) & 0xFF);
  out_header_crc[4] = (uint8_t)(payload_len & 0xFF);
  out_header_crc[5] = (uint8_t)((payload_len >> 8) & 0xFF);
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < 6; ++i) {
    crc = crc16_update(crc, out_header_crc[i]);
  }
  for (uint16_t i = 0; i < payload_len && payload; ++i) {
    crc = crc16_update(crc, payload[i]);
  }
  // return header+crc in out_header_crc (caller must be ready for 8 bytes)
  // layout: header(6) crc(2)
  out_header_crc[6] = (uint8_t)(crc & 0xFF);
  out_header_crc[7] = (uint8_t)((crc >> 8) & 0xFF);
  out_len = 8;
}

static void send_frame(uint8_t version, uint8_t ptype, const uint8_t *payload, uint16_t payload_len) {
  if (!g_serial) return;
  uint16_t seq = g_seq++;
  // compute header+crc
  uint8_t header_crc[8];
  size_t hdrcrc_len = 0;
  build_header_and_crc(version, ptype, seq, payload_len, payload, header_crc, hdrcrc_len);

  // send sync
  g_serial->write(0xA5);
  g_serial->write(0x5A);
  // send header (6 bytes)
  g_serial->write(header_crc, 6);
  // send payload
  if (payload_len && payload) g_serial->write(payload, payload_len);
  // CRC16 little-endian
  g_serial->write(header_crc + 6, 2);
}

void telemetry_init(Stream &serial) {
  g_serial = &serial;
}

void telemetry_send_heartbeat(uint32_t uptime_ms, uint16_t tx_queue_depth, uint16_t dropped_frames) {
  // Heartbeat: uptime_ms (4), tx_queue_depth (2), dropped_frames (2)
  // Optional appended diagnostics (legacy): scan_packet_count (2), scan_point_count (2)
  // Additional LD06 diagnostics appended (u16 each): zero_count, invalid_count, min_mm, max_mm, large_jump_count
  // Keep heartbeat payload compatible: uptime_ms (4) + tx_queue_depth (2) + dropped_frames (2)
  // + legacy scan counters: scan_packet_count (2), scan_point_count (2)
  const uint16_t payload_len = 12 + 2 + 2; // 12 + two u16s
  uint8_t *payload = (uint8_t*)malloc(payload_len);
  if (!payload) return;
  payload[0] = (uint8_t)(uptime_ms & 0xFF);
  payload[1] = (uint8_t)((uptime_ms >> 8) & 0xFF);
  payload[2] = (uint8_t)((uptime_ms >> 16) & 0xFF);
  payload[3] = (uint8_t)((uptime_ms >> 24) & 0xFF);
  payload[4] = (uint8_t)(tx_queue_depth & 0xFF);
  payload[5] = (uint8_t)((tx_queue_depth >> 8) & 0xFF);
  payload[6] = (uint8_t)(dropped_frames & 0xFF);
  payload[7] = (uint8_t)((dropped_frames >> 8) & 0xFF);
  payload[8] = (uint8_t)(g_scan_packet_count & 0xFF);
  payload[9] = (uint8_t)((g_scan_packet_count >> 8) & 0xFF);
  payload[10] = (uint8_t)(g_scan_point_count & 0xFF);
  payload[11] = (uint8_t)((g_scan_point_count >> 8) & 0xFF);
  send_frame(1, 0x04, payload, payload_len);
  free(payload);
}

void telemetry_send_pose_and_path(const MappingNav &nav) {
  // Gather pose
  MappingNav::Pose2D p = nav.getPose();
  // Get path cells
  const uint16_t max_cells = 128;
  MappingNav::CellCoord *cells = (MappingNav::CellCoord*)malloc(sizeof(MappingNav::CellCoord) * max_cells);
  uint16_t path_count = nav.getPathCells(cells, max_cells);

  // Payload size: 3*4 (floats) + 2 (path_count) + path_count*2 (u8,u8 packed as u8 each)
  uint16_t payload_len = 12 + 2 + path_count * 2;
  uint8_t *payload = (uint8_t*)malloc(payload_len);
  uint8_t *ptr = payload;
  // x,y,theta float32 little-endian
  memcpy(ptr, &p.x_m, 4); ptr += 4;
  memcpy(ptr, &p.y_m, 4); ptr += 4;
  memcpy(ptr, &p.theta_rad, 4); ptr += 4;
  // path_count (u16 little-endian)
  payload[12] = (uint8_t)(path_count & 0xFF);
  payload[13] = (uint8_t)((path_count >> 8) & 0xFF);
  ptr = payload + 14;
  for (uint16_t i = 0; i < path_count; ++i) {
    // clamp to 0..255 for u8
    uint8_t cx = (uint8_t)(cells[i].x & 0xFF);
    uint8_t cy = (uint8_t)(cells[i].y & 0xFF);
    *ptr++ = cx;
    *ptr++ = cy;
  }

  send_frame(1, 0x01, payload, payload_len);
  free(payload);
  free(cells);
}

void telemetry_send_localisation_debug(
    float encoder_left_delta_m,
    float encoder_right_delta_m,
    float encoder_dtheta_rad,
    float imu_dtheta_rad,
    float lidar_match_score,
    float lidar_correction_accepted,
    const MappingNav::Pose2D &pose,
    float lidar_dx_m, float lidar_dy_m, float lidar_dtheta_rad,
    int32_t encoder_left_count, int32_t encoder_right_count) {

  // Payload:
  //
  // encoder_left_delta_m  4 bytes
  // encoder_right_delta_m 4 bytes
  // encoder_dtheta_rad    4 bytes
  // imu_dtheta_rad    4 bytes
  // lidar_match_score 4 bytes (0..1000)
  // lidar_accepted    4 bytes (0 or 1)
  // pose_x_m          4 bytes
  // pose_y_m          4 bytes
  // pose_theta_rad    4 bytes
  //
  // 36-byte base remains compatible; appended correction and encoder counts = 56 bytes total.

  const uint16_t payload_len = 56;

  uint8_t payload[payload_len];

  uint8_t *ptr = payload;

  memcpy(ptr, &encoder_left_delta_m, 4);
  ptr += 4;

  memcpy(ptr, &encoder_right_delta_m, 4);
  ptr += 4;

  memcpy(ptr, &encoder_dtheta_rad, 4);
  ptr += 4;

  memcpy(ptr, &imu_dtheta_rad, 4);
  ptr += 4;

  memcpy(ptr, &lidar_match_score, 4);
  ptr += 4;

  memcpy(ptr, &lidar_correction_accepted, 4);
  ptr += 4;

  memcpy(ptr, &pose.x_m, 4);
  ptr += 4;

  memcpy(ptr, &pose.y_m, 4);
  ptr += 4;

  memcpy(ptr, &pose.theta_rad, 4);
  ptr += 4;

  memcpy(ptr, &lidar_dx_m, 4); ptr += 4;
  memcpy(ptr, &lidar_dy_m, 4); ptr += 4;
  memcpy(ptr, &lidar_dtheta_rad, 4); ptr += 4;
  memcpy(ptr, &encoder_left_count, 4); ptr += 4;
  memcpy(ptr, &encoder_right_count, 4);

  send_frame(
      1,
      PACKET_LOCALISATION_DEBUG,
      payload,
      payload_len
  );
}

void telemetry_send_grid_keyframe(const MappingNav &nav) {
  const uint8_t *grid = nav.getGridData();
  uint16_t width = MappingNav::GRID_WIDTH;
  uint16_t height = MappingNav::GRID_HEIGHT;
  uint16_t cell_mm = (uint16_t)(MappingNav::CELL_SIZE_M * 1000.0f);
  uint16_t packed_len = width * height; // one byte per cell

  uint16_t payload_len = 2 + 2 + 2 + 2 + packed_len; // width,height,cell_mm,packed_len + data
  uint8_t *payload = (uint8_t*)malloc(payload_len);
  uint8_t *ptr = payload;
  // width
  *ptr++ = (uint8_t)(width & 0xFF); *ptr++ = (uint8_t)((width >> 8) & 0xFF);
  // height
  *ptr++ = (uint8_t)(height & 0xFF); *ptr++ = (uint8_t)((height >> 8) & 0xFF);
  // cell size mm
  *ptr++ = (uint8_t)(cell_mm & 0xFF); *ptr++ = (uint8_t)((cell_mm >> 8) & 0xFF);
  // packed length
  *ptr++ = (uint8_t)(packed_len & 0xFF); *ptr++ = (uint8_t)((packed_len >> 8) & 0xFF);
  // copy grid
  memcpy(ptr, grid, packed_len);

  send_frame(1, 0x02, payload, payload_len);
  free(payload);
}

void telemetry_send_downsampled_scan(LD06 &ld06, const MappingNav &nav, uint16_t max_points) {
  uint16_t nb = ld06.getNbPointsInScan();
  constexpr uint16_t MAX_SCAN_TELEMETRY_POINTS = 60;
  static uint8_t payload[2 + MAX_SCAN_TELEMETRY_POINTS * 8];
  if (nb == 0 || max_points == 0) return;
  if (max_points > MAX_SCAN_TELEMETRY_POINTS) {
    max_points = MAX_SCAN_TELEMETRY_POINTS;
  }
  const uint16_t pcount = (nb < max_points) ? nb : max_points;
  const uint16_t payload_len = 2 + pcount * 8;
  uint8_t *ptr = payload;
  // Diagnostics are computed once per completed raw scan by
  // `telemetry_update_ld06_diagnostics()` called from the LD06 driver.
  // Here we simply reuse the last-computed LD06 diagnostics globals.
  // point_count u16
  *ptr++ = (uint8_t)(pcount & 0xFF); *ptr++ = (uint8_t)((pcount >> 8) & 0xFF);
  // Sample evenly across the completed scan without allocating temporary arrays.
  for (uint16_t out = 0; out < pcount; ++out) {
    uint16_t idx = (uint16_t)(((uint32_t)out * (uint32_t)nb) / (uint32_t)pcount);
    if (idx >= nb) idx = nb - 1;
    const DataPoint *pt = ld06.getPoints(idx);
    const float a = pt->angle * (PI / 180.0f);
    const float r = ((float)pt->distance) / 1000.0f;
    memcpy(ptr, &a, 4); ptr += 4;
    memcpy(ptr, &r, 4); ptr += 4;
  }

  // Update legacy diagnostic counters
  g_scan_packet_count++;
  g_scan_point_count = pcount;
  last_scan_point_count = pcount;

  // Increment telemetry-sent diagnostic counter immediately before send
  scan_telemetry_sent_count++;

  send_frame(1, 0x05, payload, payload_len);
}

// Diagnostic packet: layout (little-endian):
// scan_ready_count (u32), scan_telemetry_call_count (u32), scan_telemetry_sent_count (u32), last_scan_point_count (u16)
void telemetry_send_diag_scan() {
  // Expand diagnostic packet to include LD06 per-scan diagnostics
  // Layout (little-endian):
  // scan_ready_count (u32), scan_telemetry_call_count (u32), scan_telemetry_sent_count (u32),
  // last_scan_point_count (u16), ld_zero_count (u16), ld_invalid_count (u16),
  // ld_min_mm (u16), ld_max_mm (u16), ld_large_jump_count (u16)
  // add ld06 scan-order diagnostics: 11 u16 fields (22 bytes)
  const uint16_t payload_len = 4 + 4 + 4 + 2 + 2 + 2 + 2 + 2 + 2 + 16 + 22; // 62
  uint8_t payload[payload_len];
  // scan_ready_count
  payload[0] = (uint8_t)(scan_ready_count & 0xFF);
  payload[1] = (uint8_t)((scan_ready_count >> 8) & 0xFF);
  payload[2] = (uint8_t)((scan_ready_count >> 16) & 0xFF);
  payload[3] = (uint8_t)((scan_ready_count >> 24) & 0xFF);
  // scan_telemetry_call_count
  payload[4] = (uint8_t)(scan_telemetry_call_count & 0xFF);
  payload[5] = (uint8_t)((scan_telemetry_call_count >> 8) & 0xFF);
  payload[6] = (uint8_t)((scan_telemetry_call_count >> 16) & 0xFF);
  payload[7] = (uint8_t)((scan_telemetry_call_count >> 24) & 0xFF);
  // scan_telemetry_sent_count
  payload[8] = (uint8_t)(scan_telemetry_sent_count & 0xFF);
  payload[9] = (uint8_t)((scan_telemetry_sent_count >> 8) & 0xFF);
  payload[10] = (uint8_t)((scan_telemetry_sent_count >> 16) & 0xFF);
  payload[11] = (uint8_t)((scan_telemetry_sent_count >> 24) & 0xFF);
  // last_scan_point_count (u16)
  payload[12] = (uint8_t)(last_scan_point_count & 0xFF);
  payload[13] = (uint8_t)((last_scan_point_count >> 8) & 0xFF);
  // LD06 diagnostics (u16 each)
  payload[14] = (uint8_t)(g_ld_zero_count & 0xFF);
  payload[15] = (uint8_t)((g_ld_zero_count >> 8) & 0xFF);
  payload[16] = (uint8_t)(g_ld_invalid_count & 0xFF);
  payload[17] = (uint8_t)((g_ld_invalid_count >> 8) & 0xFF);
  payload[18] = (uint8_t)(g_ld_min_distance_mm & 0xFF);
  payload[19] = (uint8_t)((g_ld_min_distance_mm >> 8) & 0xFF);
  payload[20] = (uint8_t)(g_ld_max_distance_mm & 0xFF);
  payload[21] = (uint8_t)((g_ld_max_distance_mm >> 8) & 0xFF);
  payload[22] = (uint8_t)(g_ld_large_jump_count & 0xFF);
  payload[23] = (uint8_t)((g_ld_large_jump_count >> 8) & 0xFF);
  // Angle-matching diagnostics (u16 each)
  payload[24] = (uint8_t)(g_ld_matched_count & 0xFF);
  payload[25] = (uint8_t)((g_ld_matched_count >> 8) & 0xFF);
  payload[26] = (uint8_t)(g_ld_max_delta_mm & 0xFF);
  payload[27] = (uint8_t)((g_ld_max_delta_mm >> 8) & 0xFF);
  payload[28] = (uint8_t)(g_ld_count_delta_gt_100 & 0xFF);
  payload[29] = (uint8_t)((g_ld_count_delta_gt_100 >> 8) & 0xFF);
  payload[30] = (uint8_t)(g_ld_count_delta_gt_500 & 0xFF);
  payload[31] = (uint8_t)((g_ld_count_delta_gt_500 >> 8) & 0xFF);
  payload[32] = (uint8_t)(g_ld_count_delta_gt_1000 & 0xFF);
  payload[33] = (uint8_t)((g_ld_count_delta_gt_1000 >> 8) & 0xFF);
  payload[34] = (uint8_t)(g_ld_angle_of_max_delta_cdeg & 0xFF);
  payload[35] = (uint8_t)((g_ld_angle_of_max_delta_cdeg >> 8) & 0xFF);
  payload[36] = (uint8_t)(g_ld_angle_diff_mean_cdeg & 0xFF);
  payload[37] = (uint8_t)((g_ld_angle_diff_mean_cdeg >> 8) & 0xFF);
  payload[38] = (uint8_t)(g_ld_angle_diff_std_cdeg & 0xFF);
  payload[39] = (uint8_t)((g_ld_angle_diff_std_cdeg >> 8) & 0xFF);

  // LD06 scan-order diagnostics (u16 each unless noted):
  // num_points, first_angle_cdeg, last_angle_cdeg, min_angle_cdeg, max_angle_cdeg,
  // neg_steps_count, large_pos_jumps_count, largest_pos_step_cdeg, largest_neg_step_cdeg,
  // total_span_cdeg, approx_one_revolution (u16: 0/1)
  payload[40] = (uint8_t)(ld06_diag_num_points & 0xFF);
  payload[41] = (uint8_t)((ld06_diag_num_points >> 8) & 0xFF);
  payload[42] = (uint8_t)(ld06_diag_first_angle_cdeg & 0xFF);
  payload[43] = (uint8_t)((ld06_diag_first_angle_cdeg >> 8) & 0xFF);
  payload[44] = (uint8_t)(ld06_diag_last_angle_cdeg & 0xFF);
  payload[45] = (uint8_t)((ld06_diag_last_angle_cdeg >> 8) & 0xFF);
  payload[46] = (uint8_t)(ld06_diag_min_angle_cdeg & 0xFF);
  payload[47] = (uint8_t)((ld06_diag_min_angle_cdeg >> 8) & 0xFF);
  payload[48] = (uint8_t)(ld06_diag_max_angle_cdeg & 0xFF);
  payload[49] = (uint8_t)((ld06_diag_max_angle_cdeg >> 8) & 0xFF);
  payload[50] = (uint8_t)(ld06_diag_neg_steps_count & 0xFF);
  payload[51] = (uint8_t)((ld06_diag_neg_steps_count >> 8) & 0xFF);
  payload[52] = (uint8_t)(ld06_diag_large_pos_jumps_count & 0xFF);
  payload[53] = (uint8_t)((ld06_diag_large_pos_jumps_count >> 8) & 0xFF);
  payload[54] = (uint8_t)(ld06_diag_largest_pos_step_cdeg & 0xFF);
  payload[55] = (uint8_t)((ld06_diag_largest_pos_step_cdeg >> 8) & 0xFF);
  payload[56] = (uint8_t)(ld06_diag_largest_neg_step_cdeg & 0xFF);
  payload[57] = (uint8_t)((ld06_diag_largest_neg_step_cdeg >> 8) & 0xFF);
  payload[58] = (uint8_t)(ld06_diag_total_span_cdeg & 0xFF);
  payload[59] = (uint8_t)((ld06_diag_total_span_cdeg >> 8) & 0xFF);
  payload[60] = (uint8_t)(ld06_diag_approx_one_revolution & 0xFF);
  payload[61] = 0; // padding

  send_frame(1, 0x06, payload, payload_len);
}

// Compute and store LD06 per-scan diagnostics from a completed raw scan.
// This function is intentionally called once per completed raw scan by the
// LD06 driver to avoid doing expensive work in the high-frequency telemetry
// send path.
void telemetry_update_ld06_diagnostics(struct DataPointHandler *scan) {
  if (!scan) return;
  uint16_t nb = scan->index;

  // Build temporary arrays of angles (rad) and distances (m)
  float angles[LD06_MAX_PTS_SCAN];
  float dists[LD06_MAX_PTS_SCAN];
  for (uint16_t i = 0; i < nb && i < LD06_MAX_PTS_SCAN; ++i) {
    angles[i] = (scan->points[i].angle) * (PI / 180.0f);
    dists[i] = ((float)scan->points[i].distance) / 1000.0f;
  }

  // Compute simple range counters
  const uint16_t VALID_MIN_MM = 1;
  const uint16_t VALID_MAX_MM = 12000;
  uint16_t zero_count = 0;
  uint16_t invalid_count = 0;
  uint16_t min_mm = 0;
  uint16_t max_mm = 0;
  uint16_t large_jump_count = 0;
  bool first_valid = true;
  for (uint16_t i = 0; i < nb; ++i) {
    uint32_t dmm = (uint32_t)(dists[i] * 1000.0f + 0.5f);
    if (dmm == 0) { ++zero_count; continue; }
    if (dmm < VALID_MIN_MM || dmm > VALID_MAX_MM) { ++invalid_count; continue; }
    if (first_valid) { min_mm = (uint16_t)dmm; max_mm = (uint16_t)dmm; first_valid = false; }
    else { if ((uint16_t)dmm < min_mm) min_mm = (uint16_t)dmm; if ((uint16_t)dmm > max_mm) max_mm = (uint16_t)dmm; }
    // Compare to previous scan at same index
    if (g_prev_scan_nb > 0 && i < g_prev_scan_nb) {
      float prev = g_prev_scan_dists[i];
      if (prev > 0.0f) {
        float diff_m = fabsf(dists[i] - prev);
        if (diff_m > 0.05f) ++large_jump_count;
      }
    }
  }

  // Angle-matching diagnostics (compare by angle to previous scan)
  uint16_t matched_count = 0;
  float max_delta_m = 0.0f;
  uint16_t cnt_gt_100 = 0;
  uint16_t cnt_gt_500 = 0;
  uint16_t cnt_gt_1000 = 0;
  float angle_of_max_delta_rad = 0.0f;
  float ang_diff_sum = 0.0f;
  float ang_diff_sq_sum = 0.0f;
  uint16_t ang_diff_n = 0;
  const float RAD_TO_DEG_F = 180.0f / PI;
  // The nearest-angle comparison is O(n^2). Keep the diagnostic data but run
  // this expensive comparison on every fourth scan; other per-scan range and
  // quality counters below remain current for every completed scan.
  static uint8_t angle_match_divider = 0;
  ++angle_match_divider;
  if (angle_match_divider >= 4) angle_match_divider = 0;
  const bool update_angle_match = (angle_match_divider == 0);
  if (g_prev_scan_nb > 0 && update_angle_match) {
    for (uint16_t i = 0; i < nb; ++i) {
      uint32_t curr_mm = (uint32_t)(dists[i] * 1000.0f + 0.5f);
      if (curr_mm == 0) continue;
      if (curr_mm < VALID_MIN_MM || curr_mm > VALID_MAX_MM) continue;
      // find nearest previous angle (linear search, ok for <=1200)
      uint16_t best_j = 0; float best_ang_diff = 1e9f;
      for (uint16_t j = 0; j < g_prev_scan_nb; ++j) {
        float d_ang = fabsf(angles[i] - g_prev_scan_angles[j]);
        if (d_ang > PI) d_ang = 2.0f * PI - d_ang;
        if (d_ang < best_ang_diff) { best_ang_diff = d_ang; best_j = j; }
      }
      uint32_t prev_mm = (uint32_t)(g_prev_scan_dists[best_j] * 1000.0f + 0.5f);
      if (prev_mm == 0) continue;
      if (prev_mm < VALID_MIN_MM || prev_mm > VALID_MAX_MM) continue;
      uint32_t delta_mm = (curr_mm > prev_mm) ? (curr_mm - prev_mm) : (prev_mm - curr_mm);
      ++matched_count;
      if ((float)delta_mm / 1000.0f > max_delta_m) { max_delta_m = (float)delta_mm / 1000.0f; angle_of_max_delta_rad = angles[i]; }
      if (delta_mm > 100) ++cnt_gt_100; if (delta_mm > 500) ++cnt_gt_500; if (delta_mm > 1000) ++cnt_gt_1000;
      float ad = best_ang_diff * RAD_TO_DEG_F; ang_diff_sum += ad; ang_diff_sq_sum += ad * ad; ++ang_diff_n;
    }
  }

  // Store results when a comparison was performed; otherwise retain the most
  // recent angle-match snapshot. Packet 0x06 remains byte-for-byte compatible.
  if (g_prev_scan_nb > 0 && update_angle_match) {
    g_ld_matched_count = (uint16_t)matched_count;
    uint32_t max_delta_mm_u32 = (uint32_t)(max_delta_m * 1000.0f + 0.5f);
    if (max_delta_mm_u32 > 0xFFFF) max_delta_mm_u32 = 0xFFFF;
    g_ld_max_delta_mm = (uint16_t)max_delta_mm_u32;
    g_ld_count_delta_gt_100 = cnt_gt_100;
    g_ld_count_delta_gt_500 = cnt_gt_500;
    g_ld_count_delta_gt_1000 = cnt_gt_1000;
    float angle_deg = fmodf((angle_of_max_delta_rad * RAD_TO_DEG_F) + 360.0f, 360.0f);
    uint32_t angle_cdeg = (uint32_t)(angle_deg * 100.0f + 0.5f);
    if (angle_cdeg > 0xFFFF) angle_cdeg = 0xFFFF;
    g_ld_angle_of_max_delta_cdeg = (uint16_t)angle_cdeg;
    if (ang_diff_n > 0) {
      float mean = ang_diff_sum / (float)ang_diff_n;
      float var = (ang_diff_sq_sum / (float)ang_diff_n) - (mean * mean);
      if (var < 0.0f) var = 0.0f;
      float std = sqrtf(var);
      uint32_t mean_cdeg = (uint32_t)(mean * 100.0f + 0.5f);
      uint32_t std_cdeg = (uint32_t)(std * 100.0f + 0.5f);
      if (mean_cdeg > 0xFFFF) mean_cdeg = 0xFFFF;
      if (std_cdeg > 0xFFFF) std_cdeg = 0xFFFF;
      g_ld_angle_diff_mean_cdeg = (uint16_t)mean_cdeg;
      g_ld_angle_diff_std_cdeg = (uint16_t)std_cdeg;
    } else {
      g_ld_angle_diff_mean_cdeg = 0;
      g_ld_angle_diff_std_cdeg = 0;
    }
  }

  // Save current scan distances/angles for next comparison
  for (uint16_t i = 0; i < nb && i < LD06_MAX_PTS_SCAN; ++i) {
    g_prev_scan_dists[i] = dists[i];
    g_prev_scan_angles[i] = angles[i];
  }
  g_prev_scan_nb = nb;

  // Range counters
  g_ld_zero_count = zero_count; g_ld_invalid_count = invalid_count;
  g_ld_min_distance_mm = min_mm; g_ld_max_distance_mm = max_mm; g_ld_large_jump_count = large_jump_count;
}

void telemetry_send_status(
    uint8_t start_side,
    bool lidar_ok,
    bool imu_ok,
    bool tof_ok,
    bool optical_flow_ok,
    bool ultrasonic_ok,
    int16_t motor_left,
    int16_t motor_right,
    float left_rpm,
    float right_rpm,
    bool planner_goal_set,
    uint16_t goal_cell,
    uint16_t path_length,
    bool robot_started,
    bool start_button_pressed,
    bool target_navigation_active,
    bool visualiser_path_preview_active)
{
    /*
     * STATUS packet 0x07
     *
     * Byte layout:
     *
     * 0       start side
     * 1       sensor flags
     * 2-3     left motor command
     * 4-5     right motor command
     * 6-9     measured left RPM
     * 10-13   measured right RPM
     * 14      planner goal set
     * 15-16   goal cell
     * 17-18   path length
     * 19      robot started
     * 20      D25 start button pressed
     * 21      target navigation active
     * 22      visualiser path preview active
     *
     * Total = 23 bytes
     */

    const uint16_t payload_len = 23;

    uint8_t payload[payload_len];

    // Start side
    payload[0] = start_side;

    // Sensor status bitfield
    //
    // bit 0 = LiDAR
    // bit 1 = BNO055
    // bit 2 = ToF
    // bit 3 = Optical Flow
    // bit 4 = Ultrasonic

    uint8_t sensor_flags = 0;

    if (lidar_ok)
        sensor_flags |= (1 << 0);

    if (imu_ok)
        sensor_flags |= (1 << 1);

    if (tof_ok)
        sensor_flags |= (1 << 2);

    if (optical_flow_ok)
        sensor_flags |= (1 << 3);

    if (ultrasonic_ok)
        sensor_flags |= (1 << 4);

    payload[1] = sensor_flags;

    // Motor commands
    memcpy(
        &payload[2],
        &motor_left,
        sizeof(int16_t)
    );

    memcpy(
        &payload[4],
        &motor_right,
        sizeof(int16_t)
    );

    // Motor RPM
    memcpy(
        &payload[6],
        &left_rpm,
        sizeof(float)
    );

    memcpy(
        &payload[10],
        &right_rpm,
        sizeof(float)
    );

    // Planner
    payload[14] =
        planner_goal_set ? 1 : 0;

    payload[15] =
        (uint8_t)(goal_cell & 0xFF);

    payload[16] =
        (uint8_t)((goal_cell >> 8) & 0xFF);

    payload[17] =
        (uint8_t)(path_length & 0xFF);

    payload[18] =
        (uint8_t)((path_length >> 8) & 0xFF);

    payload[19] = robot_started ? 1 : 0;
    payload[20] = start_button_pressed ? 1 : 0;
    payload[21] = target_navigation_active ? 1 : 0;
    payload[22] = visualiser_path_preview_active ? 1 : 0;

    send_frame(
        1,
        PACKET_STATUS,
        payload,
        payload_len
    );
}

void telemetry_send_inflated_grid(const MappingNav &nav)
{
    const uint8_t *grid = nav.getInflatedGridData();

    const uint16_t width = MappingNav::GRID_WIDTH;
    const uint16_t height = MappingNav::GRID_HEIGHT;
    const uint16_t cell_mm =
        (uint16_t)(MappingNav::CELL_SIZE_M * 1000.0f);

    const uint16_t grid_size = width * height;

    /*
     * Payload:
     *
     * width       u16
     * height      u16
     * cell_mm     u16
     * grid_size   u16
     * grid        1 byte/cell
     */

    const uint16_t payload_len = 8 + grid_size;

    uint8_t *payload =
        (uint8_t *)malloc(payload_len);

    if (!payload)
        return;

    uint8_t *ptr = payload;

    *ptr++ = (uint8_t)(width & 0xFF);
    *ptr++ = (uint8_t)((width >> 8) & 0xFF);

    *ptr++ = (uint8_t)(height & 0xFF);
    *ptr++ = (uint8_t)((height >> 8) & 0xFF);

    *ptr++ = (uint8_t)(cell_mm & 0xFF);
    *ptr++ = (uint8_t)((cell_mm >> 8) & 0xFF);

    *ptr++ = (uint8_t)(grid_size & 0xFF);
    *ptr++ = (uint8_t)((grid_size >> 8) & 0xFF);

    memcpy(
        ptr,
        grid,
        grid_size
    );

    send_frame(
        1,
        PACKET_INFLATED_GRID,
        payload,
        payload_len
    );

    free(payload);
}
