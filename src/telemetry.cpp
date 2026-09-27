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

// CRC16-CCITT (poly 0x1021) initial 0xFFFF
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
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
  // header: version(1) type(1) seq(2) payload_len(2)
  // CRC is computed over header+payload
  size_t header_len = 6;
  size_t total = header_len + payload_len;
  uint8_t *tmp = (uint8_t*)malloc(total);
  tmp[0] = version;
  tmp[1] = ptype;
  tmp[2] = (uint8_t)(seq & 0xFF);
  tmp[3] = (uint8_t)((seq >> 8) & 0xFF);
  tmp[4] = (uint8_t)(payload_len & 0xFF);
  tmp[5] = (uint8_t)((payload_len >> 8) & 0xFF);
  if (payload_len && payload) memcpy(tmp + header_len, payload, payload_len);
  uint16_t crc = crc16_ccitt(tmp, total);
  // return header+crc in out_header_crc (caller must be ready for 8 bytes)
  // layout: header(6) crc(2)
  out_header_crc[0] = tmp[0]; out_header_crc[1] = tmp[1]; out_header_crc[2] = tmp[2]; out_header_crc[3] = tmp[3]; out_header_crc[4] = tmp[4]; out_header_crc[5] = tmp[5];
  out_header_crc[6] = (uint8_t)(crc & 0xFF);
  out_header_crc[7] = (uint8_t)((crc >> 8) & 0xFF);
  out_len = 8;
  free(tmp);
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
  if (nb == 0) return;
  // New strategy: select up to max_points output samples by target-angle bins.
  // This selects the nearest measured beam to each target angle, producing
  // stable angular coverage even when the raw scan point count varies.
  uint16_t pcount = (nb < max_points) ? nb : max_points;
  uint16_t payload_len = 2 + pcount * 8;
  uint8_t *payload = (uint8_t*)malloc(payload_len);
  uint8_t *ptr = payload;
  
  // Read all points into temporary arrays (angles in radians, distances in meters)
  float *angles = (float*)malloc(sizeof(float) * nb);
  float *dists = (float*)malloc(sizeof(float) * nb);
  for (uint16_t i = 0; i < nb; ++i) {
    DataPoint *pt = ld06.getPoints(i);
    if (!pt) {
      angles[i] = 0.0f;
      dists[i] = 0.0f;
    } else {
      angles[i] = pt->angle * (PI / 180.0f);
      dists[i] = ((float)pt->distance) / 1000.0f;
    }
  }

  // Compute per-scan diagnostics from the raw scan (before downsampling)
  uint16_t zero_count = 0;
  uint16_t invalid_count = 0;
  uint16_t min_mm = 0;
  uint16_t max_mm = 0;
  uint16_t large_jump_count = 0;
  const uint16_t VALID_MIN_MM = 1;
  const uint16_t VALID_MAX_MM = 12000;
  bool first_valid = true;
  for (uint16_t i = 0; i < nb; ++i) {
    uint32_t dmm = (uint32_t)(dists[i] * 1000.0f + 0.5f);
    if (dmm == 0) {
      ++zero_count;
      continue;
    }
    if (dmm < VALID_MIN_MM || dmm > VALID_MAX_MM) {
      ++invalid_count;
      continue;
    }
    if (first_valid) {
      min_mm = (uint16_t)dmm;
      max_mm = (uint16_t)dmm;
      first_valid = false;
    } else {
      if ((uint16_t)dmm < min_mm) min_mm = (uint16_t)dmm;
      if ((uint16_t)dmm > max_mm) max_mm = (uint16_t)dmm;
    }
    // large jump vs previous scan same index
    if (g_prev_scan_nb > 0 && i < g_prev_scan_nb) {
      float prev = g_prev_scan_dists[i];
      if (prev > 0.0f) {
        float diff_m = fabsf(dists[i] - prev);
        if (diff_m > 0.05f) ++large_jump_count; // >50mm
      }
    }
  }

  // ANGLE-MATCHING DIAGNOSTICS: compare current scan to previous scan by ANGLE
  // For each valid current beam, find nearest previous beam by angular distance
  uint16_t matched_count = 0;
  float max_delta_m = 0.0f;
  uint16_t cnt_gt_100 = 0;
  uint16_t cnt_gt_500 = 0;
  uint16_t cnt_gt_1000 = 0;
  float angle_of_max_delta_rad = 0.0f;
  // collect angle diffs (deg) for stats
  float ang_diff_sum = 0.0f;
  float ang_diff_sq_sum = 0.0f;
  uint16_t ang_diff_n = 0;
  const float RAD_TO_DEG_F = 180.0f / PI;
  if (g_prev_scan_nb > 0) {
    for (uint16_t i = 0; i < nb; ++i) {
      uint32_t curr_mm = (uint32_t)(dists[i] * 1000.0f + 0.5f);
      if (curr_mm == 0) continue;
      if (curr_mm < VALID_MIN_MM || curr_mm > VALID_MAX_MM) continue;
      // find nearest previous angle
      uint16_t best_j = 0;
      float best_ang_diff = 1e9f;
      for (uint16_t j = 0; j < g_prev_scan_nb; ++j) {
        float d_ang = fabsf(angles[i] - g_prev_scan_angles[j]);
        if (d_ang > PI) d_ang = 2.0f * PI - d_ang;
        if (d_ang < best_ang_diff) { best_ang_diff = d_ang; best_j = j; }
      }
      // get previous distance for best_j
      uint32_t prev_mm = (uint32_t)(g_prev_scan_dists[best_j] * 1000.0f + 0.5f);
      if (prev_mm == 0) continue;
      if (prev_mm < VALID_MIN_MM || prev_mm > VALID_MAX_MM) continue;
      uint32_t delta_mm = (curr_mm > prev_mm) ? (curr_mm - prev_mm) : (prev_mm - curr_mm);
      ++matched_count;
      if ((float)delta_mm / 1000.0f > max_delta_m) {
        max_delta_m = (float)delta_mm / 1000.0f;
        angle_of_max_delta_rad = angles[i];
      }
      if (delta_mm > 100) ++cnt_gt_100;
      if (delta_mm > 500) ++cnt_gt_500;
      if (delta_mm > 1000) ++cnt_gt_1000;
      // angle diff stats (degrees)
      float ad = best_ang_diff * RAD_TO_DEG_F;
      ang_diff_sum += ad;
      ang_diff_sq_sum += ad * ad;
      ++ang_diff_n;
    }
  }

  // store into globals (capping where appropriate)
  g_ld_matched_count = (uint16_t)matched_count;
  uint32_t max_delta_mm_u32 = (uint32_t)(max_delta_m * 1000.0f + 0.5f);
  if (max_delta_mm_u32 > 0xFFFF) max_delta_mm_u32 = 0xFFFF;
  g_ld_max_delta_mm = (uint16_t)max_delta_mm_u32;
  g_ld_count_delta_gt_100 = cnt_gt_100;
  g_ld_count_delta_gt_500 = cnt_gt_500;
  g_ld_count_delta_gt_1000 = cnt_gt_1000;
  // angles: map to 0..360 and store centi-degrees
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

  // Save current scan distances for next scan comparison
  for (uint16_t i = 0; i < nb && i < LD06_MAX_PTS_SCAN; ++i) {
    g_prev_scan_dists[i] = dists[i];
    g_prev_scan_angles[i] = angles[i];
  }
  g_prev_scan_nb = nb;

  // Update global counters used by heartbeat diagnostics
  g_ld_zero_count = zero_count;
  g_ld_invalid_count = invalid_count;
  g_ld_min_distance_mm = min_mm;
  g_ld_max_distance_mm = max_mm;
  g_ld_large_jump_count = large_jump_count;
  // point_count u16
  *ptr++ = (uint8_t)(pcount & 0xFF); *ptr++ = (uint8_t)((pcount >> 8) & 0xFF);
  // Reuse the `angles`/`dists` arrays populated above (no need to re-read points)

  // Downsampling: sample by dividing the raw scan into pcount index bins
  // (legacy/index-stride behavior). For each output slot pick the point at
  // index = floor(out * nb / pcount).
  for (uint16_t out = 0; out < pcount; ++out) {
    uint16_t idx = (uint16_t)(((uint32_t)out * (uint32_t)nb) / (uint32_t)pcount);
    if (idx >= nb) idx = nb - 1;
    float a = angles[idx];
    float r = dists[idx];
    memcpy(ptr, &a, 4); ptr += 4;
    memcpy(ptr, &r, 4); ptr += 4;
  }

  free(angles);
  free(dists);

  // Update legacy diagnostic counters
  g_scan_packet_count++;
  g_scan_point_count = pcount;
  last_scan_point_count = pcount;

  // Increment telemetry-sent diagnostic counter immediately before send
  scan_telemetry_sent_count++;

  send_frame(1, 0x05, payload, payload_len);
  free(payload);
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
