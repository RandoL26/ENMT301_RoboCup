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

// New diagnostic counters requested by user (non-static so other files can update)
uint32_t scan_ready_count = 0;
uint32_t scan_telemetry_call_count = 0;
uint32_t scan_telemetry_sent_count = 0;
uint16_t last_scan_point_count = 0;

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
  // Optional appended diagnostics: scan_packet_count (2), scan_point_count (2)
  uint8_t payload[12];
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
  send_frame(1, 0x04, payload, sizeof(payload));
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
  uint16_t step = (nb + max_points - 1) / max_points;
  // Estimate payload: 2 + n*(4+4)
  uint16_t pcount = (nb + step - 1) / step;
  uint16_t payload_len = 2 + pcount * 8;
  uint8_t *payload = (uint8_t*)malloc(payload_len);
  uint8_t *ptr = payload;
  // point_count u16
  *ptr++ = (uint8_t)(pcount & 0xFF); *ptr++ = (uint8_t)((pcount >> 8) & 0xFF);

  for (uint16_t i = 0, out = 0; i < nb && out < pcount; i += step, ++out) {
    DataPoint *pt = ld06.getPoints(i);
    if (!pt) {
      float ang0 = 0.0f;
      float d0 = 0.0f;
      memcpy(ptr, &ang0, 4); ptr += 4;
      memcpy(ptr, &d0, 4); ptr += 4;
      continue;
    }
    float angle_rad = pt->angle * (PI / 180.0f);
    float dist_m = pt->distance / 1000.0f; // LD06 mm -> m
    memcpy(ptr, &angle_rad, 4); ptr += 4;
    memcpy(ptr, &dist_m, 4); ptr += 4;
  }

  // Update legacy diagnostic counters
  g_scan_packet_count++;
  g_scan_point_count = pcount;

  // Increment telemetry-sent diagnostic counter immediately before send
  scan_telemetry_sent_count++;

  send_frame(1, 0x05, payload, payload_len);
  free(payload);
}

// Diagnostic packet: layout (little-endian):
// scan_ready_count (u32), scan_telemetry_call_count (u32), scan_telemetry_sent_count (u32), last_scan_point_count (u16)
void telemetry_send_diag_scan() {
  uint8_t payload[14];
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

  send_frame(1, 0x06, payload, sizeof(payload));
}
