//************************************
//      telemetry_protocol.cpp
//************************************

#include "telemetry_protocol.h"
#include "ch9143_bluetooth.h"
#include <string.h>

TelemetryProtocol::TelemetryProtocol() : m_link(nullptr) {}

void TelemetryProtocol::begin(CH9143Bluetooth *link) {
  m_link = link;
}

uint8_t TelemetryProtocol::packCell(uint8_t occupancy, uint8_t terrain) {
  return (uint8_t)(((terrain & 0x03u) << 2) | (occupancy & 0x03u));
}

uint16_t TelemetryProtocol::packedGridBytes(uint16_t cell_count) {
  return (uint16_t)((cell_count + 1u) / 2u);
}

uint16_t TelemetryProtocol::crc16_ccitt(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000u) {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool TelemetryProtocol::sendFrame(uint8_t packet_type,
                                  uint16_t sequence,
                                  const uint8_t *payload,
                                  uint16_t payload_len) {
  if (m_link == nullptr || !m_link->isInitialized()) {
    return false;
  }

  if (payload_len + 10u > MAX_FRAME_SIZE) {
    return false;
  }

  uint8_t frame[MAX_FRAME_SIZE];
  uint16_t idx = 0;
  frame[idx++] = SYNC0;
  frame[idx++] = SYNC1;
  frame[idx++] = VERSION;
  frame[idx++] = packet_type;
  frame[idx++] = (uint8_t)(sequence & 0xFFu);
  frame[idx++] = (uint8_t)(sequence >> 8);
  frame[idx++] = (uint8_t)(payload_len & 0xFFu);
  frame[idx++] = (uint8_t)(payload_len >> 8);

  if (payload_len > 0 && payload != nullptr) {
    memcpy(&frame[idx], payload, payload_len);
    idx = (uint16_t)(idx + payload_len);
  }

  const uint16_t crc = crc16_ccitt(&frame[2], (uint16_t)(idx - 2));
  frame[idx++] = (uint8_t)(crc & 0xFFu);
  frame[idx++] = (uint8_t)(crc >> 8);

  m_link->write(frame, idx);
  return true;
}

bool TelemetryProtocol::sendPosePath(uint16_t sequence,
                                     float x_m,
                                     float y_m,
                                     float theta_rad,
                                     const PathCell *path,
                                     uint16_t path_count) {
  if (path_count > MAX_PATH_WAYPOINTS) {
    path_count = MAX_PATH_WAYPOINTS;
  }

  uint8_t payload[16 + MAX_PATH_WAYPOINTS * 2];
  uint16_t idx = 0;

  memcpy(&payload[idx], &x_m, sizeof(float)); idx += sizeof(float);
  memcpy(&payload[idx], &y_m, sizeof(float)); idx += sizeof(float);
  memcpy(&payload[idx], &theta_rad, sizeof(float)); idx += sizeof(float);
  payload[idx++] = (uint8_t)(path_count & 0xFFu);
  payload[idx++] = (uint8_t)(path_count >> 8);

  for (uint16_t i = 0; i < path_count; ++i) {
    payload[idx++] = path[i].x;
    payload[idx++] = path[i].y;
  }

  return sendFrame(PACKET_POSE_PATH, sequence, payload, idx);
}

bool TelemetryProtocol::sendGridKeyframe(uint16_t sequence,
                                         uint16_t grid_w,
                                         uint16_t grid_h,
                                         uint16_t cell_mm,
                                         const uint8_t *packed_grid,
                                         uint16_t packed_len) {
  uint8_t payload[16 + (MAX_GRID_CELLS + 1u) / 2u];
  uint16_t idx = 0;

  memcpy(&payload[idx], &grid_w, sizeof(uint16_t)); idx += sizeof(uint16_t);
  memcpy(&payload[idx], &grid_h, sizeof(uint16_t)); idx += sizeof(uint16_t);
  memcpy(&payload[idx], &cell_mm, sizeof(uint16_t)); idx += sizeof(uint16_t);
  memcpy(&payload[idx], &packed_len, sizeof(uint16_t)); idx += sizeof(uint16_t);

  if (packed_len > 0 && packed_grid != nullptr) {
    memcpy(&payload[idx], packed_grid, packed_len);
    idx = (uint16_t)(idx + packed_len);
  }

  return sendFrame(PACKET_GRID_KEYFRAME, sequence, payload, idx);
}

bool TelemetryProtocol::sendGridDelta(uint16_t sequence,
                                      const GridDeltaCell *changes,
                                      uint16_t change_count) {
  if (change_count > MAX_DELTA_UPDATES) {
    change_count = MAX_DELTA_UPDATES;
  }

  uint8_t payload[2 + MAX_DELTA_UPDATES * 3];
  uint16_t idx = 0;

  memcpy(&payload[idx], &change_count, sizeof(uint16_t)); idx += sizeof(uint16_t);
  for (uint16_t i = 0; i < change_count; ++i) {
    memcpy(&payload[idx], &changes[i].idx, sizeof(uint16_t)); idx += sizeof(uint16_t);
    payload[idx++] = changes[i].packedCell;
  }

  return sendFrame(PACKET_GRID_DELTA, sequence, payload, idx);
}

bool TelemetryProtocol::sendHeartbeat(uint16_t sequence,
                                      uint32_t uptime_ms,
                                      uint16_t tx_queue_depth,
                                      uint16_t dropped_frames) {
  uint8_t payload[8];
  uint16_t idx = 0;
  memcpy(&payload[idx], &uptime_ms, sizeof(uint32_t)); idx += sizeof(uint32_t);
  memcpy(&payload[idx], &tx_queue_depth, sizeof(uint16_t)); idx += sizeof(uint16_t);
  memcpy(&payload[idx], &dropped_frames, sizeof(uint16_t)); idx += sizeof(uint16_t);
  return sendFrame(PACKET_HEARTBEAT, sequence, payload, idx);
}
