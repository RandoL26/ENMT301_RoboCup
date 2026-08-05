//************************************
//      telemetry_protocol.h
//************************************

#ifndef TELEMETRY_PROTOCOL_H_
#define TELEMETRY_PROTOCOL_H_

#include <Arduino.h>
#include <stdint.h>

class CH9143Bluetooth;

class TelemetryProtocol {
public:
  static const uint8_t VERSION = 1;
  static const uint8_t PACKET_POSE_PATH = 0x01;
  static const uint8_t PACKET_GRID_KEYFRAME = 0x02;
  static const uint8_t PACKET_GRID_DELTA = 0x03;
  static const uint8_t PACKET_HEARTBEAT = 0x04;

  static const uint16_t MAX_PATH_WAYPOINTS = 64;
  static const uint16_t MAX_GRID_CELLS = 4704;
  static const uint16_t MAX_DELTA_UPDATES = 64;
  static const uint16_t MAX_FRAME_SIZE = 2800;

  struct PathCell {
    uint8_t x;
    uint8_t y;
  };

  struct GridDeltaCell {
    uint16_t idx;
    uint8_t packedCell;
  };

  TelemetryProtocol();

  void begin(CH9143Bluetooth *link);

  bool sendPosePath(uint16_t sequence,
                    float x_m,
                    float y_m,
                    float theta_rad,
                    const PathCell *path,
                    uint16_t path_count);

  bool sendGridKeyframe(uint16_t sequence,
                        uint16_t grid_w,
                        uint16_t grid_h,
                        uint16_t cell_mm,
                        const uint8_t *packed_grid,
                        uint16_t packed_len);

  bool sendGridDelta(uint16_t sequence,
                     const GridDeltaCell *changes,
                     uint16_t change_count);

  bool sendHeartbeat(uint16_t sequence,
                     uint32_t uptime_ms,
                     uint16_t tx_queue_depth,
                     uint16_t dropped_frames);

  static uint8_t packCell(uint8_t occupancy, uint8_t terrain);
  static uint16_t packedGridBytes(uint16_t cell_count);

private:
  CH9143Bluetooth *m_link;

  static const uint8_t SYNC0 = 0xA5;
  static const uint8_t SYNC1 = 0x5A;

  static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);
  bool sendFrame(uint8_t packet_type, uint16_t sequence, const uint8_t *payload, uint16_t payload_len);
};

#endif /* TELEMETRY_PROTOCOL_H_ */
