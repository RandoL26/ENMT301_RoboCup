#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <Arduino.h>
#include <stdint.h>
#include "MappingNav.h"

class LD06; // forward

// Initialize telemetry over a Serial-like Print/Stream. Pass Serial (USB) instance.
void telemetry_init(Stream &serial);

// Periodic sends
void telemetry_send_heartbeat(uint32_t uptime_ms, uint16_t tx_queue_depth, uint16_t dropped_frames);
void telemetry_send_pose_and_path(const MappingNav &nav);
void telemetry_send_grid_keyframe(const MappingNav &nav);
void telemetry_send_downsampled_scan(LD06 &ld06, const MappingNav &nav, uint16_t max_points = 60);

// Temporary diagnostic counters (defined in telemetry.cpp)
extern uint32_t scan_ready_count;
extern uint32_t scan_telemetry_call_count;
extern uint32_t scan_telemetry_sent_count;
extern uint16_t last_scan_point_count;

// Allow LD06 driver to hand completed raw scans to telemetry diagnostics
// for expensive per-scan analysis (keeps heavy work out of the realtime
// telemetry send path). Forward-declare DataPointHandler to avoid header
// ordering issues.
struct DataPointHandler;
void telemetry_update_ld06_diagnostics(struct DataPointHandler *scan);

// Send a diagnostic packet with the above counters (temporary)
void telemetry_send_diag_scan();

#endif // TELEMETRY_H_
