#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <Arduino.h>
#include <stdint.h>
#include "MappingNav.h"

// Additional telemetry packets
#define PACKET_STATUS          0x07
#define PACKET_INFLATED_GRID   0x08
#define PACKET_LOCALISATION_DEBUG 0x09

// Start side
enum TelemetryStartSide : uint8_t {
    TELEMETRY_START_LEFT = 0,
    TELEMETRY_START_RIGHT = 1
};

// Send robot/system status.
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
    bool visualiser_path_preview_active
);

void telemetry_send_localisation_debug(
    float encoder_left_delta_m,
    float encoder_right_delta_m,
    float encoder_dtheta_rad,
    float imu_dtheta_rad,
    float lidar_match_score,
    float lidar_correction_accepted,
    const MappingNav::Pose2D &pose,
    float lidar_dx_m, float lidar_dy_m, float lidar_dtheta_rad,
    int32_t encoder_left_count, int32_t encoder_right_count
);

// Send inflated obstacle grid.
void telemetry_send_inflated_grid(const MappingNav &nav);

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
