//************************************
//       ToFGeometry.h
//************************************
// Shared ToF sensor geometry utilities
// Provides consistent coordinate transforms for:
// - POI detection
// - ToF coverage mapping
// - Search planner evaluation

#ifndef TOF_GEOMETRY_H
#define TOF_GEOMETRY_H

#include <stdint.h>
#include "MappingNav.h"
#include "POIDetector.h"

//====================================
// TOF GEOMETRY CALCULATOR
//====================================
// Centralizes ToF sensor coordinate transforms to ensure
// consistency across all modules using the sensor

class ToFGeometry {
public:
    // Calculate ToF sensor origin position in world frame
    // Inputs:
    //   robot_pose: authoritative MappingNav pose (metres and radians)
    //   tof_extrinsics: sensor mount offset and yaw
    // Outputs:
    //   sensor_x_m, sensor_y_m: world position of sensor
    //   sensor_yaw_rad: world heading of sensor beam
    static void getSensorWorldFrame(
        const MappingNav::Pose2D &robot_pose,
        const ToFExtrinsics &tof_extrinsics,
        float &sensor_x_m,
        float &sensor_y_m,
        float &sensor_yaw_rad
    );
    
    // Calculate where a ToF measurement hits in world coordinates
    // Inputs:
    //   sensor_x_m, sensor_y_m: sensor position
    //   sensor_yaw_rad: sensor beam direction
    //   range_m: ToF measurement distance
    // Outputs:
    //   hit_x_m, hit_y_m: world hit position
    static void getToFHitPosition(
        float sensor_x_m,
        float sensor_y_m,
        float sensor_yaw_rad,
        float range_m,
        float &hit_x_m,
        float &hit_y_m
    );
};

#endif // TOF_GEOMETRY_H
