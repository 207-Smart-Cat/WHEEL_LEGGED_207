#ifndef COURSE3_BRIDGE_ODOMETRY_H
#define COURSE3_BRIDGE_ODOMETRY_H

#include "zf_common_typedef.h"

typedef struct
{
    float fixed_yaw_deg;
    float target_distance_m;
    float travelled_distance_m;
    uint8_t completed;
} Course3BridgeOdometry_t;

float Course3Bridge_ComputeLength(float start_x, float start_y,
                                  float end_x, float end_y);
float Course3Bridge_MinWheelSpeed(float left_mps, float right_mps);
void Course3BridgeOdometry_Begin(Course3BridgeOdometry_t *odometry,
                                 float fixed_yaw_deg,
                                 float start_x, float start_y,
                                 float end_x, float end_y);
uint8 Course3BridgeOdometry_Update(Course3BridgeOdometry_t *odometry,
                                   float left_mps, float right_mps,
                                   float dt,
                                   float *dx, float *dy);

#endif
