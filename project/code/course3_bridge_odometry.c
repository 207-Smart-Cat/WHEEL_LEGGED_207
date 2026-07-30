#include "course3_bridge_odometry.h"

#include <math.h>

#define COURSE3_BRIDGE_PI (3.1415926535898f)

float Course3Bridge_ComputeLength(float start_x, float start_y,
                                  float end_x, float end_y)
{
    float delta_x = end_x - start_x;
    float delta_y = end_y - start_y;

    return sqrtf(delta_x * delta_x + delta_y * delta_y);
}

float Course3Bridge_MinWheelSpeed(float left_mps, float right_mps)
{
    float left_speed = fabsf(left_mps);
    float right_speed = fabsf(right_mps);

    return (left_speed < right_speed) ? left_speed : right_speed;
}

void Course3BridgeOdometry_Begin(Course3BridgeOdometry_t *odometry,
                                 float fixed_yaw_deg,
                                 float start_x, float start_y,
                                 float end_x, float end_y)
{
    odometry->fixed_yaw_deg = fixed_yaw_deg;
    odometry->target_distance_m = Course3Bridge_ComputeLength(start_x, start_y, end_x, end_y);
    odometry->travelled_distance_m = 0.0f;
    odometry->completed = 0U;
}

uint8 Course3BridgeOdometry_Update(Course3BridgeOdometry_t *odometry,
                                   float left_mps, float right_mps,
                                   float dt,
                                   float *dx, float *dy)
{
    float distance = Course3Bridge_MinWheelSpeed(left_mps, right_mps) * dt;
    float remaining_distance;
    float yaw_rad = odometry->fixed_yaw_deg * COURSE3_BRIDGE_PI / 180.0f;

    if (odometry->completed)
    {
        *dx = 0.0f;
        *dy = 0.0f;
        return 1U;
    }

    remaining_distance = odometry->target_distance_m - odometry->travelled_distance_m;
    if (distance > remaining_distance)
    {
        distance = remaining_distance;
    }

    *dx = distance * cosf(yaw_rad);
    *dy = distance * sinf(yaw_rad);
    odometry->travelled_distance_m += distance;

    if (odometry->travelled_distance_m >= odometry->target_distance_m)
    {
        odometry->completed = 1U;
    }

    return odometry->completed;
}

void Course3TravelMeter_Begin(Course3TravelMeter_t *meter, float target_distance_m)
{
    meter->target_distance_m = (target_distance_m > 0.0f) ? target_distance_m : 0.0f;
    meter->travelled_distance_m = 0.0f;
    meter->completed = (meter->target_distance_m <= 0.0f) ? 1U : 0U;
}

uint8 Course3TravelMeter_Update(Course3TravelMeter_t *meter,
                                float left_mps, float right_mps,
                                float dt)
{
    float distance;

    if (meter->completed)
    {
        return 1U;
    }

    distance = Course3Bridge_MinWheelSpeed(left_mps, right_mps) * dt;
    if (distance > meter->target_distance_m - meter->travelled_distance_m)
    {
        distance = meter->target_distance_m - meter->travelled_distance_m;
    }
    meter->travelled_distance_m += distance;
    if (meter->travelled_distance_m >= meter->target_distance_m)
    {
        meter->completed = 1U;
    }
    return meter->completed;
}
