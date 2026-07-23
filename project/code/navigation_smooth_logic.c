#include "navigation_smooth_logic.h"

uint8_t Navi_Smooth_Should_Advance(float distance,
                                   float smooth_radius,
                                   uint8_t is_terminal_point)
{
    if (is_terminal_point || smooth_radius <= 0.0f)
    {
        return 0U;
    }

    return (distance <= smooth_radius) ? 1U : 0U;
}

float Navi_Smooth_Resolve_Target_Velocity(float speed_cmd,
                                          float turn_speed_limit,
                                          float smooth_zone_speed_limit,
                                          uint8_t smooth_zone_active)
{
    float target_velocity =
        (speed_cmd < turn_speed_limit) ? speed_cmd : turn_speed_limit;

    if (smooth_zone_active &&
        smooth_zone_speed_limit > 0.0f &&
        smooth_zone_speed_limit < target_velocity)
    {
        target_velocity = smooth_zone_speed_limit;
    }

    return target_velocity;
}
