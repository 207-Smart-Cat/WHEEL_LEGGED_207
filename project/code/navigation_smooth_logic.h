#ifndef NAVIGATION_SMOOTH_LOGIC_H
#define NAVIGATION_SMOOTH_LOGIC_H

#include <stdint.h>

#define NAVI_SMOOTH_REACH_RADIUS_M 0.40f    // 平滑区域半径
#define NAVI_SMOOTH_ZONE_SPEED_LIMIT 700.0f  // 平滑区域内限速
#define NAVI_SMOOTH_POST_ADVANCE_TICKS 3U

uint8_t Navi_Smooth_Should_Advance(float distance,
                                   float smooth_radius,
                                   uint8_t is_terminal_point);

float Navi_Smooth_Resolve_Target_Velocity(float speed_cmd,
                                          float turn_speed_limit,
                                          float smooth_zone_speed_limit,
                                          uint8_t smooth_zone_active);

#endif
