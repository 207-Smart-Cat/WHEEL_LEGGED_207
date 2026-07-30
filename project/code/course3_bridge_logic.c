#include "course3_bridge_logic.h"
#include "navigation_tracking.h"

#define COURSE3_BRIDGE_VEHICLE_MODE (3U)
#define COURSE3_CH4_LOW_MID_BOUNDARY (592)
#define COURSE3_CH4_MID_HIGH_BOUNDARY (1392)

uint8 Course3Segment_IsPairedType(uint8 waypoint_type)
{
    return (waypoint_type == WP_TYPE_BRIDGE ||
            waypoint_type == WP_TYPE_BUMP ||
            waypoint_type == WP_TYPE_STAIR_RAMP) ? 1U : 0U;
}

uint8 Course3Segment_RequiresVision(uint8 waypoint_type)
{
    return (waypoint_type == WP_TYPE_BRIDGE ||
            waypoint_type == WP_TYPE_STAIR_RAMP) ? 1U : 0U;
}

uint8 Course3Segment_PointCount(uint8 waypoint_type)
{
    if (Course3Segment_RequiresVision(waypoint_type))
    {
        return 3U;
    }
    return (waypoint_type == WP_TYPE_BUMP) ? 2U : 0U;
}

uint16 Course3Segment_ExpectedAction(uint8 waypoint_type, uint8 ordinal)
{
    if (Course3Segment_RequiresVision(waypoint_type))
    {
        if (ordinal == 0U) return NAVI_VISION_SEGMENT_ACTION_CALIBRATE;
        if (ordinal == 1U) return NAVI_VISION_SEGMENT_ACTION_ENTRY;
        if (ordinal == 2U) return NAVI_VISION_SEGMENT_ACTION_END;
        return 0U;
    }
    if (waypoint_type == WP_TYPE_BUMP)
    {
        if (ordinal == 0U) return NAVI_BUMP_ACTION_START;
        if (ordinal == 1U) return NAVI_BUMP_ACTION_END;
    }
    return 0U;
}

uint8 Course3Segment_IsStartAction(uint8 waypoint_type, uint16 action_cmd)
{
    return (Course3Segment_PointCount(waypoint_type) > 0U &&
            action_cmd == Course3Segment_ExpectedAction(waypoint_type, 0U)) ? 1U : 0U;
}

uint8 Course3Segment_ShouldQueueAction(uint8 vehicle_mode,
                                       uint8 waypoint_type,
                                       uint16 action_cmd)
{
    return (vehicle_mode == COURSE3_BRIDGE_VEHICLE_MODE &&
            Course3Segment_IsPairedType(waypoint_type) &&
            Course3Segment_IsStartAction(waypoint_type, action_cmd)) ? 1U : 0U;
}

uint8 Course3Segment_ShouldApproach(uint8 vehicle_mode,
                                    uint8 waypoint_type,
                                    uint16 action_cmd,
                                    float distance_m)
{
    return (Course3Segment_ShouldQueueAction(vehicle_mode, waypoint_type, action_cmd) &&
            distance_m >= 0.0f &&
            distance_m <= NAVI_COURSE3_APPROACH_DISTANCE) ? 1U : 0U;
}

uint8 Course3Remote_SelectSpecialType(int32 ch4_value)
{
    if (ch4_value < COURSE3_CH4_LOW_MID_BOUNDARY)
    {
        return WP_TYPE_BRIDGE;
    }
    if (ch4_value < COURSE3_CH4_MID_HIGH_BOUNDARY)
    {
        return WP_TYPE_BUMP;
    }
    return WP_TYPE_STAIR_RAMP;
}

float Course3AngleSlew_Step(float current_deg,
                            float desired_deg,
                            float max_step_deg)
{
    float delta;

    while (current_deg > 180.0f) current_deg -= 360.0f;
    while (current_deg < -180.0f) current_deg += 360.0f;
    while (desired_deg > 180.0f) desired_deg -= 360.0f;
    while (desired_deg < -180.0f) desired_deg += 360.0f;

    delta = desired_deg - current_deg;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;

    if (max_step_deg <= 0.0f)
    {
        return current_deg;
    }
    if (delta > max_step_deg) delta = max_step_deg;
    if (delta < -max_step_deg) delta = -max_step_deg;

    current_deg += delta;
    while (current_deg > 180.0f) current_deg -= 360.0f;
    while (current_deg < -180.0f) current_deg += 360.0f;
    return current_deg;
}

uint8 Course3Bridge_ShouldQueueAction(uint8 vehicle_mode,
                                      uint8 waypoint_type,
                                      uint16 action_cmd)
{
    return (vehicle_mode == COURSE3_BRIDGE_VEHICLE_MODE &&
            waypoint_type == WP_TYPE_BRIDGE &&
            action_cmd == NAVI_BRIDGE_ACTION_START) ? 1U : 0U;
}
