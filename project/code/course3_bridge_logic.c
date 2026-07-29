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

uint8 Course3Segment_ShouldQueueAction(uint8 vehicle_mode,
                                       uint8 waypoint_type,
                                       uint16 action_cmd)
{
    return (vehicle_mode == COURSE3_BRIDGE_VEHICLE_MODE &&
            Course3Segment_IsPairedType(waypoint_type) &&
            action_cmd == NAVI_SEGMENT_ACTION_START) ? 1U : 0U;
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

uint8 Course3Bridge_ShouldQueueAction(uint8 vehicle_mode,
                                      uint8 waypoint_type,
                                      uint16 action_cmd)
{
    return (vehicle_mode == COURSE3_BRIDGE_VEHICLE_MODE &&
            waypoint_type == WP_TYPE_BRIDGE &&
            action_cmd == NAVI_BRIDGE_ACTION_START) ? 1U : 0U;
}
