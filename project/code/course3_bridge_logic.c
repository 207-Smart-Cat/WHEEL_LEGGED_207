#include "course3_bridge_logic.h"
#include "navigation_tracking.h"

#define COURSE3_BRIDGE_VEHICLE_MODE (3U)

uint8 Course3Bridge_ShouldQueueAction(uint8 vehicle_mode,
                                      uint8 waypoint_type,
                                      uint16 action_cmd)
{
    return (vehicle_mode == COURSE3_BRIDGE_VEHICLE_MODE &&
            waypoint_type == WP_TYPE_BRIDGE &&
            action_cmd == NAVI_BRIDGE_ACTION_START) ? 1U : 0U;
}
