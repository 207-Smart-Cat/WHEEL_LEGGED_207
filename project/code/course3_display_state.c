#include "course3_display_state.h"

#include <math.h>

#define COURSE3_VEHICLE_MODE (3U)
#define COURSE3_WAYPOINT_NORMAL (0U)
#define COURSE3_WAYPOINT_BRIDGE (3U)
#define COURSE3_WAYPOINT_JUMP   (4U)
#define COURSE3_WAYPOINT_BUMP   (5U)
#define COURSE3_WAYPOINT_RAMP   (8U)
#define COURSE3_SEARCH_AMPLITUDE_DEG (15.0f)
#define COURSE3_SEARCH_AMPLITUDE_RAD (0.2617993878f)
#define COURSE3_SEARCH_MAX_RATE_RAD_S (0.5f)

const char *Course3DisplayState_Text(uint8_t state)
{
    switch ((Course3DisplayState_t)state)
    {
        case COURSE3_DISPLAY_TRACK_ALIGN: return "TRACK ALIGN";
        case COURSE3_DISPLAY_ACTION:      return "ACTION";
        case COURSE3_DISPLAY_DONE:        return "DONE";
        case COURSE3_DISPLAY_IDLE:
        default:                          return 0;
    }
}

const char *Course3TargetType_Text(uint8_t waypoint_type)
{
    switch (waypoint_type)
    {
        case COURSE3_WAYPOINT_NORMAL: return "NORMAL";
        case COURSE3_WAYPOINT_BRIDGE: return "BRIDGE";
        case COURSE3_WAYPOINT_JUMP:   return "JUMP";
        case COURSE3_WAYPOINT_BUMP:   return "BUMP";
        case COURSE3_WAYPOINT_RAMP:   return "RAMP";
        default:                      return 0;
    }
}

uint8_t Course3Waypoint_RequiresAlign(uint8_t waypoint_type)
{
    return (waypoint_type == COURSE3_WAYPOINT_BRIDGE ||
            waypoint_type == COURSE3_WAYPOINT_JUMP ||
            waypoint_type == COURSE3_WAYPOINT_RAMP) ? 1U : 0U;
}

uint8_t Course3Vision_ShouldEnter(uint8_t vehicle_mode, uint8_t state, uint8_t already_active)
{
    return (vehicle_mode == COURSE3_VEHICLE_MODE &&
            state == COURSE3_DISPLAY_TRACK_ALIGN &&
            !already_active) ? 1U : 0U;
}

uint8_t Course3Vision_ShouldRestore(uint8_t state, uint8_t auto_vision_active)
{
    return (state != COURSE3_DISPLAY_TRACK_ALIGN && auto_vision_active) ? 1U : 0U;
}

float Course3Search_TargetOffsetDeg(uint32_t elapsed_ms)
{
    float omega = COURSE3_SEARCH_MAX_RATE_RAD_S / COURSE3_SEARCH_AMPLITUDE_RAD;
    float time_s = (float)elapsed_ms * 0.001f;

    return COURSE3_SEARCH_AMPLITUDE_DEG * sinf(omega * time_s);
}
