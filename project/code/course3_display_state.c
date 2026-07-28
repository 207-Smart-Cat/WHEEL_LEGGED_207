#include "course3_display_state.h"

#define COURSE3_VEHICLE_MODE (3U)

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

uint8_t Course3Vision_ShouldEnter(uint8_t vehicle_mode, uint8_t state, uint8_t already_active)
{
    return (vehicle_mode == COURSE3_VEHICLE_MODE &&
            state != COURSE3_DISPLAY_IDLE &&
            !already_active) ? 1U : 0U;
}

uint8_t Course3Vision_ShouldRestore(uint8_t state, uint8_t auto_vision_active)
{
    return (state == COURSE3_DISPLAY_IDLE && auto_vision_active) ? 1U : 0U;
}
