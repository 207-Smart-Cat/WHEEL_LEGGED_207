#ifndef COURSE3_DISPLAY_STATE_H
#define COURSE3_DISPLAY_STATE_H

#include <stdint.h>

typedef enum
{
    COURSE3_DISPLAY_IDLE = 0U,
    COURSE3_DISPLAY_TRACK_ALIGN,
    COURSE3_DISPLAY_ACTION,
    COURSE3_DISPLAY_DONE
} Course3DisplayState_t;

const char *Course3DisplayState_Text(uint8_t state);
const char *Course3TargetType_Text(uint8_t waypoint_type);
uint8_t Course3Waypoint_RequiresAlign(uint8_t waypoint_type);
uint8_t Course3Vision_ShouldEnter(uint8_t vehicle_mode, uint8_t state, uint8_t already_active);
uint8_t Course3Vision_ShouldRestore(uint8_t state, uint8_t auto_vision_active);
float Course3Search_TargetOffsetDeg(uint32_t elapsed_ms);

#endif
