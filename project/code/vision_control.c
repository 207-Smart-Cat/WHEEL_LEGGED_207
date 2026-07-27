#include "vision_control.h"

#define VISION_DEADBAND_PX       (5)
#define VISION_GAIN_DEG_PER_PX   (0.18f)
#define VISION_FILTER_OLD        (0.20f)
#define VISION_FILTER_NEW        (0.80f)
#define VISION_SIGN              (-1.0f)

void vision_control_reset(VisionControlState_t *state)
{
    state->raw_offset_deg = 0.0f;
    state->filtered_offset_deg = 0.0f;
}

float vision_control_update(VisionControlState_t *state, int lane_error_px)
{
    if (lane_error_px >= -VISION_DEADBAND_PX && lane_error_px <= VISION_DEADBAND_PX)
    {
        lane_error_px = 0;
    }

    state->raw_offset_deg = VISION_SIGN * VISION_GAIN_DEG_PER_PX * (float)lane_error_px;
    state->filtered_offset_deg = VISION_FILTER_OLD * state->filtered_offset_deg +
                                 VISION_FILTER_NEW * state->raw_offset_deg;
    return state->filtered_offset_deg;
}
