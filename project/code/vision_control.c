#include "vision_control.h"

#define VISION_DEADBAND_PX       (5)
#define VISION_GAIN_DEG_PER_PX   (0.32f)
#define VISION_D_GAIN_DEG_PER_PX_STEP (0.15f)
#define VISION_FILTER_OLD        (0.20f)
#define VISION_FILTER_NEW        (0.80f)
#define VISION_SIGN              (-1.0f)

void vision_control_reset(VisionControlState_t *state)
{
    state->raw_offset_deg = 0.0f;
    state->filtered_offset_deg = 0.0f;
    state->last_error_px = 0;
    state->has_last_error = 0U;
}

float vision_control_update(VisionControlState_t *state, int lane_error_px)
{
    int error_delta_px = 0;

    if (lane_error_px >= -VISION_DEADBAND_PX && lane_error_px <= VISION_DEADBAND_PX)
    {
        lane_error_px = 0;
    }

    if (state->has_last_error)
    {
        error_delta_px = lane_error_px - state->last_error_px;
    }
    state->last_error_px = lane_error_px;
    state->has_last_error = 1U;

    state->raw_offset_deg = VISION_SIGN *
                            (VISION_GAIN_DEG_PER_PX * (float)lane_error_px +
                             VISION_D_GAIN_DEG_PER_PX_STEP * (float)error_delta_px);
    state->filtered_offset_deg = VISION_FILTER_OLD * state->filtered_offset_deg +
                                 VISION_FILTER_NEW * state->raw_offset_deg;
    return state->filtered_offset_deg;
}

float vision_control_pd_output(float yaw_error_deg,
                               float yaw_rate_dps,
                               float kp,
                               float kd,
                               float output_limit)
{
    float output = kp * yaw_error_deg - kd * yaw_rate_dps;

    if (output > output_limit)
    {
        return output_limit;
    }
    if (output < -output_limit)
    {
        return -output_limit;
    }
    return output;
}
