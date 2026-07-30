#include "vision_control.h"
#include "course3_tuning.h"

static float vision_constrain(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static float vision_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

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
    state->raw_offset_deg = vision_constrain(state->raw_offset_deg,
                                             -VISION_MAX_ANGLE_OFFSET_DEG,
                                             VISION_MAX_ANGLE_OFFSET_DEG);
    state->filtered_offset_deg = state->raw_offset_deg;
    return state->raw_offset_deg;
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

void vision_turn_control_reset(VisionTurnControlState_t *state)
{
    if (state == 0)
    {
        return;
    }

    state->integral_deg_s = 0.0f;
    state->last_error_deg = 0.0f;
    state->p_output = 0.0f;
    state->i_output = 0.0f;
    state->d_output = 0.0f;
    state->output = 0.0f;
    state->has_last_error = 0U;
}

float vision_turn_control_update(VisionTurnControlState_t *state,
                                 float yaw_error_deg,
                                 float vision_yaw_rate_dps,
                                 float dt_s,
                                 unsigned char control_valid)
{
    float error_abs;
    unsigned char error_reversed;

    if (state == 0)
    {
        return 0.0f;
    }

    if (!control_valid)
    {
        vision_turn_control_reset(state);
        return 0.0f;
    }

    state->p_output = COURSE3_VISION_DIRECTION_P * yaw_error_deg;
    state->d_output = -COURSE3_VISION_DIRECTION_D * vision_yaw_rate_dps;
    error_abs = vision_abs(yaw_error_deg);
    error_reversed = (state->has_last_error &&
                      yaw_error_deg * state->last_error_deg < 0.0f) ? 1U : 0U;

    if (COURSE3_VISION_DIRECTION_I == 0.0f ||
        dt_s <= 0.0f ||
        error_reversed ||
        error_abs < COURSE3_VISION_I_ERROR_MIN_DEG ||
        error_abs > COURSE3_VISION_I_ERROR_MAX_DEG ||
        vision_abs(vision_yaw_rate_dps) > COURSE3_VISION_I_YAW_RATE_MAX_DPS)
    {
        state->integral_deg_s = 0.0f;
    }
    else
    {
        float integral_limit = COURSE3_VISION_I_OUTPUT_LIMIT /
                               vision_abs(COURSE3_VISION_DIRECTION_I);
        float candidate_integral = vision_constrain(state->integral_deg_s +
                                                     yaw_error_deg * dt_s,
                                                     -integral_limit,
                                                     integral_limit);
        float candidate_i_output = COURSE3_VISION_DIRECTION_I * candidate_integral;
        float candidate_output = state->p_output + candidate_i_output + state->d_output;

        if (!((candidate_output > COURSE3_VISION_TURN_PWM_LIMIT && yaw_error_deg > 0.0f) ||
              (candidate_output < -COURSE3_VISION_TURN_PWM_LIMIT && yaw_error_deg < 0.0f)))
        {
            state->integral_deg_s = candidate_integral;
        }
    }

    state->i_output = vision_constrain(COURSE3_VISION_DIRECTION_I * state->integral_deg_s,
                                       -COURSE3_VISION_I_OUTPUT_LIMIT,
                                       COURSE3_VISION_I_OUTPUT_LIMIT);
    state->output = vision_constrain(state->p_output + state->i_output + state->d_output,
                                     -COURSE3_VISION_TURN_PWM_LIMIT,
                                     COURSE3_VISION_TURN_PWM_LIMIT);
    state->last_error_deg = yaw_error_deg;
    state->has_last_error = 1U;
    return state->output;
}
