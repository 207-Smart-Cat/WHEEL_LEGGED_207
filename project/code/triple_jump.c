#include "triple_jump.h"

#include <math.h>
#include <stddef.h>

#define TRIPLE_JUMP_X1_MAX_M              (1.00f)
#define TRIPLE_JUMP_XN_MAX_M              (0.20f)
#define TRIPLE_JUMP_SPEED_MAX             (300.0f)
#define TRIPLE_JUMP_WHEEL_DIAMETER_M      (0.045f)
#define TRIPLE_JUMP_PI                    (3.14159265358979323846f)
#define TRIPLE_JUMP_TICK_S                (0.005f)
#define TRIPLE_JUMP_RPM_FILTER_ALPHA      (0.20f)
#define TRIPLE_JUMP_RPM_TO_MPS            \
    ((TRIPLE_JUMP_WHEEL_DIAMETER_M * TRIPLE_JUMP_PI) / 60.0f)

static uint8_t triple_jump_in_range(float value, float maximum)
{
    return (isfinite(value) && value >= 0.0f && value <= maximum) ? 1U : 0U;
}

static void triple_jump_clear_output(TripleJumpOutput_t *output)
{
    output->target_speed = 0.0f;
    output->target_yaw_deg = 0.0f;
    output->profile = JUMP_ACTION_PROFILE_FIRST;
    output->hold_yaw = 0U;
    output->start_jump = 0U;
}

static float triple_jump_target_distance(const TripleJumpContext_t *context)
{
    if (context->landing_count == 0U)
    {
        return context->config.x1_m;
    }
    if (context->landing_count == 1U)
    {
        return context->config.x2_m;
    }
    return context->config.x3_m;
}

static void triple_jump_set_active_output(const TripleJumpContext_t *context,
                                          TripleJumpOutput_t *output)
{
    output->target_yaw_deg = context->held_yaw_deg;
    output->target_speed = 0.0f;
    output->hold_yaw = 1U;
    if (context->landing_count < 3U)
    {
        output->target_speed = context->config.speed;
    }
}

uint8_t TripleJump_ConfigIsValid(const TripleJumpConfig_t *config)
{
    if (config == NULL)
    {
        return 0U;
    }
    return (triple_jump_in_range(config->x1_m, TRIPLE_JUMP_X1_MAX_M) &&
            triple_jump_in_range(config->x2_m, TRIPLE_JUMP_XN_MAX_M) &&
            triple_jump_in_range(config->x3_m, TRIPLE_JUMP_XN_MAX_M) &&
            triple_jump_in_range(config->speed, TRIPLE_JUMP_SPEED_MAX)) ? 1U : 0U;
}

void TripleJump_Init(TripleJumpContext_t *context)
{
    if (context == NULL)
    {
        return;
    }
    context->config.x1_m = 0.0f;
    context->config.x2_m = 0.0f;
    context->config.x3_m = 0.0f;
    context->config.speed = 0.0f;
    context->state = TRIPLE_JUMP_STANDBY;
    context->segment_distance_m = 0.0f;
    context->filtered_forward_rpm = 0.0f;
    context->held_yaw_deg = 0.0f;
    context->landing_count = 0U;
}

uint8_t TripleJump_Start(TripleJumpContext_t *context,
                         const TripleJumpConfig_t *config,
                         float yaw_deg)
{
    if (context == NULL || !TripleJump_ConfigIsValid(config) || !isfinite(yaw_deg))
    {
        return 0U;
    }

    context->config = *config;
    context->state = TRIPLE_JUMP_DRIVING;
    context->segment_distance_m = 0.0f;
    context->filtered_forward_rpm = 0.0f;
    context->held_yaw_deg = yaw_deg;
    context->landing_count = 0U;
    return 1U;
}

void TripleJump_Stop(TripleJumpContext_t *context, TripleJumpOutput_t *output)
{
    if (context != NULL)
    {
        context->state = TRIPLE_JUMP_STANDBY;
        context->segment_distance_m = 0.0f;
        context->filtered_forward_rpm = 0.0f;
    }
    if (output != NULL)
    {
        triple_jump_clear_output(output);
    }
}

void TripleJump_Update5ms(TripleJumpContext_t *context,
                          const TripleJumpInput_t *input,
                          TripleJumpOutput_t *output)
{
    float forward_rpm;

    if (context == NULL || input == NULL || output == NULL)
    {
        return;
    }

    triple_jump_clear_output(output);
    if (context->state == TRIPLE_JUMP_STANDBY ||
        context->state == TRIPLE_JUMP_FAULT)
    {
        return;
    }

    triple_jump_set_active_output(context, output);

    if (input->action_result == JUMP_ACTION_RESULT_FAULT)
    {
        context->state = TRIPLE_JUMP_FAULT;
        triple_jump_clear_output(output);
        return;
    }

    if (context->state == TRIPLE_JUMP_EXECUTING)
    {
        if (input->action_result == JUMP_ACTION_RESULT_LANDED)
        {
            if (context->landing_count < 3U)
            {
                context->landing_count++;
            }
            context->segment_distance_m = 0.0f;
            context->filtered_forward_rpm = 0.0f;
            context->state = TRIPLE_JUMP_RECOVERING;
            triple_jump_set_active_output(context, output);
        }
        return;
    }

    if (context->state == TRIPLE_JUMP_RECOVERING &&
        input->action_result == JUMP_ACTION_RESULT_COMPLETE)
    {
        if (context->landing_count >= 3U)
        {
            TripleJump_Stop(context, output);
            return;
        }
        context->state = TRIPLE_JUMP_DRIVING;
        return;
    }

    forward_rpm = (input->right_rpm - input->left_rpm) * 0.5f;
    if (!isfinite(forward_rpm) || forward_rpm < 0.0f)
    {
        forward_rpm = 0.0f;
    }
    context->filtered_forward_rpm +=
        TRIPLE_JUMP_RPM_FILTER_ALPHA *
        (forward_rpm - context->filtered_forward_rpm);
    context->segment_distance_m += context->filtered_forward_rpm *
                                   TRIPLE_JUMP_RPM_TO_MPS *
                                   TRIPLE_JUMP_TICK_S;

    if (context->state == TRIPLE_JUMP_DRIVING &&
        context->landing_count < 3U &&
        context->segment_distance_m >= triple_jump_target_distance(context))
    {
        context->state = TRIPLE_JUMP_EXECUTING;
        output->start_jump = 1U;
        output->profile = (context->landing_count == 0U) ?
                          JUMP_ACTION_PROFILE_FIRST :
                          JUMP_ACTION_PROFILE_FOLLOWUP;
    }
}

TripleJumpState_e TripleJump_GetState(const TripleJumpContext_t *context)
{
    return (context != NULL) ? context->state : TRIPLE_JUMP_FAULT;
}

float TripleJump_GetSegmentDistance(const TripleJumpContext_t *context)
{
    return (context != NULL) ? context->segment_distance_m : 0.0f;
}

uint8_t TripleJump_GetLandingCount(const TripleJumpContext_t *context)
{
    return (context != NULL) ? context->landing_count : 0U;
}

float TripleJump_GetHeldYaw(const TripleJumpContext_t *context)
{
    return (context != NULL) ? context->held_yaw_deg : 0.0f;
}
