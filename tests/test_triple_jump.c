#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "triple_jump.h"

static TripleJumpConfig_t valid_config(void)
{
    TripleJumpConfig_t config = {0.10f, 0.05f, 0.05f, 200.0f};
    return config;
}

static void drive_until_jump(TripleJumpContext_t *context,
                             TripleJumpInput_t *input,
                             TripleJumpOutput_t *output)
{
    uint32_t i;

    input->left_rpm = -120.0f;
    input->right_rpm = 120.0f;
    input->action_result = JUMP_ACTION_RESULT_NONE;
    for (i = 0U; i < 500U; ++i)
    {
        TripleJump_Update5ms(context, input, output);
        if (output->start_jump != 0U)
        {
            return;
        }
    }
    assert(!"jump distance was never reached");
}

static void report_action(TripleJumpContext_t *context,
                          TripleJumpInput_t *input,
                          TripleJumpOutput_t *output,
                          JumpActionResult_e result)
{
    input->action_result = result;
    TripleJump_Update5ms(context, input, output);
    input->action_result = JUMP_ACTION_RESULT_NONE;
}

static void test_config_validation(void)
{
    TripleJumpConfig_t config = valid_config();

    assert(TripleJump_ConfigIsValid(&config));
    config.x1_m = 1.01f;
    assert(!TripleJump_ConfigIsValid(&config));
    config = valid_config();
    config.x2_m = 0.21f;
    assert(!TripleJump_ConfigIsValid(&config));
    config = valid_config();
    config.x3_m = -0.01f;
    assert(!TripleJump_ConfigIsValid(&config));
    config = valid_config();
    config.speed = 301.0f;
    assert(!TripleJump_ConfigIsValid(&config));
    config = valid_config();
    config.x1_m = NAN;
    assert(!TripleJump_ConfigIsValid(&config));
}

static void test_distance_freezes_only_until_landing(void)
{
    TripleJumpContext_t context;
    TripleJumpInput_t input = {0};
    TripleJumpOutput_t output = {0};
    TripleJumpConfig_t config = valid_config();
    float frozen_distance;
    uint32_t i;

    TripleJump_Init(&context);
    assert(TripleJump_Start(&context, &config, 37.0f));
    assert(fabsf(TripleJump_GetHeldYaw(&context) - 37.0f) < 0.0001f);

    drive_until_jump(&context, &input, &output);
    assert(output.profile == JUMP_ACTION_PROFILE_FIRST);
    assert(TripleJump_GetState(&context) == TRIPLE_JUMP_EXECUTING);
    frozen_distance = TripleJump_GetSegmentDistance(&context);

    input.left_rpm = -3000.0f;
    input.right_rpm = 3000.0f;
    for (i = 0U; i < 100U; ++i)
    {
        TripleJump_Update5ms(&context, &input, &output);
    }
    assert(fabsf(TripleJump_GetSegmentDistance(&context) - frozen_distance) < 0.0001f);

    report_action(&context, &input, &output, JUMP_ACTION_RESULT_LANDED);
    assert(TripleJump_GetLandingCount(&context) == 1U);
    assert(fabsf(TripleJump_GetSegmentDistance(&context)) < 0.0001f);
    assert(TripleJump_GetState(&context) == TRIPLE_JUMP_RECOVERING);

    input.left_rpm = -120.0f;
    input.right_rpm = 120.0f;
    TripleJump_Update5ms(&context, &input, &output);
    assert(TripleJump_GetSegmentDistance(&context) > 0.0f);
    assert(output.target_speed == config.speed);

    report_action(&context, &input, &output, JUMP_ACTION_RESULT_COMPLETE);
    assert(TripleJump_GetState(&context) == TRIPLE_JUMP_DRIVING);
}

static void test_three_landings_finish_in_standby(void)
{
    TripleJumpContext_t context;
    TripleJumpInput_t input = {0};
    TripleJumpOutput_t output = {0};
    TripleJumpConfig_t config = {0.0f, 0.0f, 0.0f, 180.0f};
    uint8_t jump_number;

    TripleJump_Init(&context);
    assert(TripleJump_Start(&context, &config, -179.0f));

    for (jump_number = 1U; jump_number <= 3U; ++jump_number)
    {
        TripleJump_Update5ms(&context, &input, &output);
        assert(output.start_jump != 0U);
        assert(output.profile == ((jump_number == 1U) ?
                                  JUMP_ACTION_PROFILE_FIRST :
                                  JUMP_ACTION_PROFILE_FOLLOWUP));
        report_action(&context, &input, &output, JUMP_ACTION_RESULT_LANDED);
        assert(TripleJump_GetLandingCount(&context) == jump_number);
        if (jump_number < 3U)
        {
            assert(output.target_speed == config.speed);
        }
        else
        {
            assert(output.target_speed == 0.0f);
        }
        report_action(&context, &input, &output, JUMP_ACTION_RESULT_COMPLETE);
    }

    assert(TripleJump_GetState(&context) == TRIPLE_JUMP_STANDBY);
    assert(output.target_speed == 0.0f);
    assert(output.hold_yaw == 0U);
}

static void test_fault_and_stop_are_safe(void)
{
    TripleJumpContext_t context;
    TripleJumpInput_t input = {0};
    TripleJumpOutput_t output = {0};
    TripleJumpConfig_t config = valid_config();

    TripleJump_Init(&context);
    assert(TripleJump_Start(&context, &config, 10.0f));
    drive_until_jump(&context, &input, &output);
    report_action(&context, &input, &output, JUMP_ACTION_RESULT_FAULT);
    assert(TripleJump_GetState(&context) == TRIPLE_JUMP_FAULT);
    assert(output.target_speed == 0.0f);

    TripleJump_Stop(&context, &output);
    assert(TripleJump_GetState(&context) == TRIPLE_JUMP_STANDBY);
    assert(output.target_speed == 0.0f);
    assert(output.hold_yaw == 0U);
}

static void test_reverse_motion_never_reduces_distance(void)
{
    TripleJumpContext_t context;
    TripleJumpInput_t input = {0};
    TripleJumpOutput_t output = {0};
    TripleJumpConfig_t config = valid_config();
    float before_reverse;
    uint32_t i;

    TripleJump_Init(&context);
    assert(TripleJump_Start(&context, &config, 0.0f));
    input.left_rpm = -80.0f;
    input.right_rpm = 80.0f;
    for (i = 0U; i < 20U; ++i)
    {
        TripleJump_Update5ms(&context, &input, &output);
    }
    before_reverse = TripleJump_GetSegmentDistance(&context);
    input.left_rpm = 80.0f;
    input.right_rpm = -80.0f;
    for (i = 0U; i < 20U; ++i)
    {
        TripleJump_Update5ms(&context, &input, &output);
    }
    assert(TripleJump_GetSegmentDistance(&context) >= before_reverse);
}

int main(void)
{
    test_config_validation();
    test_distance_freezes_only_until_landing();
    test_three_landings_finish_in_standby();
    test_fault_and_stop_are_safe();
    test_reverse_motion_never_reduces_distance();
    return 0;
}
