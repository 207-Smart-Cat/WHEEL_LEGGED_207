#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "bump_mode_logic.h"

static int float_near(float actual, float expected)
{
    return fabsf(actual - expected) < 0.0001f;
}

static void test_defaults_and_adjustments(void)
{
    BumpConfig_t config = BumpMode_DefaultConfig();

    assert(float_near(config.pwm_gain, 0.32f));
    assert(float_near(config.target_speed, 314.0f));
    assert(BumpMode_AssistPwmLimit() == 6000);
    assert(float_near(BUMP_DISTANCE_COMPENSATION_M, 1.0f));
    assert(float_near(BumpMode_AdjustGain(1.99f, 1, 1U), 2.00f));
    assert(float_near(BumpMode_AdjustTargetSpeed(790.0f, 1, 1U), 800.0f));
}

static void test_target_ownership(void)
{
    BumpTargetArbiter_t arbiter = {0U};
    float command = 777.0f;

    assert(BumpMode_ArbitrateTarget(&arbiter, 1U, 1U, 0U, 1U,
                                    314.0f, &command) == 1U);
    assert(float_near(command, 314.0f));

    command = 777.0f;
    assert(BumpMode_ArbitrateTarget(&arbiter, 0U, 0U, 0U, 1U,
                                    314.0f, &command) == 1U);
    assert(float_near(command, 0.0f));
}

static void test_record_round_trip(void)
{
    BumpConfig_t source = {0.47f, 520.0f};
    BumpConfig_t loaded = {0.0f, 0.0f};
    BumpConfigRecord_t record;

    BumpMode_BuildRecord(&record, source);
    assert(BumpMode_LoadRecord(&record, &loaded) == 1U);
    assert(float_near(loaded.pwm_gain, 0.47f));
    assert(float_near(loaded.target_speed, 520.0f));

    record.integrity ^= 0x00010000UL;
    assert(BumpMode_LoadRecord(&record, &loaded) == 0U);
    assert(float_near(loaded.pwm_gain, 0.32f));
    assert(float_near(loaded.target_speed, 314.0f));
}

static void test_reverse_assist(void)
{
    BumpReverseAssistState_t state;
    float command = 0.0f;
    uint16_t i;

    BumpMode_ReverseAssistReset(&state);
    for (i = 0U; i < BUMP_REVERSE_MOTION_TICKS; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U,
                                            314.0f, 16.0f, &command) == 1U);
        assert(float_near(command, 314.0f));
    }
    for (i = 0U; i < BUMP_REVERSE_ZERO_TICKS - 1U; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U,
                                            314.0f, 2.0f, &command) == 1U);
        assert(float_near(command, 314.0f));
    }
    assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U,
                                        314.0f, 2.0f, &command) == 1U);
    assert(float_near(command, -314.0f));

    for (i = 0U; i < BUMP_REVERSE_HOLD_TICKS - 1U; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U,
                                            314.0f, 0.0f, &command) == 1U);
        assert(float_near(command, -314.0f));
    }
    assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U,
                                        314.0f, 0.0f, &command) == 1U);
    assert(float_near(command, 314.0f));
}

int main(void)
{
    test_defaults_and_adjustments();
    test_target_ownership();
    test_record_round_trip();
    test_reverse_assist();
    return 0;
}
