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

    assert(float_near(BumpMode_AdjustGain(0.32f, 1, 0U), 0.33f));
    assert(float_near(BumpMode_AdjustGain(0.32f, -1, 1U), 0.22f));
    assert(float_near(BumpMode_AdjustGain(1.99f, 1, 1U), 2.00f));
    assert(float_near(BumpMode_AdjustGain(0.01f, -1, 1U), 0.00f));

    assert(float_near(BumpMode_AdjustTargetSpeed(314.0f, 1, 0U), 324.0f));
    assert(float_near(BumpMode_AdjustTargetSpeed(314.0f, -1, 1U), 264.0f));
    assert(float_near(BumpMode_AdjustTargetSpeed(790.0f, 1, 1U), 800.0f));
    assert(float_near(BumpMode_AdjustTargetSpeed(10.0f, -1, 1U), 0.0f));
}

static void test_sanitization_and_current_speed(void)
{
    BumpConfig_t invalid = {-0.5f, 900.0f};
    BumpConfig_t sanitized = BumpMode_SanitizeConfig(invalid);

    assert(float_near(sanitized.pwm_gain, 0.0f));
    assert(float_near(sanitized.target_speed, 800.0f));
    assert(float_near(BumpMode_CurrentSpeed(-220, 180), 200.0f));
    assert(float_near(BumpMode_CurrentSpeed(100, -100), -100.0f));
}

static void test_target_ownership_is_fail_safe(void)
{
    float command = -1.0f;

    assert(BumpMode_ResolveTarget(1U, 1U, 0U, 1U, 300.0f, &command) == 1U);
    assert(float_near(command, 300.0f));

    assert(BumpMode_ResolveTarget(1U, 0U, 0U, 1U, 300.0f, &command) == 1U);
    assert(float_near(command, 0.0f));

    assert(BumpMode_ResolveTarget(1U, 1U, 1U, 1U, 300.0f, &command) == 1U);
    assert(float_near(command, 0.0f));

    assert(BumpMode_ResolveTarget(1U, 1U, 0U, 0U, 300.0f, &command) == 1U);
    assert(float_near(command, 0.0f));

    assert(BumpMode_ResolveTarget(0U, 1U, 0U, 1U, 300.0f, &command) == 0U);
    assert(float_near(command, 0.0f));
}

static void test_target_arbiter_reasserts_priority_and_releases_at_zero(void)
{
    BumpTargetArbiter_t arbiter = {0U};
    float command = 777.0f;

    assert(BumpMode_ArbitrateTarget(&arbiter, 1U, 0U, 0U, 1U, 300.0f, &command) == 1U);
    assert(float_near(command, 0.0f));

    command = 777.0f; /* Simulate an intervening navigation write. */
    assert(BumpMode_ArbitrateTarget(&arbiter, 1U, 1U, 0U, 1U, 300.0f, &command) == 1U);
    assert(float_near(command, 300.0f));

    command = 777.0f;
    assert(BumpMode_ArbitrateTarget(&arbiter, 0U, 0U, 0U, 1U, 300.0f, &command) == 1U);
    assert(float_near(command, 0.0f));

    command = 777.0f;
    assert(BumpMode_ArbitrateTarget(&arbiter, 0U, 0U, 0U, 1U, 300.0f, &command) == 0U);
    assert(float_near(command, 777.0f));
}

static void test_record_round_trip_and_fallback(void)
{
    BumpConfig_t source = {0.47f, 520.0f};
    BumpConfig_t loaded = {0.0f, 0.0f};
    BumpConfigRecord_t record;

    BumpMode_BuildRecord(&record, source);
    assert(BumpMode_LoadRecord(&record, &loaded) == 1U);
    assert(float_near(loaded.pwm_gain, 0.47f));
    assert(float_near(loaded.target_speed, 520.0f));

    record.integrity ^= 0x00010000UL;
    loaded.pwm_gain = 0.9f;
    loaded.target_speed = 700.0f;
    assert(BumpMode_LoadRecord(&record, &loaded) == 0U);
    assert(float_near(loaded.pwm_gain, 0.32f));
    assert(float_near(loaded.target_speed, 314.0f));

    memset(&record, 0xFF, sizeof(record));
    assert(BumpMode_LoadRecord(&record, &loaded) == 0U);
    assert(float_near(loaded.pwm_gain, 0.32f));
    assert(float_near(loaded.target_speed, 314.0f));
}

static void test_reverse_assist_requires_prior_motion_and_recovers(void)
{
    BumpReverseAssistState_t state;
    float command = 0.0f;
    uint16_t i;

    BumpMode_ReverseAssistReset(&state);

    for (i = 0U; i < 120U; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U, 314.0f, 0.0f, &command) == 1U);
        assert(float_near(command, 314.0f));
    }

    for (i = 0U; i < 150U; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U, 314.0f, 16.0f, &command) == 1U);
        assert(float_near(command, 314.0f));
    }

    for (i = 0U; i < 99U; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U, 314.0f, 2.0f, &command) == 1U);
        assert(float_near(command, 314.0f));
    }

    assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U, 314.0f, 2.0f, &command) == 1U);
    assert(float_near(command, -314.0f));

    for (i = 0U; i < 499U; ++i)
    {
        assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U, 314.0f, 0.0f, &command) == 1U);
        assert(float_near(command, -314.0f));
    }

    assert(BumpMode_ReverseAssistUpdate(&state, 1U, 1U, 1U, 314.0f, 0.0f, &command) == 1U);
    assert(float_near(command, 314.0f));

    assert(BumpMode_ReverseAssistUpdate(&state, 1U, 0U, 1U, 314.0f, 0.0f, &command) == 1U);
    assert(float_near(command, 0.0f));
}

int main(void)
{
    test_defaults_and_adjustments();
    test_sanitization_and_current_speed();
    test_target_ownership_is_fail_safe();
    test_target_arbiter_reasserts_priority_and_releases_at_zero();
    test_record_round_trip_and_fallback();
    test_reverse_assist_requires_prior_motion_and_recovers();
    return 0;
}
