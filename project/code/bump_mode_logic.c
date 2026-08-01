#include "bump_mode_logic.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#include "course3_tuning.h"

#define BUMP_CONFIG_MAGIC          (0x42554D50UL)
#define BUMP_CONFIG_VERSION        (1U)
#define BUMP_GAIN_SHORT_STEP       (0.01f)
#define BUMP_GAIN_LONG_STEP        (0.10f)
#define BUMP_SPEED_SHORT_STEP      (10.0f)
#define BUMP_SPEED_LONG_STEP       (50.0f)

static float bump_clampf(float value, float minimum, float maximum)
{
    if ((value != value) || (value > FLT_MAX) || (value < -FLT_MAX))
    {
        return minimum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint32_t bump_record_integrity(const BumpConfigRecord_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    const size_t length = offsetof(BumpConfigRecord_t, integrity);
    uint32_t hash = 2166136261UL;
    size_t i;

    for (i = 0U; i < length; ++i)
    {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

BumpConfig_t BumpMode_DefaultConfig(void)
{
    BumpConfig_t config;
    config.pwm_gain = BUMP_PWM_GAIN_DEFAULT;
    config.target_speed = BUMP_TARGET_SPEED_DEFAULT;
    return config;
}

BumpConfig_t BumpMode_SanitizeConfig(BumpConfig_t config)
{
    config.pwm_gain = bump_clampf(config.pwm_gain, BUMP_PWM_GAIN_MIN, BUMP_PWM_GAIN_MAX);
    config.target_speed = bump_clampf(config.target_speed,
                                      BUMP_TARGET_SPEED_MIN,
                                      BUMP_TARGET_SPEED_MAX);
    return config;
}

float BumpMode_AdjustGain(float value, int direction, uint8_t long_press)
{
    float step = long_press ? BUMP_GAIN_LONG_STEP : BUMP_GAIN_SHORT_STEP;
    value += (direction >= 0) ? step : -step;
    return bump_clampf(value, BUMP_PWM_GAIN_MIN, BUMP_PWM_GAIN_MAX);
}

float BumpMode_AdjustTargetSpeed(float value, int direction, uint8_t long_press)
{
    float step = long_press ? BUMP_SPEED_LONG_STEP : BUMP_SPEED_SHORT_STEP;
    value += (direction >= 0) ? step : -step;
    return bump_clampf(value, BUMP_TARGET_SPEED_MIN, BUMP_TARGET_SPEED_MAX);
}

float BumpMode_CurrentSpeed(int16_t raw_left_speed, int16_t raw_right_speed)
{
    return ((float)raw_right_speed - (float)raw_left_speed) * 0.5f;
}

int32_t BumpMode_AssistPwmLimit(void)
{
    return BUMP_ASSIST_PWM_LIMIT;
}

uint8_t BumpMode_ResolveTarget(uint8_t mode_active,
                               uint8_t run_enabled,
                               uint8_t emergency_active,
                               uint8_t safety_ready,
                               float configured_target,
                               float *target_command)
{
    if (target_command == NULL)
    {
        return 0U;
    }

    *target_command = 0.0f;
    if (!mode_active)
    {
        return 0U;
    }

    if (run_enabled && !emergency_active && safety_ready)
    {
        *target_command = bump_clampf(configured_target,
                                     BUMP_TARGET_SPEED_MIN,
                                     BUMP_TARGET_SPEED_MAX);
    }
    return 1U;
}

uint8_t BumpMode_ArbitrateTarget(BumpTargetArbiter_t *arbiter,
                                 uint8_t mode_active,
                                 uint8_t run_enabled,
                                 uint8_t emergency_active,
                                 uint8_t safety_ready,
                                 float configured_target,
                                 float *target_command)
{
    if ((arbiter == NULL) || (target_command == NULL))
    {
        return 0U;
    }

    if (mode_active)
    {
        arbiter->was_active = 1U;
        return BumpMode_ResolveTarget(1U,
                                      run_enabled,
                                      emergency_active,
                                      safety_ready,
                                      configured_target,
                                      target_command);
    }

    if (arbiter->was_active)
    {
        arbiter->was_active = 0U;
        *target_command = 0.0f;
        return 1U;
    }

    return 0U;
}

void BumpMode_ReverseAssistReset(BumpReverseAssistState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->motion_ticks = 0U;
    state->zero_ticks = 0U;
    state->reverse_ticks_remaining = 0U;
}

uint8_t BumpMode_ReverseAssistUpdate(BumpReverseAssistState_t *state,
                                     uint8_t mode_active,
                                     uint8_t run_enabled,
                                     uint8_t safety_ready,
                                     float configured_target,
                                     float current_speed,
                                     float *target_command)
{
    float target;
    float abs_speed;

    if ((state == NULL) || (target_command == NULL))
    {
        return 0U;
    }

    if (!mode_active)
    {
        BumpMode_ReverseAssistReset(state);
        return 0U;
    }

    *target_command = 0.0f;
    if (!run_enabled || !safety_ready)
    {
        BumpMode_ReverseAssistReset(state);
        return 1U;
    }

    target = bump_clampf(configured_target, BUMP_TARGET_SPEED_MIN, BUMP_TARGET_SPEED_MAX);
    abs_speed = fabsf(current_speed);

    if (state->reverse_ticks_remaining > 0U)
    {
        *target_command = -target;
        state->reverse_ticks_remaining--;
        return 1U;
    }

    if (abs_speed > BUMP_REVERSE_MOTION_SPEED)
    {
        if (state->motion_ticks < BUMP_REVERSE_MOTION_TICKS)
        {
            state->motion_ticks++;
        }
        state->zero_ticks = 0U;
    }
    else if (abs_speed < BUMP_REVERSE_ZERO_SPEED)
    {
        if (state->motion_ticks >= BUMP_REVERSE_MOTION_TICKS)
        {
            if (state->zero_ticks < BUMP_REVERSE_ZERO_TICKS)
            {
                state->zero_ticks++;
            }
            if (state->zero_ticks >= BUMP_REVERSE_ZERO_TICKS)
            {
                state->motion_ticks = 0U;
                state->zero_ticks = 0U;
                state->reverse_ticks_remaining = BUMP_REVERSE_HOLD_TICKS;
                *target_command = -target;
                state->reverse_ticks_remaining--;
                return 1U;
            }
        }
    }
    else
    {
        state->zero_ticks = 0U;
    }

    *target_command = target;
    return 1U;
}

void BumpMode_BuildRecord(BumpConfigRecord_t *record, BumpConfig_t config)
{
    if (record == NULL)
    {
        return;
    }

    config = BumpMode_SanitizeConfig(config);
    record->magic = BUMP_CONFIG_MAGIC;
    record->version = BUMP_CONFIG_VERSION;
    record->reserved = 0U;
    record->pwm_gain = config.pwm_gain;
    record->target_speed = config.target_speed;
    record->integrity = bump_record_integrity(record);
}

uint8_t BumpMode_LoadRecord(const BumpConfigRecord_t *record, BumpConfig_t *config)
{
    if (config == NULL)
    {
        return 0U;
    }

    *config = BumpMode_DefaultConfig();
    if ((record == NULL) ||
        (record->magic != BUMP_CONFIG_MAGIC) ||
        (record->version != BUMP_CONFIG_VERSION) ||
        (record->integrity != bump_record_integrity(record)) ||
        (record->pwm_gain != record->pwm_gain) ||
        (record->target_speed != record->target_speed) ||
        (record->pwm_gain < BUMP_PWM_GAIN_MIN) ||
        (record->pwm_gain > BUMP_PWM_GAIN_MAX) ||
        (record->target_speed < BUMP_TARGET_SPEED_MIN) ||
        (record->target_speed > BUMP_TARGET_SPEED_MAX))
    {
        return 0U;
    }

    config->pwm_gain = record->pwm_gain;
    config->target_speed = record->target_speed;
    return 1U;
}
