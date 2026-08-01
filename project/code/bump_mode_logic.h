#ifndef BUMP_MODE_LOGIC_H
#define BUMP_MODE_LOGIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUMP_PWM_GAIN_DEFAULT      (0.32f)
#define BUMP_PWM_GAIN_MIN          (0.00f)
#define BUMP_PWM_GAIN_MAX          (2.00f)
#define BUMP_TARGET_SPEED_DEFAULT  (314.0f)
#define BUMP_TARGET_SPEED_MIN      (0.0f)
#define BUMP_TARGET_SPEED_MAX      (800.0f)
#define BUMP_ASSIST_PWM_LIMIT      (6000)
#define BUMP_REVERSE_MOTION_SPEED  (15.0f)
#define BUMP_REVERSE_ZERO_SPEED    (3.0f)
#define BUMP_REVERSE_MOTION_TICKS  (150U)
#define BUMP_REVERSE_ZERO_TICKS    (100U)
#define BUMP_REVERSE_HOLD_TICKS    (500U)
#define BUMP_ACTIVE_DIRECTION_P    (95.0f)
#define BUMP_ACTIVE_INTEGRAL_GAIN  (8.0f)
#define BUMP_ACTIVE_PWM_GAIN       (1.5f)
#define BUMP_DISTANCE_COMPENSATION_M (1.0f)

typedef struct
{
    float pwm_gain;
    float target_speed;
} BumpConfig_t;

typedef struct
{
    uint8_t was_active;
} BumpTargetArbiter_t;

typedef struct
{
    uint16_t motion_ticks;
    uint16_t zero_ticks;
    uint16_t reverse_ticks_remaining;
} BumpReverseAssistState_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    float pwm_gain;
    float target_speed;
    uint32_t integrity;
} BumpConfigRecord_t;

BumpConfig_t BumpMode_DefaultConfig(void);
BumpConfig_t BumpMode_SanitizeConfig(BumpConfig_t config);
float BumpMode_AdjustGain(float value, int direction, uint8_t long_press);
float BumpMode_AdjustTargetSpeed(float value, int direction, uint8_t long_press);
float BumpMode_CurrentSpeed(int16_t raw_left_speed, int16_t raw_right_speed);
int32_t BumpMode_AssistPwmLimit(void);
uint8_t BumpMode_ResolveTarget(uint8_t mode_active,
                               uint8_t run_enabled,
                               uint8_t emergency_active,
                               uint8_t safety_ready,
                               float configured_target,
                               float *target_command);
uint8_t BumpMode_ArbitrateTarget(BumpTargetArbiter_t *arbiter,
                                 uint8_t mode_active,
                                 uint8_t run_enabled,
                                 uint8_t emergency_active,
                                 uint8_t safety_ready,
                                 float configured_target,
                                 float *target_command);
void BumpMode_ReverseAssistReset(BumpReverseAssistState_t *state);
uint8_t BumpMode_ReverseAssistUpdate(BumpReverseAssistState_t *state,
                                     uint8_t mode_active,
                                     uint8_t run_enabled,
                                     uint8_t safety_ready,
                                     float configured_target,
                                     float current_speed,
                                     float *target_command);
void BumpMode_BuildRecord(BumpConfigRecord_t *record, BumpConfig_t config);
uint8_t BumpMode_LoadRecord(const BumpConfigRecord_t *record, BumpConfig_t *config);

#ifdef __cplusplus
}
#endif

#endif
