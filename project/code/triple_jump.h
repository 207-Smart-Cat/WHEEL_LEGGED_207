#ifndef TRIPLE_JUMP_H
#define TRIPLE_JUMP_H

#include <stdint.h>

#include "jump_action_types.h"

typedef struct
{
    float x1_m;
    float x2_m;
    float x3_m;
    float speed;
} TripleJumpConfig_t;

typedef enum
{
    TRIPLE_JUMP_STANDBY = 0,
    TRIPLE_JUMP_DRIVING,
    TRIPLE_JUMP_EXECUTING,
    TRIPLE_JUMP_RECOVERING,
    TRIPLE_JUMP_FAULT
} TripleJumpState_e;

typedef struct
{
    float left_rpm;
    float right_rpm;
    JumpActionResult_e action_result;
} TripleJumpInput_t;

typedef struct
{
    float target_speed;
    float target_yaw_deg;
    JumpActionProfile_e profile;
    uint8_t hold_yaw;
    uint8_t start_jump;
} TripleJumpOutput_t;

typedef struct
{
    TripleJumpConfig_t config;
    TripleJumpState_e state;
    float segment_distance_m;
    float filtered_forward_rpm;
    float held_yaw_deg;
    uint8_t landing_count;
} TripleJumpContext_t;

uint8_t TripleJump_ConfigIsValid(const TripleJumpConfig_t *config);
void TripleJump_Init(TripleJumpContext_t *context);
uint8_t TripleJump_Start(TripleJumpContext_t *context,
                         const TripleJumpConfig_t *config,
                         float yaw_deg);
void TripleJump_Stop(TripleJumpContext_t *context, TripleJumpOutput_t *output);
void TripleJump_Update5ms(TripleJumpContext_t *context,
                          const TripleJumpInput_t *input,
                          TripleJumpOutput_t *output);
TripleJumpState_e TripleJump_GetState(const TripleJumpContext_t *context);
float TripleJump_GetSegmentDistance(const TripleJumpContext_t *context);
uint8_t TripleJump_GetLandingCount(const TripleJumpContext_t *context);
float TripleJump_GetHeldYaw(const TripleJumpContext_t *context);

#endif
