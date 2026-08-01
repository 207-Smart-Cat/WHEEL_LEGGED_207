#ifndef TRIPLE_JUMP_RUNTIME_H
#define TRIPLE_JUMP_RUNTIME_H

#include "zf_common_headfile.h"

extern volatile uint8 triple_jump_runtime_state;
extern volatile uint8 triple_jump_runtime_landings;
extern volatile uint8 triple_jump_runtime_fault;
extern volatile float triple_jump_runtime_distance_m;
extern volatile float triple_jump_runtime_yaw_deg;

void TripleJumpRuntime_Init(void);
void TripleJumpRuntime_Task5ms(void);
void TripleJumpRuntime_ForceStop(void);

#endif
