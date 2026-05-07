#ifndef JUMP_CONTROL_H
#define JUMP_CONTROL_H

#include "zf_common_headfile.h"

typedef enum
{
    JUMP_FREE = 0,
    JUMP_PREPARE,
    JUMP_BURST,
    JUMP_AIR_RETRACT,
    JUMP_EXE_BUFFER,
    JUMP_RECOVER
} JumpState;

extern volatile JumpState jump_state;
extern volatile uint8 jump_engine_suspend;
extern volatile uint8 jump_encoder_suspend;
extern volatile int jump_stop;
extern volatile int jump_position;

void jump_start(void);
void jump_process_control(float *current_x, float *current_y);
void jump_abort(void);
uint8 jump_is_active(void);
uint8 jump_should_suspend_engine(void);
uint8 jump_should_suspend_encoder(void);

#endif
