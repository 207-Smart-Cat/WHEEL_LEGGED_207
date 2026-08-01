#ifndef JUMP_CONTROL_H
#define JUMP_CONTROL_H

#include "zf_common_headfile.h"
#include "jump_action_types.h"

typedef enum
{
    JUMP_FREE = 0,
    JUMP_PREPARE,
    JUMP_BURST,
    JUMP_AIR_RETRACT,
    JUMP_EXE_BUFFER,
    JUMP_RECOVER,
    JUMP_END
} JumpState;

typedef enum
{
    JUMP_BLOCK_NONE = 0,
    JUMP_BLOCK_STARTED,
    JUMP_BLOCK_BUSY,
    JUMP_BLOCK_REMOTE_OFF,
    JUMP_BLOCK_REMOTE_LOST,
    JUMP_BLOCK_REMOTE_STANDBY,
    JUMP_BLOCK_NOT_ARMED,
    JUMP_BLOCK_NO_EDGE
} JumpTriggerBlockReason;

extern volatile JumpState jump_state;
extern volatile uint8 jump_engine_suspend;
extern volatile uint8 jump_encoder_suspend;
extern volatile int jump_stop;
extern volatile int jump_position;
extern volatile uint8 jump_dbg_state;
extern volatile uint16 jump_dbg_elapsed_ms;
extern volatile uint8 jump_dbg_trigger_block_reason;
extern volatile uint32 jump_dbg_trigger_count;

int jump_calc_prepare_pwm(uint16 elapsed_ms, uint16 prepare_ms, int target_pwm);
void jump_drive_symmetric_pwm(int pwm1);
uint8 JumpAction_Start(JumpActionProfile_e profile);
JumpActionResult_e JumpAction_Task5ms(float accel_z_g);
void JumpAction_Abort(void);
uint8 JumpAction_BaselineReady(void);
uint8 jump_start(void);
void jump_process_control(float *current_x, float *current_y);
void jump_abort(void);
void jump_force_idle(void);
uint8 jump_is_active(void);
uint8 jump_should_suspend_engine(void);
uint8 jump_should_suspend_encoder(void);
void jump_set_trigger_block_reason(JumpTriggerBlockReason reason);

#endif
