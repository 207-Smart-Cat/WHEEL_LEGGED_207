#include "triple_jump_runtime.h"

#include "control.h"
#include "imu.h"
#include "ipc_shared_data.h"
#include "jump_control.h"
#include "param.h"
#include "runtime_status.h"
#include "small_driver_uart_control.h"
#include "triple_jump.h"
#include "vehicle_supervisor.h"

volatile uint8 triple_jump_runtime_state = TRIPLE_JUMP_STANDBY;
volatile uint8 triple_jump_runtime_landings = 0U;
volatile uint8 triple_jump_runtime_fault = 0U;
volatile float triple_jump_runtime_distance_m = 0.0f;
volatile float triple_jump_runtime_yaw_deg = 0.0f;

static TripleJumpContext_t triple_jump_context;
static TripleJumpOutput_t triple_jump_output;
static uint8 triple_jump_initialized = 0U;

static void triple_jump_runtime_publish(void)
{
    triple_jump_runtime_state = (uint8)TripleJump_GetState(&triple_jump_context);
    triple_jump_runtime_landings = TripleJump_GetLandingCount(&triple_jump_context);
    triple_jump_runtime_distance_m = TripleJump_GetSegmentDistance(&triple_jump_context);
    triple_jump_runtime_yaw_deg = TripleJump_GetHeldYaw(&triple_jump_context);
}

static void triple_jump_runtime_stop(uint8 fault)
{
    JumpAction_Abort();
    TripleJump_Stop(&triple_jump_context, &triple_jump_output);
    target_velocity = 0.0f;
    target_angle = IMU_data.filter_result.yaw;
    triple_jump_runtime_fault = fault;
    triple_jump_runtime_publish();
}

void TripleJumpRuntime_Init(void)
{
    TripleJump_Init(&triple_jump_context);
    triple_jump_output.target_speed = 0.0f;
    triple_jump_output.target_yaw_deg = 0.0f;
    triple_jump_output.profile = JUMP_ACTION_PROFILE_FIRST;
    triple_jump_output.hold_yaw = 0U;
    triple_jump_output.start_jump = 0U;
    triple_jump_runtime_fault = 0U;
    triple_jump_initialized = 1U;
    triple_jump_runtime_publish();
}

void TripleJumpRuntime_ForceStop(void)
{
    if (!triple_jump_initialized)
    {
        TripleJumpRuntime_Init();
    }
    triple_jump_runtime_stop(0U);
}

void TripleJumpRuntime_Task5ms(void)
{
    TripleJumpConfig_t requested_config;
    TripleJumpInput_t input;
    JumpActionResult_e action_result;
    uint8 start_requested;
    uint8 stop_requested;

    if (!triple_jump_initialized)
    {
        TripleJumpRuntime_Init();
    }

    action_result = JumpAction_Task5ms(IMU_data.accel[2]);
    start_requested = IPC_Consume_Triple_Jump_Start_Core0(&requested_config);
    stop_requested = IPC_Consume_Triple_Jump_Stop_Core0();

    if (start_requested)
    {
        Runtime_Set_Module_Enabled(RUNTIME_MODULE_NAVIGATION, 0U);
        jump_force_idle();
        triple_jump_runtime_fault = 0U;
        if (!TripleJump_Start(&triple_jump_context,
                              &requested_config,
                              IMU_data.filter_result.yaw))
        {
            triple_jump_runtime_stop(1U);
        }
    }
    if (stop_requested)
    {
        triple_jump_runtime_stop(0U);
        return;
    }

    if (Vehicle_Is_Emergency_Stop())
    {
        triple_jump_runtime_stop(1U);
        return;
    }

    input.left_rpm = (float)motor_value.receive_left_speed_data;
    input.right_rpm = (float)motor_value.receive_right_speed_data;
    input.action_result = action_result;
    TripleJump_Update5ms(&triple_jump_context, &input, &triple_jump_output);

    if (triple_jump_output.start_jump)
    {
        if (!JumpAction_Start(triple_jump_output.profile))
        {
            input.action_result = JUMP_ACTION_RESULT_FAULT;
            TripleJump_Update5ms(&triple_jump_context,
                                 &input,
                                 &triple_jump_output);
            triple_jump_runtime_fault = 1U;
        }
    }

    if (TripleJump_GetState(&triple_jump_context) == TRIPLE_JUMP_FAULT)
    {
        target_velocity = 0.0f;
        target_angle = IMU_data.filter_result.yaw;
        triple_jump_runtime_fault = 1U;
        JumpAction_Abort();
    }
    else
    {
        target_velocity = triple_jump_output.target_speed;
        if (triple_jump_output.hold_yaw)
        {
            target_angle = triple_jump_output.target_yaw_deg;
        }
    }

    triple_jump_runtime_publish();
}
