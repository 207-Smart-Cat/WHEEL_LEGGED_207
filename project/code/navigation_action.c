#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "ipc_shared_data.h"
#include "param.h"
#include "jump_control.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "vehicle_supervisor.h"
#include "navigation_touch_logic.h"
#include "bumpy_control.h"
#include "runtime_status.h"
#include "bridge_roll_peak.h"
#include "remote.h"

#include <stdlib.h>

// ==================== 动作状态机实例 ====================
ActionFSM_t action_fsm = {FSM_IDLE, 0, 0};
ActionSequence_t action_seq = {0};
uint8_t is_action_busy = 0;  // 0: 循迹控制 1: 动作接管

// ==================== 外部依赖变量 ====================
extern Navi_WayPoint_t point_map[NAVI_POINT_MAX]; 
extern Navi_Controller_t navi_ctrl;
extern float target_velocity; 
extern float target_angle;

// *******************************************************************************
// 动作参数
// ******************************************************************************
//--------------- 排雷旋转 ---------------
#define MINE_ROTATE_TARGET_DEG      1080.0f
#define MINE_ROTATE_LEAD_DEG        70.0f
#define MINE_ROTATE_TIMEOUT_MS      15000U
#define MINE_ROTATE_CCW_CMD         2U
static float mine_rotate_start_yaw = 0.0f;
static int8_t mine_rotate_dir = 1;
static float mine_rotate_target_deg = MINE_ROTATE_TARGET_DEG;
static uint8_t action_done_pending = 0;
static uint16_t action_done_idx = 0;
#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
static uint8_t jump_sequence_done_count = 0;
#endif
static float jump_course_back_yaw = 0.0f;
static NaviJumpTrigger_t jump_active_trigger = NAVI_JUMP_TRIGGER_WAYPOINT;
static float jump_hold_control_yaw = 0.0f;
static float jump_hold_nav_yaw = 0.0f;
static float jump_motion_count = 0.0f;
static float jump_motion_start_x = 0.0f;
static float jump_motion_start_y = 0.0f;
static float jump_prepare_forward_speed = NAVI_JUMP_RUNUP_SPEED;
static NaviJumpTouchLogic_t jump_touch_logic = {0};
static uint8_t jump_touch_inhibit_after_landing = 0;
static uint8_t jump_landing_detected_latched = 0U;
static uint8_t jump_landing_confirm_count = 0U;
static uint8_t remote_jump_active = 0;
static volatile uint8_t remote_jump_request = 0U;
static uint8_t remote_bump_active = 0U;
static volatile uint8_t remote_bump_request = 0U;
static float remote_bump_hold_control_yaw = 0.0f;
static float remote_bump_hold_nav_yaw = 0.0f;
static float remote_bump_post_start_x = 0.0f;
static float remote_bump_post_start_y = 0.0f;
static uint32_t remote_bump_post_elapsed_ms = 0U;
static Course3AlignSamples_t course3_align_samples;
static uint32_t course3_align_last_frame = 0;
static float course3_align_map_yaw = 0.0f;
static uint32_t course3_align_last_valid_ms = 0;
static uint8_t course3_align_searching = 0;
static BridgeRollPeakTracker_t bridge_roll_tracker;
static float bridge_original_leg_y = 0.0f;
static uint8_t bridge_hold_active = 0U;
static uint8_t course3_display_state = COURSE3_DISPLAY_IDLE;
static uint8_t course3_display_done_pending_clear = 0U;

#define COURSE3_ALIGN_SPEED             (70.0f)
#define COURSE3_SEARCH_SPEED            (100.0f)
#define COURSE3_ALIGN_CENTER_PX         (5)
#define COURSE3_ALIGN_LOST_MS           (3000U)
#define COURSE3_BRIDGE_SPEED             (400.0f)
#define COURSE3_BRIDGE_LEG_Y             (0.05f)
#define COURSE3_BRIDGE_HOLD_MS           (1000U)
#define COURSE3_ACTION_DIRECTION_P        (50.0f)
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
static uint8_t jump_pose_update_active = 0;
#endif

uint8_t navigation_jump_is_active(void)
{
    return (action_fsm.state == FSM_JUMP_EXPLORE ||
            action_fsm.state == FSM_JUMP_BACKOFF ||
            action_fsm.state == FSM_JUMP_RUNUP ||
            action_fsm.state == FSM_JUMP_PREPARE ||
            action_fsm.state == FSM_JUMP_TAKEOFF ||
            action_fsm.state == FSM_JUMP_AIRBORNE ||
            action_fsm.state == FSM_JUMP_LANDING ||
            action_fsm.state == FSM_JUMP_NEXT_APPROACH ||
            action_fsm.state == FSM_JUMP_RAMP_DOWN ||
            action_fsm.state == FSM_JUMP_TURN_BACK ||
            action_fsm.state == FSM_JUMP_RAMP_UP ||
            action_fsm.state == FSM_JUMP_STAIR_DOWN) ? 1U : 0U;
}

uint8_t Navi_Action_Servo_Takeover_Active(void)
{
    return (action_fsm.state == FSM_JUMP_PREPARE ||
            action_fsm.state == FSM_JUMP_TAKEOFF ||
            action_fsm.state == FSM_JUMP_AIRBORNE ||
            action_fsm.state == FSM_JUMP_LANDING) ? 1U : 0U;
}

uint8_t Navi_Action_Vision_Align_Active(void)
{
    return (action_fsm.state == FSM_COURSE3_TRACK_ALIGN) ? 1U : 0U;
}

uint8_t Navi_Action_Get_Course3_Display_State(void)
{
    uint8_t state = course3_display_state;

    if (state == COURSE3_DISPLAY_DONE)
    {
        if (course3_display_done_pending_clear)
        {
            course3_display_done_pending_clear = 0U;
        }
        else
        {
            course3_display_state = COURSE3_DISPLAY_IDLE;
        }
    }
    return state;
}

uint8_t Navi_Action_Course3_Execution_Active(void)
{
    return (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3 &&
            navi_ctrl.navi_mode_driver == 1U &&
            navi_ctrl.point_current_idx < navi_ctrl.point_total_count) ? 1U : 0U;
}

uint8_t Navi_Action_Get_Course3_Target_Type(void)
{
    return Navi_Action_Course3_Execution_Active() ?
           (uint8_t)point_map[navi_ctrl.point_current_idx].type : (uint8_t)WP_TYPE_HOME;
}

float Navi_Action_Get_Course3_Target_X(void)
{
    return Navi_Action_Course3_Execution_Active() ? point_map[navi_ctrl.point_current_idx].x : 0.0f;
}

float Navi_Action_Get_Course3_Target_Y(void)
{
    return Navi_Action_Course3_Execution_Active() ? point_map[navi_ctrl.point_current_idx].y : 0.0f;
}

float Navi_Action_Get_Course3_Target_Yaw(void)
{
    return Navi_Action_Course3_Execution_Active() ? point_map[navi_ctrl.point_current_idx].yaw : 0.0f;
}

float Navi_Action_Get_Course3_Error_X(void)
{
    return Navi_Action_Get_Course3_Target_X() - robot_pose.x;
}

float Navi_Action_Get_Course3_Error_Y(void)
{
    return Navi_Action_Get_Course3_Target_Y() - robot_pose.y;
}

float Navi_Action_Get_Course3_Error_Yaw(void)
{
    return Navi_Action_Course3_Execution_Active() ?
           navi_limit_angle180(Navi_Action_Get_Course3_Target_Yaw() - robot_pose.yaw) : 0.0f;
}

float Navi_Action_Get_Course3_Target_Distance(void)
{
    float error_x = Navi_Action_Get_Course3_Error_X();
    float error_y = Navi_Action_Get_Course3_Error_Y();

    return Navi_Action_Course3_Execution_Active() ? sqrtf(error_x * error_x + error_y * error_y) : 0.0f;
}

static uint8_t navi_bump_is_hard_stop_reason(BumpyExitReason_t reason)
{
    return (reason == BUMP_EXIT_EMERGENCY ||
            reason == BUMP_EXIT_SENSOR_INVALID) ? 1U : 0U;
}

/*
 * 完赛优先：BUMP局部动作失败时释放当前节点并允许导航继续。
 * 急停和位姿无效仍由车辆监督层保持停车，本函数不会解除故障。
 */
static void navi_bump_skip_and_complete(uint16_t target_idx,
                                        BumpyExitReason_t reason)
{
    target_velocity = 0.0f;
    if (!core_b_cmd.vision_enabled)
    {
        target_angle = (float)IMU_data.filter_result.yaw;
    }
    Bumpy_Action_Reset();
    action_done_pending = 1U;
    action_done_idx = target_idx;
    if (action_seq.current_ptr < action_seq.total_count &&
        action_seq.list[action_seq.current_ptr].wp_index == target_idx)
    {
        action_seq.current_ptr++;
    }
    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0U;
    action_fsm.is_airborne_expect = 0U;
    is_action_busy = 0U;
    IPC_LOG_Printf("BUMP_ACTION,SKIP_COMPLETE,wp=%u,reason=%u,hard=%u\r\n",
                   (unsigned int)target_idx,
                   (unsigned int)reason,
                   (unsigned int)navi_bump_is_hard_stop_reason(reason));
}

#define NAVI_JUMP_AIRBORNE_SPEED      NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_LANDING_SPEED       (0.0f)
#define NAVI_JUMP_EXPLORE_SPEED       (120.0f)
#define NAVI_JUMP_BACKOFF_SPEED       (-120.0f)
#define NAVI_JUMP_TOUCH_LOW_SPEED     (10)
#define NAVI_JUMP_TOUCH_HIGH_SPEED    (50)
#define NAVI_JUMP_TOUCH_HIGH_SAMPLES  (2U)
#define NAVI_JUMP_TOUCH_LOW_CONFIRM   (2U)
#define NAVI_JUMP_TOUCH_INHIBIT_MIN_MS       (200U)
#define NAVI_JUMP_TOUCH_INHIBIT_FORWARD_M    (0.05f)
#define NAVI_JUMP_BACKOFF_TARGET_M           (0.30f)
#define NAVI_JUMP_TAKEOFF_RESERVE_M           (0.25f)
#define NAVI_JUMP_RUNUP_TARGET_M              (NAVI_JUMP_BACKOFF_TARGET_M - NAVI_JUMP_TAKEOFF_RESERVE_M)
#define NAVI_JUMP_TAKEOFF_SPEED               NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_BURST_PWM                   (1300)
#define NAVI_JUMP_AIR_RETRACT_X               (0.000f)
#define NAVI_JUMP_AIR_RETRACT_Y               (0.025f)
#define NAVI_JUMP_RETRACT_MOVE_MS              (120U)
#define NAVI_JUMP_RETRACT_HOLD_MS              (50U)
#define NAVI_JUMP_RETRACT_MIN_MS               (80U)
#define NAVI_JUMP_RETRACT_TOTAL_MS             (NAVI_JUMP_RETRACT_MOVE_MS + NAVI_JUMP_RETRACT_HOLD_MS)
#define NAVI_JUMP_EXE_BUFFER_X                 (0.000f)
#define NAVI_JUMP_EXE_BUFFER_Y                 (0.030f)
#define NAVI_JUMP_RECOVER_PWM         (420)
#define NAVI_JUMP_PREPARE_RAMP_MS     (260U)
#define NAVI_JUMP_PREPARE_MS          (260U)
#define NAVI_JUMP_BURST_MS            (180U)
#define NAVI_JUMP_RECOVER_MS          (100U)
#define NAVI_JUMP_LANDING_MAX_MS      (150U)
#define NAVI_JUMP_LAND_ACCEL_G        (1.0f)
#define NAVI_JUMP_LAND_DETECT_ARM_MS          (30U)
#define NAVI_JUMP_LAND_CONFIRM_SAMPLES        (2U)
#define NAVI_JUMP_DEBUG_LOG                   (0U)

#define NAVI_REMOTE_BUMP_PROFILE_ID           (1U)
#define NAVI_REMOTE_BUMP_POST_DISTANCE_M      (1.20f)
#define NAVI_REMOTE_BUMP_POST_SPEED           (220.0f)
#define NAVI_REMOTE_BUMP_POST_TIMEOUT_MS      (10000U)

#define NAVI_TRIPLE_JUMP_TOTAL_COUNT      (3U)

#define NAVI_JUMP_RETRACT_PWM          (420)
#define NAVI_JUMP_BUFFER_PWM           (520)

typedef struct
{
    float distance_m;
    float speed;
} NaviJumpFollowupConfig_t;

#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
static const NaviJumpFollowupConfig_t k_jump_followup_config[2] =
{
    {
        NAVI_JUMP_SECOND_APPROACH_M,
        NAVI_JUMP_SECOND_APPROACH_SPEED
    },
    {
        NAVI_JUMP_THIRD_APPROACH_M,
        NAVI_JUMP_THIRD_APPROACH_SPEED
    }
};
#endif

#if (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_TRIPLE_ONLY) && \
    (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
#error "Invalid NAVI_TRIPLE_JUMP_AFTER_MODE"
#endif

#if (NAVI_JUMP_POSE_UPDATE_MODE != 1U) && (NAVI_JUMP_POSE_UPDATE_MODE != 2U)
#error "Invalid NAVI_JUMP_POSE_UPDATE_MODE"
#endif

#if NAVI_JUMP_DEBUG_LOG
static uint8_t navi_jump_debug_index(void)
{
#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
    return jump_sequence_done_count + 1U;
#else
    return 1U;
#endif
}

static void navi_jump_debug_log_event(const char *event)
{
    IPC_LOG_Printf(
        "JUMP,%s,index=%u,timer=%lu,x=%s%d.%02d,y=%s%d.%02d,dist=%s%d.%02d,v=%s%d.%02d,az=%s%d.%02d\r\n",
        event,
        (unsigned int)navi_jump_debug_index(),
        (unsigned long)action_fsm.state_timer_ms,
        F_ARG(robot_pose.x),
        F_ARG(robot_pose.y),
        F_ARG(jump_motion_count),
        F_ARG(target_velocity),
        F_ARG(IMU_data.accel[2]));
}

#define NAVI_JUMP_LOG_EVENT(event) navi_jump_debug_log_event(event)
#else
#define NAVI_JUMP_LOG_EVENT(event) ((void)0)
#endif

static void navi_jump_landing_detection_reset(void)
{
    jump_landing_detected_latched = 0U;
    jump_landing_confirm_count = 0U;
}

static void navi_jump_update_landing_latch(uint32_t elapsed_ms)
{
    if (jump_landing_detected_latched ||
        elapsed_ms < NAVI_JUMP_LAND_DETECT_ARM_MS)
    {
        return;
    }

    if (IMU_data.accel[2] >= (1.5f * NAVI_JUMP_LAND_ACCEL_G))
    {
        if (jump_landing_confirm_count < NAVI_JUMP_LAND_CONFIRM_SAMPLES)
        {
            jump_landing_confirm_count++;
        }

        if (jump_landing_confirm_count >= NAVI_JUMP_LAND_CONFIRM_SAMPLES)
        {
            jump_landing_detected_latched = 1U;
            NAVI_JUMP_LOG_EVENT("LAND_DETECTED");
        }
    }
    else
    {
        jump_landing_confirm_count = 0U;
    }
}

static void navi_jump_motion_reset(void)
{
    jump_motion_count = 0.0f;
    jump_motion_start_x = robot_pose.x;
    jump_motion_start_y = robot_pose.y;
}

static float navi_jump_get_forward_displacement(void)
{
    float dx = robot_pose.x - jump_motion_start_x;
    float dy = robot_pose.y - jump_motion_start_y;
    float yaw_rad = (float)ANGLE_TO_RAD(jump_hold_nav_yaw);

    return dx * cosf(yaw_rad) + dy * sinf(yaw_rad);
}

static void navi_jump_motion_update_backoff(void)
{
    float forward_displacement = navi_jump_get_forward_displacement();

    jump_motion_count = (forward_displacement < 0.0f) ?
                        -forward_displacement : 0.0f;
}

static void navi_jump_motion_update_runup(void)
{
    float forward_displacement = navi_jump_get_forward_displacement();

    jump_motion_count = (forward_displacement > 0.0f) ?
                        forward_displacement : 0.0f;
}

static const NaviJumpFollowupConfig_t *navi_jump_get_followup_config(void)
{
#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
    uint8_t index;

    if (jump_sequence_done_count == 0U)
    {
        return NULL;
    }

    index = jump_sequence_done_count - 1U;
    if (index >= 2U)
    {
        return NULL;
    }

    return &k_jump_followup_config[index];
#else
    return NULL;
#endif
}

static void navi_jump_touch_window_reset(void)
{
    Navi_JumpTouchLogic_Reset(&jump_touch_logic);
}

static void navi_jump_touch_inhibit_reset(void)
{
    jump_touch_inhibit_after_landing = 0;
    navi_jump_touch_window_reset();
}

static uint8_t navi_jump_touch_update(void)
{
    float avg_speed_abs = (fabsf((float)motor_value.receive_left_speed_data) +
                           fabsf((float)motor_value.receive_right_speed_data)) * 0.5f;

    return Navi_JumpTouchLogic_Update(&jump_touch_logic,
                                      avg_speed_abs,
                                      (float)NAVI_JUMP_TOUCH_HIGH_SPEED,
                                      (float)NAVI_JUMP_TOUCH_LOW_SPEED,
                                      NAVI_JUMP_TOUCH_HIGH_SAMPLES,
                                      NAVI_JUMP_TOUCH_LOW_CONFIRM);
}

static void navi_jump_pose_update_begin(void)
{
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
    if (!jump_pose_update_active) {
        Navi_Set_Manual_Update_Mode(1);
        jump_pose_update_active = 1;
    }
#endif
}

static void navi_jump_pose_add_fixed_step(void)
{
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
    if (jump_pose_update_active) {
        Navi_Manual_Add_Pose(NAVI_JUMP_FIXED_FORWARD_M, NAVI_JUMP_FIXED_RIGHT_M, 1);
    }
#endif
}

static void navi_jump_pose_update_end(void)
{
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
    if (jump_pose_update_active) {
        Navi_Set_Manual_Update_Mode(0);
        jump_pose_update_active = 0;
    }
#endif
}

// ============================================================================
// ==================== REMOTE CH6 NAVIGATION JUMP BRIDGE =====================
// ============================================================================
// CH6 trigger entry: reuse navigation_action.c FSM_JUMP_* jump flow.
// The waypoint navigation flow is kept intact; Remote uses this start/tick bridge.
// ============================================================================
uint8_t Navi_Action_Remote_Jump_Active(void)
{
    return remote_jump_active;
}

uint8_t Navi_Action_Remote_Bump_Active(void)
{
    return remote_bump_active;
}

uint8_t Navi_Action_Remote_Test_Active(void)
{
    return (remote_jump_active || remote_bump_active) ? 1U : 0U;
}

static uint8_t navi_jump_is_remote_trigger(void)
{
    return (jump_active_trigger == NAVI_JUMP_TRIGGER_REMOTE) ? 1U : 0U;
}

void Navi_Action_Request_Remote_Jump(void)
{
    remote_jump_request = 1U;
}

void Navi_Action_Request_Remote_Bump(void)
{
    remote_bump_request = 1U;
}

static void navi_action_remote_bump_clear(uint8_t log_fault,
                                          BumpyExitReason_t reason)
{
    target_velocity = 0.0f;
    target_angle = remote_bump_hold_control_yaw;
    remote_bump_active = 0U;
    remote_bump_request = 0U;
    remote_bump_post_elapsed_ms = 0U;
    is_action_busy = 0U;
    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0U;
    action_fsm.is_airborne_expect = 0U;
    Bumpy_Action_Reset();

    if (log_fault)
    {
        IPC_LOG_Printf("BUMP_TEST,FAULT,reason=%u\r\n",
                       (unsigned int)reason);
    }
}

static BumpyExitReason_t navi_action_remote_bump_safety_reason(void)
{
    if (Vehicle_Is_Emergency_Stop())
    {
        return BUMP_EXIT_EMERGENCY;
    }
    if (Remote_GetStatus() != REMOTE_CONNECTED)
    {
        return BUMP_EXIT_REMOTE_LOST;
    }
    if (Remote_GetChannelData(5) <= 1000)
    {
        return BUMP_EXIT_USER_STOP;
    }
    if (!robot_pose.is_valid)
    {
        return BUMP_EXIT_SENSOR_INVALID;
    }
    if (remote_bump_post_elapsed_ms >= NAVI_REMOTE_BUMP_POST_TIMEOUT_MS)
    {
        return BUMP_EXIT_TIMEOUT;
    }
    return BUMP_EXIT_NONE;
}

uint8_t Navi_Action_Start_Remote_Bump(void)
{
    if (Vehicle_Is_Emergency_Stop() ||
        remote_jump_active ||
        remote_bump_active ||
        remote_jump_request ||
        navigation_jump_is_active() ||
        jump_is_active() ||
        Bumpy_Action_Is_Active() ||
        is_action_busy)
    {
        return 0U;
    }

    if (!Bumpy_Action_Start(NAVI_REMOTE_BUMP_PROFILE_ID))
    {
        return 0U;
    }

    remote_bump_active = 1U;
    is_action_busy = 1U;
    action_fsm.state = FSM_BUMP_DETECT;
    action_fsm.state_timer_ms = 0U;
    action_fsm.is_airborne_expect = 0U;
    remote_bump_hold_control_yaw = navi_limit_angle180(target_angle);
    remote_bump_hold_nav_yaw = navi_limit_angle180(robot_pose.yaw);
    remote_bump_post_start_x = robot_pose.x;
    remote_bump_post_start_y = robot_pose.y;
    remote_bump_post_elapsed_ms = 0U;

    IPC_LOG_Printf("BUMP_TEST,START,profile=%u\r\n",
                   (unsigned int)NAVI_REMOTE_BUMP_PROFILE_ID);
    return 1U;
}

void Navi_Action_Process_Remote_Bump_Request_5ms(void)
{
    if (!remote_bump_request)
    {
        return;
    }

    remote_bump_request = 0U;
    IPC_LOG_Printf("BUMP_TEST,REQUEST\r\n");

    if (Vehicle_Is_Emergency_Stop() ||
        Navi_Action_Remote_Test_Active() ||
        navigation_jump_is_active() ||
        jump_is_active() ||
        Bumpy_Action_Is_Active() ||
        is_action_busy)
    {
        return;
    }

    if (!Navi_Action_Start_Remote_Bump())
    {
        target_velocity = 0.0f;
    }
}

uint8_t Navi_Jump_Start(NaviJumpTrigger_t trigger,
                        float control_yaw,
                        float nav_yaw)
{
    if (trigger != NAVI_JUMP_TRIGGER_WAYPOINT &&
        trigger != NAVI_JUMP_TRIGGER_REMOTE)
    {
        return 0U;
    }

    if (remote_jump_active || remote_bump_active || is_action_busy ||
        navigation_jump_is_active() || jump_is_active() ||
        Bumpy_Action_Is_Active())
    {
        return 0U;
    }

    remote_jump_active = (trigger == NAVI_JUMP_TRIGGER_REMOTE) ? 1U : 0U;
    jump_active_trigger = trigger;
    is_action_busy = 1U;
    action_fsm.state = FSM_JUMP_EXPLORE;
    action_fsm.state_timer_ms = 0U;
    action_fsm.is_airborne_expect = 0U;

#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
    jump_sequence_done_count = 0U;
#endif
    jump_course_back_yaw = 0.0f;
    jump_hold_control_yaw = navi_limit_angle180(control_yaw);
    jump_hold_nav_yaw = navi_limit_angle180(nav_yaw);
    jump_prepare_forward_speed = NAVI_JUMP_RUNUP_SPEED;
    navi_jump_motion_reset();
    navi_jump_touch_inhibit_reset();
    navi_jump_landing_detection_reset();
    navi_jump_pose_update_end();

    jump_stop = 0;
    jump_position = 0;
    jump_engine_suspend = 0U;
    target_velocity = NAVI_JUMP_EXPLORE_SPEED;
    target_angle = jump_hold_control_yaw;

    return 1U;
}

uint8_t Navi_Action_Start_Remote_Jump(void)
{
    return Navi_Jump_Start(NAVI_JUMP_TRIGGER_REMOTE,
                           target_angle,
                           robot_pose.yaw);
}

void Navi_Action_Process_Remote_Jump_Request_5ms(void)
{
    if (!remote_jump_request)
    {
        return;
    }

    remote_jump_request = 0U;

    if (Vehicle_Is_Emergency_Stop())
    {
        return;
    }

    if (Navi_Action_Remote_Test_Active() ||
        navigation_jump_is_active() ||
        jump_is_active() ||
        Bumpy_Action_Is_Active() ||
        is_action_busy)
    {
        return;
    }

    (void)Navi_Action_Start_Remote_Jump();
}

static void navi_action_remote_jump_clear(void)
{
    target_velocity = 0.0f;
    remote_jump_active = 0;
    is_action_busy = 0;
    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0;
    action_fsm.is_airborne_expect = 0;
    navi_jump_pose_update_end();
    jump_engine_suspend = 0;
    jump_stop = 0;
    jump_position = 0;
}

static void navi_jump_finish(uint16_t target_idx)
{
    target_velocity = 0.0f;
    target_angle = jump_hold_control_yaw;

    navi_jump_pose_update_end();

    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0U;
    action_fsm.is_airborne_expect = 0U;
    is_action_busy = 0U;
    jump_engine_suspend = 0U;
    jump_position = 0;
    jump_stop = 0;

    if (!navi_jump_is_remote_trigger())
    {
        action_done_pending = 1U;
        action_done_idx = target_idx;

        if (action_seq.current_ptr < action_seq.total_count &&
            action_seq.list[action_seq.current_ptr].wp_index == target_idx)
        {
            action_seq.current_ptr++;
        }
    }
}

// ============================================================================
// ================== END REMOTE CH6 NAVIGATION JUMP BRIDGE ===================
// ============================================================================

// ==============================================================================
// 全局路径预处理：提取需要动作接管的特殊航点
// ==============================================================================
void navi_parse_global_path(void) {
    uint8_t mode = Runtime_Get_Vehicle_Mode();
    action_seq.total_count = 0;
    action_seq.current_ptr = 0;
    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0;
    action_fsm.is_airborne_expect = 0;
    is_action_busy = 0;
    action_done_pending = 0;
    action_done_idx = 0;
    course3_display_state = COURSE3_DISPLAY_IDLE;
    course3_display_done_pending_clear = 0U;
    remote_jump_request = 0U;
    remote_bump_request = 0U;
    remote_bump_active = 0U;
    remote_bump_post_elapsed_ms = 0U;
    jump_active_trigger = NAVI_JUMP_TRIGGER_WAYPOINT;
#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
    jump_sequence_done_count = 0;
#endif
    jump_course_back_yaw = 0.0f;
    jump_hold_control_yaw = 0.0f;
    jump_hold_nav_yaw = 0.0f;
    jump_prepare_forward_speed = NAVI_JUMP_RUNUP_SPEED;
    navi_jump_motion_reset();
    navi_jump_touch_inhibit_reset();
    navi_jump_landing_detection_reset();
    navi_jump_pose_update_end();
    Bumpy_Action_Reset();
    jump_engine_suspend = 0;

    for (int i = 0; i < navi_ctrl.point_total_count; i++) {
        // 科目三单边桥和台阶经过 Track_align -> Action -> Done。
        if (point_map[i].type == WP_TYPE_MINE_SWEEP ||
            point_map[i].type == WP_TYPE_BUMP ||
            (point_map[i].type == WP_TYPE_JUMP && mode != VEHICLE_MODE_COURSE_3) ||
            (mode == VEHICLE_MODE_COURSE_3 &&
             (point_map[i].type == WP_TYPE_BRIDGE || point_map[i].type == WP_TYPE_JUMP)))  {
            if (point_map[i].type == WP_TYPE_JUMP) {
                point_map[i].action_cmd = NAVI_JUMP_ACTION_MODE;
            }
            action_seq.list[action_seq.total_count].wp_index = i;
            action_seq.list[action_seq.total_count].type = point_map[i].type;
            action_seq.total_count++;
            if (action_seq.total_count >= MAX_ACTION_NUM) break;
        }
    }
}

// ==============================================================================
// 异步动作状态机
// ==============================================================================
static void navi_action_fsm_update(uint16_t target_idx, float distance) {
    WayPoint_Type upcoming_type = WP_TYPE_NORMAL;
    if (!Navi_Action_Remote_Test_Active()) {
        upcoming_type = point_map[target_idx].type;
    }
    action_fsm.state_timer_ms += (uint32_t)(ENCODER_DT * 1000.0f);
    // 注意：状态切换时要清零 state_timer_ms。

    switch (action_fsm.state) {
        case FSM_IDLE:
            // 空闲期用于提前预测特殊动作，并恢复通用动作标志。
            action_fsm.is_airborne_expect = 0;
            navi_jump_pose_update_end();
            jump_engine_suspend = 0;
//                is_action_busy = 1;                              // 接管控制权
            
            // ==============================================================================
            // 提前预测区：根据即将到达的特殊航点选择动作分支。
            // ==============================================================================
            if (upcoming_type == WP_TYPE_MINE_SWEEP && navi_isreach_target_point(target_idx))  {                        // 排雷动作
                action_fsm.state = FSM_MINE_APPROACH; 
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;         // 锁定动作接管
            }
            else if (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3 &&
                     (upcoming_type == WP_TYPE_BRIDGE || upcoming_type == WP_TYPE_JUMP) &&
                     navi_isreach_target_point(target_idx))
            {
                Course3Align_Reset(&course3_align_samples);
                course3_align_last_frame = core_b_cmd.vision_frame_seq;
                course3_align_map_yaw = navi_limit_angle180(point_map[target_idx].yaw);
                course3_align_last_valid_ms = 0U;
                course3_align_searching = 0U;
                action_fsm.state = FSM_COURSE3_TRACK_ALIGN;
                action_fsm.state_timer_ms = 0U;
                is_action_busy = 1U;
                course3_display_state = COURSE3_DISPLAY_TRACK_ALIGN;
            }
            else if (upcoming_type == WP_TYPE_JUMP && distance < (DISTANCE_THRESHOLD * 3.0f)) {                        // 跳跃动作
                point_map[target_idx].action_cmd = NAVI_JUMP_ACTION_MODE;
#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_DISABLED)
                action_fsm.state = FSM_MINE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
#elif (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_SINGLE)
                (void)Navi_Jump_Start(NAVI_JUMP_TRIGGER_WAYPOINT,
                                      target_angle,
                                      point_map[target_idx].yaw);
#elif (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
                (void)Navi_Jump_Start(NAVI_JUMP_TRIGGER_WAYPOINT,
                                      target_angle,
                                      point_map[target_idx].yaw);
#else
#error "Invalid NAVI_JUMP_ACTION_MODE"
#endif
            }
            else if (upcoming_type == WP_TYPE_BUMP &&
                     navi_isreach_target_point(target_idx)) {
                if (Bumpy_Action_Start(point_map[target_idx].action_cmd)) {
                    action_fsm.state = FSM_BUMP_DETECT;
                    action_fsm.state_timer_ms = 0U;
                    action_fsm.is_airborne_expect = 0U;
                    is_action_busy = 1U;
                } else {
                    IPC_LOG_Printf("BUMP_ACTION,START_FAILED,wp=%u,profile=%u\r\n",
                                   (unsigned int)target_idx,
                                   (unsigned int)point_map[target_idx].action_cmd);
                    navi_bump_skip_and_complete(target_idx,
                                                BUMP_EXIT_DISABLED);
                }
            }
            else if (upcoming_type == WP_TYPE_BRIDGE && distance < (DISTANCE_THRESHOLD * 8.0f)) {
                action_fsm.state = FSM_BRIDGE_APPROACH;
            }
            else if (upcoming_type == WP_TYPE_CONE_CONE && distance < (DISTANCE_THRESHOLD * 8.0f)) {
                action_fsm.state = FSM_CONE_APPROACH;
            }
            else if (upcoming_type == WP_TYPE_STOP && distance < (DISTANCE_THRESHOLD * 8.0f)) {
                action_fsm.state = FSM_STOP_PARKING;
            }
            else if (upcoming_type == WP_TYPE_NORMAL && distance < (DISTANCE_THRESHOLD * 2.0f)) {
                action_fsm.state = FSM_IDLE;
            }
            break;

        case FSM_COURSE3_TRACK_ALIGN:
        {
            uint8_t new_frame = (core_b_cmd.vision_frame_seq != course3_align_last_frame) ? 1U : 0U;
            is_action_busy = 1U;
            if (new_frame)
            {
                course3_align_last_frame = core_b_cmd.vision_frame_seq;
                if (core_b_cmd.vision_valid)
                {
                    if (course3_align_searching)
                    {
                        Course3Align_Reset(&course3_align_samples);
                        course3_align_searching = 0U;
                    }
                    course3_align_last_valid_ms = action_fsm.state_timer_ms;
                    target_velocity = COURSE3_ALIGN_SPEED;
                    target_angle = navi_limit_angle180(IMU_data.filter_result.yaw + core_b_cmd.vision_angle_offset_deg);
                    if (abs(core_b_cmd.vision_lane_error_px) <= COURSE3_ALIGN_CENTER_PX)
                    {
                        Course3Align_AddSample(&course3_align_samples,
                                               core_b_cmd.vision_lane_error_px,
                                               IMU_data.filter_result.yaw);
                    }
                }
            }

            if (!course3_align_searching &&
                (action_fsm.state_timer_ms - course3_align_last_valid_ms >= COURSE3_ALIGN_LOST_MS))
            {
                course3_align_searching = 1U;
                Course3Align_Reset(&course3_align_samples);
            }

            if (course3_align_searching)
            {
                target_velocity = COURSE3_SEARCH_SPEED;
                target_angle = navi_limit_angle180(course3_align_map_yaw +
                               Course3Search_TargetOffsetDeg(action_fsm.state_timer_ms));
            }
            else if (Course3Align_IsComplete(&course3_align_samples))
            {
                target_angle = navi_limit_angle180(Course3Align_ComputeYaw(&course3_align_samples));
                if (point_map[target_idx].type == WP_TYPE_BRIDGE)
                {
                    bridge_original_leg_y = y_current;
                    /* 车身换轴后，导航坐标中的横滚为 -IMU pitch（右倾为正）。 */
                    BridgeRollPeak_Reset(&bridge_roll_tracker, -IMU_data.filter_result.pitch);
                    bridge_hold_active = 0U;
                }
                action_fsm.state = FSM_COURSE3_ACTION;
                action_fsm.state_timer_ms = 0U;
                course3_display_state = COURSE3_DISPLAY_ACTION;
            }
            break;
        }

        case FSM_COURSE3_ACTION:
            /* 普通点、单边桥、台阶共用的科目三 Action 方向环参数。 */
            Direction_p = COURSE3_ACTION_DIRECTION_P;
            if (point_map[target_idx].type != WP_TYPE_BRIDGE)
            {
                action_fsm.state = FSM_COURSE3_DONE;
                action_fsm.state_timer_ms = 0U;
                course3_display_state = COURSE3_DISPLAY_DONE;
                course3_display_done_pending_clear = 1U;
                break;
            }

            is_action_busy = 1U;
            target_velocity = COURSE3_BRIDGE_SPEED;
            y_current = COURSE3_BRIDGE_LEG_Y;
            if (!bridge_hold_active)
            {
                if (BridgeRollPeak_Update(&bridge_roll_tracker, -IMU_data.filter_result.pitch))
                {
                    bridge_hold_active = 1U;
                    action_fsm.state_timer_ms = 0U;
                }
            }
            else if (action_fsm.state_timer_ms >= COURSE3_BRIDGE_HOLD_MS)
            {
                action_fsm.state = FSM_COURSE3_DONE;
                action_fsm.state_timer_ms = 0U;
                course3_display_state = COURSE3_DISPLAY_DONE;
                course3_display_done_pending_clear = 1U;
            }
            break;

        case FSM_COURSE3_DONE:
            if (point_map[target_idx].type == WP_TYPE_BRIDGE)
            {
                y_current = bridge_original_leg_y;
                bridge_hold_active = 0U;
            }
            action_done_pending = 1U;
            action_done_idx = target_idx;
            if (action_seq.current_ptr < action_seq.total_count &&
                action_seq.list[action_seq.current_ptr].wp_index == target_idx)
            {
                action_seq.current_ptr++;
            }
            action_fsm.state = FSM_IDLE;
            action_fsm.state_timer_ms = 0U;
            is_action_busy = 0U;
            break;
            
            
        // ------------------ 排雷旋转动作 ------------------
      case FSM_MINE_APPROACH:
            // 初始化排雷/测试旋转动作。
            mine_rotate_start_yaw = (float)robot_pose.cumulative_yaw;
            mine_rotate_dir = (point_map[target_idx].action_cmd == MINE_ROTATE_CCW_CMD) ? -1 : 1;
            mine_rotate_target_deg = (point_map[target_idx].type == WP_TYPE_JUMP) ? 360.0f : MINE_ROTATE_TARGET_DEG;
            is_action_busy = 1;
            target_velocity = 0.0f;
            
            target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * MINE_ROTATE_LEAD_DEG);

            IPC_LOG_Printf("\r\n============= >>> [定点排雷] 已到达旋转点 [%d]，开始%s旋转三圈 <<< =============\r\n",
                   target_idx,
                   mine_rotate_dir > 0 ? "顺时针" : "逆时针");
            
            // 进入持续旋转执行状态。
            action_fsm.state = FSM_MINE_PROCESSING;
            action_fsm.state_timer_ms = 0;
            break;

        case FSM_MINE_PROCESSING:
        {
            float rotated_deg;
            float rotated_abs;
            float remaining_deg;
            float rotate_lead_deg;

            target_velocity = 0.0f;
            
            rotated_deg = ((float)robot_pose.cumulative_yaw - mine_rotate_start_yaw) * mine_rotate_dir;
            rotated_abs = fabsf(rotated_deg);
            remaining_deg = mine_rotate_target_deg - rotated_abs;
            rotate_lead_deg = (remaining_deg < MINE_ROTATE_LEAD_DEG) ? remaining_deg : MINE_ROTATE_LEAD_DEG;

            // 旋转角度达到目标，动作完成。
            if (rotated_abs >= mine_rotate_target_deg)
            {
                IPC_LOG_Printf(" [定点排雷] 三圈旋转完成，退出动作，回到休闲状态。\r\n");
                
                target_velocity = 0.0f;
                target_angle = (float)IMU_data.filter_result.yaw;
                action_done_pending = 1;
                action_done_idx = target_idx;
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                    action_seq.current_ptr++;
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
            }
            else
            {
                target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * rotate_lead_deg);
            }
            // 超时保护：避免动作永久接管。
            if (action_fsm.state_timer_ms > MINE_ROTATE_TIMEOUT_MS)
            {
                IPC_LOG_Printf(" [定点排雷] 三圈旋转超时退出，安全回退到休闲状态。\r\n");
                
                // 超时同样认为动作结束，交还循迹层切点。
                target_velocity = 0.0f;
                target_angle = (float)IMU_data.filter_result.yaw;
                action_done_pending = 1;
                action_done_idx = target_idx;
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                    action_seq.current_ptr++;
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
            }
            break;
        }
        // ------------------ 跳跃动作 ------------------
        case FSM_JUMP_EXPLORE:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_EXPLORE_SPEED;
            target_angle = jump_hold_control_yaw;

            if (jump_touch_inhibit_after_landing)
            {
                navi_jump_touch_window_reset();
                navi_jump_motion_update_runup();
                if (action_fsm.state_timer_ms >= NAVI_JUMP_TOUCH_INHIBIT_MIN_MS &&
                    jump_motion_count >= NAVI_JUMP_TOUCH_INHIBIT_FORWARD_M)
                {
                    jump_touch_inhibit_after_landing = 0U;
                    navi_jump_motion_reset();
                }
                break;
            }

            if (navi_jump_touch_update())
            {
                target_velocity = 0.0f;
                target_angle = jump_hold_control_yaw;
                navi_jump_motion_reset();
                action_fsm.state = FSM_JUMP_BACKOFF;
                action_fsm.state_timer_ms = 0U;
                is_action_busy = 1U;
                NAVI_JUMP_LOG_EVENT("ENTER,BACKOFF");
            }
            break;

        case FSM_JUMP_BACKOFF:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_BACKOFF_SPEED;
            target_angle = jump_hold_control_yaw;
            navi_jump_motion_update_backoff();

            if (jump_motion_count >= NAVI_JUMP_BACKOFF_TARGET_M) {
                navi_jump_motion_reset();
                action_fsm.state = FSM_JUMP_RUNUP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                NAVI_JUMP_LOG_EVENT("ENTER,RUNUP");
            }
            break;

        case FSM_JUMP_RUNUP:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_RUNUP_SPEED;
            target_angle = jump_hold_control_yaw;
            navi_jump_motion_update_runup();

            if (jump_motion_count >= NAVI_JUMP_RUNUP_TARGET_M) {
                jump_prepare_forward_speed = NAVI_JUMP_RUNUP_SPEED;
                navi_jump_pose_update_begin();
                action_fsm.state = FSM_JUMP_PREPARE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                NAVI_JUMP_LOG_EVENT("ENTER,PREPARE");
            }
            break;

        case FSM_JUMP_NEXT_APPROACH:
        {
            const NaviJumpFollowupConfig_t *config;

            is_action_busy = 1U;
            action_fsm.is_airborne_expect = 0U;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0U;

            config = navi_jump_get_followup_config();
            if (config == NULL)
            {
                navi_jump_finish(target_idx);
                break;
            }

            target_velocity = config->speed;
            target_angle = jump_hold_control_yaw;
            navi_jump_motion_update_runup();

            if (jump_motion_count >= config->distance_m)
            {
                jump_prepare_forward_speed = config->speed;
                navi_jump_pose_update_begin();
                action_fsm.state = FSM_JUMP_PREPARE;
                action_fsm.state_timer_ms = 0U;
                is_action_busy = 1U;
                NAVI_JUMP_LOG_EVENT("ENTER,PREPARE");
            }
            break;
        }

        case FSM_JUMP_PREPARE:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = jump_prepare_forward_speed;
            target_angle = jump_hold_control_yaw;
            jump_drive_symmetric_pwm(
                jump_calc_prepare_pwm((uint16)action_fsm.state_timer_ms,
                                      (uint16)NAVI_JUMP_PREPARE_RAMP_MS));

            if (action_fsm.state_timer_ms >= NAVI_JUMP_PREPARE_MS) {
                action_fsm.state = FSM_JUMP_TAKEOFF;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                NAVI_JUMP_LOG_EVENT("ENTER,TAKEOFF");
            }
            break;

        case FSM_JUMP_TAKEOFF:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_TAKEOFF_SPEED;
            target_angle = jump_hold_control_yaw;
            jump_drive_symmetric_pwm(NAVI_JUMP_BURST_PWM);

            if (action_fsm.state_timer_ms >= NAVI_JUMP_BURST_MS) {
                navi_jump_landing_detection_reset();
                action_fsm.state = FSM_JUMP_AIRBORNE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                action_fsm.is_airborne_expect = 1;
                NAVI_JUMP_LOG_EVENT("ENTER,AIRBORNE");
            }
            break;

        case FSM_JUMP_AIRBORNE:
        {
            uint32_t elapsed_ms = action_fsm.state_timer_ms;
            uint32_t buffer_ms = 0U;

            is_action_busy = 1U;
            action_fsm.is_airborne_expect = 1U;
            jump_engine_suspend = 0U;
            target_velocity = NAVI_JUMP_AIRBORNE_SPEED;
            target_angle = jump_hold_control_yaw;

            navi_jump_update_landing_latch(elapsed_ms);

            if (jump_landing_detected_latched &&
                elapsed_ms >= NAVI_JUMP_RETRACT_MIN_MS)
            {
                target_velocity = 0.0f;
                action_fsm.state = FSM_JUMP_LANDING;
                action_fsm.state_timer_ms = 0U;
                action_fsm.is_airborne_expect = 0U;
                NAVI_JUMP_LOG_EVENT("ENTER,LANDING");
                break;
            }

            if (elapsed_ms < NAVI_JUMP_RETRACT_MOVE_MS)
            {                
                jump_drive_symmetric_pwm(NAVI_JUMP_RETRACT_PWM);   // 快速收腿
            }
            else if (elapsed_ms < NAVI_JUMP_RETRACT_TOTAL_MS)
            {                
                jump_drive_symmetric_pwm(NAVI_JUMP_RETRACT_PWM);   // 保持短腿
            }
            else
            {                
                jump_drive_symmetric_pwm(NAVI_JUMP_BUFFER_PWM);    // 放出落地缓冲
                
                buffer_ms = elapsed_ms - NAVI_JUMP_RETRACT_TOTAL_MS;

                if (jump_landing_detected_latched ||
                    buffer_ms >= NAVI_JUMP_LANDING_MAX_MS)
                {
                    target_velocity = 0.0f;
                    action_fsm.state = FSM_JUMP_LANDING;
                    action_fsm.state_timer_ms = 0U;
                    action_fsm.is_airborne_expect = 0U;
                    is_action_busy = 1U;
                    NAVI_JUMP_LOG_EVENT("ENTER,LANDING");
                }
            }
            break;
        }

        case FSM_JUMP_LANDING:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_LANDING_SPEED;
            target_angle = jump_hold_control_yaw;
            jump_drive_symmetric_pwm(NAVI_JUMP_RECOVER_PWM);

            if (action_fsm.state_timer_ms >= NAVI_JUMP_RECOVER_MS) {
                navi_jump_pose_add_fixed_step();
                navi_jump_pose_update_end();
#if (NAVI_JUMP_ACTION_MODE == NAVI_JUMP_ACTION_TRIPLE)
                jump_sequence_done_count++;
                if (jump_sequence_done_count < NAVI_TRIPLE_JUMP_TOTAL_COUNT)
                {
                    navi_jump_motion_reset();
                    navi_jump_touch_window_reset();
                    jump_touch_inhibit_after_landing = 0U;
                    action_fsm.state = FSM_JUMP_NEXT_APPROACH;
                    action_fsm.state_timer_ms = 0U;
                    action_fsm.is_airborne_expect = 0U;
                    is_action_busy = 1U;
                    jump_engine_suspend = 0U;
                    jump_position = 0;
                    jump_stop = 0;
                    target_velocity = 0.0f;
                    target_angle = jump_hold_control_yaw;
                    NAVI_JUMP_LOG_EVENT("ENTER,NEXT_APPROACH");
                    break;
                }
#if (NAVI_TRIPLE_JUMP_AFTER_MODE == NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
                if (!navi_jump_is_remote_trigger())
                {
                    jump_course_back_yaw = navi_limit_angle180(jump_hold_control_yaw + 180.0f);
                    action_fsm.state = FSM_JUMP_RAMP_DOWN;
                    action_fsm.state_timer_ms = 0U;
                    action_fsm.is_airborne_expect = 0U;
                    is_action_busy = 1U;
                    jump_engine_suspend = 0U;
                    jump_position = 0;
                    jump_stop = 0;
                    break;
                }
#endif
#endif
                navi_jump_finish(target_idx);
            }
            break;

        case FSM_JUMP_RAMP_DOWN:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_RAMP_SPEED;
            target_angle = jump_hold_control_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_RAMP_DOWN_MS) {
                action_fsm.state = FSM_JUMP_TURN_BACK;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_TURN_BACK:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = 0.0f;
            target_angle = jump_course_back_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_TURN_BACK_MS) {
                action_fsm.state = FSM_JUMP_RAMP_UP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_RAMP_UP:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_RAMP_SPEED;
            target_angle = jump_course_back_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_RAMP_UP_MS) {
                action_fsm.state = FSM_JUMP_STAIR_DOWN;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_STAIR_DOWN:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_STAIR_SPEED;
            target_angle = jump_course_back_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_STAIR_DOWN_MS) {
                if (!remote_jump_active) {
                    action_done_pending = 1;
                    action_done_idx = target_idx;
                    if (action_seq.current_ptr < action_seq.total_count &&
                        action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                        action_seq.current_ptr++;
                    }
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
                action_fsm.is_airborne_expect = 0;
                jump_engine_suspend = 0;
                jump_position = 0;
                jump_stop = 0;
            }
            break;

        case FSM_BUMP_DETECT:
        {
            BumpyActionResult_t result = Bumpy_Action_Process_5ms();
            is_action_busy = 1U;
            if (result == BUMP_ACTION_RESULT_ENTER_CROSSING) {
                action_fsm.state = FSM_BUMP_CROSSING;
                action_fsm.state_timer_ms = 0U;
            } else if (result == BUMP_ACTION_RESULT_FAULT ||
                       result == BUMP_ACTION_RESULT_IDLE) {
                if (remote_bump_active) {
                    navi_action_remote_bump_clear(
                        1U, Bumpy_Action_Get_Exit_Reason());
                } else {
                    navi_bump_skip_and_complete(
                        target_idx, Bumpy_Action_Get_Exit_Reason());
                }
            } else if (Bumpy_Project_Get_State() ==
                       BUMP_STATE_ENTER_CONFIRM) {
                action_fsm.state = FSM_BUMP_PRE_ENTER;
                action_fsm.state_timer_ms = 0U;
                IPC_LOG_Printf("BUMP_ACTION,PRE_ENTER,speed=200\r\n");
            }
            break;
        }

        case FSM_BUMP_PRE_ENTER:
        {
            BumpyActionResult_t result = Bumpy_Action_Process_5ms();
            is_action_busy = 1U;
            action_fsm.is_airborne_expect = 0U;
            if (result == BUMP_ACTION_RESULT_ENTER_CROSSING) {
                action_fsm.state = FSM_BUMP_CROSSING;
                action_fsm.state_timer_ms = 0U;
            } else if (result == BUMP_ACTION_RESULT_FAULT ||
                       result == BUMP_ACTION_RESULT_IDLE) {
                if (remote_bump_active) {
                    navi_action_remote_bump_clear(
                        1U, Bumpy_Action_Get_Exit_Reason());
                } else {
                    navi_bump_skip_and_complete(
                        target_idx, Bumpy_Action_Get_Exit_Reason());
                }
            } else if (Bumpy_Project_Get_State() !=
                       BUMP_STATE_ENTER_CONFIRM) {
                action_fsm.state = FSM_BUMP_DETECT;
                action_fsm.state_timer_ms = 0U;
                IPC_LOG_Printf("BUMP_ACTION,PRE_ENTER_CANCEL\r\n");
            }
            break;
        }

        case FSM_BUMP_CROSSING:
        {
            BumpyActionResult_t result = Bumpy_Action_Process_5ms();
            is_action_busy = 1U;
            if (result == BUMP_ACTION_RESULT_ENTER_RECOVER) {
                action_fsm.state = FSM_BUMP_RECOVER;
                action_fsm.state_timer_ms = 0U;
            } else if (result == BUMP_ACTION_RESULT_FAULT ||
                       result == BUMP_ACTION_RESULT_IDLE) {
                if (remote_bump_active) {
                    navi_action_remote_bump_clear(
                        1U, Bumpy_Action_Get_Exit_Reason());
                } else {
                    navi_bump_skip_and_complete(
                        target_idx, Bumpy_Action_Get_Exit_Reason());
                }
            }
            break;
        }

        case FSM_BUMP_RECOVER:
        {
            BumpyActionResult_t result = Bumpy_Action_Process_5ms();
            is_action_busy = 1U;
            if (result == BUMP_ACTION_RESULT_DONE) {
                if (remote_bump_active) {
                    remote_bump_post_start_x = robot_pose.x;
                    remote_bump_post_start_y = robot_pose.y;
                    remote_bump_post_elapsed_ms = 0U;
                    remote_bump_hold_control_yaw =
                        navi_limit_angle180(target_angle);
                    remote_bump_hold_nav_yaw =
                        navi_limit_angle180(robot_pose.yaw);
                    action_fsm.state = FSM_BUMP_TEST_POST_DRIVE;
                    action_fsm.state_timer_ms = 0U;
                    action_fsm.is_airborne_expect = 0U;
                    is_action_busy = 1U;
                    IPC_LOG_Printf(
                        "BUMP_TEST,ENTER_POST_DRIVE,distance=1.20\r\n");
                } else {
                    action_done_pending = 1U;
                    action_done_idx = target_idx;
                    if (action_seq.current_ptr < action_seq.total_count &&
                        action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                        action_seq.current_ptr++;
                    }
                    action_fsm.state = FSM_IDLE;
                    action_fsm.state_timer_ms = 0U;
                    action_fsm.is_airborne_expect = 0U;
                    is_action_busy = 0U;
                }
            } else if (result == BUMP_ACTION_RESULT_FAULT ||
                       result == BUMP_ACTION_RESULT_IDLE) {
                if (remote_bump_active) {
                    navi_action_remote_bump_clear(
                        1U, Bumpy_Action_Get_Exit_Reason());
                } else {
                    navi_bump_skip_and_complete(
                        target_idx, Bumpy_Action_Get_Exit_Reason());
                }
            }
            break;
        }

        case FSM_BUMP_TEST_POST_DRIVE:
        {
            float dx;
            float dy;
            float yaw_rad;
            float forward_distance;
            BumpyExitReason_t safety_reason;

            is_action_busy = 1U;
            action_fsm.is_airborne_expect = 0U;
            target_velocity = NAVI_REMOTE_BUMP_POST_SPEED;
            target_angle = remote_bump_hold_control_yaw;

            dx = robot_pose.x - remote_bump_post_start_x;
            dy = robot_pose.y - remote_bump_post_start_y;
            yaw_rad = (float)ANGLE_TO_RAD(remote_bump_hold_nav_yaw);
            forward_distance = dx * cosf(yaw_rad) + dy * sinf(yaw_rad);
            if (forward_distance < 0.0f)
            {
                forward_distance = 0.0f;
            }

            remote_bump_post_elapsed_ms += 5U;
            safety_reason = navi_action_remote_bump_safety_reason();
            if (safety_reason != BUMP_EXIT_NONE)
            {
                navi_action_remote_bump_clear(1U, safety_reason);
            }
            else if (forward_distance >= NAVI_REMOTE_BUMP_POST_DISTANCE_M)
            {
                target_velocity = 0.0f;
                target_angle = remote_bump_hold_control_yaw;
                remote_bump_active = 0U;
                remote_bump_request = 0U;
                remote_bump_post_elapsed_ms = 0U;
                is_action_busy = 0U;
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0U;
                action_fsm.is_airborne_expect = 0U;
                Bumpy_Action_Reset();
                IPC_LOG_Printf("BUMP_TEST,DONE\r\n");
            }
            break;
        }

        // ------------------ 桥梁动作 ------------------
        case FSM_BRIDGE_APPROACH:
            // 预留：接近桥梁时的减速、姿态准备等动作。
                      
            break;

        case FSM_BRIDGE_ON_BOARD:
            // 预留：桥面行驶时的腿长和姿态自适应动作。
                      
            break;

        // ------------------ 绕锥桶动作 ------------------
        case FSM_CONE_APPROACH:
            // 预留：接近锥桶时的减速和转向准备。
                      
            break;

        case FSM_CONE_NAVIGATE:
            // 预留：绕行锥桶时的路径/角度控制。
                      
            break;

        // ------------------ 终点停车 ------------------
        case FSM_STOP_PARKING:
            target_velocity = 0.0f;
            vofa_mode_driver = 0.0f;
            navi_ctrl.navi_mode_driver = 0; 

            static uint8_t end_printed_flag = 0;
            if (!end_printed_flag) {
            IPC_LOG_Printf("=============  >>> [事件] 终点已到达，动作接管并安全停车！ =============\r\n");                  
            end_printed_flag = 1;
            }
            break;

        default:
            action_fsm.state = FSM_IDLE;
            break;
    }
}

uint8_t Navi_Action_Consume_Done(uint16_t curr_idx)
{
    if (action_done_pending && action_done_idx == curr_idx)
    {
        action_done_pending = 0;
        return 1;
    }

    return 0;
}

// ============================================================================
// ==================== REMOTE CH6 NAVIGATION JUMP BRIDGE =====================
// ============================================================================
void Navi_Jump_Task_5ms(void)
{
    if (!remote_jump_active)
    {
        return;
    }

    if (Vehicle_Is_Emergency_Stop())
    {
        navi_action_remote_jump_clear();
        return;
    }

    navi_action_fsm_update(0, 0.0f);

    if (action_fsm.state == FSM_IDLE)
    {
        navi_action_remote_jump_clear();
    }
}

void Navi_Bump_Test_Task_5ms(void)
{
    BumpyExitReason_t safety_reason;

    if (!remote_bump_active)
    {
        return;
    }

    safety_reason = navi_action_remote_bump_safety_reason();
    if (safety_reason != BUMP_EXIT_NONE)
    {
        navi_action_remote_bump_clear(1U, safety_reason);
        return;
    }

    navi_action_fsm_update(0U, 0.0f);

    if (remote_bump_active && action_fsm.state == FSM_IDLE)
    {
        navi_action_remote_bump_clear(
            1U, Bumpy_Action_Get_Exit_Reason());
    }
}

// ============================================================================
// ================== END REMOTE CH6 NAVIGATION JUMP BRIDGE ===================
// ============================================================================


// ==============================================================================
// 动作管理入口：由循迹层周期调用
// ==============================================================================
void Navi_Action_Manager(uint16_t  curr_idx) {
    if (Navi_Action_Remote_Test_Active()) return;

    if (action_seq.total_count == 0 || action_seq.current_ptr >= action_seq.total_count) return;

    uint16_t  target_wp_idx = action_seq.list[action_seq.current_ptr].wp_index;
    
    // 动作接管期间直接更新 FSM，不做普通切点逻辑。
    if (is_action_busy) {
        float real_distance = 0.0;
        float dummy_azimuth = 0.0;
        navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance);
        navi_action_fsm_update(target_wp_idx, real_distance);
        return; 
    }
                                                            
    if (curr_idx != target_wp_idx) {
        return;
    }

    target_wp_idx = action_seq.list[action_seq.current_ptr].wp_index;
    
    // 周期性更新动作 FSM。
        float real_distance = 0.0f;
        float dummy_azimuth = 0.0f;
        if (navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance)) {
            navi_action_fsm_update(target_wp_idx, real_distance);
        }
}



//===========================================================================================================
//================================================动作封装函数=======================================================
//========================================================================================================
