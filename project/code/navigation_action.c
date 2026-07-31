#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "ipc_shared_data.h"
#include "param.h"
#include "jump_control.h"
#include "imu.h"
#include "small_driver_uart_control.h"
#include "vehicle_supervisor.h"
#include "navigation_touch_logic.h"
#include "runtime_status.h"
#include "bridge_roll_peak.h"
#include "course3_bridge_logic.h"
#include "course3_tuning.h"

#include <stdlib.h>
#include <string.h>

// ==================== 动作状态机实例 ====================
ActionFSM_t action_fsm = {FSM_IDLE, 0, 0};
ActionSequence_t action_seq = {0};
uint8_t is_action_busy = 0;  // 0: 循迹控制 1: 动作接管

// ==================== 外部依赖变量 ====================
extern Navi_WayPoint_t point_map[NAVI_POINT_MAX]; 
extern Navi_Controller_t navi_ctrl;
extern float target_velocity;
extern float target_angle;
extern pid_param_t motor_speed;
extern pid_param_t motor_direction;
extern int bridge_high;
extern void pid_low_init(void);
extern void Turn_Reset(void);
extern void Vision_Align_Cal_Reset(void);

// *******************************************************************************
// 动作参数
// ******************************************************************************
//--------------- 排雷旋转 ---------------
#define MINE_ROTATE_BASE_DEG        720.0f
#define MINE_ROTATE_LEAD_DEG        70.0f
#define MINE_ROTATE_TIMEOUT_MS      15000U
#define MINE_ROTATE_CCW_CMD         2U
static float mine_rotate_start_yaw = 0.0f;
static int8_t mine_rotate_dir = 1;
static float mine_rotate_target_deg = MINE_ROTATE_BASE_DEG;
static uint8_t action_done_pending = 0;
static uint16_t action_done_idx = 0;
#if (NAVI_JUMP_ACTION_MODE == 2U)
static uint8_t jump_sequence_done_count = 0;
static float jump_course_back_yaw = 0.0f;
#endif
static float jump_sequence_hold_yaw = 0.0f;
static float jump_motion_count = 0.0f;
static float jump_motion_last_x = 0.0f;
static NaviJumpTouchLogic_t jump_touch_logic = {0};
static uint8_t jump_touch_inhibit_after_landing = 0;
static uint8_t remote_jump_active = 0;
static Course3AlignSamples_t course3_align_samples;
static uint32_t course3_align_last_frame = 0;
static float course3_align_map_yaw = 0.0f;
static uint32_t course3_align_last_valid_ms = 0;
static uint8_t course3_align_searching = 0;
static BridgeRollPeakTracker_t bridge_roll_tracker;
static float bridge_original_leg_y = 0.0f;
static uint8_t bridge_hold_active = 0U;
static uint8_t course3_bridge_action_active = 0U;
static uint8_t course3_bridge_action_initialized = 0U;
static uint8_t course3_bridge_low_restored = 0U;
static uint16_t course3_bridge_start_idx = 0U;
static uint16_t course3_bridge_end_idx = 0U;
static uint16_t course3_bridge_sequence_idx = 0U;
static float course3_bridge_target_yaw = 0.0f;
typedef enum
{
    COURSE3_AUX_SEGMENT_NONE = 0,
    COURSE3_AUX_SEGMENT_BUMP,
    COURSE3_AUX_SEGMENT_STAIR_RAMP
} Course3AuxSegmentKind_t;

typedef struct
{
    Course3AuxSegmentKind_t kind;
    uint8_t initialized;
    uint8_t direction_saved;
    uint16_t start_idx;
    uint16_t end_idx;
    float saved_direction_p;
    uint8_t anti_stall_saved;
    uint16_t sequence_idx;
    uint8_t saved_anti_stall_enabled;
    float target_yaw;
} Course3AuxSegmentControl_t;

static Course3AuxSegmentControl_t course3_aux_segment;
typedef struct
{
    uint8_t active;
    uint8_t waiting_at_entry;
    WayPoint_Type type;
    uint16_t calibrate_idx;
    uint16_t entry_idx;
    uint16_t end_idx;
} Course3VisionSegmentControl_t;

static Course3VisionSegmentControl_t course3_vision_segment;

static uint8_t course3_display_state = COURSE3_DISPLAY_IDLE;
static uint8_t course3_display_done_pending_clear = 0U;
static uint8_t course3_aux_begin(WayPoint_Type type,
                                 uint16_t sequence_idx,
                                 uint16_t start_idx,
                                 uint16_t end_idx,
                                 float target_yaw);

#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
static uint8_t jump_pose_update_active = 0;
#endif

uint8_t navigation_jump_is_active(void)
{
    return (action_fsm.state == FSM_JUMP_EXPLORE ||
            action_fsm.state == FSM_JUMP_EDGE_TOUCH ||
            action_fsm.state == FSM_JUMP_BACKOFF ||
            action_fsm.state == FSM_JUMP_RUNUP ||
            action_fsm.state == FSM_JUMP_PREPARE ||
            action_fsm.state == FSM_JUMP_TAKEOFF ||
            action_fsm.state == FSM_JUMP_AIRBORNE ||
            action_fsm.state == FSM_JUMP_LANDING ||
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

uint8_t Navi_Action_Bridge_Vision_Cal_Active(void)
{
    uint16_t idx = navi_ctrl.point_current_idx;

    return (action_fsm.state == FSM_COURSE3_TRACK_ALIGN &&
            idx < navi_ctrl.point_total_count &&
            Course3Segment_ShouldQueueAction(Runtime_Get_Vehicle_Mode(),
                                             (uint8)point_map[idx].type,
                                             point_map[idx].action_cmd) &&
            Course3Segment_RequiresVisionForMode(Runtime_Get_Vehicle_Mode(),
                                                 (uint8)point_map[idx].type)) ? 1U : 0U;
}
uint8_t Navi_Action_Vision_Calibration_Waiting(void)
{
    return (course3_vision_segment.active &&
            course3_vision_segment.waiting_at_entry) ? 1U : 0U;
}

uint8_t Navi_Action_Get_Course3_Vision_Phase(void)
{
    if (course3_vision_segment.active)
    {
        return course3_vision_segment.waiting_at_entry ?
               COURSE3_VISION_PHASE_ENTRY_WAIT : COURSE3_VISION_PHASE_CALIBRATING;
    }
    if (course3_bridge_action_active)
    {
        return COURSE3_VISION_PHASE_BRIDGE;
    }
    if (course3_aux_segment.kind == COURSE3_AUX_SEGMENT_STAIR_RAMP)
    {
        return COURSE3_VISION_PHASE_RAMP;
    }
    return COURSE3_VISION_PHASE_NONE;
}

uint32_t Navi_Action_Get_Course3_Vision_Batch_Count(void)
{
    return 0U;
}

uint8_t Navi_Action_Get_Course3_Latest_Yaw_Valid(void)
{
    return 0U;
}

float Navi_Action_Get_Course3_Latest_Yaw(void)
{
    return 0.0f;
}

float Navi_Action_Get_Course3_Calibration_Travelled(void)
{
    return Navi_Course3_Calibration_Meter_Get_Travelled();
}

float Navi_Action_Get_Course3_Calibration_Target(void)
{
    return Navi_Course3_Calibration_Meter_Get_Target();
}
float Navi_Action_Get_Course3_Action_Travelled(void)
{
    return Navi_Course3_Bridge_Odometry_Get_Travelled();
}

float Navi_Action_Get_Course3_Action_Target(void)
{
    return Navi_Course3_Bridge_Odometry_Get_Target();
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

static uint8_t course3_segment_find_end_index(uint16_t start_idx,
                                              WayPoint_Type type,
                                              uint16_t *end_idx)
{
    uint16_t idx;

    if (end_idx == NULL)
    {
        return 0U;
    }

    for (idx = (uint16_t)(start_idx + 1U); idx < navi_ctrl.point_total_count; idx++)
    {
        if (point_map[idx].type == type &&
            point_map[idx].action_cmd == NAVI_SEGMENT_ACTION_END)
        {
            *end_idx = idx;
            return 1U;
        }

        if (Course3Segment_ShouldQueueAction(Runtime_Get_Vehicle_Mode(),
                                             (uint8)point_map[idx].type,
                                             point_map[idx].action_cmd))
        {
            break;
        }
    }


    return 0U;
}
static uint8_t course3_vision_segment_find_indices(uint16_t calibrate_idx,
                                                   WayPoint_Type type,
                                                   uint16_t *entry_idx,
                                                   uint16_t *end_idx)
{
    uint16_t entry;
    uint16_t end;

    if (entry_idx == NULL || end_idx == NULL ||
        (type != WP_TYPE_BRIDGE && type != WP_TYPE_STAIR_RAMP))
    {
        return 0U;
    }
    entry = (uint16_t)(calibrate_idx + 1U);
    end = (uint16_t)(calibrate_idx + 2U);
    if (end >= navi_ctrl.point_total_count ||
        point_map[entry].type != type || point_map[end].type != type ||
        point_map[entry].action_cmd != NAVI_VISION_SEGMENT_ACTION_ENTRY ||
        point_map[end].action_cmd != NAVI_VISION_SEGMENT_ACTION_END)
    {
        return 0U;
    }

    *entry_idx = entry;
    *end_idx = end;
    return 1U;
}

static float course3_segment_get_map_target_yaw(uint16_t start_idx,
                                                uint16_t end_idx)
{
    float segment_azimuth = navi_get_two_points_azimuth(
        point_map[start_idx].x,
        point_map[start_idx].y,
        point_map[end_idx].x,
        point_map[end_idx].y);
    float navigation_turn_error =
        navi_limit_angle180(segment_azimuth - robot_pose.yaw);

    /* The steering loop uses IMU yaw with the opposite sign to map yaw. */
    return navi_limit_angle180(IMU_data.filter_result.yaw -
                               navigation_turn_error);
}

static uint8_t course3_inertial_segment_begin(uint16_t start_idx,
                                              WayPoint_Type type)
{
    uint16_t end_idx = (uint16_t)(start_idx + 1U);
    float map_target_yaw;

    if (Runtime_Get_Vehicle_Mode() != VEHICLE_MODE_COURSE_3_INERTIAL ||
        !Course3Segment_IsPairedType((uint8)type) ||
        end_idx >= navi_ctrl.point_total_count ||
        !point_map[end_idx].valid || point_map[end_idx].type != type ||
        point_map[end_idx].action_cmd != NAVI_SEGMENT_ACTION_END)
    {
        return 0U;
    }

    map_target_yaw = course3_segment_get_map_target_yaw(start_idx, end_idx);
    if (type == WP_TYPE_BRIDGE)
    {
        course3_bridge_target_yaw = map_target_yaw;
        course3_bridge_sequence_idx = start_idx;
        course3_bridge_start_idx = start_idx;
        course3_bridge_end_idx = end_idx;
        course3_bridge_action_active = 1U;
        course3_bridge_action_initialized = 0U;
        course3_bridge_low_restored = 0U;
        navi_ctrl.point_current_idx = end_idx;
    }
    else if (!course3_aux_begin(type,
                                start_idx,
                                start_idx,
                                end_idx,
                                map_target_yaw))
    {
        return 0U;
    }

    target_angle = map_target_yaw;
    action_fsm.state = FSM_COURSE3_ACTION;
    action_fsm.state_timer_ms = 0U;
    is_action_busy = 1U;
    course3_display_state = COURSE3_DISPLAY_ACTION;
    return 1U;
}


static void course3_aux_restore_direction(void)
{
    if (course3_aux_segment.direction_saved)
    {
        Direction_p = course3_aux_segment.saved_direction_p;
        PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
    }
    course3_aux_segment.direction_saved = 0U;
}

static void course3_aux_restore_anti_stall(void)
{
    if (course3_aux_segment.anti_stall_saved)
    {
        Runtime_Set_Module_Enabled(RUNTIME_MODULE_ANTI_STALL,
                                   course3_aux_segment.saved_anti_stall_enabled);
    }
    course3_aux_segment.anti_stall_saved = 0U;
}

void Navi_Action_Reset_New_Course3_Segments(void)
{
    if (course3_aux_segment.kind != COURSE3_AUX_SEGMENT_NONE ||
        course3_vision_segment.active || course3_bridge_action_active)
    {
        target_velocity = 0.0f;
        action_fsm.state = FSM_IDLE;
        action_fsm.state_timer_ms = 0U;
        is_action_busy = 0U;
        Turn_Reset();
        navi_tracking_speed_profile_reset();
        Vision_Align_Cal_Reset();
    }
    Navi_Course3_Calibration_Meter_End();
    Navi_Course3_Bridge_Odometry_End();
    course3_aux_restore_direction();
    course3_aux_restore_anti_stall();
    memset(&course3_aux_segment, 0, sizeof(course3_aux_segment));
    memset(&course3_vision_segment, 0, sizeof(course3_vision_segment));
    course3_bridge_action_active = 0U;
    course3_bridge_action_initialized = 0U;
}

static uint8_t course3_aux_begin(WayPoint_Type type,
                                 uint16_t sequence_idx,
                                 uint16_t start_idx,
                                 uint16_t end_idx,
                                 float target_yaw)
{
    if ((type != WP_TYPE_BUMP && type != WP_TYPE_STAIR_RAMP) ||
        end_idx >= navi_ctrl.point_total_count || point_map[end_idx].type != type)
    {
        return 0U;
    }

    memset(&course3_aux_segment, 0, sizeof(course3_aux_segment));
    course3_aux_segment.kind = (type == WP_TYPE_BUMP) ?
                               COURSE3_AUX_SEGMENT_BUMP : COURSE3_AUX_SEGMENT_STAIR_RAMP;
    course3_aux_segment.start_idx = start_idx;
    course3_aux_segment.sequence_idx = sequence_idx;
    course3_aux_segment.end_idx = end_idx;
    course3_aux_segment.saved_direction_p = Direction_p;
    course3_aux_segment.direction_saved = 1U;
    course3_aux_segment.target_yaw = target_yaw;
    if (type == WP_TYPE_BUMP)
    {
        course3_aux_segment.saved_anti_stall_enabled =
            Runtime_Is_Module_Enabled(RUNTIME_MODULE_ANTI_STALL);
        course3_aux_segment.anti_stall_saved = 1U;
        Runtime_Set_Module_Enabled(RUNTIME_MODULE_ANTI_STALL, 1U);
    }

    navi_ctrl.point_current_idx = end_idx;
    return 1U;
}

uint8_t Navi_Action_Course3_Execution_Active(void)
{
    return (Course3Mode_IsCourse3(Runtime_Get_Vehicle_Mode()) &&
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
    if (!Navi_Action_Course3_Execution_Active())
    {
        return 0.0f;
    }
    if (course3_vision_segment.active || course3_bridge_action_active ||
        course3_aux_segment.kind != COURSE3_AUX_SEGMENT_NONE)
    {
        return target_angle;
    }
    return point_map[navi_ctrl.point_current_idx].yaw;
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

#define NAVI_JUMP_FORWARD_SPEED       NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_AIRBORNE_SPEED      NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_LANDING_SPEED       NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_EXPLORE_SPEED       (120.0f)
#define NAVI_JUMP_BACKOFF_SPEED       (-120.0f)
#define NAVI_JUMP_TOUCH_LOW_SPEED     (10)
#define NAVI_JUMP_TOUCH_HIGH_SPEED    (50)
#define NAVI_JUMP_TOUCH_HIGH_SAMPLES  (2U)
#define NAVI_JUMP_TOUCH_LOW_CONFIRM   (2U)
#define NAVI_JUMP_TOUCH_INHIBIT_AFTER_LANDING_MS (1000U)
#define NAVI_JUMP_BACKOFF_X_TARGET    (0.35f)
#define NAVI_JUMP_RUNUP_X_TARGET      (0.20f)
#define NAVI_JUMP_BURST_PWM           (1300)
#define NAVI_JUMP_AIR_RETRACT_X       (-0.00f)
#define NAVI_JUMP_AIR_RETRACT_Y       (0.015f)
#define NAVI_JUMP_EXE_BUFFER_X        (+0.00f)
#define NAVI_JUMP_EXE_BUFFER_Y        (0.035f)
#define NAVI_JUMP_RECOVER_PWM         (420)
#define NAVI_JUMP_PREPARE_MS          (100U)
#define NAVI_JUMP_BURST_MS            (180U)
#define NAVI_JUMP_AIR_RETRACT_MS      (40U)
#define NAVI_JUMP_RECOVER_MS          (50U)
#define NAVI_JUMP_LANDING_MAX_MS      (600U)
#define NAVI_JUMP_LAND_ACCEL_G        (1.0f)

#define NAVI_TRIPLE_JUMP_TOTAL_COUNT      (3U)

#if (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_TRIPLE_ONLY) && \
    (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
#error "Invalid NAVI_TRIPLE_JUMP_AFTER_MODE"
#endif

#if (NAVI_JUMP_POSE_UPDATE_MODE != 1U) && (NAVI_JUMP_POSE_UPDATE_MODE != 2U)
#error "Invalid NAVI_JUMP_POSE_UPDATE_MODE"
#endif

static void navi_jump_motion_reset(void)
{
    jump_motion_count = 0.0f;
    jump_motion_last_x = robot_pose.x;
}

static void navi_jump_motion_update(void)
{
    jump_motion_count += fabsf(robot_pose.x - jump_motion_last_x);
    jump_motion_last_x = robot_pose.x;
}

static void navi_jump_motion_update_forward_x(void)
{
    float dx = robot_pose.x - jump_motion_last_x;

    if (dx > 0.0f) {
        jump_motion_count += dx;
    }
    jump_motion_last_x = robot_pose.x;
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

uint8_t Navi_Action_Start_Remote_Jump(void)
{
    if (remote_jump_active || is_action_busy || navigation_jump_is_active() || jump_is_active())
    {
        return 0;
    }

    remote_jump_active = 1;
    is_action_busy = 1;
    action_fsm.state = FSM_JUMP_EXPLORE;
    action_fsm.state_timer_ms = 0;
    action_fsm.is_airborne_expect = 0;

#if (NAVI_JUMP_ACTION_MODE == 2U)
    jump_sequence_done_count = 0;
    jump_course_back_yaw = 0.0f;
#endif
    jump_sequence_hold_yaw = navi_limit_angle180(target_angle);
    navi_jump_motion_reset();
    navi_jump_touch_inhibit_reset();
    navi_jump_pose_update_end();

    jump_stop = 0;
    jump_position = 0;
    jump_engine_suspend = 0;
    target_velocity = NAVI_JUMP_EXPLORE_SPEED;
    target_angle = jump_sequence_hold_yaw;

    return 1;
}

static void navi_action_remote_jump_clear(void)
{
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

// ============================================================================
// ================== END REMOTE CH6 NAVIGATION JUMP BRIDGE ===================
// ============================================================================

// ==============================================================================
// 全局路径预处理：提取需要动作接管的特殊航点
// ==============================================================================
void navi_parse_global_path(void) {
    uint8_t mode = Runtime_Get_Vehicle_Mode();
    Navi_Action_Reset_New_Course3_Segments();
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
    course3_bridge_action_active = 0U;
    course3_bridge_action_initialized = 0U;
    course3_bridge_low_restored = 0U;
    course3_bridge_start_idx = 0U;
    course3_bridge_end_idx = 0U;
    course3_bridge_target_yaw = 0.0f;
#if (NAVI_JUMP_ACTION_MODE == 2U)
    jump_sequence_done_count = 0;
    jump_course_back_yaw = 0.0f;
#endif
    jump_sequence_hold_yaw = 0.0f;
    navi_jump_motion_reset();
    navi_jump_touch_inhibit_reset();
    navi_jump_pose_update_end();
    jump_engine_suspend = 0;

    for (int i = 0; i < navi_ctrl.point_total_count; i++) {
        // 科目三单边桥和台阶经过 Track_align -> Action -> Done。
        if (point_map[i].type == WP_TYPE_MINE_SWEEP ||
            (point_map[i].type == WP_TYPE_JUMP && !Course3Mode_IsCourse3(mode)) ||
            (Course3Mode_IsCourse3(mode) &&
             ((Course3Mode_UsesVision(mode) && point_map[i].type == WP_TYPE_JUMP) ||
              Course3Segment_ShouldQueueAction(mode, (uint8)point_map[i].type, point_map[i].action_cmd))))  {
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
    if (!remote_jump_active) {
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
            else if (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3_INERTIAL &&
                     Course3Segment_IsPairedType((uint8)upcoming_type) &&
                     Course3Segment_IsStartActionForMode(Runtime_Get_Vehicle_Mode(),
                                                        (uint8)upcoming_type,
                                                        point_map[target_idx].action_cmd) &&
                     navi_isreach_target_point(target_idx))
            {
                if (!course3_inertial_segment_begin(target_idx, upcoming_type))
                {
                    target_velocity = 0.0f;
                    navi_ctrl.navi_mode_driver = 0U;
                    vofa_mode_driver = 0.0f;
                }
            }
            else if (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3 &&
                     upcoming_type == WP_TYPE_BUMP &&
                     point_map[target_idx].action_cmd == NAVI_SEGMENT_ACTION_START &&
                     navi_isreach_target_point(target_idx))
            {
                uint16_t bump_end_idx = 0U;

                if (!course3_segment_find_end_index(target_idx, WP_TYPE_BUMP, &bump_end_idx) ||
                    !course3_aux_begin(WP_TYPE_BUMP, target_idx, target_idx, bump_end_idx, 0.0f))
                {
                    target_velocity = 0.0f;
                    navi_ctrl.navi_mode_driver = 0U;
                    vofa_mode_driver = 0.0f;
                    break;
                }
                action_fsm.state = FSM_COURSE3_ACTION;
                action_fsm.state_timer_ms = 0U;
                is_action_busy = 1U;
                course3_display_state = COURSE3_DISPLAY_ACTION;
            }
            else if (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3 &&
                     ((upcoming_type == WP_TYPE_JUMP &&
                       navi_isreach_target_point(target_idx)) ||
                      (Course3Segment_ShouldQueueAction(Runtime_Get_Vehicle_Mode(),
                                                        (uint8)upcoming_type,
                                                        point_map[target_idx].action_cmd) &&
                       Course3Segment_RequiresVisionForMode(Runtime_Get_Vehicle_Mode(),
                                                            (uint8)upcoming_type) &&
                       Navi_Course3_Vision_Approach_Is_Complete(target_idx))))
            {
                Course3Align_Reset(&course3_align_samples);
                if (Course3Segment_ShouldQueueAction(Runtime_Get_Vehicle_Mode(),
                                                     (uint8)upcoming_type,
                                                     point_map[target_idx].action_cmd) &&
                    Course3Segment_RequiresVisionForMode(Runtime_Get_Vehicle_Mode(),
                                                        (uint8)upcoming_type))
                {
                    uint16_t entry_idx = 0U;
                    uint16_t end_idx = 0U;
                    float calibration_distance;

                    if (!course3_vision_segment_find_indices(target_idx, upcoming_type,
                                                             &entry_idx, &end_idx))
                    {
                        target_velocity = 0.0f;
                        navi_ctrl.navi_mode_driver = 0U;
                        vofa_mode_driver = 0.0f;
                        break;
                    }
                    memset(&course3_vision_segment, 0, sizeof(course3_vision_segment));
                    course3_vision_segment.active = 1U;
                    course3_vision_segment.type = upcoming_type;
                    course3_vision_segment.calibrate_idx = target_idx;
                    course3_vision_segment.entry_idx = entry_idx;
                    course3_vision_segment.end_idx = end_idx;
                    calibration_distance = navi_get_two_points_distance(
                        point_map[target_idx].x, point_map[target_idx].y,
                        point_map[entry_idx].x, point_map[entry_idx].y);
                    Navi_Course3_Calibration_Meter_Begin(calibration_distance);
                    Vision_Align_Cal_Reset();
                }
                course3_align_last_frame = core_b_cmd.vision_frame_seq;
                course3_align_map_yaw = navi_limit_angle180(point_map[target_idx].yaw);
                course3_align_last_valid_ms = 0U;
                course3_align_searching = 0U;
                course3_bridge_action_active = 0U;
                course3_bridge_action_initialized = 0U;
                course3_bridge_low_restored = 0U;
                action_fsm.state = FSM_COURSE3_TRACK_ALIGN;
                action_fsm.state_timer_ms = 0U;
                is_action_busy = 1U;
                course3_display_state = COURSE3_DISPLAY_TRACK_ALIGN;
            }
            else if (upcoming_type == WP_TYPE_JUMP && distance < (DISTANCE_THRESHOLD * 3.0f)) {                        // 跳跃动作
                point_map[target_idx].action_cmd = NAVI_JUMP_ACTION_MODE;
#if (NAVI_JUMP_ACTION_MODE == 0U)
                action_fsm.state = FSM_MINE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
#elif (NAVI_JUMP_ACTION_MODE == 1U)
                jump_sequence_hold_yaw = navi_limit_angle180(point_map[target_idx].yaw);
                navi_jump_motion_reset();
                navi_jump_touch_inhibit_reset();
                action_fsm.state = FSM_JUMP_EXPLORE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                target_velocity = NAVI_JUMP_EXPLORE_SPEED;
                target_angle = jump_sequence_hold_yaw;
#elif (NAVI_JUMP_ACTION_MODE == 2U)
                jump_sequence_done_count = 0;
                jump_course_back_yaw = 0.0f;
                jump_sequence_hold_yaw = navi_limit_angle180(point_map[target_idx].yaw);
                navi_jump_motion_reset();
                navi_jump_touch_inhibit_reset();
                action_fsm.state = FSM_JUMP_EXPLORE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                target_velocity = NAVI_JUMP_EXPLORE_SPEED;
                target_angle = jump_sequence_hold_yaw;
#else
#error "Invalid NAVI_JUMP_ACTION_MODE"
#endif
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

            if (course3_vision_segment.active)
            {
                course3_vision_segment.waiting_at_entry =
                    Navi_Course3_Calibration_Meter_Is_Complete() ? 1U : 0U;
                target_velocity = course3_vision_segment.waiting_at_entry ?
                                  0.0f : COURSE3_VISION_CAL_SPEED;

                if (course3_vision_segment.waiting_at_entry)
                {
                    WayPoint_Type segment_type = course3_vision_segment.type;
                    uint16_t calibrate_idx = course3_vision_segment.calibrate_idx;
                    uint16_t entry_idx = course3_vision_segment.entry_idx;
                    uint16_t end_idx = course3_vision_segment.end_idx;
                    float map_target_yaw =
                        course3_segment_get_map_target_yaw(entry_idx, end_idx);

                    Navi_Course3_Calibration_Meter_End();
                    Vision_Align_Cal_Reset();
                    course3_vision_segment.active = 0U;
                    course3_vision_segment.waiting_at_entry = 0U;

                    if (segment_type == WP_TYPE_BRIDGE)
                    {
                        course3_bridge_target_yaw = map_target_yaw;
                        course3_bridge_sequence_idx = calibrate_idx;
                        course3_bridge_start_idx = entry_idx;
                        course3_bridge_end_idx = end_idx;
                        course3_bridge_action_active = 1U;
                        course3_bridge_action_initialized = 0U;
                        course3_bridge_low_restored = 0U;
                    }
                    else if (!course3_aux_begin(WP_TYPE_STAIR_RAMP,
                                                calibrate_idx,
                                                entry_idx,
                                                end_idx,
                                                map_target_yaw))
                    {
                        target_velocity = 0.0f;
                        navi_ctrl.navi_mode_driver = 0U;
                        vofa_mode_driver = 0.0f;
                        break;
                    }
                    action_fsm.state = FSM_COURSE3_ACTION;
                    action_fsm.state_timer_ms = 0U;
                    course3_display_state = COURSE3_DISPLAY_ACTION;
                    target_velocity = 0.0f;
                    target_angle = map_target_yaw;
                }
                break;
            }

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
            if (course3_aux_segment.kind != COURSE3_AUX_SEGMENT_NONE)
            {
                float segment_azimuth = 0.0f;
                float segment_distance = 0.0f;
                uint8_t segment_complete = 0U;

                is_action_busy = 1U;
                if (!course3_aux_segment.initialized)
                {
                    Direction_p = (course3_aux_segment.kind == COURSE3_AUX_SEGMENT_BUMP) ?
                                  COURSE3_BUMP_DIRECTION_P : COURSE3_AUX_SEGMENT_DIRECTION_P;
                    PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
                    Turn_Reset();
                    navi_tracking_speed_profile_reset();
                    if (course3_aux_segment.kind == COURSE3_AUX_SEGMENT_STAIR_RAMP)
                    {
                        Navi_Course3_Bridge_Odometry_Begin(course3_aux_segment.target_yaw,
                            point_map[course3_aux_segment.start_idx].x,
                            point_map[course3_aux_segment.start_idx].y,
                            point_map[course3_aux_segment.end_idx].x,
                            point_map[course3_aux_segment.end_idx].y);
                    }
                    course3_aux_segment.initialized = 1U;
                    action_fsm.state_timer_ms = 0U;
                }

                if (course3_aux_segment.kind == COURSE3_AUX_SEGMENT_BUMP)
                {
                    if (!navi_calcnavinfo(course3_aux_segment.end_idx,
                                          &segment_azimuth, &segment_distance))
                    {
                        target_velocity = 0.0f;
                        course3_aux_restore_direction();
                        course3_aux_restore_anti_stall();
                        memset(&course3_aux_segment, 0, sizeof(course3_aux_segment));
                        Turn_Reset();
                        navi_tracking_speed_profile_reset();
                        navi_ctrl.navi_mode_driver = 0U;
                        vofa_mode_driver = 0.0f;
                        action_fsm.state = FSM_IDLE;
                        is_action_busy = 0U;
                        break;
                    }
                    target_angle = navi_limit_angle180(IMU_data.filter_result.yaw -
                        navi_limit_angle180(segment_azimuth - robot_pose.yaw));
                    segment_complete = (segment_distance <= DISTANCE_THRESHOLD) ? 1U : 0U;
                }
                else
                {
                    target_angle = course3_aux_segment.target_yaw;
                    segment_complete = Navi_Course3_Bridge_Odometry_Is_Complete();
                }
                target_velocity = COURSE3_AUX_SEGMENT_SPEED;
                Direction_p = (course3_aux_segment.kind == COURSE3_AUX_SEGMENT_BUMP) ?
                              COURSE3_BUMP_DIRECTION_P : COURSE3_AUX_SEGMENT_DIRECTION_P;
                PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);

                if (segment_complete)
                {
                    uint16_t completed_sequence_idx = course3_aux_segment.sequence_idx;
                    uint16_t completed_end_idx = course3_aux_segment.end_idx;
                    uint8_t was_ramp =
                        (course3_aux_segment.kind == COURSE3_AUX_SEGMENT_STAIR_RAMP) ? 1U : 0U;

                    target_velocity = 0.0f;
                    if (was_ramp)
                    {
                        Navi_Course3_Bridge_Odometry_End();
                    }
                    course3_aux_restore_direction();
                    course3_aux_restore_anti_stall();
                    memset(&course3_aux_segment, 0, sizeof(course3_aux_segment));
                    Turn_Reset();
                    navi_tracking_speed_profile_reset();
                    if (action_seq.current_ptr < action_seq.total_count &&
                        action_seq.list[action_seq.current_ptr].wp_index == completed_sequence_idx)
                    {
                        action_seq.current_ptr++;
                    }
                    action_done_pending = 1U;
                    action_done_idx = completed_end_idx;
                    action_fsm.state = FSM_IDLE;
                    action_fsm.state_timer_ms = 0U;
                    is_action_busy = 0U;
                    course3_display_state = COURSE3_DISPLAY_DONE;
                    course3_display_done_pending_clear = 1U;
                }
                break;
            }

            if (!course3_bridge_action_active && point_map[target_idx].type != WP_TYPE_BRIDGE)
            {
                Direction_p = COURSE3_ACTION_DIRECTION_P;
                PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
                action_fsm.state = FSM_COURSE3_DONE;
                action_fsm.state_timer_ms = 0U;
                course3_display_state = COURSE3_DISPLAY_DONE;
                course3_display_done_pending_clear = 1U;
                break;
            }

            is_action_busy = 1U;
            if (course3_bridge_action_active)
            {
                if (!course3_bridge_action_initialized)
                {
                    if (course3_bridge_end_idx < navi_ctrl.point_total_count)
                    {
                        navi_ctrl.point_current_idx = course3_bridge_end_idx;
                    }
                    y_current = COURSE3_BRIDGE_LEG_Y;
                    bridge_high = 1;
                    Speed_p = COURSE3_BRIDGE_SPEED_P;
                    PidChange(&motor_speed, Speed_p, Speed_i, Speed_d);
                    Direction_p = COURSE3_BRIDGE_DIRECTION_P;
                    PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
                    Turn_Reset();
                    navi_tracking_speed_profile_reset();
                    Navi_Course3_Bridge_Odometry_Begin(course3_bridge_target_yaw,
                                                        point_map[course3_bridge_start_idx].x,
                                                        point_map[course3_bridge_start_idx].y,
                                                        point_map[course3_bridge_end_idx].x,
                                                        point_map[course3_bridge_end_idx].y);
                    course3_bridge_action_initialized = 1U;
                    action_fsm.state_timer_ms = 0U;
                }

                target_angle = course3_bridge_target_yaw;
                target_velocity = COURSE3_BRIDGE_SPEED;
                Direction_p = COURSE3_BRIDGE_DIRECTION_P;
                PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
                if (Navi_Course3_Bridge_Odometry_Is_Complete())
                {
                    uint16_t completed_sequence_idx = course3_bridge_sequence_idx;
                    uint16_t completed_end_idx = course3_bridge_end_idx;

                    pid_low_init();
                    Turn_Reset();
                    navi_tracking_speed_profile_reset();
                    course3_bridge_low_restored = 1U;
                    course3_bridge_action_active = 0U;
                    Navi_Course3_Bridge_Odometry_End();
                    course3_bridge_action_initialized = 0U;
                    if (action_seq.current_ptr < action_seq.total_count &&
                        action_seq.list[action_seq.current_ptr].wp_index == completed_sequence_idx)
                    {
                        action_seq.current_ptr++;
                    }
                    action_done_pending = 1U;
                    action_done_idx = completed_end_idx;
                    action_fsm.state = FSM_IDLE;
                    action_fsm.state_timer_ms = 0U;
                    is_action_busy = 0U;
                    course3_display_state = COURSE3_DISPLAY_DONE;
                    course3_display_done_pending_clear = 1U;
                }
                break;
            }

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
                if (!course3_bridge_low_restored)
                {
                    y_current = bridge_original_leg_y;
                }
                bridge_hold_active = 0U;
                course3_bridge_low_restored = 0U;
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
            mine_rotate_target_deg = (point_map[target_idx].type == WP_TYPE_JUMP) ?
                                     360.0f : MINE_ROTATE_BASE_DEG;
            is_action_busy = 1;
            target_velocity = 0.0f;
            
            target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * MINE_ROTATE_LEAD_DEG);

            IPC_LOG_Printf("\r\n============= >>> [定点排雷] 旋转点[%d]，开始%s旋转两圈 <<< =============\r\n",
                   target_idx,
                   mine_rotate_dir > 0 ? "顺时针" : "逆时针");
            
            // 进入持续旋转执行状态。
            action_fsm.state = FSM_MINE_PROCESSING;
            action_fsm.state_timer_ms = 0;
            break;

        case FSM_MINE_PROCESSING:
        {
            float rotated_deg;
            float rotated_progress;
            float remaining_deg;
            float rotate_lead_deg;

            target_velocity = 0.0f;
            
            rotated_deg = ((float)robot_pose.cumulative_yaw - mine_rotate_start_yaw) * mine_rotate_dir;
            rotated_progress = (rotated_deg > 0.0f) ? rotated_deg : 0.0f;
            remaining_deg = mine_rotate_target_deg - rotated_progress;
            rotate_lead_deg = (remaining_deg < MINE_ROTATE_LEAD_DEG) ? remaining_deg : MINE_ROTATE_LEAD_DEG;

            // 旋转角度达到目标，动作完成。
            if (rotated_progress >= mine_rotate_target_deg)
            {
                IPC_LOG_Printf(" [定点排雷] 两圈旋转完成。\r\n");
                
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
                IPC_LOG_Printf(" [定点排雷] 两圈旋转超时退出。\r\n");
                
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
            target_angle = jump_sequence_hold_yaw;

            if (jump_touch_inhibit_after_landing)
            {
                navi_jump_touch_window_reset();
                if (action_fsm.state_timer_ms >= NAVI_JUMP_TOUCH_INHIBIT_AFTER_LANDING_MS)
                {
                    jump_touch_inhibit_after_landing = 0;
                }
                break;
            }

            if (navi_jump_touch_update())
            {
                action_fsm.state = FSM_JUMP_EDGE_TOUCH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_EDGE_TOUCH:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = 0.0f;
            target_angle = jump_sequence_hold_yaw;
            navi_jump_motion_reset();
            action_fsm.state = FSM_JUMP_BACKOFF;
            action_fsm.state_timer_ms = 0;
            break;

        case FSM_JUMP_BACKOFF:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_BACKOFF_SPEED;
            target_angle = jump_sequence_hold_yaw;
            navi_jump_motion_update();

            if (jump_motion_count >= NAVI_JUMP_BACKOFF_X_TARGET) {
                navi_jump_motion_reset();
                action_fsm.state = FSM_JUMP_RUNUP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_RUNUP:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_RUNUP_SPEED;
            target_angle = jump_sequence_hold_yaw;
            navi_jump_motion_update_forward_x();

            if (jump_motion_count >= NAVI_JUMP_RUNUP_X_TARGET) {
                navi_jump_pose_update_begin();
                action_fsm.state = FSM_JUMP_PREPARE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_PREPARE:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_FORWARD_SPEED;
            target_angle = jump_sequence_hold_yaw;
            jump_drive_symmetric_pwm(jump_calc_prepare_pwm((uint16)action_fsm.state_timer_ms));

            if (action_fsm.state_timer_ms >= NAVI_JUMP_PREPARE_MS) {
                action_fsm.state = FSM_JUMP_TAKEOFF;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_TAKEOFF:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_FORWARD_SPEED;
            target_angle = jump_sequence_hold_yaw;
            jump_drive_symmetric_pwm(NAVI_JUMP_BURST_PWM);

            if (action_fsm.state_timer_ms >= NAVI_JUMP_BURST_MS) {
                action_fsm.state = FSM_JUMP_AIRBORNE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                action_fsm.is_airborne_expect = 1;
            }
            break;

        case FSM_JUMP_AIRBORNE:
        {
            uint32_t buffer_ms;

            is_action_busy = 1;
            action_fsm.is_airborne_expect = 1;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_AIRBORNE_SPEED;
            target_angle = jump_sequence_hold_yaw;

            if (action_fsm.state_timer_ms < NAVI_JUMP_AIR_RETRACT_MS) {
                jump_drive_symmetric_xy(NAVI_JUMP_AIR_RETRACT_X, NAVI_JUMP_AIR_RETRACT_Y);
            } else {
                jump_drive_symmetric_xy(NAVI_JUMP_EXE_BUFFER_X, NAVI_JUMP_EXE_BUFFER_Y);
                buffer_ms = action_fsm.state_timer_ms - NAVI_JUMP_AIR_RETRACT_MS;
                if (IMU_data.accel[2] >= (1.5f * NAVI_JUMP_LAND_ACCEL_G) || buffer_ms >= NAVI_JUMP_LANDING_MAX_MS) {
                    action_fsm.state = FSM_JUMP_LANDING;
                    action_fsm.state_timer_ms = 0;
                    is_action_busy = 1;
                    action_fsm.is_airborne_expect = 0;
                }
            }
            break;
        }

        case FSM_JUMP_LANDING:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_LANDING_SPEED;
            target_angle = jump_sequence_hold_yaw;
            jump_drive_symmetric_pwm(NAVI_JUMP_RECOVER_PWM);

            if (action_fsm.state_timer_ms >= NAVI_JUMP_RECOVER_MS) {
                navi_jump_pose_add_fixed_step();
                navi_jump_pose_update_end();
#if (NAVI_JUMP_ACTION_MODE == 2U)
                jump_sequence_done_count++;
                if (jump_sequence_done_count < NAVI_TRIPLE_JUMP_TOTAL_COUNT) {
                    navi_jump_motion_reset();
                    navi_jump_touch_window_reset();
                    jump_touch_inhibit_after_landing = 1;
                    action_fsm.state = FSM_JUMP_EXPLORE;
                    action_fsm.state_timer_ms = 0;
                    is_action_busy = 1;
                    action_fsm.is_airborne_expect = 0;
                    jump_engine_suspend = 0;
                    jump_position = 0;
                    jump_stop = 0;
                    target_velocity = NAVI_JUMP_EXPLORE_SPEED;
                    target_angle = jump_sequence_hold_yaw;
                    break;
                }
#if (NAVI_TRIPLE_JUMP_AFTER_MODE == NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
                jump_course_back_yaw = navi_limit_angle180(jump_sequence_hold_yaw + 180.0f);
                action_fsm.state = FSM_JUMP_RAMP_DOWN;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                action_fsm.is_airborne_expect = 0;
                jump_engine_suspend = 0;
                jump_position = 0;
                jump_stop = 0;
                break;
#endif
#endif
#if (NAVI_JUMP_ACTION_MODE != 2U) || (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
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
#endif
            }
            break;

        case FSM_JUMP_RAMP_DOWN:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_RAMP_SPEED;
            target_angle = jump_sequence_hold_yaw;
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
void Navi_Action_Remote_Jump_Tick(void)
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

// ============================================================================
// ================== END REMOTE CH6 NAVIGATION JUMP BRIDGE ===================
// ============================================================================


// ==============================================================================
// 动作管理入口：由循迹层周期调用
// ==============================================================================
void Navi_Action_Manager(uint16_t  curr_idx) {
    if (remote_jump_active) return;

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
