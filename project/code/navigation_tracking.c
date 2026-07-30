/********************************************************************************************************************* 
* 坐标x轴正北，y轴正东。顺时针时yaw增加    

*yaw_filter 的范围：-180 到 180

，EKF 内部的所有的状态量、控制量、观测量，必须全部统一为 国际标准单位（SI：米 $m$、秒 $s$、弧度 $rad$）。

*********************************************************************************************************************/
#include "navigation_tracking.h"

#include "navigation_data_handling.h"

#include "control.h"

#include "navigation_action.h"

#include "imu.h"

#include "param.h"

#include "ipc_shared_data.h"

#include "wifi.h"
#include "runtime_status.h"
#include "navigation_smooth_logic.h"
#include "vehicle_supervisor.h"
#include "course3_bridge_logic.h"

//====================================================全局变量定义=======================================================
Navi_Controller_t navi_ctrl;

Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // 预设的“路”

float wifi_cmd_trigger = 0.0;        // 0: 待机, 1: 追加一个航点, 2: 撤销一个航点   3 ：清空当前地图,
float wifi_in_action= 0.0;            //动作指令    
float wifi_remote_type = 0.0f;

float vofa_trigger_record = 0.0f;
float vofa_mode_driver = 0.0f;
float vofa_mode_map = 0.0f;
float vofa_print_pose_en = 0.0f;
float vofa_print_pose_period = 3000.0f; // 默认 3000ms 打印一次


//==================================================================================================

//====================================================全局静态变量定义=======================================================
//打点专用的后台独立存储区
Navi_WayPoint_t record_point_map[NAVI_POINT_MAX];
uint16_t record_point_count = 0;
static Navi_WayPoint_t static_point_map[NAVI_POINT_MAX]; 
static uint16_t static_point_count = 0;

static uint8_t navi_record_origin_cal_pending = 0U;
static uint8_t navi_start_cal_pending = 0U;

typedef struct
{
    uint8_t active;
    uint16_t target_idx;
    WayPoint_Type type;
} NaviCourse3Approach_t;

static NaviCourse3Approach_t navi_course3_approach;
static uint8_t navi_course3_angle_slew_initialized = 0U;
static float navi_course3_angle_slew_cmd = 0.0f;



//====================================================变量声明=======================================================
extern RobotState_t robot_pose;                                  // 全局实时位姿
extern IMU_t IMU_data;
extern uint8_t  is_action_busy ;         // 0:循迹控制         1:动作接管

//====================================================函数声明=============================================
void navi_record_update_status(void);
void navi_record_fill_preview(uint16 start, IpcNavRecordPreviewPoint_t *out, uint16 max_count, uint16 *actual_start, uint16 *actual_count);
static void navi_record_undo_last(void);
static void navi_speed_profile_reset(void);
static float navi_get_reach_threshold(uint16_t target_idx);
static float navi_calc_speed_plan_distance(uint16_t curr_idx, float current_distance, uint16_t *speed_target_idx);
static float navi_calc_turn_speed_limit(float turn_error_deg);
static float navi_update_tracking_velocity(float distance, float stop_threshold, uint8_t reached, uint8_t nav_valid);
static void Navi_VOFA_Preview_Task(uint8_t current_print_cmd);
static uint8_t navi_is_course1_smooth_point(uint16_t target_idx, uint16_t total_points);
static uint8_t navi_record_segment_state(WayPoint_Type *open_type);
static uint8_t navi_segment_validate_route(void);
static void navi_bridge_reset(uint8_t restore_low_height);
static uint8_t navi_course3_apply_line_lookahead(uint16_t target_idx,
                                                 float *azimuth,
                                                 float *turn_error);
static void navi_course3_approach_reset(uint8_t restore_bridge_height);
static void navi_course3_approach_handoff(void);
static uint8_t navi_course3_approach_update(uint16_t target_idx,
                                            float distance,
                                            uint8_t nav_info_valid);
static void navi_course3_angle_slew_reset(void);
static float navi_course3_angle_slew_apply(float desired_angle);

void navi_record_update_status(void)
{
    navi_ctrl.record_status.count = (float)record_point_count;
    if (record_point_count == 0)
    {
        navi_ctrl.record_status.last_idx = -1.0f;
        navi_ctrl.record_status.last_type = -1.0f;
        navi_ctrl.record_status.last_x = 0.0f;
        navi_ctrl.record_status.last_y = 0.0f;
        return;
    }

    uint16_t idx = record_point_count - 1U;
    navi_ctrl.record_status.last_idx = (float)idx;
    navi_ctrl.record_status.last_type = (float)record_point_map[idx].type;
    navi_ctrl.record_status.last_x = record_point_map[idx].x;
    navi_ctrl.record_status.last_y = record_point_map[idx].y;
}

void navi_record_fill_preview(uint16 start, IpcNavRecordPreviewPoint_t *out, uint16 max_count, uint16 *actual_start, uint16 *actual_count)
{
    uint16 i;
    uint16 safe_start = start;

    if (out == NULL || actual_start == NULL || actual_count == NULL)
    {
        return;
    }

    if (record_point_count == 0)
    {
        safe_start = 0;
    }
    else if (safe_start >= record_point_count)
    {
        safe_start = (uint16)(record_point_count - 1U);
    }

    *actual_start = safe_start;
    *actual_count = record_point_count;

    for (i = 0; i < max_count; i++)
    {
        uint16 idx = (uint16)(safe_start + i);
        out[i].idx = idx;
        if (idx < record_point_count && record_point_map[idx].valid)
        {
            out[i].valid = 1;
            out[i].type = (uint8)record_point_map[idx].type;
            out[i].x = record_point_map[idx].x;
            out[i].y = record_point_map[idx].y;
            out[i].yaw = record_point_map[idx].yaw;
        }
        else
        {
            out[i].valid = 0;
            out[i].type = 0;
            out[i].x = 0.0f;
            out[i].y = 0.0f;
            out[i].yaw = 0.0f;
        }
    }
}

static void navi_record_undo_last(void)
{
    if (record_point_count == 0)
    {
        IPC_LOG_Printf("\r\n>>> [NAVI_RECORD] map empty, cannot undo <<<\r\n");
        navi_record_update_status();
        return;
    }

    record_point_count--;
    memset(&record_point_map[record_point_count], 0, sizeof(record_point_map[record_point_count]));
    navi_ctrl.point_total_count = record_point_count;
    if (record_point_count == 0)
    {
        navi_ctrl.origin_set_flag = 0;
    }
    navi_record_update_status();
    IPC_Nav_Record_Mark_Dirty();
    IPC_LOG_Printf("\r\n>>> [NAVI_RECORD] undo last point, remain %d <<<\r\n", record_point_count);
}

static uint8_t navi_record_segment_state(WayPoint_Type *open_type)
{
    WayPoint_Type open = WP_TYPE_NORMAL;
    uint16_t i;

    if (open_type == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < record_point_count; i++)
    {
        WayPoint_Type type = record_point_map[i].type;

        if (!record_point_map[i].valid || !Course3Segment_IsPairedType((uint8)type))
        {
            continue;
        }

        if (open == WP_TYPE_NORMAL && record_point_map[i].action_cmd == NAVI_SEGMENT_ACTION_START)
        {
            open = type;
        }
        else if (open == type && record_point_map[i].action_cmd == NAVI_SEGMENT_ACTION_END)
        {
            open = WP_TYPE_NORMAL;
        }
        else
        {
            *open_type = WP_TYPE_NORMAL;
            return 0U;
        }
    }

    *open_type = open;
    return 1U;
}

uint8 Navi_Record_Get_Last_Action(void)
{
    return (record_point_count > 0U) ?
           (uint8)record_point_map[record_point_count - 1U].action_cmd : 0U;
}

uint8 Navi_Record_Get_Open_Segment_Type(void)
{
    WayPoint_Type open_type = WP_TYPE_NORMAL;

    if (!navi_record_segment_state(&open_type) || open_type == WP_TYPE_NORMAL)
    {
        return 0xFFU;
    }
    return (uint8)open_type;
}

static void navi_bridge_reset(uint8_t restore_low_height)
{
    if (restore_low_height)
    {
        pid_low_init();
    }
}

static uint8_t navi_segment_validate_route(void)
{
    WayPoint_Type open_type = WP_TYPE_NORMAL;
    uint16_t start_idx = 0U;
    uint16_t i;

    if (Runtime_Get_Vehicle_Mode() != VEHICLE_MODE_COURSE_3)
    {
        return 1U;
    }

    for (i = 0U; i < navi_ctrl.point_total_count; i++)
    {
        WayPoint_Type type = point_map[i].type;

        if (!point_map[i].valid || !Course3Segment_IsPairedType((uint8)type))
        {
            if (open_type != WP_TYPE_NORMAL)
            {
                IPC_LOG_Printf("\r\n[NAVI_SEGMENT] point %d interrupts open %s segment.\r\n",
                               i, get_enum_name(open_type));
                return 0U;
            }
            continue;
        }

        if (open_type == WP_TYPE_NORMAL)
        {
            if (point_map[i].action_cmd != NAVI_SEGMENT_ACTION_START)
            {
                IPC_LOG_Printf("\r\n[NAVI_SEGMENT] point %d (%s) is not a valid start.\r\n",
                               i, get_enum_name(type));
                return 0U;
            }
            start_idx = i;
            open_type = type;
        }
        else
        {
            if (type != open_type ||
                point_map[i].action_cmd != NAVI_SEGMENT_ACTION_END ||
                i != (uint16_t)(start_idx + 1U))
            {
                IPC_LOG_Printf("\r\n[NAVI_SEGMENT] point %d is not the paired %s end.\r\n",
                               i, get_enum_name(open_type));
                return 0U;
            }
            if (navi_get_two_points_distance(point_map[start_idx].x, point_map[start_idx].y,
                                             point_map[i].x, point_map[i].y) < 0.01f)
            {
                IPC_LOG_Printf("\r\n[NAVI_SEGMENT] %s points %d and %d overlap.\r\n",
                               get_enum_name(open_type), start_idx, i);
                return 0U;
            }
            open_type = WP_TYPE_NORMAL;
        }
    }

    if (open_type != WP_TYPE_NORMAL)
    {
        IPC_LOG_Printf("\r\n[NAVI_SEGMENT] %s start point %d has no end point.\r\n",
                       get_enum_name(open_type), start_idx);
        return 0U;
    }

    return 1U;
}

static uint8_t navi_course3_apply_line_lookahead(uint16_t target_idx,
                                                 float *azimuth,
                                                 float *turn_error)
{
    Navi_WayPoint_t *start;
    Navi_WayPoint_t *end;
    float segment_x;
    float segment_y;
    float segment_length_sq;
    float segment_length;
    float projection;
    float lookahead_ratio;
    float lookahead_x;
    float lookahead_y;

    if (azimuth == NULL || turn_error == NULL ||
        Runtime_Get_Vehicle_Mode() != VEHICLE_MODE_COURSE_3 ||
        target_idx == 0U || target_idx >= navi_ctrl.point_total_count ||
        !point_map[target_idx].valid || point_map[target_idx].type != WP_TYPE_NORMAL)
    {
        return 0U;
    }

    start = &point_map[target_idx - 1U];
    end = &point_map[target_idx];
    if (!start->valid)
    {
        return 0U;
    }

    segment_x = end->x - start->x;
    segment_y = end->y - start->y;
    segment_length_sq = segment_x * segment_x + segment_y * segment_y;
    if (segment_length_sq < 0.0001f)
    {
        return 0U;
    }

    segment_length = sqrtf(segment_length_sq);
    projection = ((robot_pose.x - start->x) * segment_x +
                  (robot_pose.y - start->y) * segment_y) / segment_length_sq;
    projection = constrain_float(projection, 0.0f, 1.0f);
    lookahead_ratio = projection + NAVI_COURSE3_LINE_LOOKAHEAD_DISTANCE / segment_length;
    lookahead_ratio = constrain_float(lookahead_ratio, 0.0f, 1.0f);
    lookahead_x = start->x + segment_x * lookahead_ratio;
    lookahead_y = start->y + segment_y * lookahead_ratio;

    if (navi_get_two_points_distance(robot_pose.x, robot_pose.y,
                                     lookahead_x, lookahead_y) < 0.01f)
    {
        lookahead_x = end->x;
        lookahead_y = end->y;
    }

    *azimuth = navi_get_two_points_azimuth(robot_pose.x, robot_pose.y,
                                           lookahead_x, lookahead_y);
    *turn_error = navi_limit_angle180(*azimuth - robot_pose.yaw);
    return 1U;
}

static void navi_course3_approach_reset(uint8_t restore_bridge_height)
{
    (void)restore_bridge_height;
    memset(&navi_course3_approach, 0, sizeof(navi_course3_approach));
}

static void navi_course3_approach_handoff(void)
{
    memset(&navi_course3_approach, 0, sizeof(navi_course3_approach));
}

static uint8_t navi_course3_approach_update(uint16_t target_idx,
                                            float distance,
                                            uint8_t nav_info_valid)
{
    Navi_WayPoint_t *target;

    if (Runtime_Get_Vehicle_Mode() != VEHICLE_MODE_COURSE_3 ||
        target_idx >= navi_ctrl.point_total_count ||
        target_idx >= NAVI_POINT_MAX)
    {
        navi_course3_approach_reset(1U);
        return 0U;
    }

    target = &point_map[target_idx];
    if (!target->valid ||
        !Course3Segment_IsPairedType((uint8)target->type) ||
        target->action_cmd != NAVI_SEGMENT_ACTION_START)
    {
        navi_course3_approach_reset(1U);
        return 0U;
    }

    if (navi_course3_approach.active)
    {
        if (navi_course3_approach.target_idx != target_idx ||
            navi_course3_approach.type != target->type)
        {
            navi_course3_approach_reset(1U);
            return 0U;
        }
        return 1U;
    }

    if (!nav_info_valid ||
        !Course3Segment_ShouldApproach(Runtime_Get_Vehicle_Mode(),
                                       (uint8)target->type,
                                       target->action_cmd,
                                       distance))
    {
        return 0U;
    }

    navi_course3_approach.active = 1U;
    navi_course3_approach.target_idx = target_idx;
    navi_course3_approach.type = target->type;
    Turn_Reset();
    navi_speed_profile_reset();
    IPC_LOG_Printf("\r\n[NAVI_COURSE3] approach %s start point %d: distance <= %.2f m, speed=%d.\r\n",
                   get_enum_name(target->type),
                   target_idx,
                   (double)NAVI_COURSE3_APPROACH_DISTANCE,
                   (int)NAVI_COURSE3_APPROACH_SPEED);
    return 1U;
}

static void navi_course3_angle_slew_reset(void)
{
    navi_course3_angle_slew_initialized = 0U;
    navi_course3_angle_slew_cmd = 0.0f;
}

static float navi_course3_angle_slew_apply(float desired_angle)
{
    float max_step = NAVI_COURSE3_ANGLE_SLEW_RATE_DEG_S * ENCODER_DT;

    if (Runtime_Get_Vehicle_Mode() != VEHICLE_MODE_COURSE_3)
    {
        return desired_angle;
    }
    if (!navi_course3_angle_slew_initialized)
    {
        navi_course3_angle_slew_cmd = navi_limit_angle180(target_angle);
        navi_course3_angle_slew_initialized = 1U;
    }

    navi_course3_angle_slew_cmd = Course3AngleSlew_Step(navi_course3_angle_slew_cmd,
                                                        desired_angle,
                                                        max_step);
    return navi_course3_angle_slew_cmd;
}

static pid_param_t navi_speed_pid;
static float navi_speed_last_output = 0.0f;

static void navi_speed_profile_reset(void)
{
    PidInit(&navi_speed_pid);
    navi_speed_last_output = 0.0f;
}

void navi_tracking_speed_profile_reset(void)
{
    navi_speed_profile_reset();
}

static float navi_get_reach_threshold(uint16_t target_idx)
{
    if (target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid)
    {
        return DISTANCE_THRESHOLD;
    }

    switch (point_map[target_idx].type)
    {
        case WP_TYPE_MINE_SWEEP:
            return DISTANCE_THRESHOLD * 0.5f;
        case WP_TYPE_CONE_CONE:
        case WP_TYPE_BRIDGE:
        case WP_TYPE_JUMP:
        case WP_TYPE_NORMAL:
        case WP_TYPE_STOP:
        case WP_TYPE_HOME:
        default:
            return DISTANCE_THRESHOLD;
    }
}

static float navi_calc_speed_plan_distance(uint16_t curr_idx, float current_distance, uint16_t *speed_target_idx)
{
    float path_distance = current_distance;
    uint16_t idx;

    if (speed_target_idx != NULL)
    {
        *speed_target_idx = curr_idx;
    }

    if (curr_idx >= navi_ctrl.point_total_count)
    {
        return current_distance;
    }

    idx = curr_idx;
    while (idx < (navi_ctrl.point_total_count - 1U) &&
           (point_map[idx].type == WP_TYPE_NORMAL ||
            point_map[idx].type == WP_TYPE_HOME ||
            (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3 &&
             Course3Segment_IsPairedType((uint8)point_map[idx].type))))
    {
        double dx = point_map[idx + 1U].x - point_map[idx].x;
        double dy = point_map[idx + 1U].y - point_map[idx].y;
        path_distance += (float)sqrt(dx * dx + dy * dy);
        idx++;
    }

    if (speed_target_idx != NULL)
    {
        *speed_target_idx = idx;
    }

    return path_distance;
}

static float navi_calc_turn_speed_limit(float turn_error_deg)
{
    float abs_turn = fabsf(turn_error_deg);

    if (abs_turn <= 20.0f)
    {
        return 300.0f;
    }
    if (abs_turn >= 90.0f)
    {
        return 100.0f;
    }

    return 300.0f - (abs_turn - 20.0f) * (200.0f / 70.0f);
}

static float navi_update_tracking_velocity(float distance, float stop_threshold, uint8_t reached, uint8_t nav_valid)
{
    float speed_kp = navi_speed_kp ? navi_speed_kp : Navi_Speed_Kp_init;
    float speed_ki = navi_speed_ki ? navi_speed_ki : Navi_Speed_Ki_init;
    float speed_kd = navi_speed_kd ? navi_speed_kd : Navi_Speed_Kd_init;
    float max_velocity = navi_speed_max > 0.0f ? navi_speed_max : Navi_Speed_Max_init;
    float max_step = navi_speed_max_step > 0.0f ? navi_speed_max_step : Navi_Speed_MaxStep_init;
    float raw_output;
    float delta;

    if (!nav_valid || reached || distance <= stop_threshold || max_velocity <= 0.0f)
    {
        navi_speed_profile_reset();
        return 0.0f;
    }

    PidChange(&navi_speed_pid, speed_kp, speed_ki, speed_kd);
    raw_output = PidLocCtrl(&navi_speed_pid, distance);
    raw_output = constrain_float(raw_output, 0.0f, max_velocity);

    delta = constrain_float(raw_output - navi_speed_last_output, -max_step, max_step);
    navi_speed_last_output = constrain_float(navi_speed_last_output + delta, 0.0f, max_velocity);

    return navi_speed_last_output;
}

static uint8_t navi_is_course1_smooth_point(uint16_t target_idx, uint16_t total_points)
{
    if (Runtime_Get_Vehicle_Mode() != VEHICLE_MODE_COURSE_1)
    {
        return 0U;
    }

    if (target_idx >= total_points || target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid)
    {
        return 0U;
    }

    if (target_idx >= (total_points - 1U) || point_map[target_idx].type == WP_TYPE_STOP)
    {
        return 0U;
    }

    return (point_map[target_idx].type == WP_TYPE_HOME ||
            point_map[target_idx].type == WP_TYPE_NORMAL) ? 1U : 0U;
}

//==================================================== function implementation =============================================
void Navi_Tracking_Init(void) {
  
    navi_ctrl.point_total_count = 0;
    navi_ctrl.point_current_idx = 0;
    navi_ctrl.navi_mode_map = 0;         //默认使用静态地图
    navi_ctrl.origin_set_flag = 0;
    navi_ctrl.trigger_record_type = WP_TYPE_NORMAL;
    navi_record_origin_cal_pending = 0U;
    navi_start_cal_pending = 0U;
    Navi_Yaw_Calibration_Cancel();
    record_point_count = 0;
    memset(record_point_map, 0, sizeof(record_point_map));        //系统启动时，生成一次静态地图保存在后台待命
    memset(static_point_map, 0, sizeof(static_point_map));
    navi_load_comprehensive_test_map();
}

//-------------------------------------------------------------------------------------------------------------------

//  路径预处理：线性插值

//当两点间距 >DISTANCE_THRESHOLD 时，自动插入中间点，确保前瞻逻辑平滑

//-------------------------------------------------------------------------------------------------------------------
void navi_path_optimize(void) {
    uint8_t count = navi_ctrl.point_total_count;
    if (count < 2) return;
    for (uint8_t i = 0; i < count - 1; i++) {
        float dist = navi_get_two_points_distance(point_map[i].x , point_map[i].y,  point_map[i+1].x , point_map[i+1].y);
        uint8_t paired_segment =
            (Course3Segment_IsPairedType((uint8)point_map[i].type) ||
             Course3Segment_IsPairedType((uint8)point_map[i + 1U].type)) ? 1U : 0U;
        if (!paired_segment && dist > INTERPOLATION_STEP && navi_ctrl.point_total_count < NAVI_POINT_MAX) {
            for (uint8_t j = navi_ctrl.point_total_count; j > i + 1; j--) {
                point_map[j] = point_map[j-1];
            }
            point_map[i+1].x = (point_map[i].x + point_map[i+2].x) / 2.0f;
            point_map[i+1].y = (point_map[i].y + point_map[i+2].y) / 2.0f;
            float angle_diff = navi_limit_angle180(point_map[i+2].yaw - point_map[i].yaw);
            point_map[i+1].yaw = navi_limit_angle180(point_map[i].yaw + angle_diff * 0.5f);
            point_map[i+1].type = WP_TYPE_NORMAL;
            point_map[i+1].valid = 1;
            navi_ctrl.point_total_count++;
            count++;
            i++; 
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------

// 记录当前Navi位置为航点          task_navigation_control函数中已调用 

// 首次调用时设置原点，后续调用记录指定类型的航点

// type: 航点类型

//-------------------------------------------------------------------------------------------------------------------
void navi_auto_record_task(void) {
    static float last_vofa_trigger = 0.0f;
    
    if (navi_ctrl.navi_mode_driver != 2) {
        last_vofa_trigger = vofa_trigger_record;
        return;
    }

    if (navi_record_origin_cal_pending)
    {
        target_velocity = 0.0f;
        vofa_trigger_record = 0.0f;
        navi_ctrl.trigger_record = 0U;

        if (Navi_Yaw_Calibration_Is_Active())
        {
            return;
        }

        if (!Navi_Yaw_Calibration_Consume_Done(NAVI_YAW_CAL_CONTEXT_RECORD_HOME))
        {
            navi_record_origin_cal_pending = 0U;
            IPC_LOG_Printf("\r\n[NAVI_YAW_CAL] HOME calibration cancelled.\r\n");
            return;
        }

        Navi_Data_Set_Origin(0);
        memset(record_point_map, 0, sizeof(record_point_map));
        record_point_map[0].x = 0.0f;
        record_point_map[0].y = 0.0f;
        record_point_map[0].yaw = 0.0f;
        record_point_map[0].type = WP_TYPE_HOME;
        record_point_map[0].action_cmd = 0U;
        record_point_map[0].valid = 1U;
        record_point_count = 1U;
        navi_ctrl.origin_set_flag = 1U;
        navi_record_origin_cal_pending = 0U;
        navi_record_update_status();
        IPC_Nav_Record_Mark_Dirty();
        IPC_LOG_Printf("\r\n[NAVI_YAW_CAL] HOME calibration done; point 1 recorded.\r\n");
        return;
    }

    if (vofa_trigger_record > 2.5f) {
        navi_record_undo_last();
        vofa_trigger_record = 0.0f;
        last_vofa_trigger = 0.0f;
        return;
    }

    if (!robot_pose.is_valid) {
        last_vofa_trigger = vofa_trigger_record;
        return;
    }

    if (vofa_trigger_record > 1.5f) {      // Navi_TrigRecord = 2：收到后立即打点一次，不依赖 0->1 边沿。
        navi_ctrl.trigger_record = 1;
        vofa_trigger_record = 0.0;
        last_vofa_trigger = 0.0;
    } 

    else if (vofa_trigger_record > 0.5f && last_vofa_trigger <= 0.5f) {  // Navi_TrigRecord = 1：保持原有 0->1 边沿触发打点。
        navi_ctrl.trigger_record = 1;
        vofa_trigger_record = 0.0;
        last_vofa_trigger = 0.0;
    } else {
        last_vofa_trigger = vofa_trigger_record;
    }
    
    if (!navi_ctrl.trigger_record)   return;

    if (record_point_count >= NAVI_POINT_MAX) {
        navi_ctrl.trigger_record = 0;
        IPC_LOG_Printf("\r\n>>> [打点失败] 记录地图已满，无法继续添加航点 <<<\r\n");
        return;
    }

    if (navi_ctrl.origin_set_flag == 0) {
        navi_ctrl.trigger_record = 0U;
        target_velocity = 0.0f;
        target_angle = IMU_data.filter_result.yaw;
        Turn_Reset();
        Navi_Yaw_Calibration_Start(NAVI_YAW_CAL_CONTEXT_RECORD_HOME);
        navi_record_origin_cal_pending = 1U;
        IPC_LOG_Printf("\r\n[NAVI_YAW_CAL] Keep still: collecting HOME yaw for 2 seconds.\r\n");
        return;
    } else {
        uint16_t idx = record_point_count;
        WayPoint_Type record_type = (WayPoint_Type)wifi_remote_type;
        uint16_t action_cmd = (uint16_t)wifi_in_action;

        if (Runtime_Get_Vehicle_Mode() == VEHICLE_MODE_COURSE_3)
        {
            WayPoint_Type open_type = WP_TYPE_NORMAL;
            if (!navi_record_segment_state(&open_type))
            {
                navi_ctrl.trigger_record = 0;
                IPC_LOG_Printf("\r\n>>> [打点失败] 成对路段航点序列无效，请清空或撤销后重试 <<<\r\n");
                return;
            }
            if (open_type != WP_TYPE_NORMAL && record_type != open_type)
            {
                navi_ctrl.trigger_record = 0;
                IPC_LOG_Printf("\r\n>>> [打点失败] 请先记录%s结束点 <<<\r\n", get_enum_name(open_type));
                return;
            }
            if (Course3Segment_IsPairedType((uint8)record_type))
            {
                action_cmd = (open_type == record_type) ?
                             NAVI_SEGMENT_ACTION_END : NAVI_SEGMENT_ACTION_START;
                if (action_cmd == NAVI_SEGMENT_ACTION_END && record_point_count > 0U &&
                    navi_get_two_points_distance(record_point_map[record_point_count - 1U].x,
                                                 record_point_map[record_point_count - 1U].y,
                                                 robot_pose.x,
                                                 robot_pose.y) < 0.01f)
                {
                    navi_ctrl.trigger_record = 0U;
                    IPC_LOG_Printf("\r\n>>> [NAVI_RECORD] segment start/end distance must be at least 1 cm <<<\r\n");
                    return;
                }
            }
        }

        record_point_map[idx].x = robot_pose.x;
        record_point_map[idx].y = robot_pose.y;
        record_point_map[idx].yaw = robot_pose.yaw;
        record_point_map[idx].type = record_type;
        record_point_map[idx].action_cmd = action_cmd;
        record_point_map[idx].valid = 1;
        record_point_count++;
        navi_record_update_status();
        IPC_Nav_Record_Mark_Dirty();

        IPC_LOG_Printf(" >>> [手动打点] 记录点[%03d]: X=%s%d.%02d, Y=%s%d.%02d | 类型:%s | 动作指令:%d <<<\r\n",
            idx,
            F_ARG(record_point_map[idx].x),
            F_ARG(record_point_map[idx].y),
            get_enum_name(record_point_map[idx].type),
            record_point_map[idx].action_cmd);
    }

    navi_ctrl.trigger_record = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//静态地图导入 (精度测定专用)
// 目的：直接赋值坐标点，检查小车跑坐标单位1时，实际物理位移
//-------------------------------------------------------------------------------------------------------------------
void navi_load_comprehensive_test_map(void) {
    Navi_Data_Set_Origin(1);  
    static_point_count = 0;
    uint8_t i = 0;  
    
    /* ========================================================================= */
    /* 区块 1：基础精度测定地图 (原始版本，用于测定X/Y坐标轴运行基准精度)          */
    /* ========================================================================= */
    
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f,   0.0f, WP_TYPE_HOME,   0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.6f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.2f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.2f,  0.6f,  90.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.6f,  0.6f, 180.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.6f, 180.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f, 270.0f, WP_TYPE_STOP,   0, 1};
    

    /* ========================================================================= */
    /* 区块 2：S形状弯（绕圆锥桶）专属测试地图                                   */
    /* 测试要点：观察小车在连续改变Y轴坐标时，车身倾角和寻迹是否平滑             */
    /* ========================================================================= */
    /*
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f,   0.0f, WP_TYPE_HOME,   0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.5f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.0f, -0.6f, -45.0f, WP_TYPE_CONE_CONE, 0, 1}; // 左偏绕桶
    static_point_map[i++] = (Navi_WayPoint_t){1.5f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};    // 回中
    static_point_map[i++] = (Navi_WayPoint_t){2.0f,  0.6f,  45.0f, WP_TYPE_CONE_CONE, 0, 1}; // 右偏绕桶
    static_point_map[i++] = (Navi_WayPoint_t){2.5f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};    // 回正
    static_point_map[i++] = (Navi_WayPoint_t){3.0f,  0.0f,   0.0f, WP_TYPE_STOP,   0, 1};
    */

    /* ========================================================================= */
    /* 区块 3：定点排雷专属测试地图                                              */
    /* 测试要点：观察小车到达排雷点后，是否能成功切入 FSM_MINE_PROCESSING 状态   */
    /* ========================================================================= */
    /*
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f,   0.0f, WP_TYPE_HOME,   0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.8f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.5f,  0.0f,   0.0f, WP_TYPE_MINE_SWEEP, 0, 1};// 定点排雷任务
    static_point_map[i++] = (Navi_WayPoint_t){2.2f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){3.0f,  0.0f,   0.0f, WP_TYPE_STOP,   0, 1};
    */

    /* ========================================================================= */
    /* 区块 4：轮腿组综合测试赛道 (当前未注释，处于生效状态)                     */
    /* 测试要点：跳跃 -> 单边桥 -> 排雷 的综合状态机衔接能力                     */
    /* ========================================================================= */
    /*
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f,   0.0f, WP_TYPE_HOME,   0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.0f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.8f,  0.0f,   0.0f, WP_TYPE_JUMP,   0, 1};    // 爆发抬腿跳跃
    static_point_map[i++] = (Navi_WayPoint_t){2.8f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){3.5f,  0.0f,   0.0f, WP_TYPE_BRIDGE, 0, 1};  // 侧倾自适应单边桥
    static_point_map[i++] = (Navi_WayPoint_t){4.5f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){5.2f,  0.0f,   0.0f, WP_TYPE_MINE_SWEEP, 0, 1};// 定点排雷
    static_point_map[i++] = (Navi_WayPoint_t){6.0f,  0.0f,   0.0f, WP_TYPE_STOP,   0, 1};    // 终点停车
    */
    
    /* ========================================================================= */
    /* 区块 5：跳跃动作测试赛道                                                       */
    /* 测试要点：常规循迹-->跳跃-->停车                                                 */
    /* ========================================================================= */  
    /*
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f,   0.0f, WP_TYPE_HOME,   0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.2f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.8f,  0.0f,   0.0f, WP_TYPE_JUMP,   0, 1};         //起跳台阶航点
    static_point_map[i++] = (Navi_WayPoint_t){2.1f,  0.0f,   0.0f, WP_TYPE_STOP,   0, 1};
    */
    
    
    // 最终记录有效地图航点总数
    /* ========================================================================= */
    /* 方案 6：原地跳跃安全验证地图（默认注释，不启用）                         */
    /* 说明：WP_TYPE_JUMP + action_cmd=0 不跳跃，只原地旋转一圈，用于验证打点。 */
    /*       真正跳跃请把 action_cmd 改为非 0。                                */
    /* ========================================================================= */
    /*
    static_point_map[i++] = (Navi_WayPoint_t){0.0f, 0.0f, 0.0f, WP_TYPE_HOME, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.0f, 0.0f, 0.0f, WP_TYPE_JUMP, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.0f, 0.0f, 0.0f, WP_TYPE_STOP, 0, 1};
    */

    /* ========================================================================= */
    /* 方案 7：正常前跳测试地图（默认注释，不启用）                              */
    /* 说明：此处 action_cmd=1 表示真正跳跃；改回 0 则只旋转一圈验证触发。       */
    /* ========================================================================= */
    /*
    static_point_map[i++] = (Navi_WayPoint_t){0.0f, 0.0f, 0.0f, WP_TYPE_HOME, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.8f, 0.0f, 0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.2f, 0.0f, 0.0f, WP_TYPE_JUMP, 1, 1};
    static_point_map[i++] = (Navi_WayPoint_t){2.0f, 0.0f, 0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){2.3f, 0.0f, 0.0f, WP_TYPE_STOP, 0, 1};
    */
    static_point_count = i;  
}

//-------------------------------------------------------------------------------------------------------------------
//   导航模式识别         
//   0：停止与地图选择     1：发车巡航模式       2：专属打点录制模式
//-------------------------------------------------------------------------------------------------------------------
void task_navigation_control(void) {
    Navi_Yaw_Calibration_Tick();

    //一、接收目前导航的工作状态
    navi_ctrl.navi_mode_driver = (uint8_t)vofa_mode_driver;
    navi_ctrl.navi_mode_map    = (uint8_t)vofa_mode_map;
    uint8_t current_print_cmd   = (uint8_t)vofa_print_pose_en;
    
    // ============================= 1. 状态与地图管理引擎 ==========================  
    static uint8_t last_mode_driver = 0;
    static uint8_t last_mode_map = 255;
    static uint8_t active_nav_map_type = 0;              //真正生效的地图类型
    
    if (last_mode_driver != navi_ctrl.navi_mode_driver || last_mode_map != navi_ctrl.navi_mode_map)  {
        if (navi_ctrl.navi_mode_driver != 2U && navi_record_origin_cal_pending)
        {
            Navi_Yaw_Calibration_Cancel();
            navi_record_origin_cal_pending = 0U;
        }
        if (navi_ctrl.navi_mode_driver != 1U && navi_start_cal_pending)
        {
            Navi_Yaw_Calibration_Cancel();
            navi_start_cal_pending = 0U;
        }

        // --- 驱动模式切换处理 ---
        if (navi_ctrl.navi_mode_driver == 0) {     
            if (last_mode_driver != 0) {
                navi_course3_approach_reset(1U);
                navi_course3_angle_slew_reset();
                navi_bridge_reset(1U);
                Navi_Action_Reset_New_Course3_Segments();
                IPC_LOG_Printf(">>> [状态] 系统已切入 停车与地图选择模式 <<<\r\n");
            }

        }
        else if (navi_ctrl.navi_mode_driver == 1 && last_mode_driver != 1) {            //发车时，将选择的地图复印到导航地图上
            memset(point_map, 0, sizeof(point_map)); 
            active_nav_map_type = navi_ctrl.navi_mode_map; // 更新并锁定待发车的地图
            navi_ctrl.point_current_idx = 0;
            
            if (active_nav_map_type == 0) {
                memcpy(point_map, static_point_map, sizeof(static_point_map));
                navi_ctrl.point_total_count = static_point_count;
                IPC_LOG_Printf("\r\n[发车加载] 已重载【静态地图】\r\n");
            }
            else if (active_nav_map_type == 1) {
                memcpy(point_map, record_point_map, sizeof(record_point_map));
                navi_ctrl.point_total_count = record_point_count;
                IPC_LOG_Printf("\r\n[发车加载] 已重载【打点地图】，共 %d 个点待命\r\n", navi_ctrl.point_total_count);
            }

            navi_bridge_reset(1U);
            if (navi_ctrl.point_total_count < 2U)
            {
                target_velocity = 0.0f;
                vofa_mode_driver = 0.0f;
                navi_ctrl.navi_mode_driver = 0U;
                IPC_LOG_Printf("\r\n[NAVI] Start rejected: route needs at least HOME and one target.\r\n");
            }
            else if (!navi_segment_validate_route())
            {
                target_velocity = 0.0f;
                vofa_mode_driver = 0.0f;
                navi_ctrl.navi_mode_driver = 0;
                IPC_LOG_Printf("\r\n>>> [发车失败] 单边桥、颠簸和台阶斜坡必须按开始点、结束点成对记录 <<<\r\n");
            }
            else
            {
                #if ENABLE_PATH_INTERPOLATION         // 线性插值宏控制
                if (navi_ctrl.point_total_count >= 2) {
                    navi_path_optimize();
                    IPC_LOG_Printf(" [路径加载] 已开启线性插值并完成加密，当前总点数: %d\r\n", navi_ctrl.point_total_count);
                }
                #endif

                target_velocity = 0.0f;
                target_angle = IMU_data.filter_result.yaw;
                Turn_Reset();
                navi_speed_profile_reset();
                navi_course3_approach_reset(1U);
                navi_course3_angle_slew_reset();
                is_action_busy = 0;
                Navi_Yaw_Calibration_Start(NAVI_YAW_CAL_CONTEXT_NAV_START);
                navi_start_cal_pending = 1U;
                IPC_LOG_Printf("\r\n[NAVI_YAW_CAL] Keep still: collecting start yaw for 2 seconds.\r\n");
            }
        }
        else if (navi_ctrl.navi_mode_driver == 2) {
            // Driver=2 专注打点模式。只有 map=1 时开始准备打点
            if (navi_ctrl.navi_mode_map == 1 && last_mode_map != 1) {
                IPC_LOG_Printf(" [状态变更] 进入手动打点录制模式，后台随时待命...\r\n");
                IPC_LOG_Printf(" (提示: 若未执行过 Map=2 清图，新打的点将自动追加在已有轨迹后)\r\n");
            }
        }
        
        last_mode_driver = navi_ctrl.navi_mode_driver;
        last_mode_map = navi_ctrl.navi_mode_map;
    }
       
    // ============================= 2. 全面分步式打印预览引擎 (吸收自队友代码) ==========================
    Navi_VOFA_Preview_Task((uint8_t)vofa_print_pose_en);
    
    // ============================= 3. 底层循环任务路由 ==========================
    switch(navi_ctrl.navi_mode_driver)
    {
        case 0:    
            navi_speed_profile_reset();
           Navigation_Pose_Monitor_Task();     // 打印位姿(print=1时有效)
           break;  
        
        case 1: {  // --- 循迹模式 ---自主追踪导航                
            uint16_t total_points = navi_ctrl.point_total_count;
            uint16_t curr_idx = navi_ctrl.point_current_idx;
            static uint8_t smooth_speed_hold_ticks = 0U;
            uint8_t course3_approach_active = 0U;
            if (total_points == 0)     break; 
            if (Vehicle_Is_Emergency_Stop())
            {
                target_velocity = 0.0f;
                vofa_mode_driver = 0.0f;
                navi_ctrl.navi_mode_driver = 0;
                navi_bridge_reset(1U);
                navi_course3_approach_reset(1U);
                navi_course3_angle_slew_reset();
                Navi_Action_Reset_New_Course3_Segments();
                navi_speed_profile_reset();
                break;
            }

            if (navi_start_cal_pending)
            {
                target_velocity = 0.0f;
                if (Navi_Yaw_Calibration_Is_Active())
                {
                    break;
                }

                if (!Navi_Yaw_Calibration_Consume_Done(NAVI_YAW_CAL_CONTEXT_NAV_START))
                {
                    navi_start_cal_pending = 0U;
                    vofa_mode_driver = 0.0f;
                    navi_ctrl.navi_mode_driver = 0U;
                    IPC_LOG_Printf("\r\n[NAVI_YAW_CAL] Start calibration cancelled.\r\n");
                    break;
                }

                Navi_Data_Set_Origin(0);
                navi_parse_global_path();
                navi_speed_profile_reset();
                Turn_Reset();
                is_action_busy = 0U;
                navi_start_cal_pending = 0U;
                IPC_LOG_Printf("\r\n[NAVI_YAW_CAL] Start calibration done; navigation enabled.\r\n");
                break;
            }
            
            //            //动态前瞻搜索 ---计算当前速度自适应的前瞻距离  0.2f 是速度增益系数
//
//            float dynamic_lookahead = BASE_LOOKAHEAD_DIST + (RPM_TO_M_COEFF(fabsf(now_velocity)) * LOOKAHEAD_VEL_GAIN);
//            dynamic_lookahead = fmaxf(BASE_LOOKAHEAD_DIST, fminf(1.0f, dynamic_lookahead));             // 限幅防止前瞻过远或过近：下限卡在 BASE_LOOKAHEAD_DIST，上限卡在 1.0f
//
//            //寻找最近的一个前瞻点
//            uint16_t lookahead_idx= total_points - 1;
//            for (uint16_t i = curr_idx; i < total_points; i++) {
//              
//                float dist = navi_get_two_points_distance( robot_pose.x  ,robot_pose.y  ,  point_map[i].x, point_map[i].y);
//                
//                if (dist >= dynamic_lookahead) {
//                
//                    lookashead_idx = i;
//                    
//                    break; // 找到足够远的前瞻点
//                }
//            }
            

            // 计算到前瞻点的方位和距离
            uint16_t lookahead_idx = curr_idx;
            float azimuth = 0.0f, distance = 0.0f;
            float print_turn_angle = 0.0f; 
            uint8_t nav_info_valid = navi_calcnavinfo(lookahead_idx, &azimuth, &distance);
            uint8_t reached_current = navi_isreach_target_point(curr_idx);
            uint8_t smooth_zone_active = 0U;
            uint8_t smooth_switch_triggered = 0U;
            uint16_t smooth_prev_idx = curr_idx;
            
            if (nav_info_valid) {
                print_turn_angle = navi_limit_angle180((float)azimuth - robot_pose.yaw);
                (void)navi_course3_apply_line_lookahead(curr_idx, &azimuth, &print_turn_angle);
            }

            course3_approach_active = navi_course3_approach_update(curr_idx,
                                                                  distance,
                                                                  nav_info_valid);

            if (smooth_speed_hold_ticks > 0U)
            {
                smooth_speed_hold_ticks--;
                smooth_zone_active = 1U;
            }

            if (!is_action_busy &&
                nav_info_valid &&
                navi_is_course1_smooth_point(curr_idx, total_points) &&
                Navi_Smooth_Should_Advance(distance, NAVI_SMOOTH_REACH_RADIUS_M, 0U))
            {
                smooth_zone_active = 1U;
                if (navi_switch_nexttargetpoint())
                {
                    curr_idx = navi_ctrl.point_current_idx;
                    lookahead_idx = curr_idx;
                    nav_info_valid = navi_calcnavinfo(lookahead_idx, &azimuth, &distance);
                    reached_current = navi_isreach_target_point(curr_idx);
                    print_turn_angle = 0.0f;
                    if (nav_info_valid)
                    {
                        print_turn_angle = navi_limit_angle180((float)azimuth - robot_pose.yaw);
                    }
                    smooth_speed_hold_ticks = NAVI_SMOOTH_POST_ADVANCE_TICKS;
                    smooth_switch_triggered = 1U;
                    IPC_LOG_Printf("\r\n[NAVI_SMOOTH] point %d -> %d inside radius %.2f m\r\n",
                                   (unsigned int)smooth_prev_idx,
                                   (unsigned int)curr_idx,
                                   (double)NAVI_SMOOTH_REACH_RADIUS_M);
                }
            }
            Navi_Action_Manager(navi_ctrl.point_current_idx);  // 动作接管识别 
            if (is_action_busy)
            {
                if (course3_approach_active)
                {
                    navi_course3_approach_handoff();
                    course3_approach_active = 0U;
                }
                navi_course3_angle_slew_reset();
            }
            
            if (Navi_Action_Consume_Done(curr_idx)) {
                if (curr_idx >= (total_points - 1U)) {
                    target_velocity = 0.0f;
                    vofa_mode_driver = 0.0f;
                    navi_ctrl.navi_mode_driver = 0;
                    navi_speed_profile_reset();
                    break;
                }

                IPC_LOG_Printf("\r\n[NAVI_ACTION] action point %d done, switch next point.\r\n", curr_idx);
                if (navi_switch_nexttargetpoint()) {
                    uint16_t next_idx = navi_ctrl.point_current_idx;
                    IPC_LOG_Printf(" [NAVI_ACTION] next point X=%s%d.%02d, Y=%s%d.%02d | Type:%s\r\n",
                        F_ARG(point_map[next_idx].x), F_ARG(point_map[next_idx].y),
                        get_enum_name(point_map[next_idx].type));
                }
                break;
            }
            
            if (!is_action_busy) {  
                // 1. 转向角始终由 Tracking 计算
                float desired_target_angle =
                    navi_limit_angle180(IMU_data.filter_result.yaw - print_turn_angle);
                target_angle = navi_course3_angle_slew_apply(desired_target_angle);
                
                // 2. 速度赋值被完美收束在此处
                if (course3_approach_active)
                {
                    target_velocity = nav_info_valid ? NAVI_COURSE3_APPROACH_SPEED : 0.0f;
                }
                else
                {
#if (USE_HOST_TARGET_VELOCITY == 0)
                   
                    target_velocity = DEFAULT_TRACKING_VELOCITY;       // 恒定速度，专门用于安全调试动作
                    
#elif (USE_HOST_TARGET_VELOCITY == 2)              // PID ????滮???
                uint16_t speed_target_idx = curr_idx;
                float speed_plan_distance = navi_calc_speed_plan_distance(curr_idx, distance, &speed_target_idx);
                float speed_stop_threshold = navi_get_reach_threshold(speed_target_idx);
                float speed_cmd = navi_update_tracking_velocity(
                    speed_plan_distance,
                    speed_stop_threshold,
                    0,
                    nav_info_valid
                );
                float turn_speed_limit = navi_calc_turn_speed_limit(print_turn_angle);
                uint8_t apply_smooth_speed_limit =
                    (smooth_zone_active || smooth_switch_triggered) ? 1U : 0U;

                target_velocity = Navi_Smooth_Resolve_Target_Velocity(
                    speed_cmd,
                    turn_speed_limit,
                    NAVI_SMOOTH_ZONE_SPEED_LIMIT,
                    apply_smooth_speed_limit);
#endif
                }
            }   else {
                navi_speed_profile_reset();
            }
            
            
      
 // 打印导航信息            
#if Print_location           
            if (vofa_print_pose_en > 0.5f && current_print_cmd != 2) {
                static uint16_t dynamic_print_delay = 0;
                float current_period = vofa_print_pose_period;
                if (current_period < 500.0f)         current_period = 500.0f; 
                uint16_t target_delay_ticks = (uint16_t)(current_period / (ENCODER_DT * 1000.0f));
                                                
                if (++dynamic_print_delay >= target_delay_ticks) {
                    dynamic_print_delay = 0;
                    // 更新打印信息，加入累计角度和圈数
                    IPC_LOG_Printf("[位姿监测] 车姿(%s%d.%02d, %s%d.%02d) | Yaw=%s%d.%02d | "
                                 "累计角度:%s%d.%02d | 圈数:%s%d.%02d | "
                                 "去往前瞻点(%s%d.%02d, %s%d.%02d) 类型:%s | "
                                 "距目标:%s%d.%02d | 需转向:%s%d.%02d度\r\n",
                        F_ARG(robot_pose.x), F_ARG(robot_pose.y), F_ARG(robot_pose.yaw), 
                        F_ARG((float)robot_pose.cumulative_yaw), // 累计角度
                        F_ARG(robot_pose.turns),                 // 累计圈数
                        F_ARG(point_map[lookahead_idx].x), F_ARG(point_map[lookahead_idx].y), 
                        get_enum_name(point_map[lookahead_idx].type),
                        F_ARG((float)distance), F_ARG(print_turn_angle));
                }
            }
#endif   
            
            // 到达判定：完全解耦，只要物理距离到了，Tracking 就自然往下切点
           if (reached_current) {
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == curr_idx &&
                    (is_action_busy || action_fsm.state != FSM_IDLE ||
                     point_map[curr_idx].type == WP_TYPE_MINE_SWEEP ||
                     point_map[curr_idx].type == WP_TYPE_JUMP)) {
                    break;
                }
                
                if (curr_idx >= (total_points - 1U) || point_map[curr_idx].type == WP_TYPE_STOP) {
                    target_velocity = 0.0f;
                    vofa_mode_driver = 0.0f;
                    navi_ctrl.navi_mode_driver = 0;
                    navi_course3_approach_reset(1U);
                    navi_course3_angle_slew_reset();
                    navi_speed_profile_reset();
                    IPC_LOG_Printf("\r\n[NAVI] final/stop point %d reached, navigation stopped.\r\n", curr_idx);
                    break;
                }
                
                IPC_LOG_Printf("\r\n============= >>> [到达事件] 已到达航点 [%d] <<< =============\r\n", curr_idx);
               if (navi_switch_nexttargetpoint()) {
                  uint16_t next_idx = navi_ctrl.point_current_idx;
                  IPC_LOG_Printf(" [到达航点【%d】] 下一个航点为：X=%s%d.%02d, Y=%s%d.%02d  |  类型: %s\r\n", 
                           curr_idx, F_ARG(point_map[next_idx].x), F_ARG(point_map[next_idx].y), 
                           get_enum_name(point_map[next_idx].type));
               } 
           }
           
           //           // 防切角死锁：判断是否离下一个点更近
//           else if (curr_idx < navi_ctrl.point_total_count - 1) {              
//                float dist_to_curr = navi_get_two_points_distance(robot_pose.x, robot_pose.y, point_map[curr_idx].x, point_map[curr_idx].y);
//                float dist_to_next = navi_get_two_points_distance(robot_pose.x, robot_pose.y, point_map[curr_idx+1].x, point_map[curr_idx+1].y);
//                
//                // 如果离下一个点更近，说明已经越过了当前点所在切面，强制切走！          避免动作接管期发生强制切点
//                if (dist_to_next < dist_to_curr && !is_action_busy) {
//                    navi_switch_nexttargetpoint();
//                    IPC_LOG_Printf(" [防死锁切角] 强制越过切面，切换至航点: %d\r\n", navi_ctrl.point_current_idx);
//                }
//           }  
           
           
           break;
        }

        case 2: {         
            //拖动到清空地图，就清空，然后就可以直接记录了（无论是否退出状态  2  ）
            if (navi_ctrl.navi_mode_map == 2) {             //清图
                record_point_count = 0;
                navi_ctrl.origin_set_flag = 0;  // 重新标定原点
                navi_ctrl.point_total_count = 0;
                navi_ctrl.point_current_idx = 0;

                memset(record_point_map, 0, sizeof(record_point_map));
                memset(point_map, 0, sizeof(point_map));
                navi_record_update_status();
                IPC_Nav_Record_Mark_Dirty();
                vofa_mode_map = 1.0f;
                navi_ctrl.navi_mode_map = 1;
                IPC_LOG_Printf("\r\n============= >>> [地图清空] 后台记录地图已成功清空 <<< =============\r\n");
            }

          // --- 打点与配置模式 --- 
               navi_auto_record_task();  
           break;
        } 
        
        default:  
            break;
    }
}


//=================================航点管理======================================

/**

 * @brief 获取指定索引的航点信息

 * @details 读取指定索引的航点数据，索引无效时返回无效航点（valid=0）

 * @param[in] index: 航点索引

 * @return GPS_WayPoint_t - 航点结构体，索引无效时返回valid=0的无效航点

 */

Navi_WayPoint_t navi_get_point(uint16_t index) {

    Navi_WayPoint_t invalid_point = {0};

    invalid_point.valid = 0;

    if(index >= NAVI_POINT_MAX) {

        return invalid_point;

    }

    return point_map[index];

}





/**

 * @brief 切换到下一个目标航点

 * @details 将当前目标航点索引自增，用于航点序列导航

 * @return uint8 - 1: 切换成功；0: 已到最后一个航点

 * @note 切换成功后current_target_idx指向新的目标航点

 */

uint8 navi_switch_nexttargetpoint(void) {

    if(navi_ctrl.point_current_idx >= navi_ctrl.point_total_count- 1)  return 0; // 已到最后一个航点
    
    navi_ctrl.point_current_idx++;

    return 1;

}



//----------------------------------------------------------------------------------------------------------------
// 计算当前位置到目标航点的导航信息   (X为北，Y东,顺时针角度加)

//  target_idx: 目标航点索引

// azimuth: 方位角指针（单位：度）

// distance: 直线距离指针（单位：米）

//----------------------------------------------------------------------------------------------------------------

uint8 navi_calcnavinfo(uint16_t target_idx, float *azimuth, float *distance) {

    if(target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid || !robot_pose.is_valid)                return 0;

    *distance = navi_get_two_points_distance(robot_pose.x, robot_pose.y, point_map[target_idx].x, point_map[target_idx].y);   

    

    // 边界处理：如果已经在目标点附近，方位角保持当前航向，防止抖动

    if (*distance < 0.01) *azimuth = robot_pose.yaw;

    else {

        *azimuth = navi_get_two_points_azimuth(robot_pose.x, robot_pose.y, point_map[target_idx].x, point_map[target_idx].y);

    }

    return 1;

}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介      计算平面坐标系下两点之间的距离 (单精度优化版)
// 参数说明      x1              第一个点的 X 坐标
// 参数说明      y1              第一个点的 Y 坐标
// 参数说明      x2              第二个点的 X 坐标
// 参数说明      y2              第二个点的 Y 坐标
// 返回参数      float           返回两点直线距离
//-------------------------------------------------------------------------------------------------------------------
float navi_get_two_points_distance(float x1, float y1, float x2, float y2)
{  
    float dx = x2 - x1;
    float dy = y2 - y1;
    
    // 使用 sqrtf 替代 sqrt，避免隐式转换为 double 导致运算变慢
    float distance = sqrtf(dx * dx + dy * dy); 

    return distance;  
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      计算平面坐标系下从第一个点到第二个点的方位角 (单精度优化版)
// 参数说明      x1              第一个点的 X 坐标
// 参数说明      y1              第一个点的 Y 坐标
// 参数说明      x2              第二个点的 X 坐标
// 参数说明      y2              第二个点的 Y 坐标
// 返回参数      float           返回方位角（0.0 至 360.0 度）
// 备注信息      适用条件：X轴正方向为正北，Y轴正方向为正东。正北为0度，顺时针增加。
//-------------------------------------------------------------------------------------------------------------------
float navi_get_two_points_azimuth(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;

    return RAD_TO_ANGLE(atan2f(dy, dx));
}



//-------------------------------------------------------------------------------------------------------------------

// 判断是否到达目标航点           0 ：   未到达               1：到达

// 基于平面坐标（x/y）计算当前位置与目标航点的距离，根据不同航点类型可设置特定的到达阈值

// arget_idx: 目标航点索引

//--------------------------------------------------------------------------------------------------------------

uint8 navi_isreach_target_point(uint16 target_idx) {

    if(target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid || !robot_pose.is_valid) {

        return 0;

    }

    double dx = point_map[target_idx].x - robot_pose.x;

    double dy = point_map[target_idx].y - robot_pose.y;

    double distance_sq = dx*dx + dy*dy;
    
    float current_threshold = navi_get_reach_threshold(target_idx);

    return (distance_sq <= (current_threshold * current_threshold)) ? 1 : 0;

}



/**
 * @brief  导航关闭时的位姿监测打印任务
 * @param  delta_ms：多长时间打印一次
 */
void Navigation_Pose_Monitor_Task(void)
{
    static uint16_t print_timer = 0;  // 【修改1】改为 uint16_t 防止溢出 (例如3000ms需要300次中断)

    // 1. 判断是否需要跳过打印：
    // 当 循迹模式开启(>=0.5) 或者 打印未使能(<0.5) 时，不打印并退出
    if (vofa_mode_driver >= 0.5f || (uint8_t)vofa_print_pose_en != 1) { 
        print_timer = 0; 
        return; 
    }

    // 2. 获取设定的打印周期，并限制最小值为 500ms
    float current_period = vofa_print_pose_period;
    if (current_period < 500.0f) {
        current_period = 500.0f;
    }

    // 3. 计算需要多少次中断才能触发打印 (ENCODER_DT = 10ms)
    // 例如：1000ms / 10ms = 100 次中断
    uint16_t target_ticks = (uint16_t)(current_period /( ENCODER_DT * 1000.0f));  

    // 4. 非阻塞延时控制打印频率
    if (++print_timer >= target_ticks)
    {
        print_timer = 0; // 重置计时器

        // 从 IPC 共享数据或全局变量获取最新的位姿
        float cur_x = core_a_status.nav_x;
        float cur_y = core_a_status.nav_y;
        float cur_v = core_a_status.nav_v;
        float cur_yaw = core_a_status.nav_yaw;

        // 执行打印，格式：[位姿监测] 车姿(X, Y) 速度, 航向
        LOG_Printf("[位姿监测] 车姿(%s%d.%02d, %s%d.%02d) 速度:%s%d.%02d m/s 航向:%s%d.%02d°\r\n",
                   F_S(cur_x), F_I(cur_x), F_D2(cur_x),
                   F_S(cur_y), F_I(cur_y), F_D2(cur_y),
                   F_S(cur_v), F_I(cur_v), F_D2(cur_v),
                   F_S(cur_yaw), F_I(cur_yaw), F_D2(cur_yaw));
    }
}



// 将航点类型枚举转换为对应的中文字符串
const char* get_enum_name(WayPoint_Type type) {
    switch (type) {
        case WP_TYPE_NORMAL:      return "普通循迹";
        case WP_TYPE_MINE_SWEEP:  return "定点排雷";
        case WP_TYPE_JUMP:        return "三级跳";
        case WP_TYPE_BUMP:        return "颠簸路段";
        case WP_TYPE_STAIR_RAMP:  return "台阶斜坡";
        case WP_TYPE_STOP:        return "终点返航";
        case WP_TYPE_HOME:        return "原点";
        case WP_TYPE_BRIDGE:      return "单边桥";
        case WP_TYPE_CONE_CONE:   return "绕圆锥桶";
        default:                  return "未知类型";
    }
}
     


static void Navi_VOFA_Preview_Task(uint8_t current_print_cmd) {
    static uint8_t last_print_cmd = 0;  
    static uint8_t print_step = 0;            // 打印步骤: 0=空闲, 1=打头部, 2=打坐标点, 3=打尾部
    static uint16_t print_map_idx = 0;        // 当前打印的航点进度
    static uint8_t preview_map_type = 0;      // 正在打印的地图类型
    static uint8_t print_tick_delay = 0;      // IPC发送节流阀计数器

    // 1. 边缘触发：检测到上位机发送的预览命令 (cmd == 2)
    if (current_print_cmd == 2 && last_print_cmd != 2) {
        // 判定当前需要预览的地图类型 (如果选了2清图模式，则打印上一次生效的地图)
        preview_map_type = (navi_ctrl.navi_mode_map == 2) ? 0 : navi_ctrl.navi_mode_map;
        print_step = 1;         // 挂挡，准备第一步（打头部）
        print_map_idx = 0;
        print_tick_delay = 10;  // 设为10使其在下方判断中能够立即触发第一步，无需等待
    }
    last_print_cmd = current_print_cmd;

    // 2. 状态机后台严格分步分时执行
    if (print_step > 0) {
        if (++print_tick_delay >= 10) {  // 10个tick = 100ms 节流 (每0.1秒打一行，绝对不卡车)
            print_tick_delay = 0;
            
            switch (print_step) {
                case 1: // --- 步骤 1：打印头部 ---
                    if (preview_map_type == 0 && static_point_count > 0) {
                        IPC_LOG_Printf("\r\n============ 查阅预览：当前选择【静态地图】共 %d 个点 ============\r\n", static_point_count);
                        print_step = 2; // 下一个周期进入坐标打印
                    } 
                    else if (preview_map_type == 1 && record_point_count > 0) {
                        IPC_LOG_Printf("\r\n============ 查阅预览：当前选择【打点地图】共 %d 个点 ============\r\n", record_point_count);
                        print_step = 2; 
                    }
                    else {
                        IPC_LOG_Printf("\r\n============ 查阅失败：当前地图尚未制作或为空！ =================\r\n");
                        print_step = 3; // 地图为空，直接去步骤3尾部清理
                    }
                    break;

                case 2: // --- 步骤 2：逐行打印坐标点 ---
                    if (preview_map_type == 0) { // 打印静态地图
                        if (print_map_idx < static_point_count) {
                            IPC_LOG_Printf(" 航点[%02d]: X=%s%d.%02d, Y=%s%d.%02d, 类型=%s\r\n", 
                                    print_map_idx, 
                                    F_ARG(static_point_map[print_map_idx].x), 
                                    F_ARG(static_point_map[print_map_idx].y), 
                                    get_enum_name(static_point_map[print_map_idx].type));
                            print_map_idx++;
                        }
                        if (print_map_idx >= static_point_count) print_step = 3; 
                    } 
                    else if (preview_map_type == 1) { // 打印打点地图
                        if (print_map_idx < record_point_count) {
                            IPC_LOG_Printf(" 航点[%02d]: X=%s%d.%02d, Y=%s%d.%02d, 类型=%s\r\n", 
                                    print_map_idx, 
                                    F_ARG(record_point_map[print_map_idx].x), 
                                    F_ARG(record_point_map[print_map_idx].y), 
                                    get_enum_name(record_point_map[print_map_idx].type));
                            print_map_idx++;
                        }
                        if (print_map_idx >= record_point_count) print_step = 3; 
                    }
                    break;
                
                case 3: // --- 步骤 3：打印尾部并彻底释放锁 ---
                    IPC_LOG_Printf("=================================================================\r\n");
                    // 【核心复位】：强行清零上位机指令，下一次按“2”时才能重新触发边缘检测
                    vofa_print_pose_en = 0.0f; 
                    last_print_cmd = 0;
                    print_step = 0; // 回归空闲
                    break;

                default:
                    print_step = 0;
                    break;
            }
        }
    }
}
//=====================================================静态函数定义=======================================================================




//1.坐标原点重置时间：开机时 IMU 会有几秒钟的收敛期，必须等 IMU 稳定后再将当前位置设置为 (0,0)。
//
//2.导航算法必须放在严格定时的 10ms 中断里，并且要放在 balance_control() 之后，因为导航依赖它算出来的 now_velocity（实时线速度）。
