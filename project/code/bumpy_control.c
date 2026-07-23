/*********************************************************************************************************************
 * @file    bumpy_control.c
 * @brief   颠簸路段检测、统计记录及导航穿越动作实现。
 *
 * @details
 * 1. 公共检测核心根据垂向加速度、姿态角速度和轮速波动识别颠簸路段。
 * 2. 工程适配层负责从当前项目读取传感器、遥控和运行状态，并生成调试日志。
 * 3. 导航 Action 门面负责按照配置表执行穿越、恢复和异常退出流程。
 * 4. 检测核心只生成速度、航向和腿部建议；是否写入公共控制目标由适配层决定。
 * 5. 当前默认关闭工程输出接管，避免未完成实车标定时直接影响车辆控制。
 *
 * @note    位移使用 m，速度使用 m/s；姿态使用 deg，角速度使用 rad/s；时间使用 ms 或 s。
 *********************************************************************************************************************/

#include "bumpy_control.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "imu.h"
#include "ipc_shared_data.h"
#include "jump_control.h"
#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "param.h"
#include "remote.h"
#include "runtime_status.h"
#include "vehicle_supervisor.h"

/*============================================================
 * 1. 默认配置与工程运行状态
 *============================================================*/

/*
 * 工程输出接管总开关：
 * 0：只检测、记录和上报，不覆盖公共目标速度/航向；
 * 1：允许工程适配层把算法建议写入 target_velocity/target_angle。
 * 未完成实车标定和安全验证前必须保持为 0。
 */
#define BUMPY_PROJECT_CONTROL_OUTPUT_ENABLED    (0U)

/* 以下参数是首次采集数据的起点，不是比赛冻结参数。 */
static const BumpyConfig_t g_bumpy_default_config = {
    .mode = BUMP_MODE_OBSERVE,

    .nominal_dt_ms = 10U,
    .rough_window_ms = 200U,
    .speed_window_ms = 200U,

    .forward_cmd_min = 30.0f,
    .forward_mps_min = 0.03f,

    .rough_enter_mps2 = 0.980665f,
    .pitch_rate_rms_enter_rps = 0.60f,
    .speed_std_enter_mps = 0.04f,
    .enter_confirm_ms = 120U,

    .rough_exit_mps2 = 0.441299f,
    .pitch_rate_rms_exit_rps = 0.35f,
    .roll_rate_rms_exit_rps = 0.35f,
    .speed_std_exit_mps = 0.025f,
    .pitch_stable_deg = 6.0f,
    .roll_stable_deg = 6.0f,
    .min_active_ms = 800U,
    .exit_confirm_ms = 350U,
    .recover_ms = 500U,

    .danger_pitch_deg = 25.0f,
    .danger_roll_deg = 25.0f,
    .danger_rate_rps = 6.10f,
    .danger_confirm_ms = 100U,
    .max_active_ms = 5000U,

    .stuck_forward_mps = 0.01f,
    .stuck_enable_ms = 200U,
    .stuck_confirm_ms = 500U,

    .impact_threshold_mps2 = 2.157463f,
    .impact_refractory_ms = 150U,

    .assist_speed_min = 0.0f,
    .assist_speed_max = 800.0f,
    .speed_slew_per_s = 1000.0f,
    .assist_on_enter_candidate = 0U,
    .lock_yaw = 1U,

    .static_leg_y_offset_m = 0.0f,
    .leg_slew_mps = 0.02f,
    .dynamic_leg_gain = 0.0f,
    .dynamic_leg_limit_m = 0.0f,

    .debug_sample_period_ms = 50U
};

/* 工程适配层使用的唯一检测实例、输入输出快照和日志状态。 */
static BumpyRuntime_t g_bumpy_project_runtime;
static BumpyConfig_t g_bumpy_project_config;
static BumpyInput_t g_bumpy_project_input;
static BumpyOutput_t g_bumpy_project_output;
static BumpyDebugData_t g_bumpy_project_debug;
static uint8_t g_bumpy_project_initialized = 0U;
static uint8_t g_bumpy_project_enabled = 1U;
static uint32_t g_bumpy_project_debug_elapsed_ms = 0U;

static volatile uint8_t g_bumpy_log_state_pending = 0U;
static volatile uint8_t g_bumpy_log_sample_pending = 0U;
static volatile uint8_t g_bumpy_log_report_pending = 0U;
static BumpyState_t g_bumpy_log_previous_state = BUMP_STATE_DISABLED;
static BumpyState_t g_bumpy_log_current_state = BUMP_STATE_DISABLED;
static uint32_t g_bumpy_log_event_id = 0U;

/*
 * 导航颠簸动作参数。
 * crossing_speed / recover_speed 沿用 target_velocity 的工程量纲；
 * 位移使用 m，时间使用 ms。
 */
/*============================================================
 * 2. 导航穿越动作配置与运行上下文
 *============================================================*/

/* 单个颠簸航点的穿越参数，速度量纲与项目 target_velocity 保持一致。 */
typedef struct
{
    BumpyMode_t mode;
    uint8_t feature_mask;
    float detect_speed;
    uint16_t detect_timeout_ms;
    float detect_max_distance_m;
    float crossing_speed;
    float crossing_speed_medium;
    float crossing_speed_heavy;
    float speed_step_per_tick;
    uint16_t min_crossing_ms;
    uint16_t crossing_timeout_ms;
    float expected_bump_length_m;
    float exit_distance_margin_m;
    float recover_speed;
    uint16_t recover_ms;
    float static_leg_y_offset_m;
    float leg_ramp_step_m;
    float fixed_forward_m;
    float fixed_right_m;
    uint8_t allow_timeout_finish;
} BumpyActionProfile_t;

#define BUMP_FEATURE_DISTANCE_FALLBACK   (1U << 0)
#define BUMP_FEATURE_ADAPTIVE_SPEED      (1U << 1)
#define BUMP_FEATURE_STATIC_LEG          (1U << 2)

typedef enum
{
    BUMP_ACTION_PHASE_IDLE = 0,
    BUMP_ACTION_PHASE_DETECT,
    BUMP_ACTION_PHASE_CROSSING,
    BUMP_ACTION_PHASE_RECOVER
} BumpyActionPhase_t;

typedef enum
{
    BUMP_SPEED_LEVEL_NORMAL = 0,
    BUMP_SPEED_LEVEL_MEDIUM,
    BUMP_SPEED_LEVEL_HEAVY
} BumpySpeedLevel_t;

typedef struct
{
    BumpyActionPhase_t phase;
    uint16_t profile_id;
    uint32_t detect_elapsed_ms;
    uint32_t crossing_elapsed_ms;
    uint32_t recover_elapsed_ms;
    float detect_distance_m;
    float crossing_distance_m;
    float detect_start_x_m;
    float detect_start_y_m;
    float entry_x_m;
    float entry_y_m;
    float entry_yaw_deg;
    float hold_yaw_deg;
    float active_speed_cmd;
    float leg_base_x_m;
    float leg_base_y_m;
    float leg_y_cmd_m;
    uint8_t speed_level;
    uint8_t speed_level_pending;
    uint8_t speed_level_pending_ticks;
    uint16_t speed_level_hold_ms;
    uint8_t leg_override_active;
    uint8_t started;
    uint8_t enter_detected;
    uint8_t exit_detected;
    uint8_t finish_allowed;
    uint8_t pose_update_active;
    uint8_t report_captured;
    uint32_t log_pending_mask;
    BumpyExitReason_t exit_reason;
    BumpyReport_t report;
} BumpyActionContext_t;

static const BumpyActionProfile_t g_bumpy_action_profiles[] =
{
    {
        .mode = BUMP_MODE_SPEED_YAW,
        .feature_mask = 0U,
        .detect_speed = 220.0f,
        .detect_timeout_ms = 2500U,
        .detect_max_distance_m = 0.80f,
        .crossing_speed = 320.0f,
        .crossing_speed_medium = 280.0f,
        .crossing_speed_heavy = 240.0f,
        .speed_step_per_tick = 10.0f,
        .min_crossing_ms = 800U,
        .crossing_timeout_ms = 5000U,
        .expected_bump_length_m = 1.20f,
        .exit_distance_margin_m = 0.25f,
        .recover_speed = 220.0f,
        .recover_ms = 350U,
        .static_leg_y_offset_m = 0.0f,
        .leg_ramp_step_m = 0.0f,
        .fixed_forward_m = 1.20f,
        .fixed_right_m = 0.0f,
        .allow_timeout_finish = 1U
    },
    {
        .mode = BUMP_MODE_SPEED_YAW,
        .feature_mask = BUMP_FEATURE_DISTANCE_FALLBACK |
                        BUMP_FEATURE_ADAPTIVE_SPEED,
        .detect_speed = 220.0f,
        .detect_timeout_ms = 2500U,
        .detect_max_distance_m = 0.80f,
        .crossing_speed = 320.0f,
        .crossing_speed_medium = 280.0f,
        .crossing_speed_heavy = 240.0f,
        .speed_step_per_tick = 10.0f,
        .min_crossing_ms = 800U,
        .crossing_timeout_ms = 5000U,
        .expected_bump_length_m = 1.20f,
        .exit_distance_margin_m = 0.25f,
        .recover_speed = 220.0f,
        .recover_ms = 350U,
        .static_leg_y_offset_m = 0.0f,
        .leg_ramp_step_m = 0.0f,
        .fixed_forward_m = 1.20f,
        .fixed_right_m = 0.0f,
        .allow_timeout_finish = 1U
    },
    {
        .mode = BUMP_MODE_STATIC_LEG,
        .feature_mask = BUMP_FEATURE_DISTANCE_FALLBACK |
                        BUMP_FEATURE_ADAPTIVE_SPEED |
                        BUMP_FEATURE_STATIC_LEG,
        .detect_speed = 220.0f,
        .detect_timeout_ms = 2500U,
        .detect_max_distance_m = 0.80f,
        .crossing_speed = 320.0f,
        .crossing_speed_medium = 280.0f,
        .crossing_speed_heavy = 240.0f,
        .speed_step_per_tick = 10.0f,
        .min_crossing_ms = 800U,
        .crossing_timeout_ms = 5000U,
        .expected_bump_length_m = 1.20f,
        .exit_distance_margin_m = 0.25f,
        .recover_speed = 220.0f,
        .recover_ms = 350U,
        .static_leg_y_offset_m = 0.005f,
        .leg_ramp_step_m = 0.0002f,
        .fixed_forward_m = 1.20f,
        .fixed_right_m = 0.0f,
        .allow_timeout_finish = 1U
    }
};

#define BUMPY_ACTION_PROFILE_COUNT \
    ((uint16_t)(sizeof(g_bumpy_action_profiles) / \
                sizeof(g_bumpy_action_profiles[0])))

#ifndef BUMPY_ACTION_POSE_UPDATE_MODE
#define BUMPY_ACTION_POSE_UPDATE_MODE       (1U)
#endif

#if (BUMPY_ACTION_POSE_UPDATE_MODE != 1U) && \
    (BUMPY_ACTION_POSE_UPDATE_MODE != 2U)
#error "BUMPY_ACTION_POSE_UPDATE_MODE must be 1 or 2"
#endif

#define BUMPY_ACTION_LOG_TRIGGER               (1UL << 0)
#define BUMPY_ACTION_LOG_ENTER                 (1UL << 1)
#define BUMPY_ACTION_LOG_ENTER_CROSSING        (1UL << 2)
#define BUMPY_ACTION_LOG_EXIT                  (1UL << 3)
#define BUMPY_ACTION_LOG_DETECT_TIMEOUT         (1UL << 4)
#define BUMPY_ACTION_LOG_DETECT_DISTANCE_LIMIT  (1UL << 5)
#define BUMPY_ACTION_LOG_ACTION_TIMEOUT         (1UL << 6)
#define BUMPY_ACTION_LOG_DISTANCE_FALLBACK      (1UL << 7)
#define BUMPY_ACTION_LOG_SPEED_LEVEL            (1UL << 8)
#define BUMPY_ACTION_LOG_DONE                   (1UL << 9)
#define BUMPY_ACTION_LOG_FAULT                  (1UL << 10)
#define BUMPY_ACTION_LOG_PROFILE_FALLBACK       (1UL << 11)

static BumpyActionContext_t g_bumpy_action;

/*============================================================
 * 3. 基础数学、校验与计时工具
 *============================================================*/

/* 返回浮点数绝对值，避免核心算法依赖额外宏。 */
static float bumpy_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float bumpy_maxf(float lhs, float rhs)
{
    return (lhs >= rhs) ? lhs : rhs;
}

static float bumpy_clampf(float value, float minimum, float maximum)
{
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

static float bumpy_safe_sqrt(float value)
{
    return sqrtf((value > 0.0f) ? value : 0.0f);
}

static float bumpy_limit_angle180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static uint8_t bumpy_float_valid(float value)
{
    return (value == value && value <= FLT_MAX && value >= -FLT_MAX) ? 1U : 0U;
}

static uint32_t bumpy_timer_add(uint32_t timer_ms, uint32_t dt_ms)
{
    if (timer_ms > (0xFFFFFFFFUL - dt_ms))
    {
        return 0xFFFFFFFFUL;
    }
    return timer_ms + dt_ms;
}

static uint32_t bumpy_confirm_timer(uint32_t timer_ms,
                                    uint8_t condition,
                                    uint32_t dt_ms)
{
    return condition ? bumpy_timer_add(timer_ms, dt_ms) : 0U;
}

static uint8_t bumpy_window_samples(uint16_t window_ms, uint16_t nominal_dt_ms)
{
    uint32_t samples;

    if (nominal_dt_ms == 0U)
    {
        return 1U;
    }

    samples = ((uint32_t)window_ms + nominal_dt_ms - 1U) / nominal_dt_ms;
    if (samples < 1U)
    {
        samples = 1U;
    }
    if (samples > BUMPY_WINDOW_MAX_SAMPLES)
    {
        samples = BUMPY_WINDOW_MAX_SAMPLES;
    }
    return (uint8_t)samples;
}

/* 检查配置范围、窗口容量及阈值关系，非法配置禁止进入状态机。 */
static uint8_t bumpy_config_valid(const BumpyConfig_t *config)
{
    uint32_t rough_capacity_ms;
    uint32_t speed_capacity_ms;

    if (config == NULL || config->nominal_dt_ms == 0U)
    {
        return 0U;
    }
    if (config->mode > BUMP_MODE_DYNAMIC_LEG)
    {
        return 0U;
    }

    rough_capacity_ms =
        (uint32_t)config->nominal_dt_ms * BUMPY_WINDOW_MAX_SAMPLES;
    speed_capacity_ms = rough_capacity_ms;
    if (config->rough_window_ms == 0U || config->speed_window_ms == 0U ||
        config->rough_window_ms > rough_capacity_ms ||
        config->speed_window_ms > speed_capacity_ms)
    {
        return 0U;
    }
    if (config->rough_enter_mps2 <= config->rough_exit_mps2 ||
        config->rough_exit_mps2 < 0.0f ||
        config->forward_cmd_min < 0.0f ||
        config->forward_mps_min < 0.0f)
    {
        return 0U;
    }
    if (config->enter_confirm_ms == 0U || config->exit_confirm_ms == 0U ||
        config->danger_confirm_ms == 0U || config->max_active_ms == 0U ||
        config->stuck_confirm_ms == 0U ||
        config->impact_refractory_ms == 0U)
    {
        return 0U;
    }
    if (config->assist_speed_min > config->assist_speed_max ||
        config->speed_slew_per_s < 0.0f || config->leg_slew_mps < 0.0f ||
        config->dynamic_leg_limit_m < 0.0f)
    {
        return 0U;
    }
    return 1U;
}

/* 检查输入是否为有限值，并过滤明显超出物理范围的异常数据。 */
static uint8_t bumpy_input_valid(const BumpyInput_t *input)
{
    if (input == NULL || !bumpy_float_valid(input->dt_s) ||
        input->dt_s < 0.001f || input->dt_s > 0.100f)
    {
        return 0U;
    }

#define BUMPY_CHECK_FLOAT(field) \
    do { if (!bumpy_float_valid(input->field)) return 0U; } while (0)
    BUMPY_CHECK_FLOAT(user_speed_cmd);
    BUMPY_CHECK_FLOAT(user_yaw_cmd_deg);
    BUMPY_CHECK_FLOAT(control_yaw_deg);
    BUMPY_CHECK_FLOAT(pose_x_m);
    BUMPY_CHECK_FLOAT(pose_y_m);
    BUMPY_CHECK_FLOAT(pose_yaw_deg);
    BUMPY_CHECK_FLOAT(fused_forward_mps);
    BUMPY_CHECK_FLOAT(vertical_accel_mps2);
    BUMPY_CHECK_FLOAT(pitch_deg);
    BUMPY_CHECK_FLOAT(roll_deg);
    BUMPY_CHECK_FLOAT(pitch_rate_rps);
    BUMPY_CHECK_FLOAT(roll_rate_rps);
    BUMPY_CHECK_FLOAT(yaw_rate_rps);
    BUMPY_CHECK_FLOAT(left_forward_mps);
    BUMPY_CHECK_FLOAT(right_forward_mps);
    BUMPY_CHECK_FLOAT(base_leg_x_m);
    BUMPY_CHECK_FLOAT(base_leg_y_m);
#undef BUMPY_CHECK_FLOAT

    if (bumpy_absf(input->vertical_accel_mps2) > 100.0f ||
        bumpy_absf(input->pitch_deg) > 180.0f ||
        bumpy_absf(input->roll_deg) > 180.0f ||
        bumpy_absf(input->pitch_rate_rps) > 50.0f ||
        bumpy_absf(input->roll_rate_rps) > 50.0f ||
        bumpy_absf(input->yaw_rate_rps) > 50.0f ||
        bumpy_absf(input->left_forward_mps) > 10.0f ||
        bumpy_absf(input->right_forward_mps) > 10.0f ||
        bumpy_absf(input->pose_x_m) > 1000000.0f ||
        bumpy_absf(input->pose_y_m) > 1000000.0f)
    {
        return 0U;
    }
    return 1U;
}

static uint32_t bumpy_dt_ms(const BumpyInput_t *input)
{
    uint32_t dt_ms = (uint32_t)(input->dt_s * 1000.0f + 0.5f);
    return (dt_ms > 0U) ? dt_ms : 1U;
}

static void bumpy_clear_output(BumpyOutput_t *output, BumpyState_t state)
{
    memset(output, 0, sizeof(*output));
    output->state = state;
    output->exit_reason = BUMP_EXIT_NONE;
}

static void bumpy_set_state(BumpyRuntime_t *runtime, BumpyState_t state)
{
    runtime->state = state;
    runtime->state_elapsed_ms = 0U;
}

/*============================================================
 * 4. 滑动窗口、特征与统计量维护
 *============================================================*/

/* 清空加速度/角速度/轮速窗口，并按配置重新计算有效采样点数。 */
static void bumpy_reset_windows(BumpyRuntime_t *runtime,
                                const BumpyConfig_t *config)
{
    memset(runtime->accel_buffer, 0, sizeof(runtime->accel_buffer));
    memset(runtime->pitch_rate_sq_buffer, 0,
           sizeof(runtime->pitch_rate_sq_buffer));
    memset(runtime->roll_rate_sq_buffer, 0,
           sizeof(runtime->roll_rate_sq_buffer));
    memset(runtime->speed_buffer, 0, sizeof(runtime->speed_buffer));

    runtime->accel_sum = 0.0f;
    runtime->accel_sq_sum = 0.0f;
    runtime->pitch_rate_sq_sum = 0.0f;
    runtime->roll_rate_sq_sum = 0.0f;
    runtime->rough_index = 0U;
    runtime->rough_count = 0U;
    runtime->rough_samples =
        bumpy_window_samples(config->rough_window_ms, config->nominal_dt_ms);

    runtime->speed_sum = 0.0f;
    runtime->speed_sq_sum = 0.0f;
    runtime->speed_index = 0U;
    runtime->speed_count = 0U;
    runtime->speed_samples =
        bumpy_window_samples(config->speed_window_ms, config->nominal_dt_ms);

    memset(&runtime->feature, 0, sizeof(runtime->feature));
}

static void bumpy_clear_candidate(BumpyRuntime_t *runtime,
                                  const BumpyConfig_t *config)
{
    runtime->enter_confirm_elapsed_ms = 0U;
    runtime->exit_confirm_elapsed_ms = 0U;
    runtime->danger_elapsed_ms = 0U;
    runtime->stuck_elapsed_ms = 0U;
    runtime->active_elapsed_ms = 0U;
    runtime->segment_started = 0U;
    runtime->segment_confirmed = 0U;
    bumpy_reset_windows(runtime, config);
}

static void bumpy_set_disabled(BumpyRuntime_t *runtime,
                               const BumpyConfig_t *config,
                               BumpyExitReason_t reason)
{
    runtime->last_exit_reason = reason;
    bumpy_clear_candidate(runtime, config);
    bumpy_set_state(runtime, BUMP_STATE_DISABLED);
}

static void bumpy_save_boundary(BumpyRuntime_t *runtime,
                                const BumpyInput_t *input,
                                BumpyBoundarySnapshot_t *snapshot)
{
    snapshot->elapsed_s = runtime->total_elapsed_s;
    snapshot->left_total_m = runtime->left_total_m;
    snapshot->right_total_m = runtime->right_total_m;
    snapshot->pose_x_m = input->pose_x_m;
    snapshot->pose_y_m = input->pose_y_m;
    snapshot->pose_yaw_deg = input->pose_yaw_deg;
    snapshot->control_yaw_deg = input->control_yaw_deg;
}

/* 更新环形窗口，并计算粗糙度、姿态角速度 RMS 和速度标准差。 */
static void bumpy_update_windows(BumpyRuntime_t *runtime,
                                 const BumpyInput_t *input)
{
    float pitch_rate_sq;
    float roll_rate_sq;
    float forward_mps;
    float accel_mean;
    float accel_variance;
    float speed_mean;
    float speed_variance;
    uint8_t index;

    pitch_rate_sq = input->pitch_rate_rps * input->pitch_rate_rps;
    roll_rate_sq = input->roll_rate_rps * input->roll_rate_rps;
    forward_mps =
        (input->left_forward_mps + input->right_forward_mps) * 0.5f;

    index = runtime->rough_index;
    if (runtime->rough_count >= runtime->rough_samples)
    {
        runtime->accel_sum -= runtime->accel_buffer[index];
        runtime->accel_sq_sum -=
            runtime->accel_buffer[index] * runtime->accel_buffer[index];
        runtime->pitch_rate_sq_sum -= runtime->pitch_rate_sq_buffer[index];
        runtime->roll_rate_sq_sum -= runtime->roll_rate_sq_buffer[index];
    }
    else
    {
        runtime->rough_count++;
    }

    runtime->accel_buffer[index] = input->vertical_accel_mps2;
    runtime->pitch_rate_sq_buffer[index] = pitch_rate_sq;
    runtime->roll_rate_sq_buffer[index] = roll_rate_sq;
    runtime->accel_sum += input->vertical_accel_mps2;
    runtime->accel_sq_sum +=
        input->vertical_accel_mps2 * input->vertical_accel_mps2;
    runtime->pitch_rate_sq_sum += pitch_rate_sq;
    runtime->roll_rate_sq_sum += roll_rate_sq;
    runtime->rough_index++;
    if (runtime->rough_index >= runtime->rough_samples)
    {
        runtime->rough_index = 0U;
    }

    index = runtime->speed_index;
    if (runtime->speed_count >= runtime->speed_samples)
    {
        runtime->speed_sum -= runtime->speed_buffer[index];
        runtime->speed_sq_sum -=
            runtime->speed_buffer[index] * runtime->speed_buffer[index];
    }
    else
    {
        runtime->speed_count++;
    }
    runtime->speed_buffer[index] = forward_mps;
    runtime->speed_sum += forward_mps;
    runtime->speed_sq_sum += forward_mps * forward_mps;
    runtime->speed_index++;
    if (runtime->speed_index >= runtime->speed_samples)
    {
        runtime->speed_index = 0U;
    }

    accel_mean = runtime->accel_sum / (float)runtime->rough_count;
    accel_variance =
        runtime->accel_sq_sum / (float)runtime->rough_count -
        accel_mean * accel_mean;
    speed_mean = runtime->speed_sum / (float)runtime->speed_count;
    speed_variance =
        runtime->speed_sq_sum / (float)runtime->speed_count -
        speed_mean * speed_mean;

    runtime->feature.roughness_mps2 = bumpy_safe_sqrt(accel_variance);
    runtime->feature.pitch_rate_rms_rps = bumpy_safe_sqrt(
        runtime->pitch_rate_sq_sum / (float)runtime->rough_count);
    runtime->feature.roll_rate_rms_rps = bumpy_safe_sqrt(
        runtime->roll_rate_sq_sum / (float)runtime->rough_count);
    runtime->feature.forward_mps = forward_mps;
    runtime->feature.speed_std_mps = bumpy_safe_sqrt(speed_variance);
    runtime->feature.wheel_diff_mps =
        bumpy_absf(input->left_forward_mps - input->right_forward_mps);
    runtime->feature.rough_window_ready =
        (runtime->rough_count >= runtime->rough_samples) ? 1U : 0U;
    runtime->feature.speed_window_ready =
        (runtime->speed_count >= runtime->speed_samples) ? 1U : 0U;
}

/* 在首次进入候选区时重置本次颠簸事件的累计统计。 */
static void bumpy_reset_stats(BumpyRuntime_t *runtime,
                              const BumpyInput_t *input,
                              const BumpyConfig_t *config)
{
    runtime->stat_speed_sum = 0.0f;
    runtime->stat_speed_sq_sum = 0.0f;
    runtime->stat_rough_sum = 0.0f;
    runtime->stat_min_speed = runtime->feature.forward_mps;
    runtime->stat_max_speed = runtime->feature.forward_mps;
    runtime->stat_max_abs_accel = bumpy_absf(input->vertical_accel_mps2);
    runtime->stat_max_roughness = runtime->feature.roughness_mps2;
    runtime->stat_max_abs_pitch = bumpy_absf(input->pitch_deg);
    runtime->stat_max_abs_roll = bumpy_absf(input->roll_deg);
    runtime->stat_max_pitch_rate = bumpy_absf(input->pitch_rate_rps);
    runtime->stat_max_roll_rate = bumpy_absf(input->roll_rate_rps);
    runtime->stat_impact_count = 0U;
    runtime->stat_slip_count = 0U;
    runtime->stat_sample_count = 0U;
    runtime->impact_elapsed_ms = config->impact_refractory_ms;
}

/* 累计当前事件的速度、粗糙度、冲击次数和打滑样本。 */
static void bumpy_update_stats(BumpyRuntime_t *runtime,
                               const BumpyConfig_t *config,
                               const BumpyInput_t *input,
                               uint32_t dt_ms)
{
    float forward_mps = runtime->feature.forward_mps;

    runtime->stat_speed_sum += forward_mps;
    runtime->stat_speed_sq_sum += forward_mps * forward_mps;
    runtime->stat_rough_sum += runtime->feature.roughness_mps2;
    runtime->stat_min_speed =
        (forward_mps < runtime->stat_min_speed) ?
        forward_mps : runtime->stat_min_speed;
    runtime->stat_max_speed =
        (forward_mps > runtime->stat_max_speed) ?
        forward_mps : runtime->stat_max_speed;
    runtime->stat_max_abs_accel = bumpy_maxf(
        runtime->stat_max_abs_accel,
        bumpy_absf(input->vertical_accel_mps2));
    runtime->stat_max_roughness = bumpy_maxf(
        runtime->stat_max_roughness, runtime->feature.roughness_mps2);
    runtime->stat_max_abs_pitch = bumpy_maxf(
        runtime->stat_max_abs_pitch, bumpy_absf(input->pitch_deg));
    runtime->stat_max_abs_roll = bumpy_maxf(
        runtime->stat_max_abs_roll, bumpy_absf(input->roll_deg));
    runtime->stat_max_pitch_rate = bumpy_maxf(
        runtime->stat_max_pitch_rate, bumpy_absf(input->pitch_rate_rps));
    runtime->stat_max_roll_rate = bumpy_maxf(
        runtime->stat_max_roll_rate, bumpy_absf(input->roll_rate_rps));

    if (runtime->stat_sample_count < 0xFFFFU)
    {
        runtime->stat_sample_count++;
    }
    if (input->slip_level != 0U && runtime->stat_slip_count < 0xFFFFU)
    {
        runtime->stat_slip_count++;
    }

    runtime->impact_elapsed_ms =
        bumpy_timer_add(runtime->impact_elapsed_ms, dt_ms);
    if (bumpy_absf(input->vertical_accel_mps2) >=
            config->impact_threshold_mps2 &&
        runtime->impact_elapsed_ms >= config->impact_refractory_ms)
    {
        if (runtime->stat_impact_count < 0xFFFFU)
        {
            runtime->stat_impact_count++;
        }
        runtime->impact_elapsed_ms = 0U;
    }
}

static void bumpy_save_exit_stats(BumpyRuntime_t *runtime)
{
    runtime->exit_stat_speed_sum = runtime->stat_speed_sum;
    runtime->exit_stat_speed_sq_sum = runtime->stat_speed_sq_sum;
    runtime->exit_stat_rough_sum = runtime->stat_rough_sum;
    runtime->exit_stat_min_speed = runtime->stat_min_speed;
    runtime->exit_stat_max_speed = runtime->stat_max_speed;
    runtime->exit_stat_max_abs_accel = runtime->stat_max_abs_accel;
    runtime->exit_stat_max_roughness = runtime->stat_max_roughness;
    runtime->exit_stat_max_abs_pitch = runtime->stat_max_abs_pitch;
    runtime->exit_stat_max_abs_roll = runtime->stat_max_abs_roll;
    runtime->exit_stat_max_pitch_rate = runtime->stat_max_pitch_rate;
    runtime->exit_stat_max_roll_rate = runtime->stat_max_roll_rate;
    runtime->exit_stat_impact_count = runtime->stat_impact_count;
    runtime->exit_stat_slip_count = runtime->stat_slip_count;
    runtime->exit_stat_sample_count = runtime->stat_sample_count;
}

static void bumpy_begin_candidate(BumpyRuntime_t *runtime,
                                  const BumpyConfig_t *config,
                                  const BumpyInput_t *input,
                                  uint32_t dt_ms)
{
    bumpy_save_boundary(runtime, input, &runtime->entry_candidate);
    bumpy_reset_stats(runtime, input, config);
    runtime->segment_started = 1U;
    runtime->segment_confirmed = 0U;
    runtime->hold_control_yaw_deg = input->control_yaw_deg;
    runtime->last_assist_speed = input->user_speed_cmd;
    runtime->last_leg_y_cmd_m = input->base_leg_y_m;
    runtime->enter_confirm_elapsed_ms = dt_ms;
    bumpy_update_stats(runtime, config, input, dt_ms);
    bumpy_set_state(runtime, BUMP_STATE_ENTER_CONFIRM);
}

/*============================================================
 * 5. 进入/退出判定与辅助输出
 *============================================================*/

/* 同时满足前进门槛、粗糙度门槛和至少一个次级特征时，判定为进入候选。 */
static uint8_t bumpy_enter_candidate(const BumpyRuntime_t *runtime,
                                     const BumpyConfig_t *config,
                                     const BumpyInput_t *input)
{
    uint8_t forward_gate;
    uint8_t rough_gate;
    uint8_t secondary_gate;

    forward_gate =
        (input->user_speed_cmd > config->forward_cmd_min &&
         runtime->feature.forward_mps > config->forward_mps_min) ? 1U : 0U;
    rough_gate =
        (runtime->feature.roughness_mps2 > config->rough_enter_mps2) ? 1U : 0U;
    secondary_gate =
        (runtime->feature.pitch_rate_rms_rps >
             config->pitch_rate_rms_enter_rps ||
         runtime->feature.speed_std_mps > config->speed_std_enter_mps) ? 1U : 0U;

    return (runtime->feature.rough_window_ready &&
            runtime->feature.speed_window_ready &&
            forward_gate && rough_gate && secondary_gate) ? 1U : 0U;
}

/* 连续稳定且达到最短激活时间后，判定为退出候选。 */
static uint8_t bumpy_exit_candidate(const BumpyRuntime_t *runtime,
                                    const BumpyConfig_t *config,
                                    const BumpyInput_t *input)
{
    return (runtime->active_elapsed_ms >= config->min_active_ms &&
            runtime->feature.roughness_mps2 < config->rough_exit_mps2 &&
            runtime->feature.pitch_rate_rms_rps <
                config->pitch_rate_rms_exit_rps &&
            runtime->feature.roll_rate_rms_rps <
                config->roll_rate_rms_exit_rps &&
            runtime->feature.speed_std_mps < config->speed_std_exit_mps &&
            bumpy_absf(input->pitch_deg) < config->pitch_stable_deg &&
            bumpy_absf(input->roll_deg) < config->roll_stable_deg) ? 1U : 0U;
}

/*
 * 结束本次事件并生成报告。
 * use_exit_stats=1 时使用首次满足退出条件时冻结的统计，避免确认延时污染边界。
 */
static void bumpy_finish_segment(BumpyRuntime_t *runtime,
                                 const BumpyBoundarySnapshot_t *exit_snapshot,
                                 BumpyExitReason_t reason,
                                 uint8_t use_exit_stats)
{
    const BumpyBoundarySnapshot_t *entry = &runtime->entry_candidate;
    BumpyReport_t *report = &runtime->report;
    float dx;
    float dy;
    float yaw_rad;
    float speed_sum;
    float speed_sq_sum;
    float rough_sum;
    float speed_mean;
    float speed_variance;
    uint16_t sample_count;

    memset(report, 0, sizeof(*report));
    runtime->next_event_id++;
    report->event_id = runtime->next_event_id;
    report->mode = (uint8_t)runtime->mode;
    report->exit_reason = reason;
    report->completed_normally = (reason == BUMP_EXIT_NORMAL) ? 1U : 0U;
    report->duration_s = exit_snapshot->elapsed_s - entry->elapsed_s;

    report->entry_control_yaw_deg = entry->control_yaw_deg;
    report->entry_pose_yaw_deg = entry->pose_yaw_deg;
    report->exit_pose_yaw_deg = exit_snapshot->pose_yaw_deg;
    report->yaw_drift_deg = bumpy_limit_angle180(
        exit_snapshot->pose_yaw_deg - entry->pose_yaw_deg);

    report->left_encoder_distance_m =
        exit_snapshot->left_total_m - entry->left_total_m;
    report->right_encoder_distance_m =
        exit_snapshot->right_total_m - entry->right_total_m;
    report->encoder_forward_distance_m =
        (report->left_encoder_distance_m +
         report->right_encoder_distance_m) * 0.5f;
    report->encoder_lr_difference_m =
        report->left_encoder_distance_m - report->right_encoder_distance_m;

    dx = exit_snapshot->pose_x_m - entry->pose_x_m;
    dy = exit_snapshot->pose_y_m - entry->pose_y_m;
    yaw_rad = entry->pose_yaw_deg * 0.017453292519943295f;
    report->odom_forward_distance_m =
        dx * cosf(yaw_rad) + dy * sinf(yaw_rad);
    report->odom_right_distance_m =
        -dx * sinf(yaw_rad) + dy * cosf(yaw_rad);
    report->odom_straight_distance_m = bumpy_safe_sqrt(dx * dx + dy * dy);

    if (use_exit_stats)
    {
        speed_sum = runtime->exit_stat_speed_sum;
        speed_sq_sum = runtime->exit_stat_speed_sq_sum;
        rough_sum = runtime->exit_stat_rough_sum;
        sample_count = runtime->exit_stat_sample_count;
        report->min_forward_mps = runtime->exit_stat_min_speed;
        report->max_forward_mps = runtime->exit_stat_max_speed;
        report->max_abs_vertical_accel_mps2 =
            runtime->exit_stat_max_abs_accel;
        report->max_roughness_mps2 = runtime->exit_stat_max_roughness;
        report->max_abs_pitch_deg = runtime->exit_stat_max_abs_pitch;
        report->max_abs_roll_deg = runtime->exit_stat_max_abs_roll;
        report->max_pitch_rate_rps = runtime->exit_stat_max_pitch_rate;
        report->max_roll_rate_rps = runtime->exit_stat_max_roll_rate;
        report->impact_count = runtime->exit_stat_impact_count;
        report->slip_sample_count = runtime->exit_stat_slip_count;
    }
    else
    {
        speed_sum = runtime->stat_speed_sum;
        speed_sq_sum = runtime->stat_speed_sq_sum;
        rough_sum = runtime->stat_rough_sum;
        sample_count = runtime->stat_sample_count;
        report->min_forward_mps = runtime->stat_min_speed;
        report->max_forward_mps = runtime->stat_max_speed;
        report->max_abs_vertical_accel_mps2 = runtime->stat_max_abs_accel;
        report->max_roughness_mps2 = runtime->stat_max_roughness;
        report->max_abs_pitch_deg = runtime->stat_max_abs_pitch;
        report->max_abs_roll_deg = runtime->stat_max_abs_roll;
        report->max_pitch_rate_rps = runtime->stat_max_pitch_rate;
        report->max_roll_rate_rps = runtime->stat_max_roll_rate;
        report->impact_count = runtime->stat_impact_count;
        report->slip_sample_count = runtime->stat_slip_count;
    }

    report->active_sample_count = sample_count;
    if (sample_count > 0U)
    {
        speed_mean = speed_sum / (float)sample_count;
        speed_variance =
            speed_sq_sum / (float)sample_count - speed_mean * speed_mean;
        report->mean_forward_mps = speed_mean;
        report->forward_speed_std_mps = bumpy_safe_sqrt(speed_variance);
        report->mean_roughness_mps2 = rough_sum / (float)sample_count;
    }

    runtime->report_ready = 1U;
    runtime->last_exit_reason = reason;
    runtime->segment_started = 0U;
    runtime->segment_confirmed = 0U;
    bumpy_set_state(runtime, BUMP_STATE_RECOVER);
}

static float bumpy_slew_limit(float current,
                              float target,
                              float rate_per_s,
                              float dt_s)
{
    float maximum_step = rate_per_s * dt_s;

    if (target > current + maximum_step)
    {
        return current + maximum_step;
    }
    if (target < current - maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

/* 根据当前模式生成速度、锁航向和腿部目标建议，不直接操作硬件。 */
static void bumpy_make_assist_output(BumpyRuntime_t *runtime,
                                     const BumpyConfig_t *config,
                                     const BumpyInput_t *input,
                                     BumpyOutput_t *output)
{
    float speed_target;
    float leg_target;
    float dynamic_offset;

    if (runtime->mode < BUMP_MODE_SPEED_YAW)
    {
        return;
    }
    if (runtime->state != BUMP_STATE_ACTIVE &&
        runtime->state != BUMP_STATE_EXIT_CONFIRM &&
        !(config->assist_on_enter_candidate &&
          runtime->state == BUMP_STATE_ENTER_CONFIRM))
    {
        return;
    }

    speed_target = bumpy_clampf(input->user_speed_cmd,
                                config->assist_speed_min,
                                config->assist_speed_max);
    runtime->last_assist_speed = bumpy_slew_limit(
        runtime->last_assist_speed,
        speed_target,
        config->speed_slew_per_s,
        input->dt_s);
    output->speed_override_valid = 1U;
    output->speed_cmd = runtime->last_assist_speed;

    if (config->lock_yaw)
    {
        output->yaw_override_valid = 1U;
        output->yaw_cmd_deg = runtime->hold_control_yaw_deg;
    }

    if (runtime->mode < BUMP_MODE_STATIC_LEG)
    {
        return;
    }

    leg_target = input->base_leg_y_m + config->static_leg_y_offset_m;
    if (runtime->mode == BUMP_MODE_DYNAMIC_LEG)
    {
        dynamic_offset = -config->dynamic_leg_gain *
                         input->vertical_accel_mps2;
        dynamic_offset = bumpy_clampf(dynamic_offset,
                                      -config->dynamic_leg_limit_m,
                                      config->dynamic_leg_limit_m);
        leg_target += dynamic_offset;
    }
    runtime->last_leg_y_cmd_m = bumpy_slew_limit(
        runtime->last_leg_y_cmd_m,
        leg_target,
        config->leg_slew_mps,
        input->dt_s);
    output->leg_override_valid = 1U;
    output->leg_x_cmd_m = input->base_leg_x_m;
    output->leg_y_cmd_m = runtime->last_leg_y_cmd_m;
}

/*============================================================
 * 6. 公共检测核心接口
 *============================================================*/

/**
 * @brief  获取模块默认配置。
 * @return 只读默认配置地址，调用方不得直接修改。
 */
const BumpyConfig_t *Bumpy_Get_Default_Config(void)
{
    return &g_bumpy_default_config;
}

/**
 * @brief  初始化一个独立的颠簸检测实例。
 * @param  runtime 调用方持有的运行状态。
 * @param  config  检测参数；非法配置时退回观察模式且不启用窗口。
 */
void Bumpy_Init(BumpyRuntime_t *runtime, const BumpyConfig_t *config)
{
    if (runtime == NULL)
    {
        return;
    }
    memset(runtime, 0, sizeof(*runtime));
    runtime->state = BUMP_STATE_DISABLED;
    runtime->mode = bumpy_config_valid(config) ?
                    config->mode : BUMP_MODE_OBSERVE;
    runtime->last_exit_reason = BUMP_EXIT_NONE;
    if (bumpy_config_valid(config))
    {
        bumpy_reset_windows(runtime, config);
    }
}

/**
 * @brief  清空运行数据，同时保留当前模式和窗口采样数量。
 */
void Bumpy_Reset(BumpyRuntime_t *runtime)
{
    BumpyMode_t mode;
    uint8_t rough_samples;
    uint8_t speed_samples;

    if (runtime == NULL)
    {
        return;
    }
    mode = runtime->mode;
    rough_samples = runtime->rough_samples;
    speed_samples = runtime->speed_samples;
    memset(runtime, 0, sizeof(*runtime));
    runtime->state = BUMP_STATE_DISABLED;
    runtime->mode = mode;
    runtime->rough_samples = (rough_samples > 0U) ? rough_samples :
        bumpy_window_samples(g_bumpy_default_config.rough_window_ms,
                             g_bumpy_default_config.nominal_dt_ms);
    runtime->speed_samples = (speed_samples > 0U) ? speed_samples :
        bumpy_window_samples(g_bumpy_default_config.speed_window_ms,
                             g_bumpy_default_config.nominal_dt_ms);
}

/**
 * @brief  修改检测实例工作模式，不负责清空当前事件。
 */
void Bumpy_Set_Mode(BumpyRuntime_t *runtime, BumpyMode_t mode)
{
    if (runtime == NULL || mode > BUMP_MODE_DYNAMIC_LEG)
    {
        return;
    }
    runtime->mode = mode;
}

/**
 * @brief  推进一次颠簸检测状态机并生成控制建议。
 * @note   调用周期由 input->dt_s 指定；本工程固定按 10 ms 调用。
 */
void Bumpy_Update(BumpyRuntime_t *runtime,
                  const BumpyConfig_t *config,
                  const BumpyInput_t *input,
                  BumpyOutput_t *output)
{
    uint32_t dt_ms;
    uint8_t enter_candidate;
    uint8_t exit_candidate;
    uint8_t danger_condition;
    uint8_t stuck_condition;
    BumpyExitReason_t stop_reason = BUMP_EXIT_NONE;

    if (output == NULL)
    {
        return;
    }
    bumpy_clear_output(output,
                       (runtime != NULL) ? runtime->state : BUMP_STATE_DISABLED);
    if (runtime == NULL || config == NULL || input == NULL ||
        !bumpy_config_valid(config))
    {
        output->abnormal = 1U;
        output->exit_reason = BUMP_EXIT_SENSOR_INVALID;
        return;
    }

    dt_ms = bumpy_input_valid(input) ? bumpy_dt_ms(input) : 0U;

    if (runtime->mode == BUMP_MODE_OFF || !input->enable ||
        !input->remote_mode_active)
    {
        if (runtime->segment_confirmed)
        {
            bumpy_finish_segment(runtime, &runtime->current_snapshot,
                                 BUMP_EXIT_DISABLED, 0U);
            output->exit_event = 1U;
            output->abnormal = 1U;
            output->exit_reason = BUMP_EXIT_DISABLED;
        }
        else
        {
            bumpy_set_disabled(runtime, config, BUMP_EXIT_DISABLED);
        }
        output->state = runtime->state;
        output->report_ready = runtime->report_ready;
        return;
    }

    if (!input->remote_link_valid)
    {
        stop_reason = BUMP_EXIT_REMOTE_LOST;
    }
    else if (input->emergency_stop)
    {
        stop_reason = BUMP_EXIT_EMERGENCY;
    }
    else if (input->suppress_detection)
    {
        stop_reason = BUMP_EXIT_HIGH_PRIORITY_ACTION;
    }
    else if (!input->sensor_valid || dt_ms == 0U)
    {
        stop_reason = BUMP_EXIT_SENSOR_INVALID;
    }
    else if (input->user_speed_cmd < -config->forward_cmd_min)
    {
        stop_reason = BUMP_EXIT_USER_REVERSE;
    }
    else if (input->user_speed_cmd <= config->forward_cmd_min)
    {
        stop_reason = BUMP_EXIT_USER_STOP;
    }

    if (stop_reason != BUMP_EXIT_NONE)
    {
        if (runtime->segment_confirmed)
        {
            bumpy_finish_segment(runtime, &runtime->current_snapshot,
                                 stop_reason, 0U);
            output->exit_event = 1U;
            output->abnormal = 1U;
            output->exit_reason = stop_reason;
        }
        else
        {
            bumpy_set_disabled(runtime, config, stop_reason);
        }
        output->state = runtime->state;
        output->report_ready = runtime->report_ready;
        return;
    }

    runtime->state_elapsed_ms =
        bumpy_timer_add(runtime->state_elapsed_ms, dt_ms);
    runtime->total_elapsed_s += input->dt_s;
    runtime->left_total_m += input->left_forward_mps * input->dt_s;
    runtime->right_total_m += input->right_forward_mps * input->dt_s;
    bumpy_save_boundary(runtime, input, &runtime->current_snapshot);
    bumpy_update_windows(runtime, input);

    enter_candidate = bumpy_enter_candidate(runtime, config, input);
    exit_candidate = bumpy_exit_candidate(runtime, config, input);

    switch (runtime->state)
    {
        case BUMP_STATE_DISABLED:      /* 未启用：满足基础条件后转入待检测。 */
            runtime->last_exit_reason = BUMP_EXIT_NONE;
            bumpy_set_state(runtime, BUMP_STATE_ARMED);
            break;

        case BUMP_STATE_ARMED:         /* 已布防：等待进入候选条件。 */
            if (enter_candidate)
            {
                bumpy_begin_candidate(runtime, config, input, dt_ms);
            }
            break;

        case BUMP_STATE_ENTER_CONFIRM: /* 进入确认：连续满足阈值才判定有效。 */
            bumpy_update_stats(runtime, config, input, dt_ms);
            runtime->enter_confirm_elapsed_ms = bumpy_confirm_timer(
                runtime->enter_confirm_elapsed_ms, enter_candidate, dt_ms);
            if (!enter_candidate)
            {
                runtime->segment_started = 0U;
                runtime->enter_confirm_elapsed_ms = 0U;
                bumpy_set_state(runtime, BUMP_STATE_ARMED);
            }
            else if (runtime->enter_confirm_elapsed_ms >=
                     config->enter_confirm_ms)
            {
                runtime->segment_confirmed = 1U;
                runtime->active_elapsed_ms = 0U;
                runtime->danger_elapsed_ms = 0U;
                runtime->stuck_elapsed_ms = 0U;
                bumpy_set_state(runtime, BUMP_STATE_ACTIVE);
                output->enter_event = 1U;
            }
            break;

        case BUMP_STATE_ACTIVE:        /* 已进入颠簸区：持续统计并监控异常。 */
        case BUMP_STATE_EXIT_CONFIRM:  /* 退出确认：连续稳定后结束事件。 */
            runtime->active_elapsed_ms =
                bumpy_timer_add(runtime->active_elapsed_ms, dt_ms);
            bumpy_update_stats(runtime, config, input, dt_ms);

            danger_condition =
                (bumpy_absf(input->pitch_deg) > config->danger_pitch_deg ||
                 bumpy_absf(input->roll_deg) > config->danger_roll_deg ||
                 bumpy_absf(input->pitch_rate_rps) > config->danger_rate_rps ||
                 bumpy_absf(input->roll_rate_rps) > config->danger_rate_rps) ?
                1U : 0U;
            runtime->danger_elapsed_ms = bumpy_confirm_timer(
                runtime->danger_elapsed_ms, danger_condition, dt_ms);
            if (runtime->danger_elapsed_ms >= config->danger_confirm_ms)
            {
                bumpy_finish_segment(runtime, &runtime->current_snapshot,
                                     BUMP_EXIT_DANGER_ATTITUDE, 0U);
                output->exit_event = 1U;
                output->abnormal = 1U;
                output->exit_reason = BUMP_EXIT_DANGER_ATTITUDE;
                break;
            }

            stuck_condition =
                (runtime->active_elapsed_ms >= config->stuck_enable_ms &&
                 runtime->feature.forward_mps < config->stuck_forward_mps) ?
                1U : 0U;
            runtime->stuck_elapsed_ms = bumpy_confirm_timer(
                runtime->stuck_elapsed_ms, stuck_condition, dt_ms);
            if (runtime->stuck_elapsed_ms >= config->stuck_confirm_ms)
            {
                bumpy_finish_segment(runtime, &runtime->current_snapshot,
                                     BUMP_EXIT_STUCK, 0U);
                output->exit_event = 1U;
                output->abnormal = 1U;
                output->exit_reason = BUMP_EXIT_STUCK;
                break;
            }

            if (runtime->active_elapsed_ms >= config->max_active_ms)
            {
                bumpy_finish_segment(runtime, &runtime->current_snapshot,
                                     BUMP_EXIT_TIMEOUT, 0U);
                output->exit_event = 1U;
                output->abnormal = 1U;
                output->exit_reason = BUMP_EXIT_TIMEOUT;
                break;
            }

            if (runtime->state == BUMP_STATE_ACTIVE)
            {
                if (exit_candidate)
                {
                    bumpy_save_boundary(runtime, input,
                                        &runtime->exit_candidate);
                    bumpy_save_exit_stats(runtime);
                    runtime->exit_confirm_elapsed_ms = dt_ms;
                    bumpy_set_state(runtime, BUMP_STATE_EXIT_CONFIRM);
                }
            }
            else
            {
                runtime->exit_confirm_elapsed_ms = bumpy_confirm_timer(
                    runtime->exit_confirm_elapsed_ms, exit_candidate, dt_ms);
                if (!exit_candidate)
                {
                    runtime->exit_confirm_elapsed_ms = 0U;
                    bumpy_set_state(runtime, BUMP_STATE_ACTIVE);
                }
                else if (runtime->exit_confirm_elapsed_ms >=
                         config->exit_confirm_ms)
                {
                    bumpy_finish_segment(runtime, &runtime->exit_candidate,
                                         BUMP_EXIT_NORMAL, 1U);
                    output->exit_event = 1U;
                    output->exit_reason = BUMP_EXIT_NORMAL;
                }
            }
            break;

        case BUMP_STATE_RECOVER:       /* 恢复等待：防止刚退出后立即重复触发。 */
            if (runtime->state_elapsed_ms >= config->recover_ms)
            {
                bumpy_clear_candidate(runtime, config);
                bumpy_set_state(runtime, BUMP_STATE_ARMED);
            }
            break;

        default:
            bumpy_set_disabled(runtime, config, BUMP_EXIT_SENSOR_INVALID);
            output->abnormal = 1U;
            output->exit_reason = BUMP_EXIT_SENSOR_INVALID;
            break;
    }

    bumpy_make_assist_output(runtime, config, input, output);
    output->state = runtime->state;
    output->terrain_active =
        (runtime->state == BUMP_STATE_ACTIVE ||
         runtime->state == BUMP_STATE_EXIT_CONFIRM) ? 1U : 0U;
    output->report_ready = runtime->report_ready;
    if (runtime->last_exit_reason != BUMP_EXIT_NONE &&
        output->exit_reason == BUMP_EXIT_NONE)
    {
        output->exit_reason = runtime->last_exit_reason;
    }
}

BumpyState_t Bumpy_Get_State(const BumpyRuntime_t *runtime)
{
    return (runtime != NULL) ? runtime->state : BUMP_STATE_DISABLED;
}

uint8_t Bumpy_Is_Active(const BumpyRuntime_t *runtime)
{
    return (runtime != NULL &&
            (runtime->state == BUMP_STATE_ACTIVE ||
             runtime->state == BUMP_STATE_EXIT_CONFIRM)) ? 1U : 0U;
}

uint8_t Bumpy_Consume_Report(BumpyRuntime_t *runtime, BumpyReport_t *report)
{
    if (runtime == NULL || report == NULL || !runtime->report_ready)
    {
        return 0U;
    }
    *report = runtime->report;
    runtime->report_ready = 0U;
    return 1U;
}

/**
 * @brief  主动中止当前检测事件，并保留可用的异常报告。
 */
void Bumpy_Abort(BumpyRuntime_t *runtime, BumpyExitReason_t reason)
{
    if (runtime == NULL)
    {
        return;
    }
    if (reason == BUMP_EXIT_NONE || reason == BUMP_EXIT_NORMAL)
    {
        reason = BUMP_EXIT_DISABLED;
    }
    if (runtime->segment_confirmed)
    {
        bumpy_finish_segment(runtime, &runtime->current_snapshot, reason, 0U);
    }
    else
    {
        runtime->last_exit_reason = reason;
        runtime->segment_started = 0U;
        runtime->segment_confirmed = 0U;
        runtime->rough_index = 0U;
        runtime->rough_count = 0U;
        runtime->accel_sum = 0.0f;
        runtime->accel_sq_sum = 0.0f;
        runtime->pitch_rate_sq_sum = 0.0f;
        runtime->roll_rate_sq_sum = 0.0f;
        runtime->speed_index = 0U;
        runtime->speed_count = 0U;
        runtime->speed_sum = 0.0f;
        runtime->speed_sq_sum = 0.0f;
        memset(&runtime->feature, 0, sizeof(runtime->feature));
        bumpy_set_state(runtime, BUMP_STATE_DISABLED);
    }
}

/*============================================================
 * 7. 工程适配层与调试日志
 *============================================================*/

/* 将状态枚举转换为日志字符串，仅用于调试输出。 */
static const char *bumpy_state_name(BumpyState_t state)
{
    switch (state)
    {
        case BUMP_STATE_DISABLED:      return "DISABLED";
        case BUMP_STATE_ARMED:         return "ARMED";
        case BUMP_STATE_ENTER_CONFIRM: return "ENTER_CONFIRM";
        case BUMP_STATE_ACTIVE:        return "ACTIVE";
        case BUMP_STATE_EXIT_CONFIRM:  return "EXIT_CONFIRM";
        case BUMP_STATE_RECOVER:       return "RECOVER";
        default:                       return "UNKNOWN";
    }
}

static void bumpy_project_update_debug(BumpyState_t previous_state)
{
    g_bumpy_project_debug.previous_state = previous_state;
    g_bumpy_project_debug.state = g_bumpy_project_runtime.state;
    g_bumpy_project_debug.mode = g_bumpy_project_runtime.mode;
    g_bumpy_project_debug.last_exit_reason =
        g_bumpy_project_runtime.last_exit_reason;
    g_bumpy_project_debug.input = g_bumpy_project_input;
    g_bumpy_project_debug.output = g_bumpy_project_output;
    g_bumpy_project_debug.feature = g_bumpy_project_runtime.feature;
    g_bumpy_project_debug.state_elapsed_ms =
        g_bumpy_project_runtime.state_elapsed_ms;
    if (g_bumpy_project_runtime.report_ready)
    {
        g_bumpy_project_debug.last_report = g_bumpy_project_runtime.report;
    }
}

/**
 * @brief  初始化当前工程使用的单例检测实例，默认仅观察不接管控制。
 */
void Bumpy_Project_Init(void)
{
    g_bumpy_project_config = g_bumpy_default_config;
    g_bumpy_project_config.mode = BUMP_MODE_OBSERVE;
    Bumpy_Init(&g_bumpy_project_runtime, &g_bumpy_project_config);
    memset(&g_bumpy_project_input, 0, sizeof(g_bumpy_project_input));
    memset(&g_bumpy_project_output, 0, sizeof(g_bumpy_project_output));
    memset(&g_bumpy_project_debug, 0, sizeof(g_bumpy_project_debug));
    g_bumpy_project_enabled = 1U;
    g_bumpy_project_debug_elapsed_ms = 0U;
    g_bumpy_log_state_pending = 0U;
    g_bumpy_log_sample_pending = 0U;
    g_bumpy_log_report_pending = 0U;
    g_bumpy_project_initialized = 1U;
}

/**
 * @brief  采集工程数据并以 10 ms 周期推进检测核心。
 * @note   导航 Action 激活时立即返回，防止同一运行实例在一个周期内被推进两次。
 */
void Bumpy_Project_Process_10ms(void)
{
    BumpyState_t previous_state;
    uint32_t runtime_mask;

    /*
     * 导航 Action 使用同一套 project runtime。
     * Action 活动时禁止旧入口再次更新，避免一个周期推进两次状态机。
     */
    if (Bumpy_Action_Is_Active())
    {
        return;
    }

    if (!g_bumpy_project_initialized)
    {
        Bumpy_Project_Init();
    }

    runtime_mask = g_runtime_status.module_enable_mask;
    memset(&g_bumpy_project_input, 0, sizeof(g_bumpy_project_input));
    g_bumpy_project_input.dt_s = 0.010f;
    g_bumpy_project_input.enable = g_bumpy_project_enabled;
    g_bumpy_project_input.remote_mode_active =
        (g_runtime_status.vehicle_mode == VEHICLE_MODE_COURSE_3 &&
         (runtime_mask & RUNTIME_MODULE_BIT(RUNTIME_MODULE_REMOTE)) &&
         Remote_GetChannelData(5) > 1000) ? 1U : 0U;
    g_bumpy_project_input.emergency_stop = Vehicle_Is_Emergency_Stop();
    g_bumpy_project_input.remote_link_valid =
        (Remote_GetStatus() == REMOTE_CONNECTED) ? 1U : 0U;
    g_bumpy_project_input.suppress_detection =
        (jump_is_active() || navigation_jump_is_active() ||
         Navi_Action_Servo_Takeover_Active()) ? 1U : 0U;
    g_bumpy_project_input.sensor_valid =
        ((runtime_mask & RUNTIME_MODULE_BIT(RUNTIME_MODULE_NAVIGATION)) &&
         robot_pose.is_valid) ? 1U : 0U;

    g_bumpy_project_input.user_speed_cmd = target_velocity;
    g_bumpy_project_input.user_yaw_cmd_deg = target_angle;
    g_bumpy_project_input.control_yaw_deg = IMU_data.filter_result.yaw;

    g_bumpy_project_input.pose_x_m = (float)robot_pose.x;
    g_bumpy_project_input.pose_y_m = (float)robot_pose.y;
    g_bumpy_project_input.pose_yaw_deg = robot_pose.yaw;
    g_bumpy_project_input.fused_forward_mps = robot_pose.v;
    g_bumpy_project_input.slip_level = robot_pose.slip_level;

    g_bumpy_project_input.vertical_accel_mps2 = filter_data.accel[2];
    g_bumpy_project_input.pitch_deg = filter_data.pitch;
    g_bumpy_project_input.roll_deg = filter_data.roll;
    g_bumpy_project_input.pitch_rate_rps = filter_data.unbiased_gyro[1];
    g_bumpy_project_input.roll_rate_rps = filter_data.unbiased_gyro[0];
    g_bumpy_project_input.yaw_rate_rps = filter_data.unbiased_gyro[2];
    g_bumpy_project_input.left_forward_mps = filter_data.left_mps;
    g_bumpy_project_input.right_forward_mps = -filter_data.right_mps;
    g_bumpy_project_input.base_leg_x_m = x_current;
    g_bumpy_project_input.base_leg_y_m = y_current;

    previous_state = g_bumpy_project_runtime.state;
    Bumpy_Update(&g_bumpy_project_runtime,
                 &g_bumpy_project_config,
                 &g_bumpy_project_input,
                 &g_bumpy_project_output);

#if BUMPY_PROJECT_CONTROL_OUTPUT_ENABLED
    if (g_bumpy_project_output.speed_override_valid)
    {
        target_velocity = g_bumpy_project_output.speed_cmd;
    }
    if (g_bumpy_project_output.yaw_override_valid)
    {
        target_angle = g_bumpy_project_output.yaw_cmd_deg;
    }
#endif

    bumpy_project_update_debug(previous_state);
    if (previous_state != g_bumpy_project_runtime.state)
    {
        g_bumpy_log_previous_state = previous_state;
        g_bumpy_log_current_state = g_bumpy_project_runtime.state;
        g_bumpy_log_event_id = g_bumpy_project_runtime.next_event_id +
            (g_bumpy_project_runtime.segment_started ? 1U : 0U);
        g_bumpy_log_state_pending = 1U;
    }

    g_bumpy_project_debug_elapsed_ms = bumpy_timer_add(
        g_bumpy_project_debug_elapsed_ms,
        g_bumpy_project_config.nominal_dt_ms);
    if (Bumpy_Is_Active(&g_bumpy_project_runtime) &&
        g_bumpy_project_debug_elapsed_ms >=
            g_bumpy_project_config.debug_sample_period_ms)
    {
        g_bumpy_project_debug_elapsed_ms = 0U;
        g_bumpy_log_sample_pending = 1U;
    }
    if (g_bumpy_project_output.exit_event &&
        g_bumpy_project_runtime.report_ready)
    {
        g_bumpy_log_report_pending = 1U;
    }
}

#define BUMPY_F_SIGN(value)  (((value) < 0.0f) ? "-" : "")
#define BUMPY_F_ABS(value)   (((value) < 0.0f) ? -(value) : (value))
#define BUMPY_F_INT(value)   ((int)BUMPY_F_ABS(value))
#define BUMPY_F_DEC2(value)  ((int)((BUMPY_F_ABS(value) - \
                              (float)BUMPY_F_INT(value)) * 100.0f))
#define BUMPY_F_ARG(value)   BUMPY_F_SIGN(value), BUMPY_F_INT(value), \
                              BUMPY_F_DEC2(value)

/**
 * @brief  在主循环中发送状态、采样和汇总日志。
 * @note   禁止在中断中调用打印函数。
 */
void Bumpy_Project_Log_Task(void)
{
    uint8_t state_pending;
    uint8_t sample_pending;
    uint8_t report_pending;
    BumpyState_t previous_state;
    BumpyState_t current_state;
    uint32_t event_id;
    BumpyDebugData_t debug;
    BumpyReport_t report;

    if (!g_bumpy_project_initialized ||
        (!g_bumpy_log_state_pending &&
         !g_bumpy_log_sample_pending &&
         !g_bumpy_log_report_pending))
    {
        return;
    }
    if (!Runtime_Is_Module_Enabled(RUNTIME_MODULE_DEBUG_OUTPUT))
    {
        return;
    }

    __disable_irq();
    state_pending = g_bumpy_log_state_pending;
    sample_pending = g_bumpy_log_sample_pending;
    report_pending = g_bumpy_log_report_pending;
    previous_state = g_bumpy_log_previous_state;
    current_state = g_bumpy_log_current_state;
    event_id = g_bumpy_log_event_id;
    debug = g_bumpy_project_debug;
    report = g_bumpy_project_runtime.report;
    g_bumpy_log_state_pending = 0U;
    g_bumpy_log_sample_pending = 0U;
    g_bumpy_log_report_pending = 0U;
    __enable_irq();

    if (state_pending)
    {
        IPC_LOG_Printf("BUMP_STATE,id=%lu,%s->%s,t=%lu\r\n",
                       (unsigned long)event_id,
                       bumpy_state_name(previous_state),
                       bumpy_state_name(current_state),
                       (unsigned long)debug.state_elapsed_ms);
    }
    if (sample_pending)
    {
        IPC_LOG_Printf(
            "BUMP_SAMPLE,id=%lu,state=%d,cmd=%s%d.%02d,az=%s%d.%02d,"
            "rough=%s%d.%02d,pitch=%s%d.%02d,roll=%s%d.%02d,"
            "prms=%s%d.%02d,rrms=%s%d.%02d,v=%s%d.%02d,std=%s%d.%02d\r\n",
            (unsigned long)event_id,
            (int)debug.state,
            BUMPY_F_ARG(debug.input.user_speed_cmd),
            BUMPY_F_ARG(debug.input.vertical_accel_mps2),
            BUMPY_F_ARG(debug.feature.roughness_mps2),
            BUMPY_F_ARG(debug.input.pitch_deg),
            BUMPY_F_ARG(debug.input.roll_deg),
            BUMPY_F_ARG(debug.feature.pitch_rate_rms_rps),
            BUMPY_F_ARG(debug.feature.roll_rate_rms_rps),
            BUMPY_F_ARG(debug.feature.forward_mps),
            BUMPY_F_ARG(debug.feature.speed_std_mps));
    }
    if (report_pending)
    {
        IPC_LOG_Printf(
            "BUMP_SUMMARY,id=%lu,mode=%d,reason=%d,duration=%s%d.%02d,"
            "left=%s%d.%02d,right=%s%d.%02d,enc=%s%d.%02d,"
            "odom_f=%s%d.%02d,odom_r=%s%d.%02d,yaw=%s%d.%02d,"
            "vmean=%s%d.%02d,vstd=%s%d.%02d,rough=%s%d.%02d,"
            "rough_max=%s%d.%02d,az_max=%s%d.%02d,impact=%u,slip=%u\r\n",
            (unsigned long)report.event_id,
            (int)report.mode,
            (int)report.exit_reason,
            BUMPY_F_ARG(report.duration_s),
            BUMPY_F_ARG(report.left_encoder_distance_m),
            BUMPY_F_ARG(report.right_encoder_distance_m),
            BUMPY_F_ARG(report.encoder_forward_distance_m),
            BUMPY_F_ARG(report.odom_forward_distance_m),
            BUMPY_F_ARG(report.odom_right_distance_m),
            BUMPY_F_ARG(report.yaw_drift_deg),
            BUMPY_F_ARG(report.mean_forward_mps),
            BUMPY_F_ARG(report.forward_speed_std_mps),
            BUMPY_F_ARG(report.mean_roughness_mps2),
            BUMPY_F_ARG(report.max_roughness_mps2),
            BUMPY_F_ARG(report.max_abs_vertical_accel_mps2),
            (unsigned int)report.impact_count,
            (unsigned int)report.slip_sample_count);
    }
}

void Bumpy_Project_Set_Enabled(uint8_t enable)
{
    if (!g_bumpy_project_initialized)
    {
        Bumpy_Project_Init();
    }
    g_bumpy_project_enabled = enable ? 1U : 0U;
}

void Bumpy_Project_Set_Mode(BumpyMode_t mode)
{
    if (!g_bumpy_project_initialized)
    {
        Bumpy_Project_Init();
    }
    if (mode <= BUMP_MODE_DYNAMIC_LEG)
    {
        g_bumpy_project_config.mode = mode;
        Bumpy_Set_Mode(&g_bumpy_project_runtime, mode);
    }
}

BumpyState_t Bumpy_Project_Get_State(void)
{
    return g_bumpy_project_initialized ?
           g_bumpy_project_runtime.state : BUMP_STATE_DISABLED;
}

uint8_t Bumpy_Project_Is_Active(void)
{
    return g_bumpy_project_initialized ?
           Bumpy_Is_Active(&g_bumpy_project_runtime) : 0U;
}

uint8_t Bumpy_Project_Consume_Report(BumpyReport_t *report)
{
    if (!g_bumpy_project_initialized)
    {
        return 0U;
    }
    return Bumpy_Consume_Report(&g_bumpy_project_runtime, report);
}

void Bumpy_Project_Get_Debug_Snapshot(BumpyDebugData_t *out)
{
    if (out == NULL)
    {
        return;
    }
    __disable_irq();
    *out = g_bumpy_project_debug;
    __enable_irq();
}

/*============================================================
 * 8. 导航颠簸 Action 门面
 *
 * 导航层只负责 Start/Process/Reset；本文件内部统一管理配置、检测实例、
 * 目标速度、锁定航向、恢复阶段、位姿补偿及日志。
 *============================================================*/

static const BumpyActionProfile_t *bumpy_action_get_profile(
    uint16_t profile_id)
{
    if (profile_id >= BUMPY_ACTION_PROFILE_COUNT)
    {
        return NULL;
    }
    return &g_bumpy_action_profiles[profile_id];
}

static float bumpy_action_slew(float current, float target, float step)
{
    if (step <= 0.0f)
    {
        return target;
    }
    if (target > current + step)
    {
        return current + step;
    }
    if (target < current - step)
    {
        return current - step;
    }
    return target;
}

static void bumpy_action_pose_begin(void)
{
#if (BUMPY_ACTION_POSE_UPDATE_MODE == 2U)
    if (!g_bumpy_action.pose_update_active)
    {
        Navi_Set_Manual_Update_Mode(1U);
        g_bumpy_action.pose_update_active = 1U;
    }
#endif
}

static void bumpy_action_pose_add_fixed(
    const BumpyActionProfile_t *profile)
{
#if (BUMPY_ACTION_POSE_UPDATE_MODE == 2U)
    if (g_bumpy_action.pose_update_active && profile != NULL)
    {
        const float deg_to_rad = 0.017453292519943295f;
        float yaw_rad = g_bumpy_action.entry_yaw_deg * deg_to_rad;
        float dx = profile->fixed_forward_m * cosf(yaw_rad) -
                   profile->fixed_right_m * sinf(yaw_rad);
        float dy = profile->fixed_forward_m * sinf(yaw_rad) +
                   profile->fixed_right_m * cosf(yaw_rad);
        Navi_Manual_Add_Pose(dx, dy, 0U);
    }
#else
    (void)profile;
#endif
}

static void bumpy_action_pose_end(void)
{
#if (BUMPY_ACTION_POSE_UPDATE_MODE == 2U)
    if (g_bumpy_action.pose_update_active)
    {
        Navi_Set_Manual_Update_Mode(0U);
        g_bumpy_action.pose_update_active = 0U;
    }
#else
    g_bumpy_action.pose_update_active = 0U;
#endif
}

static void bumpy_action_fill_input(
    const BumpyActionProfile_t *profile,
    float speed_cmd)
{
    uint32_t runtime_mask = g_runtime_status.module_enable_mask;

    memset(&g_bumpy_project_input, 0, sizeof(g_bumpy_project_input));
    g_bumpy_project_input.dt_s = 0.010f;
    g_bumpy_project_input.enable = 1U;
    g_bumpy_project_input.remote_mode_active = 1U;
    g_bumpy_project_input.remote_link_valid = 1U;
    g_bumpy_project_input.emergency_stop = Vehicle_Is_Emergency_Stop();
    g_bumpy_project_input.suppress_detection = 0U;
    g_bumpy_project_input.sensor_valid =
        ((runtime_mask & RUNTIME_MODULE_BIT(RUNTIME_MODULE_NAVIGATION)) &&
         robot_pose.is_valid) ? 1U : 0U;
    g_bumpy_project_input.user_speed_cmd = speed_cmd;
    g_bumpy_project_input.user_yaw_cmd_deg = g_bumpy_action.hold_yaw_deg;
    g_bumpy_project_input.control_yaw_deg = IMU_data.filter_result.yaw;
    g_bumpy_project_input.pose_x_m = (float)robot_pose.x;
    g_bumpy_project_input.pose_y_m = (float)robot_pose.y;
    g_bumpy_project_input.pose_yaw_deg = robot_pose.yaw;
    g_bumpy_project_input.fused_forward_mps = robot_pose.v;
    g_bumpy_project_input.slip_level = robot_pose.slip_level;
    g_bumpy_project_input.vertical_accel_mps2 = filter_data.accel[2];
    g_bumpy_project_input.pitch_deg = filter_data.pitch;
    g_bumpy_project_input.roll_deg = filter_data.roll;
    g_bumpy_project_input.pitch_rate_rps = filter_data.unbiased_gyro[1];
    g_bumpy_project_input.roll_rate_rps = filter_data.unbiased_gyro[0];
    g_bumpy_project_input.yaw_rate_rps = filter_data.unbiased_gyro[2];
    g_bumpy_project_input.left_forward_mps = filter_data.left_mps;
    g_bumpy_project_input.right_forward_mps = -filter_data.right_mps;
    g_bumpy_project_input.base_leg_x_m = x_current;
    g_bumpy_project_input.base_leg_y_m = y_current;
    (void)profile;
}

static float bumpy_action_forward_distance_10ms(void)
{
    float forward_mps = 0.5f *
        (g_bumpy_project_input.left_forward_mps +
         g_bumpy_project_input.right_forward_mps);
    return (forward_mps > 0.0f) ?
           forward_mps * g_bumpy_project_input.dt_s : 0.0f;
}

static void bumpy_action_update_debug(BumpyState_t previous_state)
{
    bumpy_project_update_debug(previous_state);
    if (previous_state != g_bumpy_project_runtime.state)
    {
        g_bumpy_log_previous_state = previous_state;
        g_bumpy_log_current_state = g_bumpy_project_runtime.state;
        g_bumpy_log_event_id = g_bumpy_project_runtime.next_event_id +
            (g_bumpy_project_runtime.segment_started ? 1U : 0U);
        g_bumpy_log_state_pending = 1U;
    }
}

static void bumpy_action_capture_report(void)
{
    if (g_bumpy_project_runtime.report_ready)
    {
        g_bumpy_action.report = g_bumpy_project_runtime.report;
        g_bumpy_action.report_captured = 1U;
        g_bumpy_log_report_pending = 1U;
    }
}

static void bumpy_action_clear_leg_override(void)
{
    g_bumpy_action.leg_override_active = 0U;
    g_bumpy_project_output.leg_override_valid = 0U;
}

static BumpyActionResult_t bumpy_action_fail_now(
    BumpyExitReason_t reason,
    uint32_t extra_log_mask)
{
    if (g_bumpy_project_runtime.segment_started ||
        g_bumpy_project_runtime.segment_confirmed)
    {
        Bumpy_Abort(&g_bumpy_project_runtime, reason);
        bumpy_action_capture_report();
    }
    g_bumpy_action.exit_reason = reason;
    g_bumpy_action.finish_allowed = 0U;
    target_velocity = 0.0f;
    target_angle = (float)IMU_data.filter_result.yaw;
    bumpy_action_clear_leg_override();
    bumpy_action_pose_end();
    g_bumpy_action.phase = BUMP_ACTION_PHASE_IDLE;
    g_bumpy_action.started = 0U;
    g_bumpy_action.log_pending_mask |=
        BUMPY_ACTION_LOG_FAULT | extra_log_mask;
    return BUMP_ACTION_RESULT_FAULT;
}

static void bumpy_action_enter_recover(BumpyExitReason_t reason,
                                        uint8_t finish_allowed)
{
    const BumpyActionProfile_t *profile =
        bumpy_action_get_profile(g_bumpy_action.profile_id);
    g_bumpy_action.exit_reason = reason;
    g_bumpy_action.finish_allowed = finish_allowed ? 1U : 0U;
    g_bumpy_action.recover_elapsed_ms = 0U;
    g_bumpy_action.phase = BUMP_ACTION_PHASE_RECOVER;
    if (!g_bumpy_action.finish_allowed)
    {
        bumpy_action_clear_leg_override();
    }
    target_velocity =
        (g_bumpy_action.finish_allowed && profile != NULL) ?
        profile->recover_speed : 0.0f;
    target_angle = g_bumpy_action.hold_yaw_deg;
}

static uint8_t bumpy_action_speed_candidate(void)
{
    const BumpyFeature_t *feature = &g_bumpy_project_runtime.feature;
    if (g_bumpy_project_input.slip_level != 0U ||
        feature->roughness_mps2 >= g_bumpy_project_config.impact_threshold_mps2 ||
        feature->pitch_rate_rms_rps >=
            (0.75f * g_bumpy_project_config.danger_rate_rps) ||
        feature->roll_rate_rms_rps >=
            (0.75f * g_bumpy_project_config.danger_rate_rps) ||
        feature->speed_std_mps >=
            (2.0f * g_bumpy_project_config.speed_std_enter_mps))
    {
        return BUMP_SPEED_LEVEL_HEAVY;
    }
    if (feature->roughness_mps2 >=
            (0.75f * g_bumpy_project_config.rough_enter_mps2) ||
        feature->pitch_rate_rms_rps >=
            (0.75f * g_bumpy_project_config.pitch_rate_rms_enter_rps) ||
        feature->roll_rate_rms_rps >=
            (0.75f * g_bumpy_project_config.pitch_rate_rms_enter_rps) ||
        feature->speed_std_mps >=
            (0.75f * g_bumpy_project_config.speed_std_enter_mps))
    {
        return BUMP_SPEED_LEVEL_MEDIUM;
    }
    return BUMP_SPEED_LEVEL_NORMAL;
}

static float bumpy_action_update_speed(const BumpyActionProfile_t *profile)
{
    uint8_t candidate;
    uint8_t required_ticks;
    float target_speed;
    if (!(profile->feature_mask & BUMP_FEATURE_ADAPTIVE_SPEED))
    {
        g_bumpy_action.speed_level = BUMP_SPEED_LEVEL_NORMAL;
        g_bumpy_action.active_speed_cmd = profile->crossing_speed;
        return g_bumpy_action.active_speed_cmd;
    }

    candidate = bumpy_action_speed_candidate();
    g_bumpy_action.speed_level_hold_ms = (uint16_t)bumpy_timer_add(
        g_bumpy_action.speed_level_hold_ms,
        g_bumpy_project_config.nominal_dt_ms);
    if (candidate < g_bumpy_action.speed_level &&
        g_bumpy_action.speed_level_hold_ms < 200U)
    {
        candidate = g_bumpy_action.speed_level;
    }
    if (candidate == g_bumpy_action.speed_level)
    {
        g_bumpy_action.speed_level_pending = candidate;
        g_bumpy_action.speed_level_pending_ticks = 0U;
    }
    else
    {
        if (candidate != g_bumpy_action.speed_level_pending)
        {
            g_bumpy_action.speed_level_pending = candidate;
            g_bumpy_action.speed_level_pending_ticks = 1U;
        }
        else if (g_bumpy_action.speed_level_pending_ticks < 255U)
        {
            g_bumpy_action.speed_level_pending_ticks++;
        }
        required_ticks = (candidate > g_bumpy_action.speed_level) ? 3U : 20U;
        if (g_bumpy_action.speed_level_pending_ticks >= required_ticks)
        {
            g_bumpy_action.speed_level = candidate;
            g_bumpy_action.speed_level_pending_ticks = 0U;
            g_bumpy_action.speed_level_hold_ms = 0U;
            g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_SPEED_LEVEL;
        }
    }

    target_speed = profile->crossing_speed;
    if (g_bumpy_action.speed_level == BUMP_SPEED_LEVEL_MEDIUM)
    {
        target_speed = profile->crossing_speed_medium;
    }
    else if (g_bumpy_action.speed_level == BUMP_SPEED_LEVEL_HEAVY)
    {
        target_speed = profile->crossing_speed_heavy;
    }
    g_bumpy_action.active_speed_cmd = bumpy_action_slew(
        g_bumpy_action.active_speed_cmd,
        target_speed,
        profile->speed_step_per_tick);
    return g_bumpy_action.active_speed_cmd;
}

void Bumpy_Action_Reset(void)
{
    if (g_bumpy_action.started &&
        (g_bumpy_project_runtime.segment_started ||
         g_bumpy_project_runtime.segment_confirmed))
    {
        Bumpy_Abort(&g_bumpy_project_runtime, BUMP_EXIT_DISABLED);
    }
    bumpy_action_clear_leg_override();
    bumpy_action_pose_end();
    memset(&g_bumpy_action, 0, sizeof(g_bumpy_action));
    g_bumpy_action.phase = BUMP_ACTION_PHASE_IDLE;
    g_bumpy_action.exit_reason = BUMP_EXIT_NONE;
}

uint8_t Bumpy_Action_Start(uint16_t profile_id)
{
    const BumpyActionProfile_t *profile;
    uint8_t profile_fallback = 0U;
    profile = bumpy_action_get_profile(profile_id);
    if (profile == NULL)
    {
        profile_id = 0U;
        profile = bumpy_action_get_profile(profile_id);
        profile_fallback = 1U;
    }
    if (profile == NULL)
    {
        return 0U;
    }

    Bumpy_Action_Reset();
    g_bumpy_project_config = *Bumpy_Get_Default_Config();
    g_bumpy_project_config.mode = profile->mode;
    g_bumpy_project_config.min_active_ms = profile->min_crossing_ms;
    g_bumpy_project_config.max_active_ms = profile->crossing_timeout_ms;
    g_bumpy_project_config.recover_ms = profile->recover_ms;
    g_bumpy_project_config.static_leg_y_offset_m =
        (profile->feature_mask & BUMP_FEATURE_STATIC_LEG) ?
        profile->static_leg_y_offset_m : 0.0f;
    g_bumpy_project_config.leg_slew_mps =
        (profile->leg_ramp_step_m > 0.0f) ?
        profile->leg_ramp_step_m / 0.010f : 0.0f;
    Bumpy_Init(&g_bumpy_project_runtime, &g_bumpy_project_config);

    memset(&g_bumpy_project_input, 0, sizeof(g_bumpy_project_input));
    memset(&g_bumpy_project_output, 0, sizeof(g_bumpy_project_output));
    memset(&g_bumpy_project_debug, 0, sizeof(g_bumpy_project_debug));
    g_bumpy_project_initialized = 1U;
    g_bumpy_project_enabled = 1U;
    g_bumpy_project_debug_elapsed_ms = 0U;
    g_bumpy_log_state_pending = 0U;
    g_bumpy_log_sample_pending = 0U;
    g_bumpy_log_report_pending = 0U;

    g_bumpy_action.profile_id = profile_id;
    g_bumpy_action.phase = BUMP_ACTION_PHASE_DETECT;
    g_bumpy_action.started = 1U;
    g_bumpy_action.detect_elapsed_ms = 0U;
    g_bumpy_action.crossing_elapsed_ms = 0U;
    g_bumpy_action.recover_elapsed_ms = 0U;
    g_bumpy_action.detect_distance_m = 0.0f;
    g_bumpy_action.crossing_distance_m = 0.0f;
    g_bumpy_action.detect_start_x_m = (float)robot_pose.x;
    g_bumpy_action.detect_start_y_m = (float)robot_pose.y;
    g_bumpy_action.hold_yaw_deg = (float)IMU_data.filter_result.yaw;
    g_bumpy_action.active_speed_cmd = profile->detect_speed;
    g_bumpy_action.speed_level = BUMP_SPEED_LEVEL_NORMAL;
    g_bumpy_action.speed_level_pending = BUMP_SPEED_LEVEL_NORMAL;
    g_bumpy_action.exit_reason = BUMP_EXIT_NONE;
    g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_TRIGGER;
    if (profile_fallback)
    {
        g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_PROFILE_FALLBACK;
    }
    target_velocity = profile->detect_speed;
    target_angle = g_bumpy_action.hold_yaw_deg;
    return 1U;
}

static BumpyActionResult_t bumpy_action_update_detect_10ms(void)
{
    const BumpyActionProfile_t *profile;
    BumpyState_t previous_state;
    profile = bumpy_action_get_profile(g_bumpy_action.profile_id);
    if (profile == NULL)
    {
        return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
    }

    target_velocity = profile->detect_speed;
    target_angle = g_bumpy_action.hold_yaw_deg;
    bumpy_action_fill_input(profile, profile->detect_speed);
    if (g_bumpy_project_input.emergency_stop)
    {
        return bumpy_action_fail_now(BUMP_EXIT_EMERGENCY, 0U);
    }
    if (!g_bumpy_project_input.sensor_valid)
    {
        return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
    }

    previous_state = g_bumpy_project_runtime.state;
    Bumpy_Update(&g_bumpy_project_runtime,
                 &g_bumpy_project_config,
                 &g_bumpy_project_input,
                 &g_bumpy_project_output);
    bumpy_action_update_debug(previous_state);
    g_bumpy_action.detect_elapsed_ms = bumpy_timer_add(
        g_bumpy_action.detect_elapsed_ms,
        g_bumpy_project_config.nominal_dt_ms);
    g_bumpy_action.detect_distance_m +=
        bumpy_action_forward_distance_10ms();

    if (g_bumpy_project_output.enter_event)
    {
        g_bumpy_action.enter_detected = 1U;
        g_bumpy_action.crossing_elapsed_ms = 0U;
        g_bumpy_action.crossing_distance_m = 0.0f;
        g_bumpy_action.entry_x_m = (float)robot_pose.x;
        g_bumpy_action.entry_y_m = (float)robot_pose.y;
        g_bumpy_action.entry_yaw_deg =
            (float)IMU_data.filter_result.yaw;
        g_bumpy_action.hold_yaw_deg = g_bumpy_action.entry_yaw_deg;
        g_bumpy_action.active_speed_cmd = profile->crossing_speed;
        g_bumpy_action.phase = BUMP_ACTION_PHASE_CROSSING;
        g_bumpy_action.leg_base_x_m = x_current;
        g_bumpy_action.leg_base_y_m = y_current;
        g_bumpy_action.leg_y_cmd_m = y_current;
        if ((profile->feature_mask & BUMP_FEATURE_STATIC_LEG) &&
            g_bumpy_project_output.leg_override_valid)
        {
            g_bumpy_action.leg_override_active = 1U;
            g_bumpy_action.leg_y_cmd_m =
                g_bumpy_project_output.leg_y_cmd_m;
        }
        bumpy_action_pose_begin();
        g_bumpy_action.log_pending_mask |=
            BUMPY_ACTION_LOG_ENTER |
            BUMPY_ACTION_LOG_ENTER_CROSSING;
        return BUMP_ACTION_RESULT_ENTER_CROSSING;
    }

    if (g_bumpy_action.detect_elapsed_ms >= profile->detect_timeout_ms)
    {
        return bumpy_action_fail_now(
            BUMP_EXIT_TIMEOUT,
            BUMPY_ACTION_LOG_DETECT_TIMEOUT);
    }
    if (g_bumpy_action.detect_distance_m >= profile->detect_max_distance_m)
    {
        return bumpy_action_fail_now(
            BUMP_EXIT_TIMEOUT,
            BUMPY_ACTION_LOG_DETECT_DISTANCE_LIMIT);
    }
    return BUMP_ACTION_RESULT_RUNNING;
}

static BumpyActionResult_t bumpy_action_update_crossing_10ms(void)
{
    const BumpyActionProfile_t *profile;
    BumpyState_t previous_state;
    BumpyExitReason_t reason;
    uint8_t finish_allowed;
    float speed_cmd;
    profile = bumpy_action_get_profile(g_bumpy_action.profile_id);
    if (profile == NULL)
    {
        return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
    }

    speed_cmd = bumpy_action_update_speed(profile);
    target_velocity = speed_cmd;
    target_angle = g_bumpy_action.hold_yaw_deg;
    bumpy_action_fill_input(profile, speed_cmd);
    if (g_bumpy_project_input.emergency_stop)
    {
        return bumpy_action_fail_now(BUMP_EXIT_EMERGENCY, 0U);
    }
    if (!g_bumpy_project_input.sensor_valid)
    {
        return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
    }

    g_bumpy_action.crossing_elapsed_ms = bumpy_timer_add(
        g_bumpy_action.crossing_elapsed_ms,
        g_bumpy_project_config.nominal_dt_ms);
    g_bumpy_action.crossing_distance_m +=
        bumpy_action_forward_distance_10ms();
    previous_state = g_bumpy_project_runtime.state;
    Bumpy_Update(&g_bumpy_project_runtime,
                 &g_bumpy_project_config,
                 &g_bumpy_project_input,
                 &g_bumpy_project_output);
    bumpy_action_update_debug(previous_state);
    if ((profile->feature_mask & BUMP_FEATURE_STATIC_LEG) &&
        g_bumpy_project_output.leg_override_valid)
    {
        g_bumpy_action.leg_override_active = 1U;
        g_bumpy_action.leg_base_x_m =
            g_bumpy_project_output.leg_x_cmd_m;
        g_bumpy_action.leg_y_cmd_m =
            g_bumpy_project_output.leg_y_cmd_m;
    }

    if (g_bumpy_project_output.exit_event)
    {
        reason = g_bumpy_project_output.exit_reason;
        g_bumpy_action.exit_detected = 1U;
        finish_allowed =
            ((reason == BUMP_EXIT_NORMAL &&
              g_bumpy_action.crossing_elapsed_ms >=
                  profile->min_crossing_ms) ||
             (reason == BUMP_EXIT_TIMEOUT &&
              profile->allow_timeout_finish &&
              g_bumpy_action.enter_detected)) ? 1U : 0U;
        bumpy_action_capture_report();
        bumpy_action_enter_recover(reason, finish_allowed);
        g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_EXIT;
        if (reason == BUMP_EXIT_TIMEOUT)
        {
            g_bumpy_action.log_pending_mask |=
                BUMPY_ACTION_LOG_ACTION_TIMEOUT;
        }
        return BUMP_ACTION_RESULT_ENTER_RECOVER;
    }

    if ((profile->feature_mask & BUMP_FEATURE_DISTANCE_FALLBACK) &&
        g_bumpy_action.enter_detected &&
        g_bumpy_action.crossing_elapsed_ms >= profile->min_crossing_ms &&
        g_bumpy_action.crossing_distance_m >=
            (profile->expected_bump_length_m +
             profile->exit_distance_margin_m))
    {
        Bumpy_Abort(&g_bumpy_project_runtime,
                    BUMP_EXIT_DISTANCE_FALLBACK);
        bumpy_action_capture_report();
        g_bumpy_action.exit_detected = 1U;
        bumpy_action_enter_recover(BUMP_EXIT_DISTANCE_FALLBACK, 1U);
        g_bumpy_action.log_pending_mask |=
            BUMPY_ACTION_LOG_DISTANCE_FALLBACK |
            BUMPY_ACTION_LOG_EXIT;
        return BUMP_ACTION_RESULT_ENTER_RECOVER;
    }

    if (g_bumpy_project_output.abnormal)
    {
        reason = (g_bumpy_project_output.exit_reason != BUMP_EXIT_NONE) ?
                 g_bumpy_project_output.exit_reason :
                 BUMP_EXIT_SENSOR_INVALID;
        Bumpy_Abort(&g_bumpy_project_runtime, reason);
        bumpy_action_capture_report();
        bumpy_action_enter_recover(reason, 0U);
        g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_EXIT;
        return BUMP_ACTION_RESULT_ENTER_RECOVER;
    }

    if (g_bumpy_action.crossing_elapsed_ms >=
        profile->crossing_timeout_ms)
    {
        Bumpy_Abort(&g_bumpy_project_runtime, BUMP_EXIT_TIMEOUT);
        finish_allowed =
            (profile->allow_timeout_finish &&
             g_bumpy_action.enter_detected) ? 1U : 0U;
        bumpy_action_capture_report();
        bumpy_action_enter_recover(BUMP_EXIT_TIMEOUT, finish_allowed);
        g_bumpy_action.log_pending_mask |=
            BUMPY_ACTION_LOG_ACTION_TIMEOUT |
            BUMPY_ACTION_LOG_EXIT;
        return BUMP_ACTION_RESULT_ENTER_RECOVER;
    }
    return BUMP_ACTION_RESULT_RUNNING;
}

static BumpyActionResult_t bumpy_action_update_recover_10ms(void)
{
    const BumpyActionProfile_t *profile;
    profile = bumpy_action_get_profile(g_bumpy_action.profile_id);
    if (profile == NULL)
    {
        return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
    }
    if (Vehicle_Is_Emergency_Stop())
    {
        return bumpy_action_fail_now(BUMP_EXIT_EMERGENCY, 0U);
    }
    if (!(g_runtime_status.module_enable_mask &
          RUNTIME_MODULE_BIT(RUNTIME_MODULE_NAVIGATION)) ||
        !robot_pose.is_valid)
    {
        return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
    }

    target_velocity = g_bumpy_action.finish_allowed ?
                      profile->recover_speed : 0.0f;
    target_angle = g_bumpy_action.hold_yaw_deg;
    g_bumpy_action.recover_elapsed_ms = bumpy_timer_add(
        g_bumpy_action.recover_elapsed_ms,
        g_bumpy_project_config.nominal_dt_ms);
    if (g_bumpy_action.leg_override_active)
    {
        g_bumpy_action.leg_y_cmd_m = bumpy_action_slew(
            g_bumpy_action.leg_y_cmd_m,
            g_bumpy_action.leg_base_y_m,
            profile->leg_ramp_step_m);
        if (bumpy_absf(g_bumpy_action.leg_y_cmd_m -
                       g_bumpy_action.leg_base_y_m) < 0.00001f)
        {
            bumpy_action_clear_leg_override();
        }
    }
    bumpy_action_capture_report();
    if (g_bumpy_action.recover_elapsed_ms < profile->recover_ms)
    {
        return BUMP_ACTION_RESULT_RUNNING;
    }

    target_velocity = 0.0f;
    bumpy_action_clear_leg_override();
    if (g_bumpy_action.finish_allowed)
    {
        bumpy_action_pose_add_fixed(profile);
        bumpy_action_pose_end();
        g_bumpy_action.phase = BUMP_ACTION_PHASE_IDLE;
        g_bumpy_action.started = 0U;
        g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_DONE;
        return BUMP_ACTION_RESULT_DONE;
    }

    target_angle = (float)IMU_data.filter_result.yaw;
    bumpy_action_pose_end();
    g_bumpy_action.phase = BUMP_ACTION_PHASE_IDLE;
    g_bumpy_action.started = 0U;
    g_bumpy_action.log_pending_mask |= BUMPY_ACTION_LOG_FAULT;
    return BUMP_ACTION_RESULT_FAULT;
}

void Bumpy_Action_Log_Task(void)
{
    uint32_t log_mask;
    const BumpyActionProfile_t *profile =
        bumpy_action_get_profile(g_bumpy_action.profile_id);
    Bumpy_Project_Log_Task();
    __disable_irq();
    log_mask = g_bumpy_action.log_pending_mask;
    g_bumpy_action.log_pending_mask = 0U;
    __enable_irq();

    if (log_mask & BUMPY_ACTION_LOG_PROFILE_FALLBACK)
    {
        IPC_LOG_Printf("BUMP_ACTION,PROFILE_FALLBACK,profile=0\r\n");
    }
    if (log_mask & BUMPY_ACTION_LOG_TRIGGER)
    {
        IPC_LOG_Printf(
            "BUMP_ACTION,TRIGGER,profile=%u,detect_speed=%s%d.%02d,features=%u,pose_mode=%u\r\n",
            (unsigned int)g_bumpy_action.profile_id,
            BUMPY_F_ARG((profile != NULL) ? profile->detect_speed : 0.0f),
            (unsigned int)((profile != NULL) ? profile->feature_mask : 0U),
            (unsigned int)BUMPY_ACTION_POSE_UPDATE_MODE);
    }
    if (log_mask & BUMPY_ACTION_LOG_ENTER)
    {
        IPC_LOG_Printf(
            "BUMP_ACTION,ENTER,detect_ms=%lu,detect_m=%s%d.%02d,rough=%s%d.%02d,pitch_rms=%s%d.%02d,speed_std=%s%d.%02d\r\n",
            (unsigned long)g_bumpy_action.detect_elapsed_ms,
            BUMPY_F_ARG(g_bumpy_action.detect_distance_m),
            BUMPY_F_ARG(g_bumpy_project_runtime.feature.roughness_mps2),
            BUMPY_F_ARG(g_bumpy_project_runtime.feature.pitch_rate_rms_rps),
            BUMPY_F_ARG(g_bumpy_project_runtime.feature.speed_std_mps));
    }
    if (log_mask & BUMPY_ACTION_LOG_ENTER_CROSSING)
    {
        IPC_LOG_Printf("BUMP_ACTION,ENTER_CROSSING,yaw=%s%d.%02d\r\n",
                       BUMPY_F_ARG(g_bumpy_action.entry_yaw_deg));
    }
    if (log_mask & BUMPY_ACTION_LOG_DETECT_TIMEOUT)
    {
        IPC_LOG_Printf("BUMP_ACTION,DETECT_TIMEOUT,elapsed_ms=%lu\r\n",
                       (unsigned long)g_bumpy_action.detect_elapsed_ms);
    }
    if (log_mask & BUMPY_ACTION_LOG_DETECT_DISTANCE_LIMIT)
    {
        IPC_LOG_Printf("BUMP_ACTION,DETECT_DISTANCE_LIMIT,distance_m=%s%d.%02d\r\n",
                       BUMPY_F_ARG(g_bumpy_action.detect_distance_m));
    }
    if (log_mask & BUMPY_ACTION_LOG_SPEED_LEVEL)
    {
        IPC_LOG_Printf("BUMP_ACTION,SPEED_LEVEL,level=%u,speed=%s%d.%02d\r\n",
                       (unsigned int)g_bumpy_action.speed_level,
                       BUMPY_F_ARG(g_bumpy_action.active_speed_cmd));
    }
    if (log_mask & BUMPY_ACTION_LOG_DISTANCE_FALLBACK)
    {
        IPC_LOG_Printf("BUMP_ACTION,DISTANCE_FALLBACK,distance_m=%s%d.%02d\r\n",
                       BUMPY_F_ARG(g_bumpy_action.crossing_distance_m));
    }
    if (log_mask & BUMPY_ACTION_LOG_EXIT)
    {
        IPC_LOG_Printf(
            "BUMP_ACTION,EXIT,reason=%u,allowed=%u,entered=%u,cross_ms=%lu,cross_m=%s%d.%02d\r\n",
            (unsigned int)g_bumpy_action.exit_reason,
            (unsigned int)g_bumpy_action.finish_allowed,
            (unsigned int)g_bumpy_action.enter_detected,
            (unsigned long)g_bumpy_action.crossing_elapsed_ms,
            BUMPY_F_ARG(g_bumpy_action.crossing_distance_m));
    }
    if (log_mask & BUMPY_ACTION_LOG_ACTION_TIMEOUT)
    {
        IPC_LOG_Printf(
            "BUMP_ACTION,ACTION_TIMEOUT,allowed=%u,entered=%u\r\n",
            (unsigned int)g_bumpy_action.finish_allowed,
            (unsigned int)g_bumpy_action.enter_detected);
    }
    if (log_mask & BUMPY_ACTION_LOG_DONE)
    {
        IPC_LOG_Printf(
            "BUMP_ACTION,DONE,reason=%u,fixed_f=%s%d.%02d,fixed_r=%s%d.%02d,pose_mode=%u\r\n",
            (unsigned int)g_bumpy_action.exit_reason,
            BUMPY_F_ARG((profile != NULL) ? profile->fixed_forward_m : 0.0f),
            BUMPY_F_ARG((profile != NULL) ? profile->fixed_right_m : 0.0f),
            (unsigned int)BUMPY_ACTION_POSE_UPDATE_MODE);
    }
    if (log_mask & BUMPY_ACTION_LOG_FAULT)
    {
        IPC_LOG_Printf(
            "BUMP_ACTION,FAULT,reason=%u,entered=%u,exited=%u\r\n",
            (unsigned int)g_bumpy_action.exit_reason,
            (unsigned int)g_bumpy_action.enter_detected,
            (unsigned int)g_bumpy_action.exit_detected);
    }
}

BumpyActionResult_t Bumpy_Action_Process_10ms(void)
{
    if (!g_bumpy_action.started ||
        g_bumpy_action.phase == BUMP_ACTION_PHASE_IDLE)
    {
        return BUMP_ACTION_RESULT_IDLE;
    }
    if (g_bumpy_action.phase == BUMP_ACTION_PHASE_DETECT)
    {
        return bumpy_action_update_detect_10ms();
    }
    if (g_bumpy_action.phase == BUMP_ACTION_PHASE_CROSSING)
    {
        return bumpy_action_update_crossing_10ms();
    }
    if (g_bumpy_action.phase == BUMP_ACTION_PHASE_RECOVER)
    {
        return bumpy_action_update_recover_10ms();
    }
    return bumpy_action_fail_now(BUMP_EXIT_SENSOR_INVALID, 0U);
}

uint8_t Bumpy_Action_Is_Active(void)
{
    return (g_bumpy_action.started &&
            (g_bumpy_action.phase == BUMP_ACTION_PHASE_DETECT ||
             g_bumpy_action.phase == BUMP_ACTION_PHASE_CROSSING ||
             g_bumpy_action.phase == BUMP_ACTION_PHASE_RECOVER)) ? 1U : 0U;
}

BumpyExitReason_t Bumpy_Action_Get_Exit_Reason(void)
{
    return g_bumpy_action.exit_reason;
}

uint8_t Bumpy_Action_Get_Leg_Override(float *leg_x_cmd_m,
                                      float *leg_y_cmd_m)
{
    if (leg_x_cmd_m == NULL || leg_y_cmd_m == NULL ||
        !g_bumpy_action.leg_override_active ||
        Vehicle_Is_Emergency_Stop() || jump_is_active() ||
        navigation_jump_is_active() ||
        Navi_Action_Servo_Takeover_Active())
    {
        return 0U;
    }
    *leg_x_cmd_m = g_bumpy_action.leg_base_x_m;
    *leg_y_cmd_m = g_bumpy_action.leg_y_cmd_m;
    return 1U;
}
