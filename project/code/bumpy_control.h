/*********************************************************************************************************************
 * @file    bumpy_control.h
 * @brief   颠簸检测核心、工程适配层和导航穿越动作的公共类型与接口。
 *
 * @details
 * - 公共核心通过 Bumpy_Update() 识别颠簸路段并生成统计报告与控制建议。
 * - 工程适配接口负责当前项目的 10 ms 调度、数据采集和低频日志。
 * - 导航 Action 接口负责按配置执行颠簸航点穿越，不向外暴露内部运行数据。
 *
 * @note    反馈与报告由模块内部更新；调用方不得绕过接口直接修改内部单例状态。
 *********************************************************************************************************************/

#ifndef BUMPY_CONTROL_H
#define BUMPY_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================
 * 1. 公共常量与枚举
 *============================================================*/

#define BUMPY_WINDOW_MAX_SAMPLES    (32U)   /* 单个滑动窗口允许的最大采样数。 */

/* 颠簸模块工作模式。当前工程接入时默认且强制从 OBSERVE 开始。 */
typedef enum
{
    BUMP_MODE_OFF = 0,          /* 完全关闭检测。 */
    BUMP_MODE_OBSERVE,          /* 只检测、记录和上报，不生成控制接管。 */
    BUMP_MODE_SPEED_YAW,        /* 生成速度限制和锁航向建议。 */
    BUMP_MODE_STATIC_LEG,       /* 在速度/航向建议基础上增加固定腿高偏置。 */
    BUMP_MODE_DYNAMIC_LEG       /* 根据垂向加速度动态修正腿高。 */
} BumpyMode_t;

/* 自动检测状态机。 */
typedef enum
{
    BUMP_STATE_DISABLED = 0,    /* 未启用或被外部条件禁止。 */
    BUMP_STATE_ARMED,           /* 条件正常，等待颠簸进入候选。 */
    BUMP_STATE_ENTER_CONFIRM,   /* 连续确认进入特征，过滤瞬时冲击。 */
    BUMP_STATE_ACTIVE,          /* 已确认位于颠簸路段。 */
    BUMP_STATE_EXIT_CONFIRM,    /* 连续确认路况恢复稳定。 */
    BUMP_STATE_RECOVER          /* 退出后的防重复触发恢复期。 */
} BumpyState_t;

/* 一次颠簸记录的结束原因。 */
typedef enum
{
    BUMP_EXIT_NONE = 0,
    BUMP_EXIT_NORMAL,
    BUMP_EXIT_USER_STOP,
    BUMP_EXIT_USER_REVERSE,
    BUMP_EXIT_EMERGENCY,
    BUMP_EXIT_REMOTE_LOST,
    BUMP_EXIT_HIGH_PRIORITY_ACTION,
    BUMP_EXIT_DANGER_ATTITUDE,
    BUMP_EXIT_STUCK,
    BUMP_EXIT_TIMEOUT,
    BUMP_EXIT_SENSOR_INVALID,
    BUMP_EXIT_DISABLED,
    BUMP_EXIT_DISTANCE_FALLBACK
} BumpyExitReason_t;

/*
 * 公共核心的标准化输入。
 * user_speed_cmd 使用当前工程 target_velocity 的 RPM 量纲；
 * 位移和轮速使用 m、m/s；姿态使用 deg；角速度使用 rad/s；
 * vertical_accel_mps2 是扣除重力后的车体 Z 轴净加速度。
 */
/*============================================================
 * 2. 标准化输入、输出与配置
 *============================================================*/

typedef struct
{
    float dt_s;                 /* 本次更新周期，单位：s，允许范围 0.001~0.100。 */

    uint8_t enable;             /* 1：允许检测；0：关闭并退出当前事件。 */
    uint8_t remote_mode_active; /* 1：当前运行模式允许颠簸检测。 */
    uint8_t emergency_stop;     /* 1：急停已触发，必须异常退出。 */
    uint8_t remote_link_valid;  /* 1：控制链路有效。 */
    uint8_t suppress_detection; /* 1：高优先级动作正在接管，暂停检测。 */
    uint8_t sensor_valid;       /* 1：导航、IMU 和轮速输入有效。 */

    float user_speed_cmd;       /* 当前公共目标速度，量纲与 target_velocity 一致。 */
    float user_yaw_cmd_deg;     /* 当前公共目标航向，单位：deg。 */
    float control_yaw_deg;      /* 控制环使用的实时航向，单位：deg。 */

    float pose_x_m;
    float pose_y_m;
    float pose_yaw_deg;
    float fused_forward_mps;
    uint8_t slip_level;

    float vertical_accel_mps2;
    float pitch_deg;
    float roll_deg;
    float pitch_rate_rps;
    float roll_rate_rps;
    float yaw_rate_rps;

    float left_forward_mps;
    float right_forward_mps;

    float base_leg_x_m;
    float base_leg_y_m;
} BumpyInput_t;

/* 核心算法只生成建议；是否应用由工程适配逻辑决定。 */
typedef struct
{
    BumpyState_t state;         /* 本次更新后的检测状态。 */
    uint8_t terrain_active;     /* 1：处于 ACTIVE 或 EXIT_CONFIRM。 */
    uint8_t enter_event;        /* 单周期脉冲：本周期确认进入颠簸区。 */
    uint8_t exit_event;         /* 单周期脉冲：本周期结束颠簸事件。 */
    uint8_t report_ready;       /* 1：存在尚未消费的事件报告。 */

    uint8_t speed_override_valid; /* 1：speed_cmd 建议有效。 */
    float speed_cmd;              /* 建议目标速度，不由核心直接写入控制变量。 */

    uint8_t yaw_override_valid;   /* 1：yaw_cmd_deg 建议有效。 */
    float yaw_cmd_deg;            /* 建议锁定航向，单位：deg。 */

    uint8_t leg_override_valid;   /* 1：腿部位置建议有效。 */
    float leg_x_cmd_m;             /* 建议腿部 X 位置，单位：m。 */
    float leg_y_cmd_m;             /* 建议腿部 Y 位置，单位：m。 */

    uint8_t abnormal;
    BumpyExitReason_t exit_reason;
} BumpyOutput_t;

/* 参数单位与 BumpyInput_t 保持一致。所有阈值都需要实车数据再次标定。 */
typedef struct
{
    BumpyMode_t mode;

    uint16_t nominal_dt_ms;
    uint16_t rough_window_ms;
    uint16_t speed_window_ms;

    float forward_cmd_min;
    float forward_mps_min;

    float rough_enter_mps2;
    float pitch_rate_rms_enter_rps;
    float speed_std_enter_mps;
    uint16_t enter_confirm_ms;

    float rough_exit_mps2;
    float pitch_rate_rms_exit_rps;
    float roll_rate_rms_exit_rps;
    float speed_std_exit_mps;
    float pitch_stable_deg;
    float roll_stable_deg;
    uint16_t min_active_ms;
    uint16_t exit_confirm_ms;
    uint16_t recover_ms;

    float danger_pitch_deg;
    float danger_roll_deg;
    float danger_rate_rps;
    uint16_t danger_confirm_ms;
    uint16_t max_active_ms;

    float stuck_forward_mps;
    uint16_t stuck_enable_ms;
    uint16_t stuck_confirm_ms;

    float impact_threshold_mps2;
    uint16_t impact_refractory_ms;

    float assist_speed_min;
    float assist_speed_max;
    float speed_slew_per_s;
    uint8_t assist_on_enter_candidate;
    uint8_t lock_yaw;

    float static_leg_y_offset_m;
    float leg_slew_mps;
    float dynamic_leg_gain;
    float dynamic_leg_limit_m;

    uint16_t debug_sample_period_ms;
} BumpyConfig_t;

/*============================================================
 * 3. 边界快照、事件报告与窗口特征
 *============================================================*/

/* 入口和出口快照。累计轮程用于把确认时刻回退到首次候选时刻。 */
typedef struct
{
    float elapsed_s;
    float left_total_m;
    float right_total_m;
    float pose_x_m;
    float pose_y_m;
    float pose_yaw_deg;
    float control_yaw_deg;
} BumpyBoundarySnapshot_t;

/* 一次完整或异常颠簸事件的统计报告。 */
typedef struct
{
    uint32_t event_id;
    uint8_t mode;
    BumpyExitReason_t exit_reason;
    uint8_t completed_normally;

    float duration_s;

    float entry_control_yaw_deg;
    float entry_pose_yaw_deg;
    float exit_pose_yaw_deg;
    float yaw_drift_deg;

    float left_encoder_distance_m;
    float right_encoder_distance_m;
    float encoder_forward_distance_m;
    float encoder_lr_difference_m;

    float odom_forward_distance_m;
    float odom_right_distance_m;
    float odom_straight_distance_m;

    float min_forward_mps;
    float mean_forward_mps;
    float max_forward_mps;
    float forward_speed_std_mps;

    float max_abs_vertical_accel_mps2;
    float mean_roughness_mps2;
    float max_roughness_mps2;

    float max_abs_pitch_deg;
    float max_abs_roll_deg;
    float max_pitch_rate_rps;
    float max_roll_rate_rps;

    uint16_t impact_count;
    uint16_t slip_sample_count;
    uint16_t active_sample_count;
} BumpyReport_t;

/* 当前窗口计算出的特征，可用于调参与限频日志。 */
typedef struct
{
    float roughness_mps2;
    float pitch_rate_rms_rps;
    float roll_rate_rms_rps;
    float forward_mps;
    float speed_std_mps;
    float wheel_diff_mps;
    uint8_t rough_window_ready;
    uint8_t speed_window_ready;
} BumpyFeature_t;

/*============================================================
 * 4. 检测运行时与调试快照
 *============================================================*/

/*
 * 调用者持有的全部运行状态。不同业务入口原则上应创建独立实例，
 * 不得在同一调度周期内让多个入口推进同一个 BumpyRuntime_t。
 */
typedef struct
{
    BumpyState_t state;
    BumpyMode_t mode;
    BumpyExitReason_t last_exit_reason;

    uint32_t state_elapsed_ms;
    uint32_t active_elapsed_ms;
    uint32_t enter_confirm_elapsed_ms;
    uint32_t exit_confirm_elapsed_ms;
    uint32_t danger_elapsed_ms;
    uint32_t stuck_elapsed_ms;
    uint32_t impact_elapsed_ms;

    float total_elapsed_s;
    float left_total_m;
    float right_total_m;

    float accel_buffer[BUMPY_WINDOW_MAX_SAMPLES];
    float pitch_rate_sq_buffer[BUMPY_WINDOW_MAX_SAMPLES];
    float roll_rate_sq_buffer[BUMPY_WINDOW_MAX_SAMPLES];
    float accel_sum;
    float accel_sq_sum;
    float pitch_rate_sq_sum;
    float roll_rate_sq_sum;
    uint8_t rough_index;
    uint8_t rough_count;
    uint8_t rough_samples;

    float speed_buffer[BUMPY_WINDOW_MAX_SAMPLES];
    float speed_sum;
    float speed_sq_sum;
    uint8_t speed_index;
    uint8_t speed_count;
    uint8_t speed_samples;

    BumpyFeature_t feature;
    BumpyBoundarySnapshot_t current_snapshot;
    BumpyBoundarySnapshot_t entry_candidate;
    BumpyBoundarySnapshot_t exit_candidate;

    uint8_t segment_started;
    uint8_t segment_confirmed;
    float hold_control_yaw_deg;
    float last_assist_speed;
    float last_leg_y_cmd_m;

    float stat_speed_sum;
    float stat_speed_sq_sum;
    float stat_rough_sum;
    float stat_min_speed;
    float stat_max_speed;
    float stat_max_abs_accel;
    float stat_max_roughness;
    float stat_max_abs_pitch;
    float stat_max_abs_roll;
    float stat_max_pitch_rate;
    float stat_max_roll_rate;
    uint16_t stat_impact_count;
    uint16_t stat_slip_count;
    uint16_t stat_sample_count;

    float exit_stat_speed_sum;
    float exit_stat_speed_sq_sum;
    float exit_stat_rough_sum;
    float exit_stat_min_speed;
    float exit_stat_max_speed;
    float exit_stat_max_abs_accel;
    float exit_stat_max_roughness;
    float exit_stat_max_abs_pitch;
    float exit_stat_max_abs_roll;
    float exit_stat_max_pitch_rate;
    float exit_stat_max_roll_rate;
    uint16_t exit_stat_impact_count;
    uint16_t exit_stat_slip_count;
    uint16_t exit_stat_sample_count;

    uint32_t next_event_id;
    BumpyReport_t report;
    uint8_t report_ready;
} BumpyRuntime_t;

/* 低频任务可读取的工程适配层快照。 */
typedef struct
{
    BumpyState_t state;
    BumpyState_t previous_state;
    BumpyMode_t mode;
    BumpyExitReason_t last_exit_reason;
    BumpyInput_t input;
    BumpyOutput_t output;
    BumpyFeature_t feature;
    BumpyReport_t last_report;
    uint32_t state_elapsed_ms;
} BumpyDebugData_t;

/*============================================================
 * 5. 公共检测核心接口
 *============================================================*/

const BumpyConfig_t *Bumpy_Get_Default_Config(void);
void Bumpy_Init(BumpyRuntime_t *runtime, const BumpyConfig_t *config);
void Bumpy_Reset(BumpyRuntime_t *runtime);
void Bumpy_Set_Mode(BumpyRuntime_t *runtime, BumpyMode_t mode);
void Bumpy_Update(BumpyRuntime_t *runtime,
                  const BumpyConfig_t *config,
                  const BumpyInput_t *input,
                  BumpyOutput_t *output);
BumpyState_t Bumpy_Get_State(const BumpyRuntime_t *runtime);
uint8_t Bumpy_Is_Active(const BumpyRuntime_t *runtime);
uint8_t Bumpy_Consume_Report(BumpyRuntime_t *runtime, BumpyReport_t *report);
void Bumpy_Abort(BumpyRuntime_t *runtime, BumpyExitReason_t reason);

/*============================================================
 * 6. 当前工程适配接口
 *============================================================*/

/*
 * 当前工程的单模块适配接口。
 * Process_10ms 只需在后续确认的 10ms 调度点调用；Log_Task 必须在主循环调用，
 * 不能从中断打印。当前版本默认 OBSERVE，不会覆盖速度、航向或腿部输出。
 */
void Bumpy_Project_Init(void);
void Bumpy_Project_Process_10ms(void);      /* 10 ms 调度调用。 */
void Bumpy_Project_Log_Task(void);          /* 主循环低频调用。 */
void Bumpy_Project_Set_Enabled(uint8_t enable);
void Bumpy_Project_Set_Mode(BumpyMode_t mode);
BumpyState_t Bumpy_Project_Get_State(void);
uint8_t Bumpy_Project_Is_Active(void);
uint8_t Bumpy_Project_Consume_Report(BumpyReport_t *report);
void Bumpy_Project_Get_Debug_Snapshot(BumpyDebugData_t *out);

/*============================================================
 * 7. 导航颠簸 Action 接口
 *============================================================*/

/*
 * 导航 Action 只通过本门面启动和推进颠簸动作。
 * profile、传感器输入、运行时、位姿和报告均由 bumpy_control.c 私有管理。
 */
typedef enum
{
    BUMP_ACTION_RESULT_IDLE = 0,       /* 当前没有活动动作。 */
    BUMP_ACTION_RESULT_RUNNING,        /* 当前阶段仍在执行。 */
    BUMP_ACTION_RESULT_ENTER_CROSSING, /* confirmed entry; switch outer FSM */
    BUMP_ACTION_RESULT_ENTER_RECOVER,  /* 穿越结束，导航状态机应切入恢复态。 */
    BUMP_ACTION_RESULT_DONE,           /* 正常完成并允许继续导航。 */
    BUMP_ACTION_RESULT_FAULT           /* 异常结束，车辆目标已回退到安全状态。 */
} BumpyActionResult_t;

void Bumpy_Action_Reset(void);
uint8_t Bumpy_Action_Start(uint16_t profile_id);
BumpyActionResult_t Bumpy_Action_Process_10ms(void); /* 导航动作 10 ms 调度入口。 */
void Bumpy_Action_Log_Task(void);             /* 主循环打印 Action 日志。 */
uint8_t Bumpy_Action_Is_Active(void);
BumpyExitReason_t Bumpy_Action_Get_Exit_Reason(void);
uint8_t Bumpy_Action_Get_Leg_Override(float *leg_x_cmd_m,
                                      float *leg_y_cmd_m);

#ifdef __cplusplus
}
#endif

#endif
