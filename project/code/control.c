#include "control.h"
#include "FiveBarLinkageData.h"
#include "imu.h"
#include "param.h"
#include "ipc_shared_data.h"
#include "zf_device_imu660rc.h"
#include "runtime_status.h"
#include "process_rx.h"
#include "jump_control.h"
#include "bumpy_control.h"
#include "vehicle_supervisor.h"
#include "vision_control.h"
#include "navigation_action.h"

#define BALANCE_CONTROL_RUN_LEG_CONTROL 1
#define VISION_FRAME_TIMEOUT_TICKS 150U  // balance loop runs at 1 ms.

// 全局变量
extern float target_velocity;      // 目标速度
extern float target_angle;         // 目标角度
extern uint8_t Navi_Action_Servo_Takeover_Active(void);
float now_velocity = 0.0;          // Measured vehicle velocity.
extern float target_motor_Stand;   // 目标电机角度
float Encoder_Left, Encoder_Right; // Left and right encoder measurements.
float leg_error;

int speed_up = 0;
int bridge_high = 0;
float Turn_Pwm; // Steering PWM command.

// PID参数
extern pid_param_t motor_speed;     // Speed-loop PID parameters.
extern pid_param_t motor_Stand;     // Motor-angle PID parameters.
extern pid_param_t motor_direction; // 方向PID参数------------------------方向调整
extern pid_param_t motor_gyro;      // Gyro-rate PID parameters.
extern pid_param_t air_roll_pid;    // Airborne roll controller PID.
extern pid_param_t motor_leg_pid;   // Leg-height PID parameters.
extern float x_current, y_current;
int i = 0;
// 电机输出
signed short int Motor_Left, Motor_Right;        // 左右电机PWM输出
float Velocity_Angle_left, Velocity_Angle_right; // 左右电机速度
float out_speed_l = 0, out_speed_r = 0;
float out_angle_l = 0, out_angle_r = 0;
float out_gyro_l  = 0, out_gyro_r  = 0;
// ================= Motor output hardware mapping =================
// Keep final behavior identical to the old chain:
// Motor_Left = -cuu(logical_left); Motor_Right = cuu(logical_right);
// small_driver_set_duty(-Motor_Left, -Motor_Right).
#define MOTOR_LEFT_LIMIT_SIGN      (-1)
#define MOTOR_RIGHT_LIMIT_SIGN     (1)
#define MOTOR_LEFT_DRIVER_SIGN     (-1)
#define MOTOR_RIGHT_DRIVER_SIGN    (-1)
// 其他变量
static float balance_last_error = 0.0f;
static float gyro_last_error = 0.0f;
static float g_turn_yaw_integral = 0.0f;
static uint8_t g_vision_last_enabled = 0U;
static uint32_t g_vision_last_frame_seq = 0U;
static uint8_t g_vision_stale_ticks = 0U;
extern IMU_t IMU_data;            // IMU数据
float roll;              // 倾斜角度
int engine_change = 600; // 发动机变化量
//=======================未加入Jump Camera时的临时动作====================================
volatile int jump_stop = 0;
volatile int jump_position = 0;
float border = 94;
bool First_angle = true;
bool IMU_ready = false;
//============================================================================/
// 腿部控制参数

float temp_a, temp_b;
typedef struct
{
    float velocity;
    float encoder_bias;
    float encoder_integral;
} velocity_loop_state_t;

static velocity_loop_state_t g_velocity_forward = {0};
static float g_leg_speed_tilt_deg = 0.0f;
typedef struct
{
    float integral;
    float assist_pwm;
    uint8 clear_reason;
} anti_stall_assist_state_t;

enum
{
    ANTI_STALL_CLEAR_NONE = 0,
    ANTI_STALL_CLEAR_DISABLED = 1,
    ANTI_STALL_CLEAR_NO_TARGET = 2,
    ANTI_STALL_CLEAR_RECOVERED = 3,
    ANTI_STALL_CLEAR_SAFETY = 4
};

static anti_stall_assist_state_t g_anti_stall_assist = {0};
float leg_dbg_speed_tilt = 0.0f;
float leg_dbg_x_offset = 0.0f;
float leg_dbg_x_target = 0.0f;
float leg_dbg_x_cmd = 0.0f;
float leg_dbg_tick = 0.0f;
float leg_dbg_x_gain_used = 0.0f;
float leg_dbg_x_limit_used = 0.0f;
float leg_dbg_x_step_used = 0.0f;
float leg_dbg_x_limit_hit = 0.0f;
float anti_stall_dbg_enabled = 0.0f;
float anti_stall_dbg_integral = 0.0f;
float anti_stall_dbg_pwm = 0.0f;
float anti_stall_dbg_clear_reason = 0.0f;

static float speed_tilt_to_leg_x(float leg_tilt_deg, float leg_height)
{
    const float LEG_DEG_TO_RAD = (PI / 180.0f);
    float x_gain = (leg_x_gain > 0.0f) ? leg_x_gain : Leg_X_Gain_init;
    float x_limit = (leg_x_limit > 0.0f) ? leg_x_limit : Leg_X_Limit_init;
    float x_min_step = (leg_x_min_step > 0.0f) ? leg_x_min_step : Leg_X_Min_Step_init;
    float tilt_rad;
    float x_offset;
    float x_limited;

    leg_dbg_x_gain_used = x_gain;
    leg_dbg_x_limit_used = x_limit;

    leg_tilt_deg = constrain_float(leg_tilt_deg, -10.0f, 10.0f);
    leg_height = constrain_float(leg_height, MIN_Y, MAX_Y);
    tilt_rad = leg_tilt_deg * LEG_DEG_TO_RAD;
    x_offset = x_gain * leg_height * tanf(tilt_rad);

    if (fabsf(leg_tilt_deg) > 0.3f && fabsf(x_offset) < x_min_step)
    {
        x_offset = (leg_tilt_deg > 0.0f) ? x_min_step : -x_min_step;
    }

    x_limited = constrain_float(x_offset, -x_limit, x_limit);
    leg_dbg_x_limit_hit = (fabsf(x_limited - x_offset) > 0.00001f) ? 1.0f : 0.0f;

    return x_limited;
}

static void anti_stall_reset(uint8 reason)
{
    g_anti_stall_assist.integral = 0.0f;
    g_anti_stall_assist.assist_pwm = 0.0f;
    g_anti_stall_assist.clear_reason = reason;
    anti_stall_dbg_integral = 0.0f;
    anti_stall_dbg_pwm = 0.0f;
    anti_stall_dbg_clear_reason = (float)reason;
}

static float anti_stall_update(uint8 enabled, float target_velocity_cmd, float measured_velocity)
{
    const float ANTI_STALL_TARGET_MIN = 40.0f;
    const float ANTI_STALL_ERROR_START = 60.0f;
    const float ANTI_STALL_RECOVER_ERROR = 20.0f;
    const float ANTI_STALL_RECOVER_RATIO = 0.85f;
    const float ANTI_STALL_INTEGRAL_GAIN = 1.6f;
    const float ANTI_STALL_INTEGRAL_LIMIT = 50000.0f;
    const float ANTI_STALL_PWM_GAIN = 0.04f;
    const float ANTI_STALL_PWM_LIMIT = 4000.0f;
    float speed_error = target_velocity_cmd - measured_velocity;

    anti_stall_dbg_enabled = enabled ? 1.0f : 0.0f;

    if (!enabled)
    {
        anti_stall_reset(ANTI_STALL_CLEAR_DISABLED);
        return 0.0f;
    }

    if (target_velocity_cmd <= ANTI_STALL_TARGET_MIN)
    {
        anti_stall_reset(ANTI_STALL_CLEAR_NO_TARGET);
        anti_stall_dbg_enabled = 1.0f;
        return 0.0f;
    }

    if (!Runtime_Is_Module_Enabled(RUNTIME_MODULE_MOTOR) || jump_is_active() || Navi_Action_Servo_Takeover_Active())
    {
        anti_stall_reset(ANTI_STALL_CLEAR_SAFETY);
        anti_stall_dbg_enabled = 1.0f;
        return 0.0f;
    }

    if (measured_velocity >= (target_velocity_cmd * ANTI_STALL_RECOVER_RATIO) || speed_error <= ANTI_STALL_RECOVER_ERROR)
    {
        anti_stall_reset(ANTI_STALL_CLEAR_RECOVERED);
        anti_stall_dbg_enabled = 1.0f;
        return 0.0f;
    }

    if (speed_error < ANTI_STALL_ERROR_START)
    {
        anti_stall_reset(ANTI_STALL_CLEAR_RECOVERED);
        anti_stall_dbg_enabled = 1.0f;
        return 0.0f;
    }
    g_anti_stall_assist.integral += ANTI_STALL_INTEGRAL_GAIN * speed_error;
    g_anti_stall_assist.integral = constrain_float(g_anti_stall_assist.integral, 0.0f, ANTI_STALL_INTEGRAL_LIMIT);
    g_anti_stall_assist.assist_pwm = constrain_float(ANTI_STALL_PWM_GAIN * g_anti_stall_assist.integral, 0.0f, ANTI_STALL_PWM_LIMIT);
    g_anti_stall_assist.clear_reason = ANTI_STALL_CLEAR_NONE;
    anti_stall_dbg_integral = g_anti_stall_assist.integral;
    anti_stall_dbg_pwm = g_anti_stall_assist.assist_pwm;
    anti_stall_dbg_clear_reason = (float)g_anti_stall_assist.clear_reason;
    return g_anti_stall_assist.assist_pwm;
}


// 模糊规则参数
typedef struct
{
    float error_threshold_high;   // High error threshold.
    float error_threshold_low;    // Low error threshold.
    float d_error_threshold_high; // High error-rate threshold.
    float d_error_threshold_low;  // Low error-rate threshold.
    float kp_inc_high;            // 高KP增量
    float kp_inc_low;             // 低KP增量
    float ki_inc_high;            // 高KI增量
    float ki_inc_low;             // 低KI增量
    float kd_inc_high;            // 高KD增量
    float kd_inc_low;             // 低KD增量
} fuzzy_rules_t;

// PID限幅参数
typedef struct
{
    float kp_max, kp_min;
    float ki_max, ki_min;
    float kd_max, kd_min;
} pid_limit_t;

// 默认模糊规则
fuzzy_rules_t speed_rules = {
    .error_threshold_high = 0.05,
    .error_threshold_low = -0.05,
    .d_error_threshold_high = 0.01,
    .d_error_threshold_low = -0.01,
    .kp_inc_high = 0.01,
    .kp_inc_low = 0.005,
    .ki_inc_high = 0.00001,
    .ki_inc_low = 0.000005,
    .kd_inc_high = 0.0005,
    .kd_inc_low = 0.00025};
fuzzy_rules_t angle_rules = {
    .error_threshold_high = 8.0,    // 高误差阈值（度）
    .error_threshold_low = 3,       // 低误差阈值（度）
    .d_error_threshold_high = 30.0, // High error-rate threshold, deg/s.
    .d_error_threshold_low = 5.0,   // Low error-rate threshold, deg/s.
    .kp_inc_high = 0.01,
    .kp_inc_low = 0.01,
    .ki_inc_high = 0,
    .ki_inc_low = 0,
    .kd_inc_high = 0.005,
    .kd_inc_low = 0.005};
fuzzy_rules_t gyro_rules = {
    .error_threshold_high = 10,
    .error_threshold_low = -10,
    .d_error_threshold_high = 50,
    .d_error_threshold_low = 10,
    .kp_inc_high = 0.01,
    .kp_inc_low = 0.005,
    .ki_inc_high = 0,
    .ki_inc_low = 0,
    .kd_inc_high = 0.0005,
    .kd_inc_low = 0.00025};

// 默认PID限幅参数
pid_limit_t angle_pid_limits = {15, 5, 0.2, 0, 0.5, 1.7};
pid_limit_t gyro_pid_limits = {2.0, 1, 1.3, 0.4, 0.5, 0.1};

// 模糊逻辑调整PID参数
void fuzzy_pid_adjust(pid_param_t *pid, float error, float d_error, fuzzy_rules_t *rules, pid_limit_t *limits)
{
    // 模糊集合
    float e = error;    // 误差
    float de = d_error; // Error rate.

    // 模糊规则
    float delta_kp, delta_ki, delta_kd;

    // 根据误差和误差变化率调整PID参数
    if (fabs(e) > rules->error_threshold_high)
    {
        delta_kp = rules->kp_inc_high;
        delta_ki = 0;
        delta_kd = rules->kd_inc_high;
    }
    else if (fabs(e) < rules->error_threshold_low)
    {
        delta_kp = -rules->kp_inc_low;
        delta_ki = -rules->ki_inc_high;
        delta_kd = -rules->kd_inc_low;
    }
    else
    {
        delta_kp = 0;
        delta_ki = 0;
        delta_kd = 0;
    }

    pid->kp += delta_kp;
    pid->ki += delta_ki;
    pid->kd += delta_kd;
    if (fabs(de) > rules->d_error_threshold_high)
    {
        delta_kp = +rules->kp_inc_high;
        delta_ki = 0;
        delta_kd = +rules->kd_inc_high;
    }
    else if (fabs(e) < rules->error_threshold_low)
    {
        delta_kp = rules->kp_inc_low;
        delta_ki = 0;
        delta_kd = +rules->kd_inc_low;
    }
    else
    {
        delta_kp = 0;
        delta_ki = 0;
        delta_kd = 0;
    }

    // 更新PID参数
    pid->kp += delta_kp;
    pid->ki += delta_ki;
    pid->kd += delta_kd;

    // 限制PID参数范围
    pid->kp = fmaxf(fminf(pid->kp, limits->kp_max), limits->kp_min);
    pid->ki = fmaxf(fminf(pid->ki, limits->ki_max), limits->ki_min);
    pid->kd = fmaxf(fminf(pid->kd, limits->kd_max), limits->kd_min);
}

// 初始化PID参数
void pid_init()
{
    PidInit(&motor_speed);
    PidChange(&motor_speed, Speed_p, Speed_i, Speed_d); // Speed loop.

    PidInit(&motor_Stand);
    PidChange(&motor_Stand, Angle_p, Angle_i, Angle_d); // Angle loop.

    PidInit(&motor_direction);
    PidChange(&motor_direction, Direction_p, Direction_i, Direction_d); // Direction loop.

    PidInit(&motor_gyro);
    PidChange(&motor_gyro, Gyro_p, Gyro_i, Gyro_d); // Gyro-rate loop.

    PidInit(&air_roll_pid);
    PidChange(&air_roll_pid, Air_roll_p, Air_roll_i, Air_roll_d); // 初始化空中控制器PID
}

void Height_PID_Switch(bool high_mode)
{
    if (high_mode)
    {
        y_current = 0.030f;
        Speed_p = 0.012f;
        Direction_p = 30.0f;
        bridge_high = 1;
    }
    else
    {
        y_current = 0.040f;
        Speed_p = 0.025f;
        Direction_p = 50.0f;
        bridge_high = 0;
    }

    PidChange(&motor_speed, Speed_p, Speed_i, Speed_d);
    PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
}

void pid_high_init(void)
{
    Height_PID_Switch(true);
}

void pid_low_init(void)
{
    Height_PID_Switch(false);
}
// Detect whether the vehicle is airborne.
bool is_airborne()
{

    const float threshold = 0.5;
    return fabs(IMU_data.accel[2] - 1) > threshold;
}
// 应用空中控制
void apply_air_control(float roll_control)
{
    // Apply the roll-control output to both wheels.
    float wheel_left = roll_control;
    float wheel_right = roll_control;

    // Limit wheel output range.
    wheel_left = fmaxf(fminf(wheel_left, 1000.0), -1000.0);
    wheel_right = fmaxf(fminf(wheel_right, 1000.0), -1000.0);

    // Send wheel output.
    small_driver_set_duty(wheel_left, wheel_right);
}

// 空中控制器主函数
void air_control()
{
    static float last_roll_error = 0.0;
    // Read current roll angle.
    float current_roll = IMU_data.filter_result.roll;

    // 目标姿态（可以根据需要调整）
    float target_roll = 0.0;

    // 计算误差
    float roll_error = target_roll - current_roll;

    // Calculate error rate.
    float roll_d_error = roll_error - last_roll_error;

    // 保存当前误差
    last_roll_error = roll_error;
    // printf("data: %f,\r\n", current_roll);
    //  计算控制输出
    float roll_control = -air_roll_pid.kp * roll_error - air_roll_pid.ki * roll_error - air_roll_pid.kd * roll_d_error;

    // 限制输出范围
    roll_control = fmaxf(fminf(roll_control, 1000.0), -1000.0);

    // Apply controller output to the wheels.
    apply_air_control(roll_control);
}

void adjust_pid_based_on_leg_height(float *current_leg_height)
{
    // Calculate the normalized leg-height ratio.
    float leg_ratio = (*current_leg_height - MIN_LEG_LENGTH) / (MAX_LEG_LENGTH - MIN_LEG_LENGTH);
    leg_ratio = fmaxf(fminf(leg_ratio, 1.0), 0.0);

    // Base parameters for the lower leg-height mode.
    static bool initialized = false;
    static float original_angle_kp, original_angle_ki, original_angle_kd;
    static float original_gyro_kp, original_gyro_ki, original_gyro_kd, original_stand;

    if (!initialized)
    {
        original_angle_kp = 3;
        original_angle_ki = 0.001;
        original_angle_kd = 0.2;
        original_gyro_kp = 8;
        original_gyro_ki = 0.8;
        original_gyro_kd = 0.07;
        original_stand = 4;
        initialized = true;
    }

    // 经验公式调整角度环PID参数
    float angle_kp = original_angle_kp * (1.0 - 0.3 * leg_ratio); // Kp减少
    float angle_ki = original_angle_ki;                           // Ki保持不变
    float angle_kd = original_angle_kd * (1.0 + 0.6 * leg_ratio); // Kd增加

    // 经验公式调整陀螺仪环PID参数
    float gyro_kp = original_gyro_kp * (1.0 - 0.2 * leg_ratio); // Kp减少
    float gyro_ki = original_gyro_ki * (1.0 - 0.4 * leg_ratio); // Ki减少
    float gyro_kd = original_gyro_kd * (1.0 + 0.4 * leg_ratio); // Kd增加
    // 默认PID限幅参数
    target_motor_Stand = original_stand * (1.0 + 0.4 * leg_ratio);
    // 更新PID参数
    motor_Stand.kp = angle_kp;
    motor_Stand.ki = angle_ki;
    motor_Stand.kd = angle_kd;
    motor_gyro.kp = gyro_kp;
    motor_gyro.ki = gyro_ki;
    motor_gyro.kd = gyro_kd;
    pid_limit_t angle_pid_limits = {1.5 * angle_kp, 0.5 * angle_kp, 1.5 * angle_ki, 0.5 * angle_ki, 1.5 * angle_kd, 0.5 * angle_kd};
    pid_limit_t gyro_pid_limits = {1.5 * gyro_kp, 0.5 * gyro_kp, 1.5 * gyro_kp, 0.5 * gyro_kp, 1.5 * gyro_kp, 0.5 * gyro_kp};
}

// 初始化平衡控制（设置Kalman滤波的各个参数）
void Balance_init()
{
    pid_init(); // 初始化PID参数

    int leg1, leg2;
    servo_control(SERVO_LEG_LEFT, x_current, y_current, &leg1, &leg2);

    //============调试使用=========================
    // printf("Leg1: %d, Leg2: %d\r\n", leg1, leg2);
    //============调试使用=========================

    engine_init(leg1, leg2); // 初始化发动机
}
    // Velocity PI output provides the balance-angle compensation.
float Velocity(velocity_loop_state_t *state, float measured_velocity, float target_velocity) //===========左边为基础，右边送入务必取反
{
    state->encoder_bias = target_velocity - measured_velocity; // 计算偏差
    state->encoder_integral += state->encoder_bias;       // 积分
    // Integral output uses a bounded accumulation range.
    // 限制积分范围
    if (state->encoder_integral > 2000)
        state->encoder_integral = 2000;
    if (state->encoder_integral < -2000)
        state->encoder_integral = -2000;
    // Integral output uses a bounded accumulation range.

    // Velocity loop output is interpreted as an angle command.
    state->velocity = motor_speed.kp * state->encoder_bias + motor_speed.ki * state->encoder_integral;
    // 限制速度范围
    if (state->velocity > 8)
        state->velocity = 8;
    if (state->velocity < -8)
        state->velocity = -8;

    // 动态调整PID参数
    // fuzzy_pid_adjust(&motor_speed, state->encoder_bias, state->velocity - last_error, &speed_rules, &speed_pid_limits);

    //============调试使用=========================
    // printf("Encoder_bias|Encoder_Integral|velocity(angle):%f,%f,%f\r\n", state->encoder_bias, state->encoder_integral, state->velocity);
    // printf("target_velocity : %lf\r\n", target_velocity);
    //============调试使用=========================

    return state->velocity; // Filtered velocity output used for A/B validation.
}
// 平衡控制计算（PD控制角度环）
float Balance(float Angle, float Gyro, float target)
{
    static float angle_integral = 0.0f; // Integral term for static angle bias.
    float Angle_bias = target_motor_Stand + target - Angle;
    float Gyro_bias = 0 - Gyro; // Gyro compensation term for balance control.
    angle_integral += Angle_bias;
    angle_integral = constrain_float(angle_integral, -1000.0f, 1000.0f);

    float balance = -motor_Stand.kp * Angle_bias - motor_Stand.ki * angle_integral + Gyro_bias * motor_Stand.kd;

    balance_last_error = Angle_bias;
    if (balance > 5000)
    balance = 5000; // Output limit.
    if (balance < -5000)
        balance = -5000;

    return balance; // Return balance-controller output.
}

// Gyro-rate controller.
float GyroControl(float target_gyro, float current_gyro) // Gyro-rate loop.
{
    float gyro_error = target_gyro - current_gyro;
    static float gyro_Integral; // Integral accumulator.
    gyro_Integral += gyro_error;
    if (gyro_Integral > 1500)
        gyro_Integral = 1500;
    if (gyro_Integral < -1500)
        gyro_Integral = -1500;

    float gyro_delta = gyro_error - gyro_last_error;
    float gyro_control = +motor_gyro.kp * gyro_error + motor_gyro.ki * gyro_Integral + motor_gyro.kd * gyro_delta;
    gyro_last_error = gyro_error;
    return gyro_control; // gyro loop output sign fixed
}

// 限制PWM输出范围
int cuu(int c)
{
    // 限制输出范围
    if (c > (MAX_DUTY * (PWM_DUTY_MAX / 100)))
        return (MAX_DUTY * (PWM_DUTY_MAX / 100));
    else if (c < -(MAX_DUTY * (PWM_DUTY_MAX / 100)))
        return -(MAX_DUTY * (PWM_DUTY_MAX / 100));
    else
        return c;
}

// Centralized motor stop/apply path. Keep PID calculations above unchanged.
static void Motor_Output_Clear_Debug(void)
{
    out_speed_l = 0.0f;
    out_speed_r = 0.0f;
    out_angle_l = 0.0f;
    out_angle_r = 0.0f;
    out_gyro_l = 0.0f;
    out_gyro_r = 0.0f;
}

static void Motor_Output_Stop(void)
{
    Motor_Left = 0;
    Motor_Right = 0;
    anti_stall_dbg_enabled = 0.0f;
    anti_stall_reset(ANTI_STALL_CLEAR_SAFETY);
    Motor_Output_Clear_Debug();
    small_driver_set_duty(0, 0);
}

static void Motor_Output_Apply(float gyro_pwm, float assist_pwm, float turn_pwm, uint8 turn_enabled)
{
    int logical_left;
    int logical_right;
    int limited_left;
    int limited_right;
    int driver_left;
    int driver_right;

    if (turn_enabled)
    {
        logical_left = (int)(gyro_pwm + assist_pwm + turn_pwm);
        logical_right = (int)(gyro_pwm + assist_pwm - turn_pwm);
    }
    else
    {
        logical_left = (int)(gyro_pwm + assist_pwm);
        logical_right = (int)(gyro_pwm + assist_pwm);
    }

    limited_left = MOTOR_LEFT_LIMIT_SIGN * cuu(logical_left);
    limited_right = MOTOR_RIGHT_LIMIT_SIGN * cuu(logical_right);

    Motor_Left = (signed short int)limited_left;
    Motor_Right = (signed short int)limited_right;

    driver_left = MOTOR_LEFT_DRIVER_SIGN * limited_left;
    driver_right = MOTOR_RIGHT_DRIVER_SIGN * limited_right;

    small_driver_set_duty(driver_left, driver_right);
}

float Turn_gyro(float target_angle, float gyro)
{
    // 1. 计算原始误差
    float error = target_angle - gyro;

    // 2. 将误差归一化到 [-180, 180] 之间
// Select the direction whose angular difference is within +/-180 degrees.
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;

    return error; // PID input.
}

// Normalize steering target angle to the range [-180, 180].
float Turn_target(float target_angle)
{
    if (target_angle >= 180)
        target_angle = target_angle - 360;
    else if (target_angle <= -180)
        target_angle = 360 + target_angle;
    return -target_angle;
}

// 转向控制计算
static float turn_compute(float current_yaw, float target_yaw, uint8_t use_integral)
{
    float yaw_error = target_yaw - current_yaw;
    float yaw_rate = IMU_data.gyro[2];
    float control_output;

    while (yaw_error > 180.0f)  yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    if (fabsf(yaw_error) < 1.5f)
    {
        yaw_error = 0.0f;
    }

    if (use_integral)
    {
        const float turn_i_output_limit = 450.0f;
        const float turn_i_state_limit = 15000.0f;
        float integral_output;

        if (yaw_error == 0.0f)
        {
            g_turn_yaw_integral *= 0.995f;
        }
        else
        {
            g_turn_yaw_integral += yaw_error;
            g_turn_yaw_integral = constrain_float(g_turn_yaw_integral, -turn_i_state_limit, turn_i_state_limit);
        }

        integral_output = constrain_float(Direction_i * g_turn_yaw_integral,
                                          -turn_i_output_limit,
                                          turn_i_output_limit);
        control_output = Direction_p * yaw_error + integral_output - Direction_d * yaw_rate;
    }
    else
    {
        g_turn_yaw_integral = 0.0f;
        control_output = vision_control_pd_output(yaw_error, yaw_rate, Direction_p, Direction_d, 2200.0f);
    }

    return control_output;
}

float Turn(float current_yaw, float target_yaw)
{
    return turn_compute(current_yaw, target_yaw, 1U);
}

static float Turn_PD(float current_yaw, float target_yaw)
{
    return turn_compute(current_yaw, target_yaw, 0U);
}

void Turn_Reset(void)
{
    g_turn_yaw_integral = 0.0f;
}

static float vision_wrap_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void vision_mode_apply(void)
{
    uint8_t enabled = (core_b_cmd.vision_enabled || Navi_Action_Vision_Align_Active()) ? 1U : 0U;
    uint8_t valid = core_b_cmd.vision_valid;

    if (!enabled)
    {
        if (g_vision_last_enabled) Turn_Reset();
        g_vision_last_enabled = 0U;
        g_vision_stale_ticks = 0U;
        return;
    }

    if (core_b_cmd.vision_frame_seq != g_vision_last_frame_seq)
    {
        g_vision_last_frame_seq = core_b_cmd.vision_frame_seq;
        g_vision_stale_ticks = 0U;
    }
    else if (g_vision_stale_ticks < 255U)
    {
        g_vision_stale_ticks++;
    }

    if (!g_vision_last_enabled)
    {
        Turn_Reset();
    }
    g_vision_last_enabled = 1U;
    if (valid && g_vision_stale_ticks <= VISION_FRAME_TIMEOUT_TICKS)
    {
        target_angle = vision_wrap_angle(IMU_data.filter_result.yaw + core_b_cmd.vision_angle_offset_deg);
    }
    else
    {
        target_angle = IMU_data.filter_result.yaw;
        Turn_Reset();
    }
}

// Balance control main function.
void balance_control()
{
    static float Balance_Pwm = 0.0f;
    static uint8_t angle_loop_div = 0;
    static uint8_t speed_loop_div = 0;
#if BALANCE_CONTROL_RUN_LEG_CONTROL
    static uint8_t leg_loop_div = 0;
#endif
    float Gyro_Pwm;
    float assist_pwm = 0.0f;
    float raw_gyro_x = process_rx_gyro_x_dps((float)imu660rc_gyro_x);

    if (Vehicle_Is_Emergency_Stop())
    {
        Runtime_Set_Balance_Reason(RUNTIME_REASON_BALANCE_OFF);
        Balance_Pwm = 0.0f;
        Velocity_Angle_left = 0.0f;
        Velocity_Angle_right = 0.0f;
        Turn_Pwm = 0.0f;
        Motor_Output_Stop();
        return;
    }

    if (IPC_CoreB_Wifi_Is_Connected() == 0)
    {
        Runtime_Set_Balance_Reason(RUNTIME_REASON_WIFI_OFF);
        Balance_Pwm = 0.0f;
        Velocity_Angle_left = 0.0f;
        Velocity_Angle_right = 0.0f;
        Turn_Pwm = 0.0f;
        Motor_Output_Stop();
        return;
    }
    Runtime_Set_Balance_Reason(RUNTIME_REASON_NORMAL);
    if (First_angle && IMU_ready)
    {
        target_angle = IMU_data.filter_result.yaw;
        First_angle = false;
    }

    vision_mode_apply();

    roll = IMU_data.filter_result.roll;

    if (jump_should_suspend_engine())
    {
        Runtime_Set_Balance_Reason(RUNTIME_REASON_BALANCE_OFF);
        PidChange(&motor_speed, 0, 0, 0);
        PidChange(&motor_Stand, 0, 0, 0);
        PidChange(&motor_gyro, 0, 0, 0);
        Balance_Pwm = 0.0f;
        Velocity_Angle_left = 0.0f;
        Velocity_Angle_right = 0.0f;
        Turn_Pwm = 0.0f;
        Motor_Output_Stop();
        return;
    }
    else
    {
        PidChange(&motor_speed, Speed_p, Speed_i, Speed_d);
        PidChange(&motor_Stand, Angle_p, Angle_i, Angle_d);
        PidChange(&motor_gyro, Gyro_p, Gyro_i, Gyro_d);
        PidChange(&motor_direction, Direction_p, Direction_i, Direction_d);
    }

    angle_loop_div++;
    speed_loop_div++;
    if (angle_loop_div >= 5)
    {
        angle_loop_div = 0;

        Encoder_Left = -motor_value.receive_left_speed_data;
        Encoder_Right = -motor_value.receive_right_speed_data;
        now_velocity = (Encoder_Left - Encoder_Right) / 2.0f;

        if (speed_loop_div >= 20)
        {
            speed_loop_div = 0;
            Velocity_Angle_left = Velocity(&g_velocity_forward, now_velocity, target_velocity);
            Velocity_Angle_right = Velocity_Angle_left;
            g_leg_speed_tilt_deg = Velocity_Angle_left;
            leg_dbg_speed_tilt = g_leg_speed_tilt_deg;
        }

        Balance_Pwm = Balance(roll, raw_gyro_x, 0.0f);
    }

    Gyro_Pwm = GyroControl(Balance_Pwm, raw_gyro_x);
    Turn_Pwm = (core_b_cmd.vision_enabled || Navi_Action_Vision_Align_Active()) ?
               Turn_PD(IMU_data.filter_result.yaw, target_angle) :
               Turn(IMU_data.filter_result.yaw, target_angle);
    assist_pwm = anti_stall_update(Runtime_Is_Module_Enabled(RUNTIME_MODULE_ANTI_STALL), target_velocity, now_velocity);

    out_speed_l = Velocity_Angle_left;
    out_speed_r = Velocity_Angle_left;
    out_angle_l = Balance_Pwm;
    out_angle_r = Balance_Pwm;
    out_gyro_l = Gyro_Pwm;
    out_gyro_r = Gyro_Pwm;

    Motor_Output_Apply(Gyro_Pwm, assist_pwm, Turn_Pwm, (jump_position == 0) ? 1 : 0);

#if BALANCE_CONTROL_RUN_LEG_CONTROL
    leg_loop_div++;
    if (leg_loop_div >= 20)
    {
        leg_loop_div = 0;
        if (!Vehicle_Is_Emergency_Stop() && !jump_is_active() && !Navi_Action_Servo_Takeover_Active())
        {
            leg_control(&x_current, &y_current);
        }
    }
#endif
}

// 滤波处理
float filter_leg_control(float current_angle, float target_angle, float filter_factor)
{
    return current_angle * filter_factor + target_angle * (1 - filter_factor);
}
/* Leg control. */

void leg_control(float *x, float *y)
{
    extern float leg_Kp, leg_Ki, leg_Kd;
    static bool is_first_run = true;
    static bool hold_mode = true;
    static float angle_last = 0.0f;
    static float leg_error_target = 0.0f;
    static float x_cmd_last = 0.0f;
    static float left_y_cmd_last = 0.0f;
    static float right_y_cmd_last = 0.0f;
    static int leg1_last = 0, leg2_last = 0, leg3_last = 0, leg4_last = 0;
    float angle_now;
    float angle_filtered;
    float angle;
    float pitch_error;
    float abs_pitch_error;
    float leg_gain_p;
    float leg_target;
    float bump_leg_x_cmd;
    float bump_leg_y_cmd;
    uint8_t bump_leg_override;
    float projected_x;
    float projected_leg_error;
    float left_y_target;
    float right_y_target;
    float projection_ratio = 1.0f;
    int target_valid = 0;
    int projection_iter;
    int leg1;
    int leg2;
    int leg3;
    int leg4;
    (void)leg_Ki;
    (void)leg_Kd;
    bump_leg_override = Bumpy_Action_Get_Leg_Override(
        &bump_leg_x_cmd,
        &bump_leg_y_cmd);
    (void)bump_leg_x_cmd;

    leg_dbg_tick += 1.0f;

    if (Vehicle_Is_Emergency_Stop())
    {
        Runtime_Set_Servo_Reason(RUNTIME_REASON_SERVO_OFF);
        leg_dbg_speed_tilt = 0.0f;
        leg_dbg_x_offset = 0.0f;
        leg_dbg_x_step_used = 0.0f;
        leg_dbg_x_limit_hit = 0.0f;
        return;
    }

    if (!Runtime_Is_Module_Enabled(RUNTIME_MODULE_SERVO))
    {
        float base_x = constrain_float(*x, MIN_X, MAX_X);
        float base_y = constrain_float(*y, MIN_Y, MAX_Y);
        extern const float servo_alpha;
        Runtime_Set_Servo_Reason(RUNTIME_REASON_SERVO_OFF);

        servo_control(SERVO_LEG_LEFT, base_x, base_y, &leg1, &leg2);
        servo_control(SERVO_LEG_RIGHT, base_x, base_y, &leg3, &leg4);

        if (is_first_run)
        {
            leg1_last = leg1;
            leg2_last = leg2;
            leg3_last = leg3;
            leg4_last = leg4;
            x_cmd_last = base_x;
            is_first_run = false;
        }

        leg1 = (int)(leg1 * servo_alpha + leg1_last * (1.0f - servo_alpha));
        leg2 = (int)(leg2 * servo_alpha + leg2_last * (1.0f - servo_alpha));
        leg3 = (int)(leg3 * servo_alpha + leg3_last * (1.0f - servo_alpha));
        leg4 = (int)(leg4 * servo_alpha + leg4_last * (1.0f - servo_alpha));

        engine_left_maintain(leg1, leg2);
        engine_right_maintain(leg3, leg4);

        leg1_last = leg1;
        leg2_last = leg2;
        leg3_last = leg3;
        leg4_last = leg4;
        leg_error = 0.0f;
        leg_dbg_speed_tilt = 0.0f;
        leg_dbg_x_offset = 0.0f;
        leg_dbg_x_target = base_x;
        leg_dbg_x_cmd = base_x;
        leg_dbg_x_step_used = 0.0f;
        leg_dbg_x_limit_hit = 0.0f;
        return;
    }
const bool leg_adaptive_enable = false; // Keep fixed leg-height PID adaptation disabled.
    if (!leg_adaptive_enable)
    {
        float leg_x_step = constrain_float(leg_x_step_limit, 0.0001f, 0.02f);
        leg_dbg_x_step_used = leg_x_step;
        float base_x = constrain_float(*x, MIN_X, MAX_X);
        float base_y = constrain_float(
            bump_leg_override ? bump_leg_y_cmd : *y,
            MIN_Y,
            MAX_Y);
        float leg_x_offset = speed_tilt_to_leg_x(g_leg_speed_tilt_deg, base_y);
        float x_target = constrain_float(base_x + leg_x_offset, MIN_X, MAX_X);
        Runtime_Set_Servo_Reason(RUNTIME_REASON_SERVO_FIXED);
        leg_dbg_speed_tilt = g_leg_speed_tilt_deg;
        leg_dbg_x_offset = leg_x_offset;
        leg_dbg_x_target = x_target;

        *x = base_x;
        *y = base_y;
        left_y_cmd_last = *y;
        right_y_cmd_last = *y;
        leg_error = leg_x_offset;
        hold_mode = true;

        if (is_first_run)
        {
            x_cmd_last = x_target;
        }
        else
        {
            x_cmd_last += constrain_float(x_target - x_cmd_last, -leg_x_step, leg_x_step);
        }
        leg_dbg_x_cmd = x_cmd_last;

        servo_control(SERVO_LEG_LEFT, x_cmd_last, left_y_cmd_last, &leg1, &leg2);
        servo_control(SERVO_LEG_RIGHT, x_cmd_last, right_y_cmd_last, &leg3, &leg4);

        if (is_first_run)
        {
            leg1_last = leg1;
            leg2_last = leg2;
            leg3_last = leg3;
            leg4_last = leg4;
            is_first_run = false;
        }

        extern const float servo_alpha;
        leg1 = (int)(leg1 * servo_alpha + leg1_last * (1.0f - servo_alpha));
        leg2 = (int)(leg2 * servo_alpha + leg2_last * (1.0f - servo_alpha));
        leg3 = (int)(leg3 * servo_alpha + leg3_last * (1.0f - servo_alpha));
        leg4 = (int)(leg4 * servo_alpha + leg4_last * (1.0f - servo_alpha));

        engine_left_maintain(leg1, leg2);
        engine_right_maintain(leg3, leg4);

        leg1_last = leg1;
        leg2_last = leg2;
        leg3_last = leg3;
        leg4_last = leg4;
        return;
    }

    angle_now = IMU_data.filter_result.pitch;
    angle_filtered = leg_sensor_filter(angle_now, is_first_run);
    if (is_first_run)
    {
        angle_last = angle_filtered;
    }

    angle = 0.78f * angle_last + 0.22f * angle_filtered;
    angle_last = angle;

    {
        const float LEG_DEG_TO_RAD = (PI / 180.0f);
        const float HOLD_ENTER_PITCH = 0.8f * LEG_DEG_TO_RAD;
        const float HOLD_EXIT_PITCH = 1.8f * LEG_DEG_TO_RAD;
        const float HOLD_ENTER_ROLL = 1.2f * LEG_DEG_TO_RAD;
        const float HOLD_EXIT_ROLL = 2.5f * LEG_DEG_TO_RAD;
        const float HOLD_ENTER_RATE = 12.0f * LEG_DEG_TO_RAD;
        const float HOLD_EXIT_RATE = 20.0f * LEG_DEG_TO_RAD;
        const float LEG_SOFTZONE = 1.5f * LEG_DEG_TO_RAD;
        const float LEG_HARDZONE = 7.0f * LEG_DEG_TO_RAD;
        const float X_ACTIVEZONE = 2.0f * LEG_DEG_TO_RAD;
        const float LEG_GAIN_SCALE = 6.0f;
        const float LEG_MIN_DIFF = 0.0015f;
        const float LEG_MAX_DIFF = 0.020f;
        const float LEG_STEP_LIMIT = 0.0009f;
        const float X_STEP_LIMIT = 0.0008f;
        const float Y_STEP_LIMIT = 0.0006f;
        const float X_HOLD_BAND = 0.0008f;
        const float Y_HOLD_BAND = 0.0006f;
        float roll_angle_rad = IMU_data.filter_result.roll * LEG_DEG_TO_RAD;
        float pitch_rate = IMU_data.filter_result.unbiased_gyro_y * LEG_DEG_TO_RAD;
        float roll_rate = IMU_data.filter_result.unbiased_gyro_x * LEG_DEG_TO_RAD;
        float x_cal = 0.0f;
        float abs_roll_angle = fabsf(roll_angle_rad);
        float abs_pitch_rate = fabsf(pitch_rate);
        float abs_roll_rate = fabsf(roll_rate);

        pitch_error = -angle * LEG_DEG_TO_RAD;
        abs_pitch_error = fabsf(pitch_error);

        if (hold_mode)
        {
            if (abs_pitch_error > HOLD_EXIT_PITCH || abs_roll_angle > HOLD_EXIT_ROLL || abs_pitch_rate > HOLD_EXIT_RATE || abs_roll_rate > HOLD_EXIT_RATE)
            {
                hold_mode = false;
            }
        }
        else
        {
            if (abs_pitch_error < HOLD_ENTER_PITCH && abs_roll_angle < HOLD_ENTER_ROLL && abs_pitch_rate < HOLD_ENTER_RATE && abs_roll_rate < HOLD_ENTER_RATE)
            {
                hold_mode = true;
            }
        }

        leg_target = constrain_float(
            bump_leg_override ? bump_leg_y_cmd : *y,
            MIN_Y,
            MAX_Y);
        leg_gain_p = fabsf(leg_Kp) * LEG_GAIN_SCALE;
        pitch_error = -angle * LEG_DEG_TO_RAD;
        abs_pitch_error = fabsf(pitch_error);

        if (hold_mode)
        {
            leg_error_target = 0.0f;
            leg_error += constrain_float(-leg_error, -LEG_STEP_LIMIT, LEG_STEP_LIMIT);
            projected_x = x_cmd_last;
            projected_leg_error = 0.0f;
            left_y_target = leg_target;
            right_y_target = leg_target;
            target_valid = 1;
        }
        else
        {
            if (abs_pitch_error < LEG_SOFTZONE)
            {
                pitch_error = 0.0f;
            }
            else
            {
                pitch_error = ((pitch_error > 0.0f) ? 1.0f : -1.0f) * (abs_pitch_error - LEG_SOFTZONE);
            }

            leg_error_target = leg_gain_p * pitch_error;
            if (fabsf(leg_error_target) > 0.0f && fabsf(leg_error_target) < LEG_MIN_DIFF)
            {
                leg_error_target = (leg_error_target > 0.0f) ? LEG_MIN_DIFF : -LEG_MIN_DIFF;
            }
            if (abs_pitch_error > LEG_HARDZONE)
            {
                leg_error_target = (pitch_error >= 0.0f) ? LEG_MAX_DIFF : -LEG_MAX_DIFF;
            }
            else
            {
                leg_error_target = constrain_float(leg_error_target, -LEG_MAX_DIFF, LEG_MAX_DIFF);
            }
            leg_error += constrain_float(leg_error_target - leg_error, -LEG_STEP_LIMIT, LEG_STEP_LIMIT);

            if (abs_roll_angle > X_ACTIVEZONE)
            {
                x_cal = -0.012f * tanf((float)(IMU_data.filter_result.roll * PI / 180.0f));
            }
            x_cal = constrain_float(x_cal, -0.012f, 0.012f);

            projected_x = 0.0f;
            projected_leg_error = 0.0f;
            left_y_target = leg_target;
            right_y_target = leg_target;
            target_valid = 0;

            for (projection_iter = 0; projection_iter < 20; projection_iter++)
            {
                projected_x = x_cal * projection_ratio;
                projected_leg_error = leg_error * projection_ratio;
                left_y_target = constrain_float(leg_target - projected_leg_error, MIN_Y, MAX_Y);
                right_y_target = constrain_float(leg_target + projected_leg_error, MIN_Y, MAX_Y);

                if (servo_target_valid(SERVO_LEG_LEFT, projected_x, left_y_target) &&
                    servo_target_valid(SERVO_LEG_RIGHT, projected_x, right_y_target))
                {
                    target_valid = 1;
                    break;
                }

                projection_ratio *= 0.80f;
            }

            if (!target_valid)
            {
                projected_x = x_cmd_last;
                projected_leg_error = 0.0f;
                left_y_target = leg_target;
                right_y_target = leg_target;
                leg_error += constrain_float(-leg_error, -LEG_STEP_LIMIT, LEG_STEP_LIMIT);
            }
        }

        leg_error = projected_leg_error;

        if (is_first_run)
        {
            x_cmd_last = projected_x;
            left_y_cmd_last = left_y_target;
            right_y_cmd_last = right_y_target;
        }

        if (!hold_mode && fabsf(projected_x - x_cmd_last) >= X_HOLD_BAND)
        {
            x_cmd_last += constrain_float(projected_x - x_cmd_last, -X_STEP_LIMIT, X_STEP_LIMIT);
        }

        if (fabsf(left_y_target - left_y_cmd_last) >= Y_HOLD_BAND)
        {
            left_y_cmd_last += constrain_float(left_y_target - left_y_cmd_last, -Y_STEP_LIMIT, Y_STEP_LIMIT);
        }
        else if (hold_mode)
        {
            left_y_cmd_last = left_y_target;
        }

        if (fabsf(right_y_target - right_y_cmd_last) >= Y_HOLD_BAND)
        {
            right_y_cmd_last += constrain_float(right_y_target - right_y_cmd_last, -Y_STEP_LIMIT, Y_STEP_LIMIT);
        }
        else if (hold_mode)
        {
            right_y_cmd_last = right_y_target;
        }
    }

    *x = x_cmd_last;

    servo_control(SERVO_LEG_LEFT, *x, left_y_cmd_last, &leg1, &leg2);
    servo_control(SERVO_LEG_RIGHT, *x, right_y_cmd_last, &leg3, &leg4);

    if (is_first_run)
    {
        leg1_last = leg1;
        leg2_last = leg2;
        leg3_last = leg3;
        leg4_last = leg4;
        is_first_run = false;
    }

    extern const float servo_alpha;
    leg1 = (int)(leg1 * servo_alpha + leg1_last * (1.0f - servo_alpha));
    leg2 = (int)(leg2 * servo_alpha + leg2_last * (1.0f - servo_alpha));
    leg3 = (int)(leg3 * servo_alpha + leg3_last * (1.0f - servo_alpha));
    leg4 = (int)(leg4 * servo_alpha + leg4_last * (1.0f - servo_alpha));

    engine_left_maintain(leg1, leg2);
    engine_right_maintain(leg3, leg4);

    leg1_last = leg1;
    leg2_last = leg2;
    leg3_last = leg3;
    leg4_last = leg4;
}
// 速度补偿计算
float calculate_speed_compensation(float v, float h, float l)
{
// Calculate wheel path when crossing a single-side bridge.
    float S = sqrt(2 * h * h + (l / 2) * (l / 2));

    // 计算速度补偿
    float delta_v = (v * S - l) / l;

    return delta_v;
}

float leg_sensor_filter(float new_val, bool is_first_run)
{
#define WINDOW_SIZE 5
    static float buffer[WINDOW_SIZE] = {0};
    static float last_out = 0;
    static int count = 0;

// --- Cold-start initialization ---
    if (is_first_run)
    {
// On the first run, fill the buffer with the current sample.
        // 防止滤波器从 0 开始缓慢爬升或产生突跳
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            buffer[i] = new_val;
        }
        last_out = new_val;
        return new_val;
    }

    // --- 正常滤波逻辑 ---
    // 限幅处理
    const float LIMIT_THRESHOLD = 4.0f;
    if (fabs(new_val - last_out) > LIMIT_THRESHOLD)
    {
        new_val = (new_val > last_out) ? (last_out + LIMIT_THRESHOLD) : (last_out - LIMIT_THRESHOLD);
    }

    buffer[count % WINDOW_SIZE] = new_val;
    count++;

    float temp_arr[WINDOW_SIZE];
    memcpy(temp_arr, buffer, sizeof(buffer));

    // 排序逻辑...
    for (int i = 0; i < WINDOW_SIZE - 1; i++)
    {
        for (int j = 0; j < WINDOW_SIZE - 1 - i; j++)
        {
            if (temp_arr[j] > temp_arr[j + 1])
            {
                float temp = temp_arr[j];
                temp_arr[j] = temp_arr[j + 1];
                temp_arr[j + 1] = temp;
            }
        }
    }

    last_out = temp_arr[WINDOW_SIZE / 2];
    return last_out;
}
float max(float a, float b)
{
    return (a > b) ? a : b;
}
float min(float a, float b)
{
    return (a < b) ? a : b;
}



