#include "control.h"
#include "FiveBarLinkageData.h"
#include "imu.h"
#include "param.h"
#include "ipc_shared_data.h"
#include "zf_device_imu660rc.h"
// 全局变量
extern float target_velocity;      // 目标速度
extern float target_angle;         // 目标角度
float now_velocity = 0.0;          // 实际速度�?
extern float target_motor_Stand;   // 目标电机角度
float Encoder_Left, Encoder_Right; // 左右电机编码器�?
float leg_error;

int speed_up = 0;
float Turn_Pwm; // 转向PWM�?

// PID参数
extern pid_param_t motor_speed;     // 速度PID参数---------------------速度�?
extern pid_param_t motor_Stand;     // 电机角度PID参数---------------------角度�?
extern pid_param_t motor_direction; // 方向PID参数------------------------方向调整
extern pid_param_t motor_gyro;      // 陀螺仪PID参数------------------------角速度�?
extern pid_param_t air_roll_pid;    // 空中控制器参�?
extern pid_param_t motor_leg_pid;   // 腿高控制�?
extern float x_current, y_current;
int i = 0;
// 电机输出
signed short int Motor_Left, Motor_Right;        // 左右电机PWM输出
float Velocity_Angle_left, Velocity_Angle_right; // 左右电机速度
float out_speed_l = 0, out_speed_r = 0;
float out_angle_l = 0, out_angle_r = 0;
float out_gyro_l  = 0, out_gyro_r  = 0;
// 其他变量
static float balance_last_error = 0.0f;
static float gyro_last_error = 0.0f;
extern IMU_t IMU_data;            // IMU数据
float roll;              // 倾斜角度
int engine_change = 600; // 发动机变化量
//=======================未加入Jump Camera时的临时动作====================================
int jump_stop = 0;
int jump_position = 0;
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
float leg_dbg_speed_tilt = 0.0f;
float leg_dbg_x_offset = 0.0f;
float leg_dbg_x_target = 0.0f;
float leg_dbg_x_cmd = 0.0f;
float leg_dbg_tick = 0.0f;

static float speed_tilt_to_leg_x(float leg_tilt_deg, float leg_height)
{
    const float LEG_DEG_TO_RAD = (PI / 180.0f);
    float x_gain = (leg_x_gain > 0.0f) ? leg_x_gain : Leg_X_Gain_init;
    float x_limit = (leg_x_limit > 0.0f) ? leg_x_limit : Leg_X_Limit_init;
    float x_min_step = (leg_x_min_step > 0.0f) ? leg_x_min_step : Leg_X_Min_Step_init;
    float tilt_rad;
    float x_offset;

    leg_tilt_deg = constrain_float(leg_tilt_deg, -10.0f, 10.0f);
    leg_height = constrain_float(leg_height, MIN_Y, MAX_Y);
    tilt_rad = leg_tilt_deg * LEG_DEG_TO_RAD;
    x_offset = x_gain * leg_height * tanf(tilt_rad);

    if (fabsf(leg_tilt_deg) > 0.3f && fabsf(x_offset) < x_min_step)
    {
        x_offset = (leg_tilt_deg > 0.0f) ? x_min_step : -x_min_step;
    }

    return constrain_float(x_offset, -x_limit, x_limit);
}

// 模糊规则参数
typedef struct
{
    float error_threshold_high;   // 高误差阈�?
    float error_threshold_low;    // 低误差阈�?
    float d_error_threshold_high; // 高误差变化率阈�?
    float d_error_threshold_low;  // 低误差变化率阈�?
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
    .d_error_threshold_high = 30.0, // 高误差变化率阈值（�?秒）
    .d_error_threshold_low = 5.0,   // 低误差变化率阈值（�?秒）
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
    float de = d_error; // 误差变化�?

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
    PidChange(&motor_speed, Speed_p, Speed_i, Speed_d); // 速度�?

    PidInit(&motor_Stand);
    PidChange(&motor_Stand, Angle_p, Angle_i, Angle_d); // 角度�?

    PidInit(&motor_direction);
    PidChange(&motor_direction, Direction_p, Direction_i, Direction_d); // 转向�?

    PidInit(&motor_gyro);
    PidChange(&motor_gyro, Gyro_p, Gyro_i, Gyro_d); // 角速度�?

    PidInit(&air_roll_pid);
    PidChange(&air_roll_pid, Air_roll_p, Air_roll_i, Air_roll_d); // 初始化空中控制器PID
}
// 检测是否处于腾空状�?
bool is_airborne()
{

    const float threshold = 0.5;
    return fabs(IMU_data.accel[2] - 1) > threshold;
}
// 应用空中控制
void apply_air_control(float roll_control)
{
    // 根据控制输出调整轮子转�?
    float wheel_left = roll_control;
    float wheel_right = roll_control;

    // 限制轮子转速范�?
    wheel_left = fmaxf(fminf(wheel_left, 1000.0), -1000.0);
    wheel_right = fmaxf(fminf(wheel_right, 1000.0), -1000.0);

    // 设置轮子转�?
    small_driver_set_duty(wheel_left, wheel_right);
}

// 空中控制器主函数
void air_control()
{
    static float last_roll_error = 0.0;
    // 获取当前姿�?
    float current_roll = IMU_data.filter_result.roll;

    // 目标姿态（可以根据需要调整）
    float target_roll = 0.0;

    // 计算误差
    float roll_error = target_roll - current_roll;

    // 计算误差变化�?
    float roll_d_error = roll_error - last_roll_error;

    // 保存当前误差
    last_roll_error = roll_error;
    // printf("data: %f,\r\n", current_roll);
    //  计算控制输出
    float roll_control = -air_roll_pid.kp * roll_error - air_roll_pid.ki * roll_error - air_roll_pid.kd * roll_d_error;

    // 限制输出范围
    roll_control = fmaxf(fminf(roll_control, 1000.0), -1000.0);

    // 应用控制输出到轮�?
    apply_air_control(roll_control);
}

void adjust_pid_based_on_leg_height(float *current_leg_height)
{
    // 计算腿部高度比例�?�?之间�?
    float leg_ratio = (*current_leg_height - MIN_LEG_LENGTH) / (MAX_LEG_LENGTH - MIN_LEG_LENGTH);
    leg_ratio = fmaxf(fminf(leg_ratio, 1.0), 0.0);

    // 原始参数（腿部高度较低时�?
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
// 角度补偿量计算，使得平衡环目标改变，控制量为速度（PI速度计算，以编码器计�?
float Velocity(velocity_loop_state_t *state, float measured_velocity, float target_velocity) //===========左边为基础，右边送入务必取反
{
    state->encoder_bias = target_velocity - measured_velocity; // 计算偏差
    state->encoder_integral += state->encoder_bias;       // 积分
    //==============================更改了积分限幅措施（7000�?=====================
    // 限制积分范围
    if (state->encoder_integral > 2000)
        state->encoder_integral = 2000;
    if (state->encoder_integral < -2000)
        state->encoder_integral = -2000;
    //==============================更改了积分限幅措施（7000�?=====================

    // 计算角度（Velocity的物理意义是输出角度�?
    state->velocity = motor_speed.kp * state->encoder_bias + motor_speed.ki * state->encoder_integral;
    // 限制速度范围
    if (state->velocity > 6)
        state->velocity = 6;
    if (state->velocity < -6)
        state->velocity = -6;

    // 动态调整PID参数
    // fuzzy_pid_adjust(&motor_speed, state->encoder_bias, state->velocity - last_error, &speed_rules, &speed_pid_limits);

    //============调试使用=========================
    // printf("Encoder_bias|Encoder_Integral|velocity(angle):%f,%f,%f\r\n", state->encoder_bias, state->encoder_integral, state->velocity);
    // printf("target_velocity : %lf\r\n", target_velocity);
    //============调试使用=========================

    return state->velocity; // �ٶȻ����ŷ�ת����ǰΪ���Ű汾�� A/B ��֤
}
// 平衡控制计算（PD控制角度环）
float Balance(float Angle, float Gyro, float target)
{
    static float angle_integral = 0.0f;                                    // �ǶȻ����������������̬��̬ƫ��
    float Angle_bias = target_motor_Stand + target - Angle;                // ����Ƕ�ƫ��
    float Gyro_bias = 0 - Gyro;                                            // ΢�ֿ��������ƽ������
    angle_integral += Angle_bias;
    angle_integral = constrain_float(angle_integral, -1000.0f, 1000.0f);

    float balance = -motor_Stand.kp * Angle_bias - motor_Stand.ki * angle_integral + Gyro_bias * motor_Stand.kd;

    balance_last_error = Angle_bias; // �������
    if (balance > 5000)
        balance = 5000; //=========�޷�5000
    if (balance < -5000)
        balance = -5000;

    return balance; // ����ƽ��ֵ����������ʹ��
}

// 陀螺仪控制计算（PID计算朝向角度�?
float GyroControl(float target_gyro, float current_gyro) // ���ٶȻ�
{
    float gyro_error = target_gyro - current_gyro;                         // �������������
    static float gyro_Integral;                                            // ������
    gyro_Integral += gyro_error;
    if (gyro_Integral > 1500)
        gyro_Integral = 1500;
    if (gyro_Integral < -1500)
        gyro_Integral = -1500;

    float gyro_delta = gyro_error - gyro_last_error;
    float gyro_control = +motor_gyro.kp * gyro_error + motor_gyro.ki * gyro_Integral + motor_gyro.kd * gyro_delta;
    gyro_last_error = gyro_error; // �������    return gyro_control; // gyro loop output sign fixed
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

// 最小转向角
float Turn_gyro(float target_angle, float gyro)
{
    // 1. 计算原始误差
    float error = target_angle - gyro;

    // 2. 将误差归一化到 [-180, 180] 之间
    // 这样可以确保小车永远旋转角度差绝对值小�?180 的那个方�?
    while (error > 180)  error -= 360;
    while (error < -180) error += 360;

    return error; // 返回�?PID 作为 Input
}

// 转向目标角度计算,划分到合适区间（-180~+180�?
float Turn_target(float target_angle)
{
    if (target_angle >= 180)
        target_angle = target_angle - 360;
    else if (target_angle <= -180)
        target_angle = 360 + target_angle;
    return -target_angle;
}

// 转向控制计算
float Turn(float current_yaw, float target_yaw)
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

    control_output = Direction_p * yaw_error - Direction_d * yaw_rate;
    control_output = constrain_float(control_output, -1200.0f, 1200.0f);

    return control_output;
}

// 平衡控制主函�?
void balance_control()
{
    static float Balance_Pwm = 0.0f;
    static uint8_t angle_loop_div = 0;
    static uint8_t speed_loop_div = 0;
    static uint8_t leg_loop_div = 0;
    float Gyro_Pwm;
    float angle_gyro_x = imu660rc_gyro_transition(imu660rc_gyro_x) - 0.49f;
    float gyro_loop_x = (float)imu660rc_gyro_x - 0.49f * imu660rc_transition_factor[1];

    if (IPC_CoreB_Wifi_Is_Connected() == 0)
    {
        Balance_Pwm = 0.0f;
        Velocity_Angle_left = 0.0f;
        Velocity_Angle_right = 0.0f;
        Turn_Pwm = 0.0f;
        Motor_Left = 0;
        Motor_Right = 0;
        out_speed_l = 0.0f;
        out_speed_r = 0.0f;
        out_angle_l = 0.0f;
        out_angle_r = 0.0f;
        out_gyro_l = 0.0f;
        out_gyro_r = 0.0f;
        small_driver_set_duty(0, 0);
        return;
    }

    if (First_angle && IMU_ready)
    {
        target_angle = 180.0f;
        First_angle = false;
    }

    roll = IMU_data.filter_result.roll;

    if (jump_stop == 1)
    {
        PidChange(&motor_speed, 0, 0, 0);
        PidChange(&motor_Stand, 0, 0, 0);
        PidChange(&motor_gyro, 0, 0, 0);
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

        Balance_Pwm = Balance(roll, angle_gyro_x, 0.0f);
    }

    Gyro_Pwm = GyroControl(Balance_Pwm * imu660rc_transition_factor[1], gyro_loop_x);
    Turn_Pwm = Turn(IMU_data.filter_result.yaw, target_angle);

    if (jump_position == 1)
    {
        Motor_Left = (signed short int)Gyro_Pwm;
        Motor_Right = (signed short int)Gyro_Pwm;
    }
    else
    {
        Motor_Left = (signed short int)(Gyro_Pwm + Turn_Pwm);
        Motor_Right = (signed short int)(Gyro_Pwm - Turn_Pwm);
    }

    Motor_Left = -(signed short int)cuu(Motor_Left);
    Motor_Right = (signed short int)cuu(Motor_Right);

    out_speed_l = Velocity_Angle_left;
    out_speed_r = Velocity_Angle_left;
    out_angle_l = Balance_Pwm;
    out_angle_r = Balance_Pwm;
    out_gyro_l = Gyro_Pwm;
    out_gyro_r = Gyro_Pwm;

    small_driver_set_duty(-Motor_Left, -Motor_Right);

    leg_loop_div++;
    if (leg_loop_div >= 20)
    {
        leg_loop_div = 0;
        leg_control(&x_current, &y_current);
    }
}

// 滤波处理
float filter_leg_control(float current_angle, float target_angle, float filter_factor)
{
    return current_angle * filter_factor + target_angle * (1 - filter_factor);
}
/* 腿部控制�?*/

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

    leg_dbg_tick += 1.0f;

    const bool leg_adaptive_enable = false; // ��ƽ����ν׶��ȹ̶��Ȳ��������ȿ��Ŷ������� PID��
    if (!leg_adaptive_enable)
    {
        float leg_x_step = constrain_float(leg_x_step_limit, 0.0001f, 0.02f);
        float base_x = constrain_float(*x, MIN_X, MAX_X);
        float base_y = constrain_float(*y, MIN_Y, MAX_Y);
        float leg_x_offset = speed_tilt_to_leg_x(g_leg_speed_tilt_deg, base_y);
        float x_target = constrain_float(base_x + leg_x_offset, MIN_X, MAX_X);
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

        leg_target = constrain_float(*y, MIN_Y, MAX_Y);
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
    // 计算单边桥上的轮子路�?
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

    // --- 冷启动标注处�?---
    if (is_first_run)
    {
        // 第一次运行时，强制将缓冲区全部填充为当前�?
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








