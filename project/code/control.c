#include "control.h"
#include "FiveBarLinkageData.h"
#include "kalman_rm.h"
#include "param.h"
// 全局变量
extern float target_velocity;      // 目标速度
extern float target_angle;         // 目标角度
float now_velocity = 0.0;          // 实际速度值
extern float target_motor_Stand;   // 目标电机角度
float Encoder_Left, Encoder_Right; // 左右电机编码器值
float leg_error;

int speed_up = 0;
float Turn_Pwm; // 转向PWM值

// PID参数
extern pid_param_t motor_speed;     // 速度PID参数---------------------速度环
extern pid_param_t motor_Stand;     // 电机角度PID参数---------------------角度环
extern pid_param_t motor_direction; // 方向PID参数------------------------方向调整
extern pid_param_t motor_gyro;      // 陀螺仪PID参数------------------------角速度环
extern pid_param_t air_roll_pid;    // 空中控制器参数
extern pid_param_t motor_leg_pid;   // 腿高控制器
extern float x_current, y_current;
int i = 0;
// 电机输出
signed short int Motor_Left, Motor_Right;        // 左右电机PWM输出
float Velocity_Angle_left, Velocity_Angle_right; // 左右电机速度
// 其他变量
float last_error;                 // 上一次误差
extern Attitude_3D_Kalman filter; // 卡尔曼滤波器
extern IMU_t IMU_data;            // IMU数据
extern float v_buchang;
float roll;              // 倾斜角度
int engine_change = 600; // 发动机变化量
//=======================未加入Jump Camera时的临时动作====================================
int jump_stop = 0;
int jump_position = 0;
float border = 94;
//============================================================================/
// 腿部控制参数

float temp_a, temp_b;

// 模糊规则参数
typedef struct
{
    float error_threshold_high;   // 高误差阈值
    float error_threshold_low;    // 低误差阈值
    float d_error_threshold_high; // 高误差变化率阈值
    float d_error_threshold_low;  // 低误差变化率阈值
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
    .d_error_threshold_high = 30.0, // 高误差变化率阈值（度/秒）
    .d_error_threshold_low = 5.0,   // 低误差变化率阈值（度/秒）
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
    float de = d_error; // 误差变化率

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
    PidChange(&motor_speed, Speed_p, Speed_i, Speed_d); // 速度环

    PidInit(&motor_Stand);
    PidChange(&motor_Stand, Angle_p, Angle_i, Angle_d); // 角度环

    PidInit(&motor_direction);
    PidChange(&motor_direction, Direction_p, Direction_i, Direction_d); // 转向环

    PidInit(&motor_gyro);
    PidChange(&motor_gyro, Gyro_p, Gyro_i, Gyro_d); // 角速度环

    PidInit(&air_roll_pid);
    PidChange(&air_roll_pid, Air_roll_p, Air_roll_i, Air_roll_d); // 初始化空中控制器PID
}
// 检测是否处于腾空状态
bool is_airborne()
{

    const float threshold = 0.5;
    return fabs(IMU_data.accel[2] - 1) > threshold;
}
// 应用空中控制
void apply_air_control(float roll_control)
{
    // 根据控制输出调整轮子转速
    float wheel_left = roll_control;
    float wheel_right = roll_control;

    // 限制轮子转速范围
    wheel_left = fmaxf(fminf(wheel_left, 1000.0), -1000.0);
    wheel_right = fmaxf(fminf(wheel_right, 1000.0), -1000.0);

    // 设置轮子转速
    small_driver_set_duty(wheel_left, wheel_right);
}

// 空中控制器主函数
void air_control()
{
    static float last_roll_error = 0.0;
    // 获取当前姿态
    float current_roll = IMU_data.filter_result.roll;

    // 目标姿态（可以根据需要调整）
    float target_roll = 0.0;

    // 计算误差
    float roll_error = target_roll - current_roll;

    // 计算误差变化率
    float roll_d_error = roll_error - last_roll_error;

    // 保存当前误差
    last_roll_error = roll_error;
    // printf("data: %f,\r\n", current_roll);
    //  计算控制输出
    float roll_control = -air_roll_pid.kp * roll_error - air_roll_pid.ki * roll_error - air_roll_pid.kd * roll_d_error;

    // 限制输出范围
    roll_control = fmaxf(fminf(roll_control, 1000.0), -1000.0);

    // 应用控制输出到轮子
    apply_air_control(roll_control);
}

void adjust_pid_based_on_leg_height(float *current_leg_height)
{
    // 计算腿部高度比例（0到1之间）
    float leg_ratio = (*current_leg_height - MIN_LEG_LENGTH) / (MAX_LEG_LENGTH - MIN_LEG_LENGTH);
    leg_ratio = fmaxf(fminf(leg_ratio, 1.0), 0.0);

    // 原始参数（腿部高度较低时）
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
    servo_control(x_current, y_current, &leg1, &leg2);

    //============调试使用=========================
    // printf("Leg1: %d, Leg2: %d\r\n", leg1, leg2);
    //============调试使用=========================

    engine_init(leg1, leg2); // 初始化发动机
}
// 角度补偿量计算，使得平衡环目标改变，控制量为速度（PI速度计算，以编码器计）
float Velocity(int encoder_left, float target_velocity) //===========左边为基础，右边送入务必取反
{
    static float velocity;         // 当前速度
    static float Encoder_bias;     // 编码器偏差
    static float Encoder_Integral; // 积分项

    Encoder_bias = target_velocity - encoder_left; // 计算偏差
    Encoder_Integral += Encoder_bias;              // 积分
    //==============================更改了积分限幅措施（7000）======================
    // 限制积分范围
    if (Encoder_Integral > 2000)
        Encoder_Integral = 2000;
    if (Encoder_Integral < -2000)
        Encoder_Integral = -2000;
    //==============================更改了积分限幅措施（7000）======================

    // 计算角度（Velocity的物理意义是输出角度）
    velocity = motor_speed.kp * Encoder_bias + motor_speed.ki * Encoder_Integral;
    // 限制速度范围
    if (velocity > 18)
        velocity = 18;
    if (velocity < -18)
        velocity = -18;

    // 动态调整PID参数
    // fuzzy_pid_adjust(&motor_speed, Encoder_bias, velocity - last_error, &speed_rules, &speed_pid_limits);

    //============调试使用=========================
    // printf("Encoder_bias|Encoder_Integral|velocity(angle):%f,%f,%f\r\n", Encoder_bias, Encoder_Integral, velocity);
    // printf("target_velocity : %lf\r\n", target_velocity);
    //============调试使用=========================

    return velocity; // 角度补偿值
}
// 平衡控制计算（PD控制角度环）
float Balance(float Angle, float Gyro, float target)
{
    float Angle_bias = target_motor_Stand + target - Angle;                    // 计算角度偏差
    float Gyro_bias = 0 - Gyro;                                                // 微分控制项，用于平缓过度
    float balance = -motor_Stand.kp * Angle_bias + Gyro_bias * motor_Stand.kd; // （修改了Gyro_bias的符号）
    /*增大角度的本质在于适当减速使得车身前倾，因此为负值*/
    // printf("%lf\n", Gyro);
    //  printf("data: %f,%f,%f\r\n", Angle_bias, Gyro_bias, balance);
    //  fuzzy_pid_adjust(&motor_Stand, Angle_bias, Gyro_bias, &angle_rules, &angle_pid_limits);

    last_error = Angle_bias; // 更新误差
    if (balance > 5000)
        balance = 5000; //=========限幅5000
    if (balance < -5000)
        balance = -5000;

    //============调试使用=========================
    // printf("Angle_bias|Gyro_bias|balance : %f,%f,%f\r\n", Angle_bias, Gyro_bias, balance);
    //============调试使用=========================

    return balance; // 返回平衡值，留给后面使用
}

// 陀螺仪控制计算（PID计算朝向角度）
float GyroControl(float target_gyro, float current_gyro) // 角速度环
{
    float gyro_error = target_gyro - current_gyro; // 计算陀螺仪误差
    static float gyro_Integral;                    // 积分项
    gyro_Integral += gyro_error;                   // 积分
    if (gyro_Integral > 1500)
        gyro_Integral = 1500;
    if (gyro_Integral < -1500)
        gyro_Integral = -1500;
    // 计算控制输出
    float gyro_control = +motor_gyro.kp * gyro_error + motor_gyro.ki * gyro_Integral + motor_gyro.kd * (gyro_error - last_error);
    last_error = gyro_error; // 更新误差

    // 动态调整PID参数
    // fuzzy_pid_adjust(&motor_gyro, gyro_error, gyro_error - last_error, &gyro_rules, &gyro_pid_limits);

    //============调试使用=========================
    //    printf("gyro_error|gyro_Integral|gyro_control: %f,%f,%f\r\n", gyro_error, gyro_Integral, gyro_control);
    // printf("target_gyro:%lf\r\n", target_gyro);
    //============调试使用=========================

    // printf("gyro_error|gyro_Integral|gyro_control: %f,%f,%f\r\n", gyro_error, gyro_Integral, gyro_control);
    return -gyro_control; // 返回控制输出
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
    int y = 0;
    if (target_angle >= 0)
    {
        y = target_angle - 180;
        if (gyro <= y && gyro >= -180)
            gyro = 360 + gyro;
        gyro = gyro - target_angle;
    }
    else
    {
        y = target_angle + 180;
        if (gyro <= 180 && gyro >= y)
            gyro = gyro - 360;
        gyro = gyro - target_angle;
    }
    return gyro;
    /*它是转向 PID 控制器的输入。该函数解决了角度在180° 附近跳转时的“环绕问题”，确保小车总是向着目标方向转动更小的弧度，而不是为了转过一个微小的角度而绕一个大圈 */
}

// 转向目标角度计算,划分到合适区间（-180~+180）
float Turn_target(float target_angle)
{
    if (target_angle >= 180)
        target_angle = target_angle - 360;
    else if (target_angle <= -180)
        target_angle = 360 + target_angle;
    return target_angle;
}

// 转向控制计算
float Turn(float gyro, float target_angle)
{
    // Gyro传入的是当前的角度
    static float previous_error = 0.0; // 上一次误差
    float error = gyro - target_angle; // 当前误差
    if (fabs(error) < 3)
    { // 低通截断，避免毛刺影响
        return 0;
    }
    float derivative = error - previous_error; // 微分项
    previous_error = error;                    // 更新误差

    // 计算控制输出
    float control_output = -motor_direction.kp * error - error * func_abs(error) * motor_direction.ki - motor_direction.kd * derivative;
    // 限制输出范围
    if (control_output > 2000)
        control_output = 2000;
    if (control_output < -2000)
        control_output = -2000;

    // 动态调整PID参数

    return control_output; // 返回控制输出
}

// 平衡控制主函数
void balance_control()
{
    float Balance_Pwm_left, Balance_Pwm_right, Gyro_Pwm_left, Gyro_Pwm_right; // 平衡PWM值k

    // 获取编码器值
    Encoder_Left = -motor_value.receive_left_speed_data;
    Encoder_Right = -motor_value.receive_right_speed_data;
    now_velocity = (Encoder_Left - Encoder_Right) / 2;

    // 更新倾斜角度
    roll = IMU_data.filter_result.roll;
    i++;
    // 跳跃更新
    if (jump_stop == 1)
    {
        // 初始化速度PID
        PidInit(&motor_speed);
        PidChange(&motor_speed, 0, 0, 0);

        // 初始化电机角度PID
        PidInit(&motor_Stand);
        PidChange(&motor_Stand, 0, 0, 0);
    }
    else
    {
        // 初始化速度PID
        PidInit(&motor_speed);
        PidChange(&motor_speed, Speed_p, Speed_i, Speed_d);

        // 初始化电机角度PID
        PidInit(&motor_Stand);
        PidChange(&motor_Stand, Angle_p, Angle_i, Angle_d);
    }
    // 计算左右电机速度
    if (i % 5 == 0)
    {
        /********速度补偿单边桥启动***************************/
        if (leg_error > 0)
        {
            Velocity_Angle_left = Velocity(Encoder_Left, target_velocity - v_buchang);
            Velocity_Angle_right = Velocity(-Encoder_Right, target_velocity + v_buchang); // 右侧输入务必取反
        }
        else
        {
            Velocity_Angle_left = Velocity(Encoder_Left, target_velocity + v_buchang);
            Velocity_Angle_right = Velocity(-Encoder_Right, target_velocity - v_buchang);
        }
    }

    //    printf("jiadata:%f,%f\r\n",Encoder_Left,target_velocity);
    // 平滑处理速度
    //    left_angle = left_angle * 0.2 + Velocity_Angle_left * 0.8;
    //    right_angle = right_angle * 0.2 + Velocity_Angle_right * 0.8;
    // 计算平衡PWM值
    if (i % 2 == 0)
    {
        Balance_Pwm_left = Balance(roll, IMU_data.gyro[0], Velocity_Angle_left);
        Balance_Pwm_right = Balance(roll, IMU_data.gyro[0], Velocity_Angle_right);
    }
    // 计算陀螺仪控制PWM值

    Gyro_Pwm_left = GyroControl(-Balance_Pwm_left, IMU_data.gyro[0]);
    Gyro_Pwm_right = GyroControl(-Balance_Pwm_right, IMU_data.gyro[0]);

    // 计算最终PWM输出
    if (jump_position == 1 || target_velocity == 0)
    {
        Motor_Left = (signed short int)Gyro_Pwm_left;
        Motor_Right = (signed short int)Gyro_Pwm_right;
    }
    else
    {
        // 计算转向PWM值
        // Turn_Pwm = Turn(IMU_data.filter_result.yaw, target_angle); // 中点拟合
        // //===================仅调试去除转向功能使用=================================
        Turn_Pwm = 0;
        // //====================================================================
        if (Turn_Pwm <= 0.2) // !!!!!!!!!!!!!!!!!!!转向中的积分可能有问题哦
        {
            speed_up = 1;
        }
        else if (Turn_Pwm <= 2)
        {
            speed_up = 2;
        }
        else
        {
            speed_up = 0;
        }
        Motor_Left = (signed short int)Gyro_Pwm_left * (1 + Turn_Pwm) - (signed short int)(imu963ra_gyro_z / 2);
        Motor_Right = (signed short int)Gyro_Pwm_right * (1 - Turn_Pwm) + (signed short int)(imu963ra_gyro_z / 2);
    }
    // 限制PWM输出范围
    Motor_Left = -(signed short int)cuu(Motor_Left);
    Motor_Right = (signed short int)cuu(Motor_Right);

    // 设置PWM输出
    small_driver_set_duty(-Motor_Left, -Motor_Right);

    // printf("\n\n\n");
}

// 滤波处理
float filter_leg_control(float current_angle, float target_angle, float filter_factor)
{
    return current_angle * filter_factor + target_angle * (1 - filter_factor);
}
/* 腿部控制器 */
float g_roll_int_gain = 0.004f;
float g_roll_d_gain = 0.00000f;

void leg_control(float *x, float *y)
{
    /*
    // ---------- 1. 计算腿部位置：前后倾斜补偿 ----------
    float x_cal = 0.0450 * tan((double)((Velocity_Angle_left + Velocity_Angle_right) / 2 / 180 * 3.14));

    if (x_cal > 0.04f)
        x_cal = 0.04f;
    else if (x_cal < -0.04f)
        x_cal = -0.04f;

    //    *x = x_cal;

*/

    extern float leg_Kp, leg_Ki, leg_Kd;
    PidInit(&motor_leg_pid);
    PidChange(&motor_leg_pid, leg_Kp, leg_Ki, leg_Kd);
    motor_leg_pid.imax = 0.01f;

    // ---------- 2. 左右平衡：单边抬高 (去毛刺与平滑处理) ----------
    //======================================================
    //======================================================
    float angle_now = IMU_data.filter_result.pitch;
    float target_pitch = 0.0f;
    static float angle_last = 0.0f;
    static bool is_first_run = true;
    float angle_filtered = leg_sensor_filter(angle_now, is_first_run);
    if (is_first_run)
    {
        angle_last = angle_filtered;
        // 注意：此处先不要将 is_first_run 设为 false，留到函数末尾舵机状态初始化后再处理
    }
    float angle = 0.5f * angle_last + 0.5f * angle_filtered;
    angle_last = angle;

    float pitch_error = target_pitch - angle_filtered;
    leg_error =  PidLocCtrl(&motor_leg_pid, pitch_error); 

    float LEG_DEADZONE =2.5f;//角度低阈值限幅
    if (fabs(pitch_error) < LEG_DEADZONE) {
        leg_error = 0;
        motor_leg_pid.integrator = 0; // 处于死区时清除积分，防止由于积分积累导致的缓慢爬行
    }

    // printf("pitch_error:%lf \n",pitch_error);  //  送入正常
    // printf("kp: %lf \n",motor_leg_pid.kp);
    // printf("P/I/D %lf %lf %lf \n",motor_leg_pid.out_p,motor_leg_pid.out_i,motor_leg_pid.out_d);
    // printf("leg_error:%lf \n",leg_error);     //输出及其异常

    // if (leg_error > 0.08f)  ----------这是PID的参数，本身不要限制！！
    //     leg_error = 0.08f;
    // else if (leg_error < -0.08f)
    //     leg_error = -0.08f;

    // ---------- 3. 计算前后基准高度 ----------

    // // ---------- 4. 叠加左右补偿：单边抬高 ----------
    // float left_y = leg_target;
    // float right_y = leg_target;

    // if (leg_error > 0)
    //     left_y = leg_target + leg_error;
    // else if (leg_error < 0)
    //     right_y = leg_target - leg_error;
    // 希望两边抬高
    float leg_target = *y;
    float left_y = max(min(leg_target - leg_error, MAX_Y), MIN_Y);
    float right_y = max(min(leg_target + leg_error, MAX_Y), MIN_Y);
    printf("Left/Right:%lf %lf\n", left_y, right_y);

    // ---------- 5. 逆运动学解算 ----------
    int leg1, leg2, leg3, leg4;
    static int leg1_last = 0, leg2_last = 0, leg3_last = 0, leg4_last = 0;

    servo_control(*x, left_y, &leg1, &leg2);  // 左腿
    servo_control(*x, right_y, &leg3, &leg4); // 右腿

    if (is_first_run)
    {
        leg1_last = leg1;
        leg2_last = leg2;
        leg3_last = leg3;
        leg4_last = leg4;
        is_first_run = false;
    }

    // ---------- 6. 舵机最终输出的机械平滑 ----------
    // 在最终驱动电机前再加一道轻度平滑，防止机械机构高频抖动共振
    // alpha 越小越平滑，但响应越慢；0.3 到 0.5 通常是不错的平衡点
    extern const float servo_alpha; // 0.4--oorigin
    leg1 = (int)(leg1 * servo_alpha + leg1_last * (1.0f - servo_alpha));
    leg2 = (int)(leg2 * servo_alpha + leg2_last * (1.0f - servo_alpha));
    leg3 = (int)(leg3 * servo_alpha + leg3_last * (1.0f - servo_alpha));
    leg4 = (int)(leg4 * servo_alpha + leg4_last * (1.0f - servo_alpha));

    engine_left_maintain(leg1, leg2);
    engine_right_maintain(leg3, leg4);
    // printf("Leg Positions:%d,%d,%d,%d\n", leg1, leg2, leg3, leg4);
    // printf("%lf\n", IMU_data.filter_result.pitch);
    // 记录历史值
    leg1_last = leg1;
    leg2_last = leg2;
    leg3_last = leg3;
    leg4_last = leg4;
}
// 速度补偿计算
float calculate_speed_compensation(float v, float h, float l)
{
    // 计算单边桥上的轮子路程
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

    // --- 冷启动标注处理 ---
    if (is_first_run)
    {
        // 第一次运行时，强制将缓冲区全部填充为当前值
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