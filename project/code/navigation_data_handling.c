/*********************************************************************************************************************

*  问题1:零点对齐：当车头指向正北（世界坐标系 X 轴）时，IMU 输出的 Yaw 是否为 $0$？            问题二：方向极性：当机器人向右（东）转动时，IMU 输出的 Yaw 值是否在增加？

*              

*                

*********************************************************************************************************************/

#include "navigation_data_handling.h"

#include "navigation_tracking.h"

#include "navigation_action.h"

#include "course3_bridge_odometry.h"

#include "control.h"

#include "small_driver_uart_control.h"

#include <math.h>



//====================================================变量声明=======================================================

extern IMU_t IMU_data;                         //IMU数据结构体


//其他文件夹函数使用情况

//leg_roll_control(float leg_target, float angle_error);

//Turn(float gyro, float target_angle);

//is_airborne(void); // 引用队友 control.c 中的函数)

//engine_jump(void);

extern float Turn_target(float target_angle);         //将目标限制在-180-+180



//====================================================自定义全局变量=================================================
RobotState_t robot_pose;                                  // 全局实时位姿，供外部只读访问

Navi_Sensor_Data_t       raw_data  ,filter_data;                        // 原始预处理数据    ，基础滤波+处理后数据

static KalmanFilter_Struct nav_ekf;      // 5阶导航 EKF 结构体

static LowPassFilter_Struct   lpf_v_left,  lpf_v_right, lpf_ax, lpf_w;

static float yaw_history_sin[YAW_HISTORY_LEN] = {0};
static float yaw_history_cos[YAW_HISTORY_LEN] = {0};
static uint16_t yaw_hist_idx = 0;
static uint8_t yaw_hist_filled = 0; // 标记窗口是否已经填满过

typedef struct {
    float sum_sin;
    float sum_cos;
    uint16_t sample_count;
    uint8_t active;
    uint8_t done;
    Navi_Yaw_Calibration_Context_t context;
} NaviYawCalibrationState_t;

static NaviYawCalibrationState_t yaw_calibration;
static Course3BridgeOdometry_t course3_bridge_odometry;
static uint8_t course3_bridge_odometry_active = 0U;
static Course3TravelMeter_t course3_calibration_meter;
static uint8_t course3_calibration_meter_active = 0U;

// 在文件上方的全局变量区添加：
#if USE_WIFI_TUNE
    // 删除了废弃的 nav_q_x 和 nav_q_y，加入了 w 相关的调参变量
      float nav_q_v = 0.1f;
      float nav_q_w = 0.1f;
      float nav_q_bias_ax = 0.0001f;
      float nav_q_bias_w = 0.0001f;

      float nav_r_v_normal = 0.1f;
      float nav_r_v_slip = 10.0f;
      float nav_r_w_normal = 0.1f;
      float nav_r_w_slip = 10.0f;
      float nav_r_gyro = 0.01f;
#endif
    
#if NAVI_USE_LOCAL_FRAME
static float initial_yaw_offset = 0.0f;  // 记录开机/重置时的绝对航向角作为零点偏置
#endif

static float navi_control_yaw_to_navigation_yaw(float control_yaw_deg)
{
#if NAVI_USE_LOCAL_FRAME
    return navi_limit_angle180(-(navi_limit_angle180(control_yaw_deg - initial_yaw_offset)));
#else
    return navi_limit_angle180(-control_yaw_deg);
#endif
}


//====================================================静态函数=================================================

static void Navi_Slip_Detection(void);                //打滑检测函数，在位姿更新前调用

static float Navi_Get_Forward_Mps(void);

static float Navi_Get_YawRate_Enc(void);




//====================================================函数声明=============================================

void Navi_Course3_Bridge_Odometry_Begin(float control_yaw_deg,
                                        float start_x, float start_y,
                                        float end_x, float end_y)
{
    Course3BridgeOdometry_Begin(&course3_bridge_odometry,
                                navi_control_yaw_to_navigation_yaw(control_yaw_deg),
                                start_x, start_y, end_x, end_y);
    course3_bridge_odometry_active = 1U;
}

void Navi_Course3_Bridge_Odometry_End(void)
{
    course3_bridge_odometry_active = 0U;
}

uint8_t Navi_Course3_Bridge_Odometry_Is_Complete(void)
{
    return (course3_bridge_odometry_active && course3_bridge_odometry.completed) ? 1U : 0U;
}
float Navi_Course3_Bridge_Odometry_Get_Travelled(void)
{
    return course3_bridge_odometry.travelled_distance_m;
}

float Navi_Course3_Bridge_Odometry_Get_Target(void)
{
    return course3_bridge_odometry.target_distance_m;
}

void Navi_Course3_Calibration_Meter_Begin(float target_distance_m)
{
    Course3TravelMeter_Begin(&course3_calibration_meter, target_distance_m);
    course3_calibration_meter_active = 1U;
}

void Navi_Course3_Calibration_Meter_End(void)
{
    course3_calibration_meter_active = 0U;
}

uint8_t Navi_Course3_Calibration_Meter_Is_Complete(void)
{
    return (course3_calibration_meter_active && course3_calibration_meter.completed) ? 1U : 0U;
}

float Navi_Course3_Calibration_Meter_Get_Travelled(void)
{
    return course3_calibration_meter.travelled_distance_m;
}

float Navi_Course3_Calibration_Meter_Get_Target(void)
{
    return course3_calibration_meter.target_distance_m;
}

//-------------------------------------------------------------------------------------------------------------------

// 函数简介  扣除重力分量，提取车体真实的运动加速度 (净加速度)

// 参数说明  p_ax, p_ay, p_az：用于返回计算好的净加速度指针 (单位: m/s^2)

// 核心原理  利用旋转矩阵，将重力向量 [0, 0, 1g] 投影到当前倾斜的车体坐标系中，然后从测量值中减去。

//！！！请务必确保 IMU_data.accel 物理轴向与姿态角定义的轴向完全一致，否则倾斜时的重力分量扣不干净，会直接产生一个假加速度导致小车“自动加速”。

//-------------------------------------------------------------------------------------------------------------------

void Navi_Remove_Gravity(float *p_ax, float *p_ay, float *p_az) {

    // 1. 获取当前姿态的弧度值 (完全信任姿态卡尔曼滤波的结果)

    float pitch_rad = ANGLE_TO_RAD(raw_data.pitch);

    float roll_rad  = ANGLE_TO_RAD(raw_data.roll);



    //计算重力向量在车体坐标系下的三轴分量 (单位: g)

    // 【方向假设】：假设小车水平静止时，加速度计 Z 轴读数为正 (约 +1.0g)。

    // 当车头抬起 (Pitch > 0) 时，重力向车尾压，X轴传感器会读到正值。

    float g_comp_x = sinf(pitch_rad);

    float g_comp_y = sinf(roll_rad) * cosf(pitch_rad);

    float g_comp_z = cosf(roll_rad) * cosf(pitch_rad);



    *p_ax = (raw_data.accel[0] - g_comp_x) * GRAVITY;

    *p_ay = (raw_data.accel[1] - g_comp_y) * GRAVITY;

    *p_az = (raw_data.accel[2] - g_comp_z) * GRAVITY;

}





//-------------------------------------------------------------------------------------------------------------------

//函数简介          初始化

//-------------------------------------------------------------------------------------------------------------

void navi_data_init(void) {          

    memset(&yaw_calibration, 0, sizeof(yaw_calibration));

    navi_ekf_config();                                       //EKF滤波函数参数配置

    low_pass_filter_init(&lpf_v_left,0.8);

    low_pass_filter_init(&lpf_v_right,0.8);

    low_pass_filter_init(&lpf_ax,0.5);   
    
    low_pass_filter_init(&lpf_w, 0.2);

}

void Navi_Yaw_Calibration_Start(Navi_Yaw_Calibration_Context_t context)
{
    if (context == NAVI_YAW_CAL_CONTEXT_NONE)
    {
        return;
    }

    memset(&yaw_calibration, 0, sizeof(yaw_calibration));
    yaw_calibration.context = context;
    yaw_calibration.active = 1U;
}

void Navi_Yaw_Calibration_Tick(void)
{
    const uint16_t target_samples =
        (uint16_t)(NAVI_YAW_CALIBRATION_DURATION_MS / NAVI_YAW_CALIBRATION_TICK_MS);
    float yaw_rad;

    if (!yaw_calibration.active)
    {
        return;
    }

    yaw_rad = ANGLE_TO_RAD(IMU_data.filter_result.yaw);
    yaw_calibration.sum_sin += sinf(yaw_rad);
    yaw_calibration.sum_cos += cosf(yaw_rad);
    yaw_calibration.sample_count++;

    if (yaw_calibration.sample_count >= target_samples)
    {
#if NAVI_USE_LOCAL_FRAME
        initial_yaw_offset = RAD_TO_ANGLE(atan2f(yaw_calibration.sum_sin,
                                                yaw_calibration.sum_cos));
#endif
        yaw_calibration.active = 0U;
        yaw_calibration.done = 1U;
    }
}

void Navi_Yaw_Calibration_Cancel(void)
{
    memset(&yaw_calibration, 0, sizeof(yaw_calibration));
}

uint8_t Navi_Yaw_Calibration_Is_Active(void)
{
    return yaw_calibration.active;
}

uint8_t Navi_Yaw_Calibration_Consume_Done(Navi_Yaw_Calibration_Context_t context)
{
    if (!yaw_calibration.done || yaw_calibration.context != context)
    {
        return 0U;
    }

    yaw_calibration.done = 0U;
    yaw_calibration.context = NAVI_YAW_CAL_CONTEXT_NONE;
    return 1U;
}

uint16_t Navi_Yaw_Calibration_Get_Remaining_Ms(void)
{
    const uint16_t target_samples =
        (uint16_t)(NAVI_YAW_CALIBRATION_DURATION_MS / NAVI_YAW_CALIBRATION_TICK_MS);

    if (!yaw_calibration.active || yaw_calibration.sample_count >= target_samples)
    {
        return 0U;
    }

    return (uint16_t)((target_samples - yaw_calibration.sample_count) *
                      NAVI_YAW_CALIBRATION_TICK_MS);
}

Navi_Yaw_Calibration_Context_t Navi_Yaw_Calibration_Get_Context(void)
{
    return yaw_calibration.context;
}



//-------------------------------------------------------------------------------------------------------------------

// 函数简介     原始数据解析函数        +   基础过滤

//-------------------------------------------------------------------------------------------------------------           

void navi_parse_data(void) {
  
#if NAVI_USE_LOCAL_FRAME  
  // 扣除初始偏移量，计算相对航向角     ,并归一化
    float relative_yaw = navi_limit_angle180(IMU_data.filter_result.yaw - initial_yaw_offset);
    
    raw_data.yaw   = -relative_yaw;   // 极性翻转：使顺时针为增加
#else
    raw_data.yaw   = -IMU_data.filter_result.yaw;   // 极性翻转：使顺时针为增加 (绝对地理北极)
#endif
    
    raw_data.pitch = -IMU_data.filter_result.roll;  // 轴映射+极性翻转：队友roll映射为导航pitch，且抬头为正
    
    raw_data.roll  = -IMU_data.filter_result.pitch; // 轴映射+极性翻转：队友pitch映射为导航roll，且右倾为正oll;

    // 角速度也需要对应翻转，确保 EKF 预测模型一致  角速度全部转为 弧度/s
    raw_data.unbiased_gyro[0] = ANGLE_TO_RAD(-IMU_data.gyro[1]);  
    raw_data.unbiased_gyro[1] = ANGLE_TO_RAD(-IMU_data.gyro[0]);  
    raw_data.unbiased_gyro[2] = ANGLE_TO_RAD(-IMU_data.gyro[2]);
    
    // 加速度计轴向映射 (假设加速度轴与陀螺仪轴一致)
    raw_data.accel[0] = -IMU_data.accel[1]; // 映射到车体前进方向 (X)
    
    raw_data.accel[1] = -IMU_data.accel[0]; // 映射到车体右侧方向 (Y)
    
    raw_data.accel[2] =  IMU_data.accel[2]; // 垂直方向 (Z)

    
    raw_data.left_rpm = (int16_t)(-motor_value.receive_left_speed_data);

    raw_data.right_rmp = (int16_t)(-motor_value.receive_right_speed_data);

    raw_data.left_mps = RPM_TO_M_COEFF((float)raw_data.left_rpm);

    raw_data.right_mps = RPM_TO_M_COEFF((float)raw_data.right_rmp);  

    

   // --- A. 速度处理：低通滤波 ---  

    filter_data.left_mps  = low_pass_filter_update(&lpf_v_left, raw_data.left_mps);

    filter_data.right_mps = low_pass_filter_update(&lpf_v_right, raw_data.right_mps);

    

    Navi_Remove_Gravity(&filter_data.accel[0], &filter_data.accel[1], &filter_data.accel[2]);

    filter_data.accel[0] = low_pass_filter_update(&lpf_ax, filter_data.accel[0]);

    

    filter_data.yaw   = raw_data.yaw;

    filter_data.pitch = raw_data.pitch;

    filter_data.roll  = raw_data.roll;

    memcpy(filter_data.unbiased_gyro,raw_data.unbiased_gyro,sizeof(raw_data.unbiased_gyro));    
    
    filter_data.unbiased_gyro[2] = low_pass_filter_update(&lpf_w, raw_data.unbiased_gyro[2]);      
    
    
    // 【新增】：将当前的绝对 Yaw 转换为弧度，并存入向量滑动窗口
    float raw_yaw_rad = ANGLE_TO_RAD(IMU_data.filter_result.yaw);
    yaw_history_sin[yaw_hist_idx] = sinf(raw_yaw_rad);
    yaw_history_cos[yaw_hist_idx] = cosf(raw_yaw_rad);
    
    yaw_hist_idx++;
    if(yaw_hist_idx >= YAW_HISTORY_LEN) {
        yaw_hist_idx = 0;
        yaw_hist_filled = 1; // 标记数组已被写满一轮
    }

}



/**

 * @brief 设置当前位置为坐标原点 (x=0, y=0)

 * @details 独立于导航点记录，用于清空 EKF 累积误差或匹配赛道起始点

 */

/**
 * @brief 导航数据重置函数
 * @param reset_yaw: 1 - 重新捕获当前车头作为0度基准(标定);                    0 - 仅重置坐标和累积误差，保持原航向基准
 */
void Navi_Data_Set_Origin(uint8_t reset_yaw){
    // 1. 基础状态重置 (你提到的那几个变量)
    robot_pose.x = 0;
    robot_pose.y = 0;
    robot_pose.v = 0.0f;
    robot_pose.w = 0.0f;
    robot_pose.radius = 999.0f;
    robot_pose.bias_ax = 0.0f;
    robot_pose.bias_w = 0.0f;
    robot_pose.slip_level = 0;
    robot_pose.manual_update_mode = 0;

    // 2. 航向基准处理
#if NAVI_USE_LOCAL_FRAME
    if (reset_yaw) {
        // 场景 A: 重新标定，从滑动窗口获取新的零点偏置
        if (yaw_hist_filled) {
            float sum_sin = 0.0f, sum_cos = 0.0f;
            for(uint16_t i = 0; i < YAW_HISTORY_LEN; i++) {
                sum_sin += yaw_history_sin[i];
                sum_cos += yaw_history_cos[i];
            }
            initial_yaw_offset = RAD_TO_ANGLE(atan2f(sum_sin, sum_cos));
        } else {
            initial_yaw_offset = IMU_data.filter_result.yaw;
        }
    }
    // 如果 reset_yaw == 0，保持原有的 initial_yaw_offset 不动
#endif

    // 3. 统一更新当前航向角
#if NAVI_USE_LOCAL_FRAME
    float relative_yaw = navi_limit_angle180(IMU_data.filter_result.yaw - initial_yaw_offset);
    robot_pose.yaw = navi_limit_angle180(-relative_yaw);
#else
    robot_pose.yaw = navi_limit_angle180(-IMU_data.filter_result.yaw);
#endif

    // 4. 累计角度和圈数重置
    robot_pose.cumulative_yaw = 0.0;
    robot_pose.turns = 0.0f;
    robot_pose.last_yaw_for_cum = robot_pose.yaw;
    
    robot_pose.is_valid = 1;
}




//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@2

//-------------------------------------------------------------------------------------------------------------------

//导航 EKF 参数配置
                           
// 状态向量 X: [0:x, 1:y, 2: 3:v, 4:bias_ax]

// 观测向量 Z: [v(轮速计算)]

//-------------------------------------------------------------------------------------------------------------

void navi_ekf_config(void) {

    // 核心修改：4阶状态，1阶观测(仅线速度)，1阶控制(仅净加速度)

    kalman_filter_init(&nav_ekf, 4, 3, 1);

    nav_ekf.Abtastzeit_s = ENCODER_DT;

// 1. Q 矩阵 (4x4)
    mat_zero(&nav_ekf.Q);
    mat_set(&nav_ekf.Q, 0, 0, NAV_Q_V);
    mat_set(&nav_ekf.Q, 1, 1, NAV_Q_W);
    mat_set(&nav_ekf.Q, 2, 2, NAV_Q_BIAS_AX);
    mat_set(&nav_ekf.Q, 3, 3, NAV_Q_BIAS_W);

    // 2. R 矩阵 (3x3)
    mat_zero(&nav_ekf.R);
    mat_set(&nav_ekf.R, 0, 0, NAV_R_V_NORMAL); 
    mat_set(&nav_ekf.R, 1, 1, NAV_R_W_NORMAL);
    mat_set(&nav_ekf.R, 2, 2, NAV_R_GYRO);



    // 3. H 矩阵 (3x4) 对应 [v, w, bias_ax, bias_w]
    mat_zero(&nav_ekf.H);
    mat_set(&nav_ekf.H, 0, 0, 1.0f); // 观测1: v_enc = v
    mat_set(&nav_ekf.H, 1, 1, 1.0f); // 观测2: w_enc = w
    mat_set(&nav_ekf.H, 2, 1, 1.0f); // 观测3: gyro  = w + bias_w
    mat_set(&nav_ekf.H, 2, 3, 1.0f);
    
    // 4. 初始化
    mat_eye(&nav_ekf.P);
    mat_zero(&nav_ekf.X);
    mat_set(&nav_ekf.X, 0, 0, 0.0f); // 初始化速度(m/s)

}



//-------------------------------------------------------------------------------------------------------------------

// 函数简介  实时更新F/B矩阵                                         

// 参数说明  // 传入参数规范：a_pure(m/s^2),去重力的加速度， yaw_cur(度)

// 返回参数  无

//-------------------------------------------------------------------------------------------------------------------

// 传入参数规范：a_pure(m/s^2), yaw_cur(度)

void navi_update_F_B_U(float a_pure_mps2) {

    float dt = nav_ekf.Abtastzeit_s;

    // 1. 更新 F 矩阵 (4x4)    

    mat_eye(&nav_ekf.F); // 对角线置 1
    mat_set(&nav_ekf.F, 0, 2, -dt); // v_k = v_{k-1} - bias_ax * dt

    // 2. 更新 B 矩阵 (4x1)
    mat_zero(&nav_ekf.B);
    mat_set(&nav_ekf.B, 0, 0, dt);   // v_k += a_x * dt

    // 3. 更新 U 矩阵 (1x1)
    mat_set(&nav_ekf.U, 0, 0, a_pure_mps2);

}



//-------------------------------------------------------------------------------------------------------------------

// 核心 EKF 更新 (Navigation Core)    

//车辆线速度                 来源于驱动板速度反馈

//-------------------------------------------------------------------------------------------------------------------

void navi_ekf_update(void) {

    navi_parse_data();
    
    Navi_Slip_Detection();
        
    uint8_t airborne_flag = navi_airborne_detection() || action_fsm.is_airborne_expect;                 //获取control里面的腾空标志  !!!后一个标志位可用于设置腾空时的空中加速度计来预测位置的准确性
//    uint8_t airborne_flag = 0 ;                                                                                                                 //只要小车处于动作链的 FSM_JUMP_AIRBORNE 状态，不管你用手举得多平稳，系统都会强制放行加速度输入，保证空中跳跃推算的绝对触发。

    // 观测值 Z (编码器速度)
    float v_obs_mps = Navi_Get_Forward_Mps();
    
    // 编码器计算角速度 (假设顺时针为正：左轮快则向右转)
    float w_obs_enc = Navi_Get_YawRate_Enc();
    
    // 取出完全原始的 gyro_z，不要人工减 0.1，让卡尔曼去算它的真实零偏！
    float gyro_z_obs = raw_data.unbiased_gyro[2]; 
    float a_input = filter_data.accel[0];
    
    // ======== 2. 动态调整 R 矩阵与 ZUPT ========
    float current_R_v = NAV_R_V_NORMAL; 
    float current_R_w = NAV_R_W_NORMAL;
    
    if (robot_pose.slip_level == 1) {
        current_R_v = NAV_R_V_SLIP; 
        current_R_w = NAV_R_W_SLIP; 
    }

    // 零速约束(ZUPT)：静止时覆盖一切
    if (fabsf(v_obs_mps) < 0.01f && fabsf(w_obs_enc) < 0.05f && !airborne_flag) {
        a_input = 0.0f;  
        current_R_v = 0.0001f; 
        current_R_w = 0.0001f; 
        w_obs_enc = 0.0f;     // 静止时强制编码器角速度为0
    }
    
    mat_set(&nav_ekf.R, 0, 0, current_R_v);
    mat_set(&nav_ekf.R, 1, 1, current_R_w);
    mat_set(&nav_ekf.R, 2, 2, NAV_R_GYRO);
// ======== WiFi 动态实时更新 Q 矩阵 ========
#if USE_WIFI_TUNE
    mat_set(&nav_ekf.Q, 0, 0, NAV_Q_V);           // 线速度预测噪声
    mat_set(&nav_ekf.Q, 1, 1, NAV_Q_W);           // 角速度预测噪声
    mat_set(&nav_ekf.Q, 2, 2, NAV_Q_BIAS_AX);     // 加速度零偏预测噪声
    mat_set(&nav_ekf.Q, 3, 3, NAV_Q_BIAS_W);      // 陀螺仪零偏预测噪声
#endif  
    
   // 3.执行预测
// ==============================================================================
// 腾空状态下的导航“断流推算”策略选择
// 场景背景：起跳腾空时，车轮空转或停转，编码器数据完全失效（必须切断观测）。
// ==============================================================================
    if (airborne_flag) {     
        // 空中水平加速度为0   ，以起跳瞬间速度作为水平加速度      
        navi_update_F_B_U(0.0f); 
      
        kalman_filter_predict(&nav_ekf);
      
        // 【必须修改】：空中闭环防漂移，强制将 3 个观测值全部等于卡尔曼预测值，让残差为0
        v_obs_mps  = mat_get(&nav_ekf.X, 0, 0);  // v 是索引 0
        w_obs_enc  = mat_get(&nav_ekf.X, 1, 0);  // w 是索引 1
        // 陀螺仪的预测观测值 = 真实的角速度 w + 陀螺仪零偏 bias_w
        gyro_z_obs = mat_get(&nav_ekf.X, 1, 0) + mat_get(&nav_ekf.X, 3, 0);

    } else {
        navi_update_F_B_U(a_input);
        
        kalman_filter_predict(&nav_ekf);
    }
    
    


    


    // ======== 4. 执行更新 ========
    float obs_z[3] = { v_obs_mps, w_obs_enc, gyro_z_obs };
    kalman_filter_update(&nav_ekf, obs_z);
    
    // ======== 5. 外部捷联推算航迹 (Dead Reckoning) ========
    float opt_v = mat_get(&nav_ekf.X, 0, 0);
    float opt_w = mat_get(&nav_ekf.X, 1, 0);
    float dt = ENCODER_DT;
    float yaw_rad = ANGLE_TO_RAD(filter_data.yaw);
    

    // ====================================================================
    // 混合圆弧模型
    // ====================================================================
    float dx = 0.0f, dy = 0.0f;
    float dtheta = opt_w * dt; // dt时间内总转角
    robot_pose.radius = 999.0f;

    // Use midpoint integration for low yaw rates to avoid arc-model cancellation.
    if (fabsf(opt_w) < NAVI_ARC_MIN_YAWRATE_RADPS) {
        // [情形 1] 直行或微小抖动时：退化为二阶中点积分（防止除零溢出）
        float half_dtheta = dtheta / 2.0f;
        dx = opt_v * cosf(yaw_rad + half_dtheta) * dt;
        dy = opt_v * sinf(yaw_rad + half_dtheta) * dt;
    } else {
        // [情形 2] 中高速弯道运动：使用精确解析圆弧积分
        // 半径 R = v / w
        float radius = opt_v / opt_w; 
        robot_pose.radius = radius;
        
        // 解析积分结果
        dx = radius * (sinf(yaw_rad + dtheta) - sinf(yaw_rad));
        // 注意 Y 轴的积分结果符号是反过来的： -R * (cos(new) - cos(old)) 也就是 R * (cos(old) - cos(new))
        dy = radius * (cosf(yaw_rad) - cosf(yaw_rad + dtheta)); 
    }

    // 将增量累加进全局坐标系
    if (robot_pose.manual_update_mode == 0) {         //自动时才更新
        if (course3_calibration_meter_active)
        {
            Course3TravelMeter_Update(&course3_calibration_meter,
                                      filter_data.left_mps,
                                      filter_data.right_mps,
                                      ENCODER_DT);
        }
        if (course3_bridge_odometry_active)
        {
            Course3BridgeOdometry_Update(&course3_bridge_odometry,
                                         filter_data.left_mps,
                                         filter_data.right_mps,
                                         ENCODER_DT,
                                         &dx, &dy);
        }
        robot_pose.x += dx;
        robot_pose.y += dy;
    }

//    float half_dtheta = (opt_w * dt) / 2.0f; // opt_w 是 EKF 融合后的角速度 (弧度制)
//    robot_pose.x += opt_v * cosf(yaw_rad + half_dtheta) * dt;
//    robot_pose.y += opt_v * sinf(yaw_rad + half_dtheta) * dt;
    
//    // 纯运动学积分，完美顺滑
//    robot_pose.x += opt_v * cosf(yaw_rad) * dt;
//    robot_pose.y += opt_v * sinf(yaw_rad) * dt;

    // ======== 6. 结果写回全局状态 ========
    robot_pose.v       = opt_v;
    robot_pose.w       = opt_w;    // 此时的 opt_w 融合了 IMU 和编码器
    robot_pose.bias_ax = mat_get(&nav_ekf.X, 2, 0);
    robot_pose.bias_w  = mat_get(&nav_ekf.X, 3, 0); // 可以通过串口打印这个值，看看它有多准
    
    robot_pose.yaw     = navi_limit_angle180(filter_data.yaw);
    
    robot_pose.cumulative_yaw += navi_limit_angle180(robot_pose.yaw - robot_pose.last_yaw_for_cum);
    robot_pose.turns = (float)(robot_pose.cumulative_yaw / 360.0);
    robot_pose.last_yaw_for_cum = robot_pose.yaw;
    // ----------------------------

    robot_pose.is_valid = 1;
    
//    IPC_LOG_Printf("%f,%f,%f,%f ,%f,%f,%f\n",robot_pose.x,robot_pose.y,robot_pose.yaw,robot_pose.v,robot_pose.w,robot_pose.bias_ax,robot_pose.bias_w); 
//    printf("%f,%f,%f,%f ,%f,%f,%f\n",robot_pose.x,robot_pose.y,robot_pose.yaw,robot_pose.v,robot_pose.w,robot_pose.bias_ax,robot_pose.bias_w); 


}



//-------------------------------------------------------------------------------------------------------------------

// 打滑检测逻辑 (Slip Detection)

//打滑后给robot.is_slip置  1

//-------------------------------------------------------------------------------------------------------------------

static float Navi_Get_Forward_Mps(void)
{
    // 右轮速度在当前驱动反馈符号下与左轮相反，沿用 control.c 的线速度符号约定。
    return (filter_data.left_mps - filter_data.right_mps) * 0.5f;
}

static float Navi_Get_YawRate_Enc(void)
{
    // 右轮速度在直行时为反号，因此差速角速度使用 left + right。
    return -(filter_data.left_mps + filter_data.right_mps) / WHEEL_DISRANCE;
}
static void Navi_Slip_Detection(void) {

    float dt = ENCODER_DT;

    static float last_v_mps = 0;


    // 计算编码器加速度

    float current_v_mps = Navi_Get_Forward_Mps();

    float enc_accel = (current_v_mps - last_v_mps) / dt;

    last_v_mps = current_v_mps;



    // 2. 打滑判定（增加静态阈值防抖）

    static uint8 slip_cnt = 0;

    if (fabsf(enc_accel - filter_data.accel[0]) > SLIP_THRESHOLD)   {

     slip_cnt++;

     if (slip_cnt >= 3) {

        robot_pose.slip_level = 1; // 连续3次触发才判定打滑

        // --- 新增：给控制层的反馈 ---

//        motor_speed.integrator = 0;    // 清除速度积分，防止打滑带来的积分饱和

        }

    }

    else  {

      slip_cnt = 0;

      robot_pose.slip_level = 0;

    }

}


float navi_limit_angle180(float angle) {
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle - 180.0f;
}

// =====================================================================
// [功能] 高鲁棒性轮腿腾空检测 (严格遵循国赛通用抗干扰标准)
// [返回] 1: 当前确认腾空 ; 0: 当前确认在地面
// =====================================================================
uint8_t navi_airborne_detection(void) {
    // 4. 状态锁存：静态变量维持状态机，避免被局部变量重置
    static uint8_t is_latching_airborne = 0; 
    
    // 滤波器静态变量
    static float lpf_acc_z = 1.0f;     // 静止时默认 Z 轴为 1g
    static float lpf_gyro_norm = 0.0f; // 角速度波动幅值
    
    // 3. 连续计数器：独立维护上升沿和下降沿的滤波防抖
    static uint8_t airborne_confirm_cnt = 0;
    static uint8_t ground_confirm_cnt = 0;

    // --- 参数配置区 (可根据实车 telemetry 微调) ---
    const float ALPHA_A = 0.3f;        // 2. 一阶低通滤波系数 (加速度)，0.3具有较好平滑度
    const float ALPHA_W = 0.2f;        // 2. 一阶低通滤波系数 (角速度)
    
    const float FREEFALL_MAX = 0.4f;   // 腾空判定上限：<0.4g 视为失重状态 (真实自由落体为0)
    const float FREEFALL_MIN = -0.4f;  // 腾空判定下限：允许一定传感器过冲
    
    const float GYRO_SHOCK_MAX = 6.1f; // 1. 双判断上限：撞击地面瞬间角速度通常>500度/s，腾空飞行时相对平稳
    
    const float LANDING_GRAVITY_MIN = 0.8f; // 落地判定：需恢复至接近 1g 正常重力
    const float LANDING_GRAVITY_MAX = 1.2f;

    // --- 数据获取与滤波 ---
    // 5. 轻量高效：直接使用绝对值相加代替 sqrt 求模，极大地节省 MCU 算力
    float cur_acc_z = raw_data.accel[2]; // Z轴加速度(单位:g)
    float cur_gyro_p = raw_data.unbiased_gyro[0]; 
    float cur_gyro_r = raw_data.unbiased_gyro[1];
    
    // 执行一阶低通滤波，滤除电机高频震荡和履带/轮胎碎震
    lpf_acc_z = ALPHA_A * cur_acc_z + (1.0f - ALPHA_A) * lpf_acc_z;
    
    float gyro_sum = fabsf(cur_gyro_p) + fabsf(cur_gyro_r);
    lpf_gyro_norm = ALPHA_W * gyro_sum + (1.0f - ALPHA_W) * lpf_gyro_norm;

    // --- 逻辑判断与状态机 ---
    // 判定条件1：Z轴进入失重区间
    uint8_t cond_is_freefall = (lpf_acc_z < FREEFALL_MAX && lpf_acc_z > FREEFALL_MIN);
    // 判定条件2：角速度未出现极其剧烈的撞击尖峰
    uint8_t cond_gyro_stable = (lpf_gyro_norm < GYRO_SHOCK_MAX);
    // 判定条件3：恢复正常静止重力区间
    uint8_t cond_is_grounded = (lpf_acc_z > LANDING_GRAVITY_MIN && lpf_acc_z < LANDING_GRAVITY_MAX);

    if (is_latching_airborne == 0) {
        // 【当前在地面，侦测起跳/跌落】
        if (cond_is_freefall && cond_gyro_stable) {
            airborne_confirm_cnt++;
            ground_confirm_cnt = 0; // 互锁清零
            
            // 连续确认 (5个周期 = 50ms@10ms控制周期)，防止单点跳变误判
            if (airborne_confirm_cnt >= 5) {
                is_latching_airborne = 1;
                airborne_confirm_cnt = 0;
            }
        } else {
            airborne_confirm_cnt = 0; // 遇到哪怕1帧的不满足要求，立刻清零重新计数
        }
    } else {
        // 【当前在空中，侦测落地】
        // 落地瞬间通常伴随剧烈的 acc_z 超调(>2g)，所以不能立刻判定，必须等重力恢复平稳区
        if (cond_is_grounded) {
            ground_confirm_cnt++;
            airborne_confirm_cnt = 0;
            
            // 落地防抖通常需要更长的时间 (10个周期 = 100ms)，等避震器和腿部机构完全卸力收敛
            if (ground_confirm_cnt >= 10) {
                is_latching_airborne = 0;
                ground_confirm_cnt = 0;
            }
        } else {
            ground_confirm_cnt = 0;
        }
    }

    return is_latching_airborne;
}


//===================================================================
// 位姿更新模式
//0:自动更新   1:手动模式
//===================================================================

/**位姿更新模式
 * @param enable 1:开启手动模式(坐标停止自动更新)，0:关闭并恢复自动更新
 */
void Navi_Set_Manual_Update_Mode(uint8_t enable) {
    robot_pose.manual_update_mode = enable;
}

/**
 * @brief 手动给当前位姿坐标添加一个固定值
 * @param val1  、val2 ：偏移量1、2
 * @param frame 偏移量参考坐标系选择:  0(全局) 或 1：当前车体的坐标系
 */
void Navi_Manual_Add_Pose(float val1, float val2, uint8_t frame) {
    // 如果没有开启手动模式，直接返回，不执行补偿
    if (!robot_pose.manual_update_mode) {
        return; 
    }

    if (frame == 1) {
        // --- 车体相对坐标系模式 (方案二) ---
        // val1: forward_m, val2: right_m
        float yaw_rad = ANGLE_TO_RAD(robot_pose.yaw);
        
        // 转换到全局 dx, dy
        double dx_global = val1 * cosf(yaw_rad) - val2 * sinf(yaw_rad);
        double dy_global = val1 * sinf(yaw_rad) + val2 * cosf(yaw_rad);
        
        robot_pose.x += dx_global;
        robot_pose.y += dy_global;
    } 
    else if (frame == 0) {
        // --- 全局坐标系模式 (方案一) ---
        // val1: dx_m, val2: dy_m
        robot_pose.x += val1;
        robot_pose.y += val2;
    }
}

