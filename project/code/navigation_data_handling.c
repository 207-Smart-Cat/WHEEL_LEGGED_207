/*********************************************************************************************************************

*  问题1:零点对齐：当车头指向正北（世界坐标系 X 轴）时，IMU 输出的 Yaw 是否为 $0$？            问题二：方向极性：当机器人向右（东）转动时，IMU 输出的 Yaw 值是否在增加？

*              

*                

*********************************************************************************************************************/

#include "navigation_data_handling.h"

#include "navigation_tracking.h"

#include "navigation_action.h"



//====================================================变量声明=======================================================

extern IMU_t IMU_data;                         //IMU数据结构体

extern Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // 预设的“路”



// 引用control.c 中的控制接口

extern float now_velocity;       //当前速度(线速度)

extern float target_velocity;    // 目标速度 (线速度)

extern float target_motor_angle; // 目标角度 (用于转向)

extern float target_engine_high; // 目标腿部高度

extern float Turn_Pwm; // 转向PWM值

extern float Encoder_Left;

extern float Encoder_Right;


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


//====================================================静态函数=================================================
static void Navi_Slip_Detection(void);                //打滑检测函数，在位姿更新前调用




//====================================================函数声明=============================================



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

    navi_ekf_config();                                       //EKF滤波函数参数配置

    low_pass_filter_init(&lpf_v_left,0.8);

    low_pass_filter_init(&lpf_v_right,0.8);

    low_pass_filter_init(&lpf_ax,0.5);   
    
    low_pass_filter_init(&lpf_w, 0.2);

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

    // 角速度也需要对应翻转，确保 EKF 预测模型一致
    raw_data.unbiased_gyro[0] = -IMU_data.gyro[1];  // 对应 pitch 角速度
    
    raw_data.unbiased_gyro[1] = -IMU_data.gyro[0];  // 对应 roll 角速度
    
    raw_data.unbiased_gyro[2] = -IMU_data.gyro[2];  // 对应 yaw 角速度
    
    // 加速度计轴向映射 (假设加速度轴与陀螺仪轴一致)
    raw_data.accel[0] = -IMU_data.accel[1]; // 映射到车体前进方向 (X)
    
    raw_data.accel[1] = -IMU_data.accel[0]; // 映射到车体右侧方向 (Y)
    
    raw_data.accel[2] =  IMU_data.accel[2]; // 垂直方向 (Z)

    
    raw_data.left_rpm = (int16_t)Encoder_Left;

    raw_data.right_rmp = (int16_t)Encoder_Right;

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

}



/**

 * @brief 设置当前位置为坐标原点 (x=0, y=0)

 * @details 独立于导航点记录，用于清空 EKF 累积误差或匹配赛道起始点

 */

void Navi_Data_Set_Origin(void) {
    robot_pose.x = 0;
    
    robot_pose.y = 0;
    
    // 【新增】：锁定当前车头朝向，作为相对坐标系的 0 度参考
#if NAVI_USE_LOCAL_FRAME
    initial_yaw_offset = IMU_data.filter_result.yaw;
#endif

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
    mat_set(&nav_ekf.X, 0, 0, RPM_TO_M_COEFF(now_velocity)); // 初始化速度(m/s)

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

//车辆线速度                 来源于control.h

//-------------------------------------------------------------------------------------------------------------------

void navi_ekf_update(void) {

    navi_parse_data();
    
    Navi_Slip_Detection();
    
    action_fsm.is_airborne_expect=0;
    
    uint8_t airborne_flag = (uint8_t)is_airborne() || action_fsm.is_airborne_expect;                 //获取control里面的腾空标志  !!!后一个标志位可用于设置腾空时的空中加速度计来预测位置的准确性
                                                                                                                     //只要小车处于动作链的 FSM_JUMP_AIRBORNE 状态，不管你用手举得多平稳，系统都会强制放行加速度输入，保证空中跳跃推算的绝对触发。

    // 观测值 Z (编码器速度)
    float v_obs_mps = RPM_TO_M_COEFF(now_velocity); 
    
    // 编码器计算角速度 (假设顺时针为正：左轮快则向右转)
    float w_obs_enc = (filter_data.left_mps - filter_data.right_mps) / WHEEL_DISRANCE;
    
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

    // 纯运动学积分，完美顺滑
    robot_pose.x += opt_v * cosf(yaw_rad) * dt;
    robot_pose.y += opt_v * sinf(yaw_rad) * dt;

    // ======== 6. 结果写回全局状态 ========
    robot_pose.v       = opt_v;
    robot_pose.w       = opt_w;    // 此时的 opt_w 融合了 IMU 和编码器
    robot_pose.bias_ax = mat_get(&nav_ekf.X, 2, 0);
    robot_pose.bias_w  = mat_get(&nav_ekf.X, 3, 0); // 可以通过串口打印这个值，看看它有多准
    
    robot_pose.yaw     = navi_limit_angle180(filter_data.yaw);

    robot_pose.is_valid = 1;

}



//----------------------------------------------------------------------------------------------------------------
// 计算当前位置到目标航点的导航信息   (X为北，Y东,顺时针角度加)

//  target_idx: 目标航点索引

// azimuth: 方位角指针（单位：度）

// distance: 直线距离指针（单位：米）

//----------------------------------------------------------------------------------------------------------------

uint8 navi_calcnavinfo(uint8 target_idx, double *azimuth, double *distance) {

    if(target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid || !robot_pose.is_valid)

       return 0;

    

    double dx = point_map[target_idx].x - robot_pose.x;

    double dy = point_map[target_idx].y - robot_pose.y;

    *distance = sqrt(dx * dx + dy * dy);   

    

    // 边界处理：如果已经在目标点附近，方位角保持当前航向，防止抖动

    if (*distance < 0.01) *azimuth = robot_pose.yaw;

    else {

        double angle = RAD_TO_ANGLE(atan2(dy, dx));

        *azimuth = (angle < 0) ? (angle + 360.0) : angle;

    }

    return 1;

}



//-------------------------------------------------------------------------------------------------------------------

// 判断是否到达目标航点           0 ：   未到达               1：到达

// 基于平面坐标（x/y）计算当前位置与目标航点的距离，判断是否小于设定阈值

// arget_idx: 目标航点索引

//--------------------------------------------------------------------------------------------------------------

uint8 navi_isreach_target_point(uint8 target_idx) {

    if(target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid || !robot_pose.is_valid) {

        return 0;

    }

    double dx = point_map[target_idx].x - robot_pose.x;

    double dy = point_map[target_idx].y - robot_pose.y;

    double distance_sq = dx*dx + dy*dy;
    
    float current_threshold = DISTANCE_THRESHOLD; // 默认 0.2m
    
    switch (point_map[target_idx].type) {
        case WP_TYPE_CONE_CONE:   // 绕锥桶：要求更精准，缩小阈值
            current_threshold = 0.10f; 
            break;
        case WP_TYPE_NORMAL:      // 普通直线：为了高速顺滑，放大阈值提前切点
            current_threshold = 0.40f; 
            break;
        case WP_TYPE_STOP:        // 停车点：要求极高精度
            current_threshold = 0.05f; 
            break;
        default:
            current_threshold = 0.20f;
            break;
    }

    return (distance_sq <= (current_threshold * current_threshold)) ? 1 : 0;

}





//-------------------------------------------------------------------------------------------------------------------

// 打滑检测逻辑 (Slip Detection)

//打滑后给robot.is_slip置  1

//-------------------------------------------------------------------------------------------------------------------

static void Navi_Slip_Detection(void) {

    float dt = ENCODER_DT;

    static float last_v_mps = 0;


    // 计算编码器加速度

    float current_v_mps = RPM_TO_M_COEFF(now_velocity);

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


float navi_limit_angle180(float angle) 
{
    // 第一步：使用 fmodf 将任意大角度快速映射到 (-360, 360) 区间
    // fmodf 专门用于 float 类型，运算速度快且是 O(1) 复杂度
    angle = fmodf(angle, 360.0f);

    // 第二步：将区间进一步压缩到 (-180, 180]
    if (angle > 180.0f) 
    {
        angle -= 360.0f;
    } 
    else if (angle <= -180.0f) 
    {
        angle += 360.0f;
    }

    return angle;
}


