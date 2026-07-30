  /*********************************************************************************************************************

* 本文件主要是处理导航需要用到的姿态数据，位置数据，为导航模块提供数据资源和稳定的坐标，使其能够相信坐标信息进行导航     

* 我的坐标方向：pitch俯仰角，抬头为正； roll横滚角，左低右高为正，yaw航向角，顺时针为正。            x 轴正北，y轴正东               

* 队友欧拉角，roll俯仰，抬头减少；pitch横滚，左高右低减少；yaw顺时针减少               

*********************************************************************************************************************/

#ifndef NAVIGATION_DATA_HANDLING_H_

#define NAVIGATION_DATA_HANDLING_H_



#include "zf_common_typedef.h"

#include "filter_function.h"

#include <math.h>

#include "imu.h"

#include "param.h"



//=================================================定义  基本配置================================================

// ========================== 卡尔曼参数配置 ==========================
// 增加一个总开关：1代表WiFi在线调参模式，0代表固化比赛模式
#define USE_WIFI_TUNE  1

// 【新增】：坐标系基准选择
// 1 : 相对坐标系 (Local Frame) —— 以开机重置点车头朝向为 X 轴正北方向 (推荐调试和盲跑使用)
// 0 : 绝对坐标系 (Global Frame) —— 依赖地磁计，以地球地理正北为 X 轴 (推荐带 GPS 室外使用)
#define NAVI_USE_LOCAL_FRAME  1

//=================数据单位转换===============================

#define GNSS_PI             ( 3.1415926535898 )

#define RPM_TO_M_COEFF(x)       x * ((WHEEL_DIAMETER * GNSS_PI) / 60.0f)

#define ANGLE_TO_RAD(x)     ( (x) * GNSS_PI / 180.0 )                           // 角度转换为弧度

#define RAD_TO_ANGLE(x)     ( (x) * 180.0 / GNSS_PI )                           // 弧度转换为角度

#define GRAVITY             ( 9.80665f )                                        //重力g大小

// =====================算法阈值 ==========================

#define SLIP_THRESHOLD      2.0f      // 打滑判定阈值 (m/s^2)

#define STATIC_V_THRESHOLD  0.01f    // 静止判定速度阈值

#define NAVI_ARC_MIN_YAWRATE_RADPS 0.10f    // Use midpoint integration below this yaw rate (rad/s)

#define LOOK_AHEAD_DIST 0.6f // 前瞻距离：根据车速调整，通常 0.5m-1.0m





// =========================车模数据 ==========================

#define WHEEL_DIAMETER   0.045F             //车轮直径

#define WHEEL_DISRANCE   0.190f              //两轮轴距    


//=========================运行参数===========================

#define ENCODER_DT                    0.005f     // 5ms sampling period

#define YAW_HISTORY_LEN 600  // 600 samples at 5ms keeps about 3000ms of yaw history

#define NAVI_YAW_CALIBRATION_DURATION_MS 2000U
#define NAVI_YAW_CALIBRATION_TICK_MS        5U


#define F_S(f) ((f) < 0 ? "-" : "")
#define F_I(f) (int)((f) < 0 ? -(f) : (f))
#define F_D2(f) (int)((((f) < 0 ? -(f) : (f)) - (int)((f) < 0 ? -(f) : (f))) * 100)
#define F_D5(f) (int)((((f) < 0 ? -(f) : (f)) - (int)((f) < 0 ? -(f) : (f))) * 100000)
#define F_ARG(f) F_S(f), F_I(f), F_D2(f)
//#define FMT "%s%d.%04d"


//===================卡尔曼参数配置=================================
#if USE_WIFI_TUNE
    // 调参模式：声明外部变量，并让宏直接指向变量
    extern float nav_q_v;
    extern float nav_q_w;
    extern float nav_q_bias_ax;
    extern float nav_q_bias_w;

    extern float nav_r_v_normal;
    extern float nav_r_v_slip;
    extern float nav_r_w_normal;
    extern float nav_r_w_slip;
    extern float nav_r_gyro;

    // Q 矩阵：过程噪声映射
    #define NAV_Q_V             nav_q_v
    #define NAV_Q_W             nav_q_w
    #define NAV_Q_BIAS_AX       nav_q_bias_ax
    #define NAV_Q_BIAS_W        nav_q_bias_w

    // R 矩阵：观测噪声映射
    #define NAV_R_V_NORMAL      nav_r_v_normal
    #define NAV_R_V_SLIP        nav_r_v_slip
    #define NAV_R_W_NORMAL      nav_r_w_normal
    #define NAV_R_W_SLIP        nav_r_w_slip
    #define NAV_R_GYRO          nav_r_gyro
    
#else
    // 固化比赛模式：静态宏定义（Q 越小越信任预测/模型，R 越小越信任测量/传感器）
    // Q 矩阵：过程噪声
    #define NAV_Q_V             0.1f         // 线速度预测噪声
    #define NAV_Q_W             0.1f         // 角速度预测噪声
    #define NAV_Q_BIAS_AX       0.0001f      // 加速度零偏游走预测噪声
    #define NAV_Q_BIAS_W        0.0001f      // 陀螺仪零偏游走预测噪声
  
    // R 矩阵：观测噪声
    #define NAV_R_V_NORMAL      0.1f         // 正常行驶时线速度观测噪声
    #define NAV_R_V_SLIP        10.0f        // 打滑时线速度观测噪声(极大值表示不信任编码器)
    #define NAV_R_W_NORMAL      0.1f         // 正常行驶时角速度观测噪声
    #define NAV_R_W_SLIP        10.0f        // 打滑时角速度观测噪声
    #define NAV_R_GYRO          0.01f        // IMU陀螺仪观测噪声(极小值，强制抓取零偏)
#endif
    
 
//=================================================定义  基本配置================================================



//=================================================定义  结构体================================================

// 机器人实时位姿结构体

typedef struct {

    double x;               // 世界坐标 X (m)

    double y;              // 世界坐标 Y (m)

    float  yaw;             // 融合航向角 (-180~180°)

    float  v;               // 线速度 (m/s)

    float  w;               // 角速度 (rad /s)

    float  radius;          // Odometry turn radius (m), 999.0f for midpoint integration
    
    float  bias_ax;         //加速度零偏

    float  bias_w;          // 陀螺仪Z轴零偏 (rad/s)

    uint8_t slip_level;     // 打滑状态 (0:正常, 1:轻微, 2:严重)

    uint8_t is_valid;       // 状态是否可靠
    
    // --- 新增变量用于轴距测试 ---
    double cumulative_yaw;  // 累计真实角度 (不限制范围，度)
    float  turns;           // 累计圈数 (自动计算比例，如 1.05)
    float  last_yaw_for_cum; // 上一帧的 Yaw 值，用于检测跳转
    
    // 【新增】手动坐标更新模式标志位 (1: 开启手动补偿, 暂停自动更新; 0: 自动更新)
    uint8_t manual_update_mode;

} RobotState_t;



//定位传感数据读取

typedef struct {

  //轮速相关

   int16_t left_rpm;                                    //左电机原始转速

   int16_t right_rmp;                                   //右电机原始转速

   float left_mps;                                  //左电机线速度

   float right_mps;                                 //右电机线速度

 

  //imu相关

   float roll;

   float pitch;

   float yaw;

   float unbiased_gyro[3];

   float accel[3];

}Navi_Sensor_Data_t;  

typedef enum {
    NAVI_YAW_CAL_CONTEXT_NONE = 0,
    NAVI_YAW_CAL_CONTEXT_RECORD_HOME = 1,
    NAVI_YAW_CAL_CONTEXT_NAV_START = 2
} Navi_Yaw_Calibration_Context_t;

//=================================================定义  结构体================================================


//全局变量声明
extern RobotState_t robot_pose;                                  // 全局实时位姿，供外部只读访问
extern Navi_Sensor_Data_t filter_data;                        //滤波+处理后的初始数据数据

//=================================================声明  基础函数================================================

//数据预处理 /基础函数
void Navi_Remove_Gravity(float *p_ax, float *p_ay, float *p_az) ;                //重力扣除

void navi_data_init(void) ;                             //初始化

void navi_parse_data(void) ;                              //  原始数据解析函数  +基础过滤 (转速初处理，航向角融合，加速度初处理)    


//卡尔曼算法

void navi_ekf_config(void);                               //卡尔曼参数初始化配置

void navi_update_F_B_U(float a_pure_mps2);

void navi_ekf_update(void) ;                       //卡尔曼算法数据预测+更新+其他信息更新  ,更新当前Navi位置信息      



//外置函数
void Navi_Data_Set_Origin(uint8_t reset_yaw) ;// 独立原点设置 API  置当前位置为坐标原点 (x=0, y=0)

void Navi_Yaw_Calibration_Start(Navi_Yaw_Calibration_Context_t context);
void Navi_Yaw_Calibration_Tick(void);
void Navi_Yaw_Calibration_Cancel(void);
uint8_t Navi_Yaw_Calibration_Is_Active(void);
uint8_t Navi_Yaw_Calibration_Consume_Done(Navi_Yaw_Calibration_Context_t context);
uint16_t Navi_Yaw_Calibration_Get_Remaining_Ms(void);
Navi_Yaw_Calibration_Context_t Navi_Yaw_Calibration_Get_Context(void);

float navi_limit_angle180(float angle);              //转角限幅函数
uint8_t navi_airborne_detection();                                //腾空检测函数。


void Navi_Set_Manual_Update_Mode(uint8_t enable);
void Navi_Manual_Add_Pose(float val1, float val2, uint8_t frame);

void Navi_Course3_Bridge_Odometry_Begin(float control_yaw_deg,
                                        float start_x, float start_y,
                                        float end_x, float end_y);
void Navi_Course3_Bridge_Odometry_End(void);
uint8_t Navi_Course3_Bridge_Odometry_Is_Complete(void);
float Navi_Course3_Bridge_Odometry_Get_Travelled(void);
float Navi_Course3_Bridge_Odometry_Get_Target(void);
void Navi_Course3_Calibration_Meter_Begin(float target_distance_m);
void Navi_Course3_Calibration_Meter_End(void);
uint8_t Navi_Course3_Calibration_Meter_Is_Complete(void);
float Navi_Course3_Calibration_Meter_Get_Travelled(void);
float Navi_Course3_Calibration_Meter_Get_Target(void);

//=================================================声明  基础函数================================================



#endif //NAVIGATION_DATA_HANDLING_H
