/*********************************************************************************************************************

* 本文件主要是处理导航需要用到的姿态数据，位置数据，为导航模块提供数据资源和稳定的坐标，使其能够相信坐标信息进行导航     

* 我的坐标方向：pitch俯仰角，抬头为正； roll横滚角，左低右高为正，yaw航向角，顺时针为正。            x 轴正北，y轴正东               

* 队友欧拉角，roll俯仰，抬头减少；pitch横滚，左高右低减少；yaw顺时针减少               

*********************************************************************************************************************/

#ifndef NAVIGATION_DATA_HANDLING_H_

#define NAVIGATION_DATA_HANDLING_H_



#include "zf_common_typedef.h"

#include "filter_function.h"

#include "kalman_rm.h"

#include <math.h>

#include "control.h"



//=================================================定义  基本配置================================================

// ========================== 卡尔曼参数配置 ==========================
// 增加一个总开关：1代表WiFi在线调参模式，0代表固化比赛模式
#define USE_WIFI_TUNE  1

//===================1.数据单位转换===============================

#define GNSS_PI             ( 3.1415926535898 )

#define RPM_TO_M_COEFF(x)       x * ((WHEEL_DIAMETER * GNSS_PI) / MOTOR_GEAR_RADIO / 60.0f)

#define ANGLE_TO_RAD(x)     ( (x) * GNSS_PI / 180.0 )                           // 角度转换为弧度

#define RAD_TO_ANGLE(x)     ( (x) * 180.0 / GNSS_PI )                           // 弧度转换为角度

#define GRAVITY             ( 9.80665f )                                        //重力g大小



//====================2.卡尔曼参数配置=================================

#if USE_WIFI_TUNE

// 调参模式：声明外部变量，并让宏直接指向变量
    extern float nav_q_x;
    extern float nav_q_y;
    extern float nav_q_v;
    extern float nav_q_bias_ax;
    extern float nav_r_v_normal;
    extern float nav_r_v_slip;

    #define NAV_Q_X             nav_q_x
    #define NAV_Q_Y             nav_q_y
    #define NAV_Q_V             nav_q_v
    #define NAV_Q_BIAS_AX       nav_q_bias_ax
    #define NAV_R_V_NORMAL      nav_r_v_normal
    #define NAV_R_V_SLIP        nav_r_v_slip
    
#else

// Q 矩阵：过程噪声 (越小越信任预测模型，即编码器)

#define NAV_Q_X             0.001f    // X坐标预测噪声

#define NAV_Q_Y             0.001f    // Y坐标预测噪声

#define NAV_Q_V             0.1f      // 线速度预测噪声

#define NAV_Q_BIAS_AX       0.0001f      // 加速度零偏游走预测噪声

  

// R 矩阵：观测噪声 (越小越信任传感器测量值，即 IMU)

#define NAV_R_V_NORMAL      0.1f      // 正常行驶时线速度观测噪声

#define NAV_R_V_SLIP        10.0f     // 检测到打滑时线速度观测噪声(极大值表示不信任编码器)

#endif



// ========================== 3. 算法阈值 ==========================

#define SLIP_THRESHOLD      2.0f      // 打滑判定阈值 (m/s^2)

#define STATIC_V_THRESHOLD  0.005f    // 静止判定速度阈值

#define LOOK_AHEAD_DIST 0.6f // 前瞻距离：根据车速调整，通常 0.5m-1.0m





// ========================== 4.车模数据 ==========================

#define WHEEL_DIAMETER   0.082F             //车轮直径

#define WHEEL_DISRANCE   0.20f              //两轮轴距      

#define MOTOR_GEAR_RADIO 15.0f              //电机减速比 



//===========================5.运行参数===========================

#define ENCODER_DT                    0.01f     // 10ms 采样周期







//=================================================定义  基本配置================================================





//=================================================定义  结构体================================================

// 机器人实时位姿结构体

typedef struct {

    double x;               // 世界坐标 X (m)

    double y;              // 世界坐标 Y (m)

    float  yaw;             // 融合航向角 (-180~180°)

    float  v;               // 线速度 (m/s)

    float  w;               // 角速度 (°/s)

    uint8_t slip_level;     // 打滑状态 (0:正常, 1:轻微, 2:严重)

    uint8_t is_valid;       // 状态是否可靠

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

//=================================================定义  结构体================================================


//全局变量声明
extern RobotState_t robot_pose;                                  // 全局实时位姿，供外部只读访问




//=================================================声明  基础函数================================================

//数据预处理 /基础函数

void navi_data_init(void) ;                             //初始化

void navi_parse_data(void) ;                              //  原始数据解析函数  +基础过滤 (转速初处理，航向角融合，加速度初处理)    

void navi_filter_data(void);                            //数据基础处理，将处理后的数据提交给 navi_mgr.filter_data ,后续使用



//卡尔曼算法

void navi_ekf_config(void);                               //卡尔曼参数初始化配置

void navi_update_F_B_U(float a_pure_mps2, float yaw_cur_deg);

void navi_ekf_update(void) ;                       //卡尔曼算法数据预测+更新+其他信息更新  ,更新当前Navi位置信息      



//航点计算

uint8 navi_calcnavinfo(uint8 target_idx, double *azimuth, double *distance) ;                     // 计算当前位置到目标航点的导航信息

uint8 navi_isreach_target_point(uint8 target_idx)  ;                                       //   判断是否到达目标航点



//外置函数


void Navi_Data_Set_Origin(void); // 独立原点设置 API  置当前位置为坐标原点 (x=0, y=0)

RobotState_t Navi_Get_Robot_Pose(void);

void Navi_Remove_Gravity(float *p_ax, float *p_ay, float *p_az) ;                //重力扣除







//=================================================声明  基础函数================================================



#endif //NAVIGATION_DATA_HANDLING_H