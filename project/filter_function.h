/*********************************************************************************************************************

********************************************************************************************************************/

#ifndef FILTER_FUNCTION_H
#define FILTER_FUNCTION_H

#include "zf_common_typedef.h"
#include "matrix.h"

//=========================参数配置============================
#define TIMER_FILTER_CH  TC_TIME2_CH0       //需要的时间差的计时器      ！！！外部初始化时，一定要确定是us微秒级。且要定期清零，确保不到达最大值后停止计时。清零后较大 的dt数据要舍弃
                                             //确保只用于时间戳获取，避免计数值被误篡改


//==========================结构体定义=========================
// 卡尔曼滤波结构体（支持1-4阶，状态量维度n=1~4）
typedef struct {               //n：状态量     m：测量量
    float Abtastzeit_s;         //滤波器积分用的采样时间 [s]
    Matrix X;      // 状态矩阵 [n*1]：
    Matrix B;      // 控制矩阵 [n*p]（p为控制量维度，通常1）
    Matrix U;      // 控制量矩阵 [p*1]（如加速度、油门PWM）
    Matrix F;      // 状态转移矩阵 [n*n]
    Matrix F_T;
    Matrix H;      // 观测矩阵 [m*n]（m为观测维度，通常1~2）  ，  将状态空间转换为测量空间
    Matrix H_T;
    Matrix P;      // 状态协方差矩阵 [n*n]
    Matrix Q;      // 过程噪声协方差矩阵 [n*n]（对角矩阵）
    Matrix R;      // 观测噪声协方差矩阵 [m*m]（对角矩阵）
    Matrix Z;      //观测矩阵Z [m*1]
    Matrix K;      // 卡尔曼增益矩阵 [n*m]
    Matrix temp_X; //矩阵X更新时临时变量                                       ------------------矩阵的运算不能直接计算后赋值，矩阵每个计算都包含几个数，
    Matrix temp1;  // 临时矩阵 [n*n]（减少内存申请）                                           若赋值则前一个数被覆盖后对后一个数计算结果会产生影响
    Matrix temp2;  // 临时矩阵 [n*m]
    Matrix temp3;  // 临时矩阵 [m*m]
    Matrix temp4;  // 临时矩阵 [m*1]    
    Matrix temp_inv;
    int state_dim; // 状态维度，滤波状态量个数  
    int obs_dim;   // 观测维度，传感器实际测量的值个数  
} KalmanFilter_Struct;


// 一阶低通滤波结构体
typedef struct {
    float alpha;      // 滤波系数（0<alpha<1，越小越平滑）
    float prev_value; // 上一次滤波结果
    uint8 is_init;    // 是否初始化标记（0=未初始化，1=已初始化）
} LowPassFilter_Struct;

// 滑动平均滤波结构体
typedef struct {
    float *buf;       // 数据缓冲区
    int buf_size;     // 缓冲区大小（滑动窗口长度）
    int index;        // 当前数据索引
    int count;        // 已存储数据个数
    float sum;        // 缓冲区数据和（优化计算效率）
    uint8 is_init;    // 是否初始化标记
} MovingAvgFilter_Struct;

// 中值滤波结构体
typedef struct {
    float *buf;       // 数据缓冲区
    int buf_size;     // 缓冲区大小（建议奇数，如3/5/7）
    int index;        // 当前数据索引
    int count;        // 已存储数据个数
    uint8 is_init;    // 是否初始化标记
} MedianFilter_Struct;

// 均值滤波结构体（静态窗口，填满后计算）
typedef struct {
    float *buf;       // 数据缓冲区
    int buf_size;     // 缓冲区大小
    int index;        // 当前数据索引
    int count;        // 已存储数据个数
    float sum;        // 缓冲区数据和
    uint8 is_init;    // 是否初始化标记
} AvgFilter_Struct;

// 互补滤波结构体（融合两个传感器，如加速度计+陀螺仪）
typedef struct {
    float alpha;      // 互补系数（0<alpha<1，越大越信任陀螺仪）
    float prev_value; // 上一次融合结果（角度）
    uint32 prev_time; // 上一次更新时间戳（us，用于计算dt）
    uint8 is_init;    // 是否初始化标记
} ComplementaryFilter_Struct;

//=========================变量声明============================

//===========================函数声明============================
//卡尔曼滤波
uint8 kalman_filter_init(KalmanFilter_Struct *kf, int state_dim, int obs_dim,int ctrl_dim);// 初始化卡尔曼滤波器（state_dim:状态维度1~4；obs_dim:观测维度1~2）
void kalman_filter_predict(KalmanFilter_Struct *kf);                             // 卡尔曼预测阶段（无控制量，适配智能车多数场景）
void kalman_filter_update(KalmanFilter_Struct *kf, float *obs_data);      // 卡尔曼更新阶段（传入观测值）

// 一阶低通滤波
uint8 low_pass_filter_init(LowPassFilter_Struct *lpf, float alpha);  // 初始化
float low_pass_filter_update(LowPassFilter_Struct *lpf, float input); // 滤波更新

// 滑动平均滤波
uint8 moving_avg_filter_init(MovingAvgFilter_Struct *maf, int buf_size); // 初始化
float moving_avg_filter_update(MovingAvgFilter_Struct *maf, float input); // 滤波更新
void moving_avg_filter_deinit(MovingAvgFilter_Struct *maf);               // 释放内存

// 中值滤波
uint8 median_filter_init(MedianFilter_Struct *mf, int buf_size);    // 初始化
float median_filter_update(MedianFilter_Struct *mf, float input);   // 滤波更新
void median_filter_deinit(MedianFilter_Struct *mf);                 // 释放内存

// 均值滤波
uint8 avg_filter_init(AvgFilter_Struct *af, int buf_size);          // 初始化
float avg_filter_update(AvgFilter_Struct *af, float input);         // 滤波更新
void avg_filter_deinit(AvgFilter_Struct *af);                       // 释放内存

// 互补滤波
uint8 complementary_filter_init(ComplementaryFilter_Struct *cf, float alpha); // 初始化
float complementary_filter_update(ComplementaryFilter_Struct *cf, float input1, float input2); // 融合更新



#endif

