/*********************************************************************************************************************

*  使用卡尔曼EKF，状态量=[x ,y ,yaw ,v ,w]          ，预测方程来于编码器的速度，测量方程来源是imu的加速度和角速度。

*  通过navi_mgr.trigger_record 变量，变量 = 1 时，打点。会周期检测该变量

*对于pitch和roll方向，采用了leg_adaptive.c中，方向     

*!!!!!!!void task_navigation_control(void)函数中，有修改将使 Turn_Pwm 受你的导航方位角控制。        

*导航单位管理：

        速度      rpm，         以  rm  命名

       角速度     °/s            角度制

        角度     °             角度制



导航模块实现方式：
      模式识别：     2.遥控模式，遥控进行打点，绘制航点图                         ————>  动作处理：检测到达点位后，识别当前点位的类型，执行相关动作。
                    1.巡航模式，根据航点图，依次巡航，并检测动作


!!!!is_action_busy在识别特殊动作时，会置1.那么规定在动作任务完成时，负责将他置回0

*********************************************************************************************************************/

#ifndef  NAVIGATION_TRACKING

#define  NAVIGATION_TRACKING



#include "zf_common_typedef.h"



//=================================================定义  基本配置================================================

#define NAVI_POINT_MAX   100                  //最大记录航点数

#define DISTANCE_THRESHOLD   0.2f



//=================================================定义  结构体================================================

// 航点动作类型
typedef enum {
  
    WP_TYPE_NORMAL = 0,     // 普通循迹点
    
    WP_TYPE_BRIDGE,     // 单边桥 
        
    WP_TYPE_JUMP,           // 跳跃台阶 (爆发抬腿)
    
    WP_TYPE_MINE_SWEEP,     // 定点排雷 (今年新增项目：预留)
    
    WP_TYPE_CONE_CONE,      // 绕圆锥桶 (今年新增项目：预留)
    
    WP_TYPE_CROSSING,       // 穿越低矮障碍 (压低重心)
    
    WP_TYPE_SIDE_SLOPE,     // 侧倾坡道 (侧倾自适应)
    
    WP_TYPE_STOP,           // 终点停车 (两阶段强制停机)
    
    WP_TYPE_HOME            // 原点
} WayPoint_Type;


//航点信息

typedef struct {

    float x;

    float y;

    float yaw;          //用于平滑转弯插值

    WayPoint_Type type;      // 枚举类型

    uint16_t action_cmd;     //动作指令（如 1:穿越, 2:避障, 3:特殊动作）

    uint8_t valid;

} Navi_WayPoint_t;



// 导航核心管理结构体

typedef struct {

    uint8_t point_current_idx;    // 当前目标点索引

    uint8_t point_total_count;    // 路径点总数

    uint8_t navi_mode;      // 0: 停止, 1: 自动寻迹, 2: 记录模式

    float   point_dist_to_target; // 到目标的剩余距离

    float   point_angle_error;    // 航向偏差

    uint8_t               origin_set_flag;                         // 原点已设置标记 

    uint8_t trigger_record; // 外部触发打点标记

} Navi_Controller_t;;

// 全局变量声明



//====================================================函数声明=============================================

void Navi_Tracking_Init(void);

//航点管理

void navi_record_current_point(void) ;                                                //记录当前Navi位置为航点   task_navigation_control函数中已调用   ，随其一起在周期中断了

void navi_path_optimize(void);                                                        //路径预处理：线性插值


//业务服务
void navi_load_static_calibration_map(void);                    //静态地图

void navi_execute_integrated_action(uint8_t point_idx);           //   核心集成动作执行引擎

void task_navigation_control(void);                       //循迹模式





//外置函数

Navi_WayPoint_t navi_get_point(uint8 index) ;                   // 获取指定索引的航点信息

uint8 navi_switch_nexttargetpoint(void) ;                   // 切换到下一个目标航点







#endif