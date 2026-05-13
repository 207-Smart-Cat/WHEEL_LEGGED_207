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

#define NAVI_POINT_MAX   500                  //最大记录航点数

#define DISTANCE_THRESHOLD   0.05f            //到达判定值

#define INTERPOLATION_STEP  1.0f              //插值步长

#define BASE_LOOKAHEAD_DIST  0.5f             // 基础前瞻距离 

#define LOOKAHEAD_VEL_GAIN   0.2f             // 前瞻距离的速度增益系数

#define RECORD_MIN_DIST 0.05f  // 最小打点间距 (米)

#define WEIZHIJIANCE   1

#define NAVI_WAYPOINT_HOLD_MS 3000U


//=================================================定义  结构体================================================
// 航点动作类型
typedef enum {
  
    WP_TYPE_NORMAL = 0,     // 普通循迹点
    
    WP_TYPE_BRIDGE,     // 单边桥 
        
    WP_TYPE_JUMP,           // 跳跃台阶 (爆发抬腿)
    
    WP_TYPE_MINE_SWEEP,     // 定点排雷 (今年新增项目：预留)
    
    WP_TYPE_CONE_CONE,      // 绕圆锥桶 (今年新增项目：预留)
        
    WP_TYPE_SIDE_SLOPE,     // 侧倾坡道 (侧倾自适应)
    
    WP_TYPE_STOP,           // 终点返航
    
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

    uint16_t point_current_idx;    // 当前目标点索引

    uint16_t point_total_count;    // 路径点总数

    uint8_t navi_mode_driver;      // 0: 停止, 1: 自动寻迹, 2: 记录模式
    
    uint8_t navi_mode_map ;       //地图方案：0：静态地图        1. 打点画图          2.WiFi动态目标地图

    float   point_dist_to_target; // 到目标的剩余距离

    float   point_angle_error;    // 航向偏差

    uint8_t  origin_set_flag;                         // 原点已设置标记 

    uint8_t trigger_record; // 外部触发打点标记

} Navi_Controller_t;

// 全局变量声明
extern Navi_Controller_t navi_ctrl;

extern Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // 预设的“路”


extern float wifi_cmd_trigger;        // 摇铃变量         0: 待机, 1: 追加一个航点, 2: 清空当前地图, 3: 立刻接管执行动作
extern float wifi_remote_type;                                  //赋值航点类型,按照枚举依次从0到···
extern float wifi_in_action;            //动作指令
extern float vofa_trigger_record; // 新增：用于接收WiFi远程打点指令的浮点变量     

// ========================== VOFA+ 在线调参变量 ==========================
extern float vofa_mode_driver;       // 对应 navi_mode_driver
extern float vofa_mode_map;          // 对应 navi_mode_map
extern float vofa_print_pose_en;     // 是否开启位姿打印 (0:关, 1:开)
extern float vofa_print_pose_period; // 打印周期 (单位：ms)
extern float vofa_reserved_1;        // 备用变量 1
extern float vofa_reserved_2;        // 备用变量 2


//====================================================函数声明=============================================

void Navi_Tracking_Init(void);

//航点管理      

uint8 navi_isreach_target_point(uint16 target_idx)  ;                                       //   判断是否到达目标航点


//业务服务
void task_navigation_control(void);                       //循迹模式

uint8 navi_calcnavinfo(uint16_t target_idx, double *azimuth, double *distance) ;                     // 计算当前位置到目标航点的导航信息

void navi_auto_record_task(void) ;               //记录当前Navi位置为航点   task_navigation_control函数中已调用   ，随其一起在周期中断了

void navi_path_optimize(void);                      //路径预处理：线性插值

void navi_load_comprehensive_test_map(void);                    //静态地图



//外置函数

Navi_WayPoint_t navi_get_point(uint16_t index) ;                   // 获取指定索引的航点信息

uint8 navi_switch_nexttargetpoint(void) ;                   // 切换到下一个目标航点


float navi_get_two_points_distance(float x1, float y1, float x2, float y2);

float navi_get_two_points_azimuth(float x1, float y1, float x2, float y2);

void navi_wifi_remote_cmd(void) ;

void Navigation_Pose_Monitor_Task(uint32_t delta_ms);                           //导航关闭时的位姿监测打印任务

// 将航点类型枚举转换为对应的中文字符串
const char* get_enum_name(WayPoint_Type type);




#endif