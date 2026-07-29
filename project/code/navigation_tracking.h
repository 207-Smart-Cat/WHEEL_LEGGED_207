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

#define DISTANCE_THRESHOLD   0.10f            //到达判定值

#define INTERPOLATION_STEP  1.0f              //插值步长

//#define BASE_LOOKAHEAD_DIST  0.5f             // 基础前瞻距离 
//
//#define LOOKAHEAD_VEL_GAIN   0.2f             // 前瞻距离的速度增益系数

//#define RECORD_MIN_DIST 0.5f  // 最小打点间距 (米)

#define Print_location   1           //1：打印位姿

#define ENABLE_PATH_INTERPOLATION 0              //1：开启线性插值

#define USE_HOST_TARGET_VELOCITY 2   // 0：固定速度          1；上位机更新         2：pid动态规划速度

#define DEFAULT_TRACKING_VELOCITY 300.0f   //默认速度

#define NAVI_BRIDGE_ACTION_START  1U
#define NAVI_BRIDGE_ACTION_END    2U
#define NAVI_COURSE3_LINE_LOOKAHEAD_DISTANCE 0.40f


//=================================================定义  结构体================================================
// 航点动作类型
typedef enum {
  
    WP_TYPE_NORMAL = 0,     // 普通循迹点
    
    WP_TYPE_MINE_SWEEP= 1,     // 定点排雷    
    
    WP_TYPE_CONE_CONE= 2,      // 绕圆锥桶  
    
    WP_TYPE_BRIDGE = 3,     // 单边桥 
        
    WP_TYPE_JUMP = 4,           // 跳跃台阶 

    WP_TYPE_STOP= 6,           // 终点返航
    
    WP_TYPE_HOME = 7           // 原点
} WayPoint_Type;


//航点信息

typedef struct {

    float x;

    float y;

    float yaw;          //若使用，则为期望朝向：到达航点时车头锁定的目标绝对角度

    WayPoint_Type type;      // 航点大类（普通/单边桥/跳跃/排雷/绕桶/终点）
    
    /**
     * @brief 动作子指令/控制参数
     * @note  用于传递大类动作下的具体配置参数：
     * 1. 排雷点(MINE_SWEEP)：2 代表逆时针转，其他值代表顺时针转
     * 2. 跳跃点(JUMP)：可直接用作起跳目标速度（如 450 代表 450mm/s）
     * 3. 绕桶点(CONE)：1 代表左绕，2 代表右绕
     * 4. 普通点(NORMAL)：非 0 时可作为局部动态限速值
     */
    uint16_t action_cmd;           //对于跳跃：0--》转一圈         1--》

    uint8_t valid;

} Navi_WayPoint_t;



// 导航核心管理结构体
typedef struct {
    uint16_t point_current_idx;    // 当前目标点索引
    
    uint16_t point_total_count;    // 路径点总数
    
    uint8_t navi_mode_driver;      // 0: 地图选择, 1: 自动寻迹, 2: 记录模式
    
    uint8_t navi_mode_map ;        // 地图方案：0:静态地图, 1:打点地图  2:清空后台打点地图与前台缓存
    
    float   point_dist_to_target; // 到目标的剩余距离
    
    float   point_angle_error;    // 航向偏差
    
    uint8_t  origin_set_flag;      // 原点已设置标记 
    
    uint8_t trigger_record;        // 外部触发特殊打点标记
    WayPoint_Type  trigger_record_type;   //外部打点航点类型
    
    // 【新增1】打点遥测状态子结构（替代原有的 5 个独立全局浮点变量）
    struct {
        float count;                    // 当前地图已录制的航点总数
        float last_idx;                 // 刚录制的最后一个航点索引 (空地图为-1)
        float last_type;                // 刚录制的最后一个航点类型 (枚举值)
        float last_x;                   // 刚录制的最后一个航点 X 坐标 (单位: m)
        float last_y;                   // 刚录制的最后一个航点 Y 坐标 (单位: m)
    } record_status;
    
} Navi_Controller_t;

// 全局变量声明
extern Navi_Controller_t navi_ctrl;

extern Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // 导航时的“路”
extern Navi_WayPoint_t     record_point_map[NAVI_POINT_MAX];
extern uint16_t            record_point_count;

extern float wifi_cmd_trigger;        // 摇铃变量         0: 待机, 1: 追加一个航点, 2: 清空当前地图, 3: 立刻接管执行动作
extern float wifi_in_action;            //动作指令
extern float vofa_trigger_record; // 打点动作标志             1、2：启动打点（是否边沿触发），2：撤销打点

// ========================== VOFA+ 在线调参变量 ==========================
extern float vofa_mode_driver;       // 对应 navi_mode_driver
extern float vofa_mode_map;          // 对应 navi_mode_map
extern float vofa_print_pose_en;     // 是否开启位姿打印 (0:关, 1:开)
extern float vofa_print_pose_period; // 打印周期 (单位：ms)


//====================================================函数声明=============================================
void Navi_Tracking_Init(void);            //初始化

void navi_load_comprehensive_test_map(void);                    //静态地图

//航点管理      
uint8 navi_isreach_target_point(uint16 target_idx)  ;  //  航点到达判断

uint8 navi_switch_nexttargetpoint(void) ;                   // 切换到下一个目标航点

const char* get_enum_name(WayPoint_Type type);    // 将航点类型枚举转换为对应的中文字符串


//业务服务
void task_navigation_control(void);                       //导航的模式选择

uint8 navi_calcnavinfo(uint16_t target_idx, float *azimuth, float *distance) ;                     // 计算当前位置到目标航点的导航信息

void navi_auto_record_task(void) ;               //航点记录动作
void navi_record_update_status(void);

void navi_path_optimize(void);                      //线性插值


//外置函数
Navi_WayPoint_t navi_get_point(uint16_t index) ;                   // 获取指定索引的航点信息

float navi_get_two_points_distance(float x1, float y1, float x2, float y2);      //两点距离

float navi_get_two_points_azimuth(float x1, float y1, float x2, float y2);       //两点转角

void Navigation_Pose_Monitor_Task(void);                           //位姿打印动作


#endif
