#ifndef NAVIGATION_ACTION_H_
#define NAVIGATION_ACTION_H_

#include "zf_common_headfile.h"

#include "navigation_tracking.h"  // 引入航点类型 WayPoint_Type

// ========================== 宏定义 ==========================
#define MAX_ACTION_NUM  50    // 假设赛道上最多有 50 个特殊动作点

#define TIMER_ACTION_PIR  10       //中断周期     10ms

// ========================== 动作状态机枚举 ==========================
typedef enum {
    FSM_IDLE = 0,             // 闲置/正常巡航
    
    FSM_NORMAL_STOP,  // 普通点短暂停留
    
// --- 跳跃动作状态链 ---   
    FSM_JUMP_PREPARE,         // 准备期：压低重心，蓄力加速
    FSM_JUMP_TAKEOFF,         // 起跳期：爆发伸腿
    FSM_JUMP_AIRBORNE,        // 腾空期：空中姿态保持，通知 EKF 断流补全
    FSM_JUMP_LANDING,         // 落地期：屈腿缓冲冲击
    
    // --- 单边桥动作状态链 ---
    FSM_BRIDGE_APPROACH,      // 接近期：减速，精准对正
    FSM_BRIDGE_ON_BOARD,      // 上桥期：开启腿部独立自适应
    
    // --- 定点排雷动作状态链 ---
    FSM_MINE_PROCESSING,      // 到点后停车并旋转三圈
    
    // --- 绕圆锥桶状态链 ---
    FSM_CONE_APPROACH,        // 接近期：降速，准备大角度转向
    FSM_CONE_NAVIGATE,        // 绕行期：维持低速与低重心
    
    // --- 侧倾坡道状态链 ---
    FSM_SLOPE_APPROACH,       // 接近期：姿态调整
    FSM_SLOPE_ONBOARD,        // 上坡期：压低重心，抗侧翻
    
    // --- 终点停车状态 ---
    FSM_STOP_PARKING          // 终点停车锁定
} ActionState_e;

// ========================== 数据结构定义 ==========================
// 动作状态机管理结构体
typedef struct {
    ActionState_e state;        // 当前状态
    uint32_t state_timer_ms;    // 状态内部计时器 (用于超时保护)
    uint8_t is_airborne_expect; // 期望腾空标志位 (传给 EKF 用于断流推算)
} ActionFSM_t;

// 动作节点结构体 (用于预解析提取)
typedef struct {
    uint8_t wp_index;         // 这个动作在 point_map 中的真实序号
    WayPoint_Type type;       // 动作类型 (跳跃/单边桥等)
} ActionNode_t;

// 动作序列剧本结构体
typedef struct {
    ActionNode_t list[MAX_ACTION_NUM];
    uint8_t total_count;      // 赛道上总共有几个特殊动作
    uint8_t current_ptr;      // 我们当前正在驶向第几个动作？(指针)
} ActionSequence_t;

// ========================== 全局变量与接口声明 ==========================
extern ActionFSM_t action_fsm;
extern ActionSequence_t action_seq;
extern uint8_t is_action_busy; 

void navi_parse_global_path(void);
void navi_action_manager(uint8_t curr_idx);
void Navi_Action_Manager(uint8_t curr_idx) ;

#endif // NAVIGATION_ACTION_H_
