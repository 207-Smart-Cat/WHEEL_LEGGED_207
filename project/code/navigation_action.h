#ifndef NAVIGATION_ACTION_H_
#define NAVIGATION_ACTION_H_

#include "zf_common_headfile.h"

#include "navigation_tracking.h"  // 引入航点类型 WayPoint_Type

// ========================== 宏定义 ==========================
#define MAX_ACTION_NUM  50    // 最多缓存 50 个特殊动作点

// 跳跃动作模式选择：
// 0U = 跳跃点只转一圈测试，不执行跳跃
// 1U = 跳跃点执行一次完整跳跃
// 2U = 跳跃点执行三级跳
#define NAVI_JUMP_ACTION_MODE  2U

// 三级跳两次跳跃之间的间隔选择：
// 1U = 定时行走一段时间后进入下一跳
// 2U = 按位姿距离行走到固定距离后进入下一跳
#define NAVI_TRIPLE_JUMP_INTERVAL_MODE  1U

#define NAVI_TRIPLE_JUMP_AFTER_MODE  1U
#define NAVI_TRIPLE_JUMP_AFTER_TRIPLE_ONLY  0U
#define NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE  1U

#define NAVI_TRIPLE_JUMP_RAMP_DOWN_MS    (1500U)
#define NAVI_TRIPLE_JUMP_TURN_BACK_MS    (1300U)
#define NAVI_TRIPLE_JUMP_RAMP_UP_MS      (1700U)
#define NAVI_TRIPLE_JUMP_STAIR_DOWN_MS   (1400U)
#define NAVI_TRIPLE_JUMP_RAMP_SPEED      NAVI_JUMP_RUNUP_SPEED
#define NAVI_TRIPLE_JUMP_STAIR_SPEED     250.0f

// 跳跃动作期间的坐标更新方式：
// 1U = 使用当前自动位姿更新，空中阶段由现有 EKF/里程逻辑处理
// 2U = 暂停自动位姿更新，每完成一次跳跃后按固定前向距离补偿坐标
#define NAVI_JUMP_POSE_UPDATE_MODE  1U

// 固定坐标补偿参数，仅 NAVI_JUMP_POSE_UPDATE_MODE == 2U 时生效。
// frame=1，表示车体坐标系：forward_m 为前向，right_m 为右向。
#define NAVI_JUMP_FIXED_FORWARD_M  0.30f
#define NAVI_JUMP_FIXED_RIGHT_M    0.00f

// 跳跃前助跑参数：
// RUNUP 阶段只接管速度和航向，不接管舵机，让常规 leg_control 产生前倾助跑。
// 进入 PREPARE/TAKEOFF 后再接管舵机，避免普通腿控覆盖压腿和爆发输出。
#define NAVI_JUMP_RUNUP_SPEED      350.0f
#define NAVI_JUMP_RUNUP_MS         (250U)

// ========================== 动作状态机枚举 ==========================
typedef enum {
    FSM_IDLE = 0,             // 空闲/提前预测
    
    FSM_NORMAL_STOP,          // 普通点停车状态

    // --- 排雷动作状态 ---
    FSM_MINE_APPROACH,        // 排雷动作初始化
    FSM_MINE_PROCESSING,      // 排雷旋转执行中
    
    // --- 跳跃动作状态 ---
    FSM_JUMP_RUNUP,           // 助跑期：保持常规腿控，建立前向速度
    FSM_JUMP_PREPARE,         // 准备期：压腿蓄力
    FSM_JUMP_TAKEOFF,         // 起跳期：伸腿爆发
    FSM_JUMP_AIRBORNE,        // 空中期：收腿/缓冲
    FSM_JUMP_LANDING,         // 落地期：恢复姿态
    FSM_JUMP_TRIPLE_INTERVAL, // 三级跳间隔行走
    FSM_JUMP_RAMP_DOWN,
    FSM_JUMP_TURN_BACK,
    FSM_JUMP_RAMP_UP,
    FSM_JUMP_STAIR_DOWN,
    
    // --- 颠簸路段动作状态 ---
    FSM_BUMP,               // 前进检测真实颠簸段、穿越并由传感确认结束
    
    
    // --- 桥梁动作状态 ---
    FSM_BRIDGE_APPROACH,      // 接近桥梁
    FSM_BRIDGE_ON_BOARD,      // 桥面自适应
    

    
    // --- 绕锥桶动作状态 ---
    FSM_CONE_APPROACH,        // 接近锥桶
    FSM_CONE_NAVIGATE,        // 绕行执行中
    
    // --- 终点停车状态 ---
    FSM_STOP_PARKING          // 终点停车
} ActionState_e;

// ========================== 数据结构定义 ==========================
// 动作状态机运行时数据
typedef struct {
    ActionState_e state;        // 当前状态
    uint32_t state_timer_ms;    // 状态内计时，单位 ms
    uint8_t is_airborne_expect; // 预期空中状态标志，供 EKF/控制层参考
} ActionFSM_t;

// 特殊动作节点
typedef struct {
    uint8_t wp_index;         // 对应 point_map 中的航点索引
    WayPoint_Type type;       // 动作类型
} ActionNode_t;

// 特殊动作序列
typedef struct {
    ActionNode_t list[MAX_ACTION_NUM];
    uint8_t total_count;      // 特殊动作总数
    uint8_t current_ptr;      // 当前待执行动作指针
} ActionSequence_t;

// ========================== 全局变量和接口声明 ==========================
extern ActionFSM_t action_fsm;
extern ActionSequence_t action_seq;
extern uint8_t is_action_busy; 

uint8_t navigation_jump_is_active(void);
uint8_t Navi_Action_Servo_Takeover_Active(void);
void navi_parse_global_path(void);
void Navi_Action_Manager(uint16_t curr_idx) ;
uint8_t Navi_Action_Consume_Done(uint16_t curr_idx);

#endif // NAVIGATION_ACTION_H_
