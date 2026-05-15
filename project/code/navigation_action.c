#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "control.h"
#include "ipc_shared_data.h"


// ==================== 实例化状态机变量 ====================
ActionFSM_t action_fsm = {FSM_IDLE, 0, 0};
ActionSequence_t action_seq = {0};
uint8_t is_action_busy = 0;  // 0:循迹控制 1:动作接管

// ==================== 引入外部需要的依赖 ====================
extern Navi_WayPoint_t point_map[NAVI_POINT_MAX]; 
extern Navi_Controller_t navi_ctrl;
extern Navi_Sensor_Data_t filter_data;

extern float target_velocity;    
extern float now_velocity;
extern float target_motor_Stand; // 默认中值为 2.2
extern float x_current, y_current;
extern int jump_position;        // 告诉底层进入跳跃/腾空模式 (关闭转向)
extern int jump_stop;            // 强制切断电机PID
extern int Bridge_position;

// 定义腿长边界 (与 control.c 保持一致)
#define LEG_MIN 0.04f
#define LEG_MAX 0.10f
#define LEG_NOMINAL 0.08f

// ==============================================================================
// 全局路径预解析,找出特殊点位置 (在录制完成或发车前调用 1 次)
// ==============================================================================
void navi_parse_global_path(void) {
    action_seq.total_count = 0;
    action_seq.current_ptr = 0;

    for (int i = 0; i < navi_ctrl.point_total_count; i++) {
        if (point_map[i].type != WP_TYPE_HOME) {
            action_seq.list[action_seq.total_count].wp_index = i;
            action_seq.list[action_seq.total_count].type = point_map[i].type;
            action_seq.total_count++;
            if (action_seq.total_count >= MAX_ACTION_NUM) break;
        }
    }
}

// ==============================================================================
// 模块 2：异步动作状态机
// ==============================================================================
static void navi_action_fsm_update(uint8_t target_idx, double distance) {
    WayPoint_Type upcoming_type = point_map[target_idx].type;
    action_fsm.state_timer_ms += TIMER_ACTION_PIR; 

switch (action_fsm.state) {
        case FSM_IDLE:
            // 休闲时自动巡航 / 正常状态恢复
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;        
            jump_position = 0;    
            target_motor_Stand = 2.2f; // 恢复机械中值
            y_current = LEG_NOMINAL;
            
            // 距离判定预警分发
            if (upcoming_type == WP_TYPE_JUMP && distance < 1.0f) {
                action_fsm.state = FSM_JUMP_PREPARE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1; 
            }
            else if (upcoming_type == WP_TYPE_BRIDGE && distance < 0.8f) {
                action_fsm.state = FSM_BRIDGE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            else if (upcoming_type == WP_TYPE_MINE_SWEEP && distance < 0.5f) {
                action_fsm.state = FSM_MINE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            else if (upcoming_type == WP_TYPE_CONE_CONE && distance < 0.8f) {
                action_fsm.state = FSM_CONE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            else if (upcoming_type == WP_TYPE_SIDE_SLOPE && distance < 0.8f) {
                action_fsm.state = FSM_SLOPE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            else if (upcoming_type == WP_TYPE_STOP && distance < 0.3f) {
                action_fsm.state = FSM_STOP_PARKING;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            else if (upcoming_type == WP_TYPE_NORMAL && navi_isreach_target_point(target_idx)) {
                action_fsm.state = FSM_NORMAL_STOP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;  // 接管控制权
                
                // 动作系统抢先打印到达信息
                IPC_LOG_Printf("\r\n============= >>> [到达事件] 已精准到达航点 [%d]，开始静止 3 秒测试 <<< =============\r\n", target_idx);
            }
            break;
            
        // 【规范修改 2】：标准化普通点停留动作 (未来你的蜂鸣器、云台测试都可以加在这里)
        case FSM_NORMAL_STOP:
            target_velocity = 0;       // 强制切断速度，保持停车
            
            // --> [预留测试区]：你可以在这里加入如 BEEP_ON(); 等测试代码 <--
            
            if (action_fsm.state_timer_ms > 3000) {  // 停滞 3 秒
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;       
                
                IPC_LOG_Printf(" [动作完成] 3秒等待结束，控制权已交还底层，继续循迹。\r\n");
                // 交还控制权后，主循环 task_navigation_control 会识别到 !is_action_busy 从而自动调用 navi_switch_nexttargetpoint() 进行无缝切点。--
            }
            break;
            

        // ------------------ 跳跃动作 ------------------
        case FSM_JUMP_PREPARE:
            // 准备期：加速并下蹲蓄力
            target_velocity = 650;     // 提速
            y_current = LEG_MIN;       // 重心压到最低
            target_motor_Stand = 3.0f; // 身体前倾
            
            if (distance < 0.1f || action_fsm.state_timer_ms > 1500) {
                action_fsm.state = FSM_JUMP_TAKEOFF;
                action_fsm.state_timer_ms = 0;
            }
            break;

        case FSM_JUMP_TAKEOFF:
            // 起跳期：腿部瞬间伸展到最大，开启跳跃屏蔽模式
            y_current = LEG_MAX;       // 瞬间蹬腿 (MAX_LEG_LENGTH 0.1)
            jump_position = 1;         // 通知 control.c 切断转向，只保直立
            
            // 利用您底层的失重判断函数
            if (navi_airborne_detection() || action_fsm.state_timer_ms > 200) { 
                action_fsm.state = FSM_JUMP_AIRBORNE;
                action_fsm.state_timer_ms = 0;
            }
            break;

        case FSM_JUMP_AIRBORNE:
            // 腾空期：收腿防止磕碰，并在空中维持姿态
            y_current = LEG_MIN;       // 空中缩腿
            target_velocity = 0;       // 防止轮子在空中疯转产生陀螺效应
            
            // 底层的 control.c 会在此期间自动用 air_roll_pid 维持平衡
            
            if (!navi_airborne_detection() || action_fsm.state_timer_ms > 800) {
                action_fsm.state = FSM_JUMP_LANDING;
                action_fsm.state_timer_ms = 0;
            }

        case FSM_JUMP_LANDING:
            // 落地缓冲期
            y_current = LEG_MIN;       // 保持屈腿缓冲冲击
            target_velocity = 400;     // 恢复正常速度防摔
            target_motor_Stand = 1.6f; // 重心后仰防前翻
            
            if (action_fsm.state_timer_ms > 400) { 
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                jump_position = 0;     // 重新开启转向控制
                
            }
            break;

        // ------------------ 单边桥动作 ------------------
        case FSM_BRIDGE_APPROACH:
            target_velocity = 400;
            y_current = LEG_NOMINAL;
            if (distance < 0.1f || action_fsm.state_timer_ms > 2000) {
                action_fsm.state = FSM_BRIDGE_ON_BOARD;
                action_fsm.state_timer_ms = 0;
                Bridge_position = 0;   // 开启单边桥自适应
            }
            break;

        case FSM_BRIDGE_ON_BOARD:
            target_velocity = 400;
            if (action_fsm.state_timer_ms > 3000) { // 假定3秒过桥，可改为基于位移的判定
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                Bridge_position = 1;   // 关闭自适应
                
            }
            break;

        // ------------------ 定点排雷动作 ------------------
        case FSM_MINE_APPROACH:
            target_velocity = 200;     // 极低速靠近
            if (distance < 0.05f || action_fsm.state_timer_ms > 2000) {
                action_fsm.state = FSM_MINE_PROCESSING;
                action_fsm.state_timer_ms = 0;
            }
            break;

        case FSM_MINE_PROCESSING:
            target_velocity = 0;       // 停车
            // jump_stop = 1;          // 若需要切断PID动力可解开此注释
            if (action_fsm.state_timer_ms > 2000) { // 模拟排雷停留2秒
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                
            }
            break;

        // ------------------ 绕圆锥桶动作 ------------------
        case FSM_CONE_APPROACH:
            target_velocity = 350;     // 降速防止侧滑
            y_current = 0.05f;         // 降低重心增加抓地力
            if (distance < 0.2f) {
                action_fsm.state = FSM_CONE_NAVIGATE;
                action_fsm.state_timer_ms = 0;
            }
            break;

        case FSM_CONE_NAVIGATE:
            target_velocity = 350;
            if (action_fsm.state_timer_ms > 2500) { // 根据实际绕桩时间调整
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                y_current = 0.08f;
                
            }
            break;

        // ------------------ 侧倾坡道动作 ------------------
        case FSM_SLOPE_APPROACH:
            target_velocity = 450;
            y_current = 0.04f;         // 极致低重心
            if (distance < 0.1f) {
                action_fsm.state = FSM_SLOPE_ONBOARD;
                action_fsm.state_timer_ms = 0;
            }
            break;

        case FSM_SLOPE_ONBOARD:
            target_velocity = 500;
            // 依靠 leg_control 内置的 roll 补偿即可应对侧倾
            if (action_fsm.state_timer_ms > 3000) {
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                y_current = 0.08f;
                
            }
            break;

        // ------------------ 终点停车 ------------------
        case FSM_STOP_PARKING:      // 彻底关闭导航计算，防止回荡
            navi_ctrl.navi_mode_driver = 0; 
            target_velocity = 0.0f;
            
            static uint8_t stop_printed = 0;
            if (!stop_printed) {
            IPC_LOG_Printf("=============  >>> [事件] 终点已到达，动作接管并安全停车！ =============\r\n");                  
            stop_printed = 1;
            }
            break;

        default:
            action_fsm.state = FSM_IDLE;
            break;
    }
}

// ==============================================================================
// 模块 3：运行时动作调度器 (暴露给主循环)
// ==============================================================================
void Navi_Action_Manager(uint8_t curr_idx) {
    if (action_seq.total_count == 0) return; 

    uint8_t target_wp_idx = action_seq.list[action_seq.current_ptr].wp_index;
    
    if (is_action_busy) {
        double real_distance = 0.0;
        double dummy_azimuth = 0.0;
        // 注意：计算的距离必须是离“动作目标点(target_wp_idx)”的距离，而不是主循环当前点
        navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance);
        navi_action_fsm_update(target_wp_idx, real_distance);
        return; 
    }
    
     // 如果主循迹因为某些无动作的普通点切走了，我们必须把剧本指针也往后追平
    if (curr_idx > target_wp_idx) {
        action_seq.current_ptr++;
        if (action_seq.current_ptr >= action_seq.total_count) {
            action_seq.current_ptr = 0; // 重置或卡在最大值
        }
        return; 
    }
    
// 空闲且正好轮到这个点，开启动作预警检测
    if (curr_idx == target_wp_idx) {
        double real_distance = 0.0;
        double dummy_azimuth = 0.0;
        navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance);
        navi_action_fsm_update(target_wp_idx, real_distance);
    } else {                                                                    
        action_fsm.state = FSM_IDLE;
        is_action_busy = 0;
    }
}