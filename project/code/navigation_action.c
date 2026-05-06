#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "jump_control.h"

// ==================== 实例化状态机变量 ====================
ActionFSM_t action_fsm = {FSM_IDLE, 0, 0};
ActionSequence_t action_seq = {0};
uint8_t is_action_busy = 0;  // 0:循迹控制 1:动作接管

// ==================== 引入外部需要的依赖 ====================
extern Navi_WayPoint_t point_map[NAVI_POINT_MAX]; 
extern Navi_Controller_t navi_ctrl;
extern Navi_Sensor_Data_t filter_data;

// 引入 control.c 中的底层变量
extern float target_velocity;    
extern float target_motor_Stand; 
extern float target_engine_high; 
extern float x_current, y_current;
extern int Bridge_position;

// ==============================================================================
// 模块 1：全局路径预解析 (在录制完成或发车前调用 1 次)
// ==============================================================================
void navi_parse_global_path(void) {
    action_seq.total_count = 0;
    action_seq.current_ptr = 0;

    for (int i = 0; i < navi_ctrl.point_total_count; i++) {
        if (point_map[i].type == WP_TYPE_JUMP || 
            point_map[i].type == WP_TYPE_BRIDGE || 
            point_map[i].type == WP_TYPE_CROSSING) {
            
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
static void Navi_Action_FSM_Update(uint8_t target_idx, double distance) {
    WayPoint_Type upcoming_type = point_map[target_idx].type;
    action_fsm.state_timer_ms += 10; // 假设调用周期为 10ms

    switch (action_fsm.state) {
        case FSM_IDLE:
            action_fsm.is_airborne_expect = 0;
            // 跳跃预警：距离 < 1.0m
            if (upcoming_type == WP_TYPE_JUMP && distance < 1.0f) {
                action_fsm.state = FSM_JUMP_PREPARE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1; 
            }
            // 单边桥预警：距离 < 0.8m
            else if (upcoming_type == WP_TYPE_BRIDGE && distance < 0.8f) {
                action_fsm.state = FSM_BRIDGE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_PREPARE:
            target_velocity = 800;
            y_current = 0.04f;         // 压低重心
            target_motor_Stand = 3.0f; // 身体前倾
            if (distance < 0.1f || action_fsm.state_timer_ms > 2000) {
                jump_start();
                action_fsm.state = FSM_JUMP_TAKEOFF;
                action_fsm.state_timer_ms = 0;
            }
            break;

        case FSM_JUMP_TAKEOFF:
            y_current = 0.14f;         // 爆发起跳
            target_velocity = 900;
            if (jump_state == JUMP_AIR_RETRACT || action_fsm.state_timer_ms > 200) { 
                action_fsm.state = FSM_JUMP_AIRBORNE;
                action_fsm.state_timer_ms = 0;
                action_fsm.is_airborne_expect = 1; // 告诉EKF抛弃里程计
            }
            break;

        case FSM_JUMP_AIRBORNE:
            y_current = 0.05f;         // 空中收腿
            if (jump_state == JUMP_RECOVER || action_fsm.state_timer_ms > 600) {
                action_fsm.state = FSM_JUMP_LANDING;
                action_fsm.state_timer_ms = 0;
                action_fsm.is_airborne_expect = 0; // 告诉EKF恢复
            }
            break;

        case FSM_JUMP_LANDING:
            y_current = 0.04f;                    //屈腿缓冲
            target_velocity = 400;
            target_motor_Stand = 1.6f;
            if (!jump_is_active() && action_fsm.state_timer_ms > 300) { 
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                y_current = 0.08f;     
                navi_switch_nexttargetpoint(); // 强制切掉跳跃航点
            }
            break;

        case FSM_BRIDGE_APPROACH:
            target_velocity = 400;
            y_current = 0.09f;
            if (distance < 0.1f) {
                action_fsm.state = FSM_BRIDGE_ON_BOARD;
                action_fsm.state_timer_ms = 0;
                Bridge_position = 0;   // 开启单边桥自适应
            }
            break;

        case FSM_BRIDGE_ON_BOARD:
            target_velocity = 500;
            if (action_fsm.state_timer_ms > 3000) { 
                action_fsm.state = FSM_IDLE;
                is_action_busy = 0;
                Bridge_position = 1;   // 关闭自适应
                navi_switch_nexttargetpoint();
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
    int index_diff = target_wp_idx - curr_idx;
    if (index_diff < 0) index_diff += navi_ctrl.point_total_count;                   //防环形越界

    // 1. 越过目标点
    if (index_diff == 0 || index_diff > (navi_ctrl.point_total_count - 10)) {                              //越过了目标点（且没跑太远，在 10 个点以内），就算你跨越成功了！
        action_seq.current_ptr++;
        if (action_seq.current_ptr >= action_seq.total_count) {                                        //最后一关，就将指针归 0，准备跑下一圈（可选）
            action_seq.current_ptr = 0; 
        }
        return; 
    }

    // 2. 预警触发
    if (index_diff > 0 && index_diff < 20) {
        double real_distance = 0.0;
        double dummy_azimuth = 0.0;
        
        navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance);
        Navi_Action_FSM_Update(target_wp_idx, real_distance);
    } 
    else {                                                                    //如果目标还在 20个点开外（很安全），那么强行将状态机的状态锁定为 FSM_IDLE（闲置），并清空所有接管标志位。
        action_fsm.state = FSM_IDLE;
        action_fsm.is_airborne_expect = 0;
        is_action_busy = 0;
    }
}