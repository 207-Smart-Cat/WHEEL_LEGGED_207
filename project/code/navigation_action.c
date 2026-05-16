#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "ipc_shared_data.h"

// ==================== 实例化状态机变量 ====================
ActionFSM_t action_fsm = {FSM_IDLE, 0, 0};
ActionSequence_t action_seq = {0};
uint8_t is_action_busy = 0;  // 0:循迹控制 1:动作接管

// ==================== 引入外部需要的依赖 ====================
extern Navi_WayPoint_t point_map[NAVI_POINT_MAX];
extern Navi_Controller_t navi_ctrl;
extern float target_velocity;
extern float target_angle;

#define MINE_ROTATE_TARGET_DEG      1080.0f
#define MINE_ROTATE_LEAD_DEG        35.0f
#define MINE_ROTATE_TIMEOUT_MS      15000U
#define MINE_ROTATE_CCW_CMD         2U

static float mine_rotate_start_yaw = 0.0f;
static int8_t mine_rotate_dir = 1;
static uint8_t action_done_pending = 0;
static uint16_t action_done_idx = 0;

static void navi_action_mark_done(uint16_t target_idx)
{
    action_done_pending = 1;
    action_done_idx = target_idx;

    if (action_seq.current_ptr < action_seq.total_count &&
        action_seq.list[action_seq.current_ptr].wp_index == target_idx)
    {
        action_seq.current_ptr++;
    }

    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0;
    is_action_busy = 0;
}

static void navi_enter_mine_rotate(uint16_t target_idx)
{
    mine_rotate_start_yaw = (float)robot_pose.cumulative_yaw;
    mine_rotate_dir = (point_map[target_idx].action_cmd == MINE_ROTATE_CCW_CMD) ? -1 : 1;

    action_fsm.state = FSM_MINE_PROCESSING;
    action_fsm.state_timer_ms = 0;
    is_action_busy = 1;

    target_velocity = 0.0f;
    target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * MINE_ROTATE_LEAD_DEG);

    IPC_LOG_Printf("\r\n============= >>> [定点排雷] 已到达旋转点 [%d]，开始%s旋转三圈 <<< =============\r\n",
                   target_idx,
                   mine_rotate_dir > 0 ? "顺时针" : "逆时针");
}

// ===============================================================================
// 全局路径预解析：只提取需要动作 FSM 接管的航点。
// 普通点/起点/终点由 navigation_tracking.c 直接处理，不进入动作队列。
// ===============================================================================
void navi_parse_global_path(void)
{
    action_seq.total_count = 0;
    action_seq.current_ptr = 0;
    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0;
    action_fsm.is_airborne_expect = 0;
    is_action_busy = 0;
    action_done_pending = 0;
    action_done_idx = 0;

    for (int i = 0; i < navi_ctrl.point_total_count; i++)
    {
        if (point_map[i].type == WP_TYPE_MINE_SWEEP)
        {
            action_seq.list[action_seq.total_count].wp_index = i;
            action_seq.list[action_seq.total_count].type = point_map[i].type;
            action_seq.total_count++;
            if (action_seq.total_count >= MAX_ACTION_NUM) break;
        }
    }
}

static void navi_action_fsm_update(uint16_t target_idx)
{
    float rotated_deg;

    action_fsm.state_timer_ms += TIMER_ACTION_PIR;

    switch (action_fsm.state)
    {
        case FSM_IDLE:
            if (point_map[target_idx].type == WP_TYPE_MINE_SWEEP && navi_isreach_target_point(target_idx))
            {
                navi_enter_mine_rotate(target_idx);
            }
            break;

        case FSM_MINE_PROCESSING:
            target_velocity = 0.0f;
            target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * MINE_ROTATE_LEAD_DEG);
            rotated_deg = ((float)robot_pose.cumulative_yaw - mine_rotate_start_yaw) * mine_rotate_dir;

            if (rotated_deg >= MINE_ROTATE_TARGET_DEG)
            {
                target_velocity = 0.0f;
                IPC_LOG_Printf(" [定点排雷] 三圈旋转完成，等待主导航切换航点。\r\n");
                navi_action_mark_done(target_idx);
            }
            else if (action_fsm.state_timer_ms > MINE_ROTATE_TIMEOUT_MS)
            {
                target_velocity = 0.0f;
                IPC_LOG_Printf(" [定点排雷] 三圈旋转超时退出，等待主导航切换航点。\r\n");
                navi_action_mark_done(target_idx);
            }
            break;
        default:
            action_fsm.state = FSM_IDLE;
            is_action_busy = 0;
            break;
    }
}

uint8_t Navi_Action_Consume_Done(uint16_t curr_idx)
{
    if (action_done_pending && action_done_idx == curr_idx)
    {
        action_done_pending = 0;
        return 1;
    }

    return 0;
}

// ===============================================================================
// 运行时动作调度器：只在当前目标航点正好是动作点时接管。
// ===============================================================================
void Navi_Action_Manager(uint16_t curr_idx)
{
    uint16_t target_wp_idx;

    if (action_seq.total_count == 0 || action_seq.current_ptr >= action_seq.total_count)
    {
        return;
    }

    while (action_seq.current_ptr < action_seq.total_count &&
           curr_idx > action_seq.list[action_seq.current_ptr].wp_index)
    {
        action_seq.current_ptr++;
    }

    if (action_seq.current_ptr >= action_seq.total_count)
    {
        return;
    }

    target_wp_idx = action_seq.list[action_seq.current_ptr].wp_index;

    if (is_action_busy)
    {
        navi_action_fsm_update(target_wp_idx);
        return;
    }

    if (curr_idx == target_wp_idx)
    {
        navi_action_fsm_update(target_wp_idx);
    }
}
