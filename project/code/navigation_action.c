#include "navigation_action.h"
#include "navigation_data_handling.h"
#include "ipc_shared_data.h"
#include "param.h"
#include "jump_control.h"
#include "imu.h"

// ==================== 动作状态机实例 ====================
ActionFSM_t action_fsm = {FSM_IDLE, 0, 0};
ActionSequence_t action_seq = {0};
uint8_t is_action_busy = 0;  // 0: 循迹控制 1: 动作接管

// ==================== 外部依赖变量 ====================
extern Navi_WayPoint_t point_map[NAVI_POINT_MAX]; 
extern Navi_Controller_t navi_ctrl;
extern float target_velocity; 
extern float target_angle;

// *******************************************************************************
// 动作参数
// ******************************************************************************
//--------------- 排雷旋转 ---------------
#define MINE_ROTATE_TARGET_DEG      1080.0f
#define MINE_ROTATE_LEAD_DEG        70.0f
#define MINE_ROTATE_TIMEOUT_MS      15000U
#define MINE_ROTATE_CCW_CMD         2U
static float mine_rotate_start_yaw = 0.0f;
static int8_t mine_rotate_dir = 1;
static float mine_rotate_target_deg = MINE_ROTATE_TARGET_DEG;
static uint8_t action_done_pending = 0;
static uint16_t action_done_idx = 0;
#if (NAVI_JUMP_ACTION_MODE == 2U)
static uint8_t jump_sequence_done_count = 0;
static float jump_course_back_yaw = 0.0f;
#endif
#if (NAVI_TRIPLE_JUMP_INTERVAL_MODE == 2U)
static float jump_interval_start_x = 0.0f;
static float jump_interval_start_y = 0.0f;
#endif
static float jump_sequence_hold_yaw = 0.0f;
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
static uint8_t jump_pose_update_active = 0;
#endif

uint8_t navigation_jump_is_active(void)
{
    return (action_fsm.state == FSM_JUMP_PREPARE ||
            action_fsm.state == FSM_JUMP_TAKEOFF ||
            action_fsm.state == FSM_JUMP_AIRBORNE ||
            action_fsm.state == FSM_JUMP_LANDING ||
            action_fsm.state == FSM_JUMP_TRIPLE_INTERVAL ||
            action_fsm.state == FSM_JUMP_RAMP_DOWN ||
            action_fsm.state == FSM_JUMP_TURN_BACK ||
            action_fsm.state == FSM_JUMP_RAMP_UP ||
            action_fsm.state == FSM_JUMP_STAIR_DOWN) ? 1U : 0U;
}

uint8_t Navi_Action_Servo_Takeover_Active(void)
{
    return (action_fsm.state == FSM_JUMP_PREPARE ||
            action_fsm.state == FSM_JUMP_TAKEOFF ||
            action_fsm.state == FSM_JUMP_AIRBORNE ||
            action_fsm.state == FSM_JUMP_LANDING) ? 1U : 0U;
}

#define NAVI_JUMP_FORWARD_SPEED       NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_AIRBORNE_SPEED      NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_LANDING_SPEED       NAVI_JUMP_RUNUP_SPEED
#define NAVI_JUMP_BURST_PWM           (1300)
#define NAVI_JUMP_AIR_RETRACT_X       (-0.00f)
#define NAVI_JUMP_AIR_RETRACT_Y       (0.015f)
#define NAVI_JUMP_EXE_BUFFER_X        (+0.00f)
#define NAVI_JUMP_EXE_BUFFER_Y        (0.035f)
#define NAVI_JUMP_RECOVER_PWM         (420)
#define NAVI_JUMP_PREPARE_MS          (260U)
#define NAVI_JUMP_BURST_MS            (180U)
#define NAVI_JUMP_AIR_RETRACT_MS      (40U)
#define NAVI_JUMP_RECOVER_MS          (50U)
#define NAVI_JUMP_LANDING_MAX_MS      (600U)
#define NAVI_JUMP_LAND_ACCEL_G        (1.0f)

#define NAVI_TRIPLE_JUMP_TOTAL_COUNT      (3U)
#define NAVI_TRIPLE_JUMP_INTERVAL_SPEED   NAVI_JUMP_RUNUP_SPEED
#define NAVI_TRIPLE_JUMP_INTERVAL_MS      (100U)
#define NAVI_TRIPLE_JUMP_INTERVAL_DX_M    (0.10f)

#if (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_TRIPLE_ONLY) && \
    (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
#error "Invalid NAVI_TRIPLE_JUMP_AFTER_MODE"
#endif

#if (NAVI_TRIPLE_JUMP_INTERVAL_MODE != 1U) && (NAVI_TRIPLE_JUMP_INTERVAL_MODE != 2U)
#error "Invalid NAVI_TRIPLE_JUMP_INTERVAL_MODE"
#endif

#if (NAVI_JUMP_POSE_UPDATE_MODE != 1U) && (NAVI_JUMP_POSE_UPDATE_MODE != 2U)
#error "Invalid NAVI_JUMP_POSE_UPDATE_MODE"
#endif

static void navi_jump_pose_update_begin(void)
{
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
    if (!jump_pose_update_active) {
        Navi_Set_Manual_Update_Mode(1);
        jump_pose_update_active = 1;
    }
#endif
}

static void navi_jump_pose_add_fixed_step(void)
{
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
    if (jump_pose_update_active) {
        Navi_Manual_Add_Pose(NAVI_JUMP_FIXED_FORWARD_M, NAVI_JUMP_FIXED_RIGHT_M, 1);
    }
#endif
}

static void navi_jump_pose_update_end(void)
{
#if (NAVI_JUMP_POSE_UPDATE_MODE == 2U)
    if (jump_pose_update_active) {
        Navi_Set_Manual_Update_Mode(0);
        jump_pose_update_active = 0;
    }
#endif
}

// ==============================================================================
// 全局路径预处理：提取需要动作接管的特殊航点
// ==============================================================================
void navi_parse_global_path(void) {
    action_seq.total_count = 0;
    action_seq.current_ptr = 0;
    action_fsm.state = FSM_IDLE;
    action_fsm.state_timer_ms = 0;
    action_fsm.is_airborne_expect = 0;
    is_action_busy = 0;
    action_done_pending = 0;
    action_done_idx = 0;
#if (NAVI_JUMP_ACTION_MODE == 2U)
    jump_sequence_done_count = 0;
    jump_course_back_yaw = 0.0f;
#endif
#if (NAVI_TRIPLE_JUMP_INTERVAL_MODE == 2U)
    jump_interval_start_x = 0.0f;
    jump_interval_start_y = 0.0f;
#endif
    jump_sequence_hold_yaw = 0.0f;
    navi_jump_pose_update_end();
    jump_engine_suspend = 0;

    for (int i = 0; i < navi_ctrl.point_total_count; i++) {
        if (point_map[i].type == WP_TYPE_MINE_SWEEP ||  point_map[i].type == WP_TYPE_JUMP)  {
            if (point_map[i].type == WP_TYPE_JUMP) {
                point_map[i].action_cmd = NAVI_JUMP_ACTION_MODE;
            }
            action_seq.list[action_seq.total_count].wp_index = i;
            action_seq.list[action_seq.total_count].type = point_map[i].type;
            action_seq.total_count++;
            if (action_seq.total_count >= MAX_ACTION_NUM) break;
        }
    }
}

// ==============================================================================
// 异步动作状态机
// ==============================================================================
static void navi_action_fsm_update(uint16_t target_idx, float distance) {
    WayPoint_Type upcoming_type = point_map[target_idx].type;
    action_fsm.state_timer_ms += (uint32_t)(ENCODER_DT * 1000.0f);
    // 注意：状态切换时要清零 state_timer_ms。

    switch (action_fsm.state) {
        case FSM_IDLE:
            // 空闲期用于提前预测特殊动作，并恢复通用动作标志。
            action_fsm.is_airborne_expect = 0;
            navi_jump_pose_update_end();
            jump_engine_suspend = 0;
//                is_action_busy = 1;                              // 接管控制权
            
            // ==============================================================================
            // 提前预测区：根据即将到达的特殊航点选择动作分支。
            // ==============================================================================
            if (upcoming_type == WP_TYPE_MINE_SWEEP && navi_isreach_target_point(target_idx))  {                        // 排雷动作
                action_fsm.state = FSM_MINE_APPROACH; 
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;         // 锁定动作接管
            }
            else if (upcoming_type == WP_TYPE_JUMP && distance < (DISTANCE_THRESHOLD * 3.0f)) {                        // 跳跃动作
                point_map[target_idx].action_cmd = NAVI_JUMP_ACTION_MODE;
#if (NAVI_JUMP_ACTION_MODE == 0U)
                action_fsm.state = FSM_MINE_APPROACH;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
#elif (NAVI_JUMP_ACTION_MODE == 1U)
                jump_sequence_hold_yaw = (float)IMU_data.filter_result.yaw;
                action_fsm.state = FSM_JUMP_RUNUP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                target_velocity = NAVI_JUMP_RUNUP_SPEED;
                target_angle = jump_sequence_hold_yaw;
#elif (NAVI_JUMP_ACTION_MODE == 2U)
                jump_sequence_done_count = 0;
                jump_course_back_yaw = 0.0f;
                jump_sequence_hold_yaw = (float)IMU_data.filter_result.yaw;
                action_fsm.state = FSM_JUMP_RUNUP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                target_velocity = NAVI_JUMP_RUNUP_SPEED;
                target_angle = jump_sequence_hold_yaw;
#else
#error "Invalid NAVI_JUMP_ACTION_MODE"
#endif
            }
            else if (upcoming_type == WP_TYPE_BRIDGE && distance < (DISTANCE_THRESHOLD * 8.0f)) {
                action_fsm.state = FSM_BRIDGE_APPROACH;
            }
            else if (upcoming_type == WP_TYPE_CONE_CONE && distance < (DISTANCE_THRESHOLD * 8.0f)) {
                action_fsm.state = FSM_CONE_APPROACH;
            }
            else if (upcoming_type == WP_TYPE_STOP && distance < (DISTANCE_THRESHOLD * 8.0f)) {
                action_fsm.state = FSM_STOP_PARKING;
            }
            else if (upcoming_type == WP_TYPE_NORMAL && distance < (DISTANCE_THRESHOLD * 2.0f)) {
                action_fsm.state = FSM_IDLE;
            }
            break;
            
            
        // ------------------ 排雷旋转动作 ------------------
      case FSM_MINE_APPROACH:
            // 初始化排雷/测试旋转动作。
            mine_rotate_start_yaw = (float)robot_pose.cumulative_yaw;
            mine_rotate_dir = (point_map[target_idx].action_cmd == MINE_ROTATE_CCW_CMD) ? -1 : 1;
            mine_rotate_target_deg = (point_map[target_idx].type == WP_TYPE_JUMP) ? 360.0f : MINE_ROTATE_TARGET_DEG;
            is_action_busy = 1;
            target_velocity = 0.0f;
            
            target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * MINE_ROTATE_LEAD_DEG);

            IPC_LOG_Printf("\r\n============= >>> [定点排雷] 已到达旋转点 [%d]，开始%s旋转三圈 <<< =============\r\n",
                   target_idx,
                   mine_rotate_dir > 0 ? "顺时针" : "逆时针");
            
            // 进入持续旋转执行状态。
            action_fsm.state = FSM_MINE_PROCESSING;
            action_fsm.state_timer_ms = 0;
            break;

        case FSM_MINE_PROCESSING:
        {
            float rotated_deg;
            float rotated_abs;
            float remaining_deg;
            float rotate_lead_deg;

            target_velocity = 0.0f;
            
            rotated_deg = ((float)robot_pose.cumulative_yaw - mine_rotate_start_yaw) * mine_rotate_dir;
            rotated_abs = fabsf(rotated_deg);
            remaining_deg = mine_rotate_target_deg - rotated_abs;
            rotate_lead_deg = (remaining_deg < MINE_ROTATE_LEAD_DEG) ? remaining_deg : MINE_ROTATE_LEAD_DEG;

            // 旋转角度达到目标，动作完成。
            if (rotated_abs >= mine_rotate_target_deg)
            {
                IPC_LOG_Printf(" [定点排雷] 三圈旋转完成，退出动作，回到休闲状态。\r\n");
                
                target_velocity = 0.0f;
                target_angle = (float)IMU_data.filter_result.yaw;
                action_done_pending = 1;
                action_done_idx = target_idx;
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                    action_seq.current_ptr++;
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
            }
            else
            {
                target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - mine_rotate_dir * rotate_lead_deg);
            }
            // 超时保护：避免动作永久接管。
            if (action_fsm.state_timer_ms > MINE_ROTATE_TIMEOUT_MS)
            {
                IPC_LOG_Printf(" [定点排雷] 三圈旋转超时退出，安全回退到休闲状态。\r\n");
                
                // 超时同样认为动作结束，交还循迹层切点。
                target_velocity = 0.0f;
                target_angle = (float)IMU_data.filter_result.yaw;
                action_done_pending = 1;
                action_done_idx = target_idx;
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                    action_seq.current_ptr++;
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
            }
            break;
        }
        // ------------------ 跳跃动作 ------------------
        case FSM_JUMP_RUNUP:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_RUNUP_SPEED;
            target_angle = jump_sequence_hold_yaw;

            if (action_fsm.state_timer_ms >= NAVI_JUMP_RUNUP_MS) {
                navi_jump_pose_update_begin();
                action_fsm.state = FSM_JUMP_PREPARE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_PREPARE:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_stop = 0;
            jump_position = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_FORWARD_SPEED;
#if (NAVI_JUMP_ACTION_MODE == 2U)
            target_angle = jump_sequence_hold_yaw;
#endif
            jump_drive_symmetric_pwm(jump_calc_prepare_pwm((uint16)action_fsm.state_timer_ms));

            if (action_fsm.state_timer_ms >= NAVI_JUMP_PREPARE_MS) {
                action_fsm.state = FSM_JUMP_TAKEOFF;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_TAKEOFF:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_FORWARD_SPEED;
#if (NAVI_JUMP_ACTION_MODE == 2U)
            target_angle = jump_sequence_hold_yaw;
#endif
            jump_drive_symmetric_pwm(NAVI_JUMP_BURST_PWM);

            if (action_fsm.state_timer_ms >= NAVI_JUMP_BURST_MS) {
                action_fsm.state = FSM_JUMP_AIRBORNE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                action_fsm.is_airborne_expect = 1;
            }
            break;

        case FSM_JUMP_AIRBORNE:
        {
            uint32_t buffer_ms;

            is_action_busy = 1;
            action_fsm.is_airborne_expect = 1;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_AIRBORNE_SPEED;
#if (NAVI_JUMP_ACTION_MODE == 2U)
            target_angle = jump_sequence_hold_yaw;
#endif

            if (action_fsm.state_timer_ms < NAVI_JUMP_AIR_RETRACT_MS) {
                jump_drive_symmetric_xy(NAVI_JUMP_AIR_RETRACT_X, NAVI_JUMP_AIR_RETRACT_Y);
            } else {
                jump_drive_symmetric_xy(NAVI_JUMP_EXE_BUFFER_X, NAVI_JUMP_EXE_BUFFER_Y);
                buffer_ms = action_fsm.state_timer_ms - NAVI_JUMP_AIR_RETRACT_MS;
                if (IMU_data.accel[2] >= (1.5f * NAVI_JUMP_LAND_ACCEL_G) || buffer_ms >= NAVI_JUMP_LANDING_MAX_MS) {
                    action_fsm.state = FSM_JUMP_LANDING;
                    action_fsm.state_timer_ms = 0;
                    is_action_busy = 1;
                    action_fsm.is_airborne_expect = 0;
                }
            }
            break;
        }

        case FSM_JUMP_LANDING:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            target_velocity = NAVI_JUMP_LANDING_SPEED;
#if (NAVI_JUMP_ACTION_MODE == 2U)
            target_angle = jump_sequence_hold_yaw;
#endif
            jump_drive_symmetric_pwm(NAVI_JUMP_RECOVER_PWM);

            if (action_fsm.state_timer_ms >= NAVI_JUMP_RECOVER_MS) {
                navi_jump_pose_add_fixed_step();
                navi_jump_pose_update_end();
#if (NAVI_JUMP_ACTION_MODE == 2U)
                jump_sequence_done_count++;
                if (jump_sequence_done_count < NAVI_TRIPLE_JUMP_TOTAL_COUNT) {
#if (NAVI_TRIPLE_JUMP_INTERVAL_MODE == 2U)
                    jump_interval_start_x = robot_pose.x;
                    jump_interval_start_y = robot_pose.y;
#endif
                    action_fsm.state = FSM_JUMP_TRIPLE_INTERVAL;
                    action_fsm.state_timer_ms = 0;
                    is_action_busy = 1;
                    action_fsm.is_airborne_expect = 0;
                    jump_engine_suspend = 0;
                    jump_position = 0;
                    jump_stop = 0;
                    break;
                }
#if (NAVI_TRIPLE_JUMP_AFTER_MODE == NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
                jump_course_back_yaw = navi_limit_angle180(jump_sequence_hold_yaw + 180.0f);
                action_fsm.state = FSM_JUMP_RAMP_DOWN;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
                action_fsm.is_airborne_expect = 0;
                jump_engine_suspend = 0;
                jump_position = 0;
                jump_stop = 0;
                break;
#endif
#endif
#if (NAVI_JUMP_ACTION_MODE != 2U) || (NAVI_TRIPLE_JUMP_AFTER_MODE != NAVI_TRIPLE_JUMP_AFTER_FULL_COURSE)
                action_done_pending = 1;
                action_done_idx = target_idx;
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                    action_seq.current_ptr++;
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
                action_fsm.is_airborne_expect = 0;
                jump_engine_suspend = 0;
                jump_position = 0;
                jump_stop = 0;
#endif
            }
            break;

        case FSM_JUMP_TRIPLE_INTERVAL:
        {
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_INTERVAL_SPEED;
            target_angle = jump_sequence_hold_yaw;

#if (NAVI_TRIPLE_JUMP_INTERVAL_MODE == 1U)
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_INTERVAL_MS) {
                action_fsm.state = FSM_JUMP_RUNUP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
#elif (NAVI_TRIPLE_JUMP_INTERVAL_MODE == 2U)
            {
                float dx = robot_pose.x - jump_interval_start_x;
                float dy = robot_pose.y - jump_interval_start_y;
                float interval_dist_sq = dx * dx + dy * dy;
                float target_dist_sq = NAVI_TRIPLE_JUMP_INTERVAL_DX_M * NAVI_TRIPLE_JUMP_INTERVAL_DX_M;

                if (interval_dist_sq >= target_dist_sq) {
                    action_fsm.state = FSM_JUMP_RUNUP;
                    action_fsm.state_timer_ms = 0;
                    is_action_busy = 1;
                }
            }
#endif
            break;
        }
        case FSM_JUMP_RAMP_DOWN:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_RAMP_SPEED;
            target_angle = jump_sequence_hold_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_RAMP_DOWN_MS) {
                action_fsm.state = FSM_JUMP_TURN_BACK;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_TURN_BACK:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = 0.0f;
            target_angle = jump_course_back_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_TURN_BACK_MS) {
                action_fsm.state = FSM_JUMP_RAMP_UP;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_RAMP_UP:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_RAMP_SPEED;
            target_angle = jump_course_back_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_RAMP_UP_MS) {
                action_fsm.state = FSM_JUMP_STAIR_DOWN;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 1;
            }
            break;

        case FSM_JUMP_STAIR_DOWN:
            is_action_busy = 1;
            action_fsm.is_airborne_expect = 0;
            jump_engine_suspend = 0;
            jump_position = 0;
            jump_stop = 0;
            target_velocity = NAVI_TRIPLE_JUMP_STAIR_SPEED;
            target_angle = jump_course_back_yaw;
            if (action_fsm.state_timer_ms >= NAVI_TRIPLE_JUMP_STAIR_DOWN_MS) {
                action_done_pending = 1;
                action_done_idx = target_idx;
                if (action_seq.current_ptr < action_seq.total_count &&
                    action_seq.list[action_seq.current_ptr].wp_index == target_idx) {
                    action_seq.current_ptr++;
                }
                action_fsm.state = FSM_IDLE;
                action_fsm.state_timer_ms = 0;
                is_action_busy = 0;
                action_fsm.is_airborne_expect = 0;
                jump_engine_suspend = 0;
                jump_position = 0;
                jump_stop = 0;
            }
            break;

        // ------------------ 桥梁动作 ------------------
        case FSM_BRIDGE_APPROACH:
            // 预留：接近桥梁时的减速、姿态准备等动作。
                      
            break;

        case FSM_BRIDGE_ON_BOARD:
            // 预留：桥面行驶时的腿长和姿态自适应动作。
                      
            break;

        // ------------------ 绕锥桶动作 ------------------
        case FSM_CONE_APPROACH:
            // 预留：接近锥桶时的减速和转向准备。
                      
            break;

        case FSM_CONE_NAVIGATE:
            // 预留：绕行锥桶时的路径/角度控制。
                      
            break;

        // ------------------ 终点停车 ------------------
        case FSM_STOP_PARKING:
            target_velocity = 0.0f;
            vofa_mode_driver = 0.0f;
            navi_ctrl.navi_mode_driver = 0; 

            static uint8_t end_printed_flag = 0;
            if (!end_printed_flag) {
            IPC_LOG_Printf("=============  >>> [事件] 终点已到达，动作接管并安全停车！ =============\r\n");                  
            end_printed_flag = 1;
            }
            break;

        default:
            action_fsm.state = FSM_IDLE;
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


// ==============================================================================
// 动作管理入口：由循迹层周期调用
// ==============================================================================
void Navi_Action_Manager(uint16_t  curr_idx) {
    if (action_seq.total_count == 0 || action_seq.current_ptr >= action_seq.total_count) return; 

    uint16_t  target_wp_idx = action_seq.list[action_seq.current_ptr].wp_index;
    
    // 动作接管期间直接更新 FSM，不做普通切点逻辑。
    if (is_action_busy) {
        float real_distance = 0.0;
        float dummy_azimuth = 0.0;
        navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance);
        navi_action_fsm_update(target_wp_idx, real_distance);
        return; 
    }
                                                            
    if (curr_idx != target_wp_idx) {
        return;
    }

    target_wp_idx = action_seq.list[action_seq.current_ptr].wp_index;
    
    // 周期性更新动作 FSM。
        float real_distance = 0.0f;
        float dummy_azimuth = 0.0f;
        if (navi_calcnavinfo(target_wp_idx, &dummy_azimuth, &real_distance)) {
            navi_action_fsm_update(target_wp_idx, real_distance);
        }
}



//===========================================================================================================
//================================================动作封装函数=======================================================
//========================================================================================================



