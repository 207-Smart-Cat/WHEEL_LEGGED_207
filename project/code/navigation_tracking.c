/********************************************************************************************************************* 

*先指定航点类型，再切换航点标志

* 打滑代码使用了加速度acc_x  ，具体方向需要根据实际更改             

* 坐标x轴正北，y轴正东。顺时针时yaw增加    

*需要确认 IMU_data.filter_result.yaw 读出来确实是 0~360 或180$ 且顺时针增加。

*yaw_filter 的范围：-180 到 180





，EKF 内部的所有的状态量、控制量、观测量，必须全部统一为 国际标准单位（SI：米 $m$、秒 $s$、弧度 $rad$）。

*********************************************************************************************************************/
#include "navigation_tracking.h"

#include "navigation_data_handling.h"

#include "control.h"

#include "navigation_action.h"

#include "imu.h"

#include "param.h"

#include "ipc_shared_data.h"

#include "wifi.h" // 包含 LOG_Printf 支持串口和WiFi双路透传

//====================================================全局变量定义=======================================================

Navi_Controller_t navi_ctrl;

Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // foreground map currently used by navigation

static Navi_WayPoint_t record_point_map[NAVI_POINT_MAX];      // background map recorded manually
static uint16_t record_point_count = 0;

static Navi_WayPoint_t static_point_map[NAVI_POINT_MAX];      // built-in static test map
static uint16_t static_point_count = 0;

float wifi_cmd_trigger = 0.0;        // 摇铃变量         0: 待机, 1: 追加一个航点, 2: 清空当前地图, 3: 立刻接管执行动作

float wifi_remote_type= 0.0;                                  //赋值航点类型,按照枚举依次从0到···

float wifi_in_action= 0.0;            //动作指令    

float vofa_trigger_record = 0.0f;

//====================================================全局变量定义=======================================================
float vofa_mode_driver = 0.0f;
float vofa_mode_map = 0.0f;
float vofa_print_pose_en = 0.0f;
float vofa_print_pose_period = 3000.0f; // 默认 1000ms 打印一次
float vofa_reserved_1 = 0.0f;
float vofa_reserved_2 = 0.0f;


      
// =========================================================



/*腿部姿态设置*/

//====================================================全局静态变量定义=======================================================

//====================================================变量声明=======================================================

extern RobotState_t robot_pose;                                  // 全局实时位姿

//extern Navi_Sensor_Data_t       filter_data;                       //基础滤波+处理后数据

extern IMU_t IMU_data;

extern uint8_t  is_action_busy ;       // 0:循迹控制         1:动作接管

extern float now_velocity;       //当前速度(线速度)

//====================================================函数声明=============================================

static void navi_speed_profile_reset(void);
static float navi_update_tracking_velocity(float distance, uint8_t reached, uint8_t nav_valid);



//====================================================具体函数编写层=============================================

static pid_param_t navi_speed_pid;
static float navi_speed_last_output = 0.0f;
static void navi_speed_profile_reset(void)
{
    PidInit(&navi_speed_pid);
    navi_speed_last_output = 0.0f;
}

static float navi_update_tracking_velocity(float distance, uint8_t reached, uint8_t nav_valid)
{
    uint8_t use_default_params = (navi_speed_kp == 0.0f && navi_speed_ki == 0.0f && navi_speed_kd == 0.0f &&
                                  navi_speed_max == 0.0f && navi_speed_max_step == 0.0f);
    float speed_kp = use_default_params ? Navi_Speed_Kp_init : navi_speed_kp;
    float speed_ki = use_default_params ? Navi_Speed_Ki_init : navi_speed_ki;
    float speed_kd = use_default_params ? Navi_Speed_Kd_init : navi_speed_kd;
    float max_velocity = (navi_speed_max > 0.0f) ? navi_speed_max : Navi_Speed_Max_init;
    float max_step = (navi_speed_max_step > 0.0f) ? navi_speed_max_step : Navi_Speed_MaxStep_init;
    float error = distance;
    float raw_output;
    float delta;

    if (!nav_valid) {
        navi_speed_profile_reset();
        return 0.0f;
    }
    if (reached) {
        navi_speed_profile_reset();
        return 0.0f;
    }
    if (distance <= DISTANCE_THRESHOLD) {
        navi_speed_profile_reset();
        return 0.0f;
    }

    if (error < 0.0f) {
        error = 0.0f;
    }
    if (max_velocity < 0.0f) {
        max_velocity = 0.0f;
    }
    if (max_step <= 0.0f) {
        max_step = Navi_Speed_MaxStep_init;
    }
    if (max_velocity <= 0.0f) {
        navi_speed_profile_reset();
        return 0.0f;
    }

    PidChange(&navi_speed_pid, speed_kp, speed_ki, speed_kd);
    raw_output = PidLocCtrl(&navi_speed_pid, error);
    raw_output = constrain_float(raw_output, 0.0f, max_velocity);

    delta = constrain_float(raw_output - navi_speed_last_output, -max_step, max_step);
    navi_speed_last_output = constrain_float(navi_speed_last_output + delta, 0.0f, max_velocity);

    return navi_speed_last_output;
}

//初始化
void Navi_Tracking_Init(void) {
    navi_ctrl.point_total_count = 0;
    navi_ctrl.point_current_idx = 0;
    navi_ctrl.navi_mode_map = 0;
    navi_ctrl.origin_set_flag = 0;
    navi_ctrl.trigger_record = 0;

    record_point_count = 0;
    static_point_count = 0;
    memset(point_map, 0, sizeof(point_map));
    memset(record_point_map, 0, sizeof(record_point_map));
    memset(static_point_map, 0, sizeof(static_point_map));

    navi_load_comprehensive_test_map();
}

//-------------------------------------------------------------------------------------------------------------------

//  路径预处理：线性插值

//当两点间距 >DISTANCE_THRESHOLD 时，自动插入中间点，确保前瞻逻辑平滑

//-------------------------------------------------------------------------------------------------------------------

void navi_path_optimize(void) {

    uint8_t count = navi_ctrl.point_total_count;

    if (count < 2) return;

    for (uint8_t i = 0; i < count - 1; i++) {

        float dist = navi_get_two_points_distance( point_map[i].x , point_map[i].y,  point_map[i+1].x , point_map[i+1].y ) ;
        
        // 如果间距超过 0.4米 且 数组未满，插入一个中点

        if (dist > INTERPOLATION_STEP && navi_ctrl.point_total_count < NAVI_POINT_MAX) {

            // 整体后移腾出空间

            for (uint8_t j = navi_ctrl.point_total_count; j > i + 1; j--) {

                point_map[j] = point_map[j-1];

            }
            
            // 坐标线性插值
            point_map[i+1].x = (point_map[i].x + point_map[i+2].x) / 2.0f;
            point_map[i+1].y = (point_map[i].y + point_map[i+2].y) / 2.0f;
            
            // 角度插值（处理过零点）
            float angle_diff = navi_limit_angle180(point_map[i+2].yaw - point_map[i].yaw);
            point_map[i+1].yaw = navi_limit_angle180(point_map[i].yaw + angle_diff * 0.5f);

            
            point_map[i+1].type = WP_TYPE_NORMAL;
            point_map[i+1].valid = 1;
            
            navi_ctrl.point_total_count++;

            count++;

            i++; // 跳过新生成的点

        }

    }

}


//-------------------------------------------------------------------------------------------------------------------

// 记录当前Navi位置为航点          task_navigation_control函数中已调用 

// 首次调用时设置原点，后续调用记录指定类型的航点

// type: 航点类型（WP_TYPE_HOME/WP_TYPE_NORMAL/WP_TYPE_STOP/WP_TYPE_OBSTACLE）

//-------------------------------------------------------------------------------------------------------------------
void navi_auto_record_task(void) {
    static float last_vofa_trigger = 0.0f;

    if (navi_ctrl.navi_mode_driver != 2 || navi_ctrl.navi_mode_map != 1 || !robot_pose.is_valid) {
        last_vofa_trigger = vofa_trigger_record;
        return;
    }

    if (vofa_trigger_record > 0.5f && last_vofa_trigger <= 0.5f) {
        navi_ctrl.trigger_record = 1;
        vofa_trigger_record = 0.0f;
    }
    last_vofa_trigger = vofa_trigger_record;

    if (!navi_ctrl.trigger_record) {
        return;
    }

    if (record_point_count >= NAVI_POINT_MAX) {
        navi_ctrl.trigger_record = 0;
        IPC_LOG_Printf("\r\n>>> [打点失败] 记录地图已满，无法继续添加航点 <<<\r\n");
        return;
    }

    if (navi_ctrl.origin_set_flag == 0) {
        Navi_Data_Set_Origin();
        memset(record_point_map, 0, sizeof(record_point_map));

        record_point_map[0].x = 0.0f;
        record_point_map[0].y = 0.0f;
        record_point_map[0].yaw = 0.0f;
        record_point_map[0].type = WP_TYPE_HOME;
        record_point_map[0].action_cmd = 0;
        record_point_map[0].valid = 1;

        record_point_count = 1;
        navi_ctrl.origin_set_flag = 1;
        IPC_LOG_Printf("\r\n>>> [手动打点] 第1次触发，坐标系原点(Home)已成功建立 <<<\r\n");
    } else {
        uint16_t idx = record_point_count;

        record_point_map[idx].x = robot_pose.x;
        record_point_map[idx].y = robot_pose.y;
        record_point_map[idx].yaw = robot_pose.yaw;
        record_point_map[idx].type = (WayPoint_Type)wifi_remote_type;
        record_point_map[idx].action_cmd = (uint16_t)wifi_in_action;
        record_point_map[idx].valid = 1;
        record_point_count++;

        IPC_LOG_Printf(" >>> [手动打点] 记录点[%03d]: X=%s%d.%02d, Y=%s%d.%02d | 类型:%s | 动作指令:%d <<<\r\n",
            idx,
            F_ARG(record_point_map[idx].x),
            F_ARG(record_point_map[idx].y),
            get_enum_name(record_point_map[idx].type),
            record_point_map[idx].action_cmd);
    }

    navi_ctrl.trigger_record = 0;
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//静态地图导入 (精度测定专用)
// 目的：直接赋值坐标点，检查小车跑坐标单位1时，实际物理位移
//-------------------------------------------------------------------------------------------------------------------
void navi_load_comprehensive_test_map(void) {
    static_point_count = 0;
    memset(static_point_map, 0, sizeof(static_point_map));

    uint8_t i = 0;

    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f,   0.0f, WP_TYPE_HOME,   0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.2f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){1.2f,  0.6f,  90.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.6f, 180.0f, WP_TYPE_NORMAL, 0, 1};
    static_point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f, 270.0f, WP_TYPE_STOP,   0, 1};

    static_point_count = i;
}

//-------------------------------------------------------------------------------------------------------------------

//   WiFi动态插值

//-------------------------------------------------------------------------------------------------------------------
void navi_wifi_remote_cmd(void) {
  
    uint8_t  cmd_trigger = (uint8_t)  wifi_cmd_trigger;
  
    if (cmd_trigger == 0) return; // 没人摇铃，直接退出

    switch (cmd_trigger) {
        case 1: // 1: 追加单个航点到点阵中
            if (navi_ctrl.point_total_count < NAVI_POINT_MAX) {

            }
            break;

        case 2: // 2: 清空现有地图 (发新地图前先发这个)
            navi_ctrl.point_total_count = 0;
            navi_ctrl.point_current_idx = 0;
            navi_ctrl.origin_set_flag = 0;
            break;

//        case 3: // 3: 无视航点，立刻强制执行轮腿跨越/跳跃动作 ,  直接将动作代码丢给 action.c 的状态机
//            action_fsm.state = (ActionState_e)wifi_in_action;
//            is_action_busy = 1; 
//            break;
    }

    // 处理完毕，清空摇铃，等待下一次注入
    wifi_cmd_trigger = 0; 
}



//-------------------------------------------------------------------------------------------------------------------

//   导航模式识别         2：遥控器打点模式     1：已有航点，巡航模式           0：停止

//-------------------------------------------------------------------------------------------------------------------

void task_navigation_control(void) {
    static uint8_t last_mode_driver = 0;
    static uint8_t last_mode_map = 255;
    static uint8_t last_print_cmd = 0;
    static uint8_t active_nav_map_type = 0;
    static uint8_t is_printing_map = 0;
    static uint16_t print_map_idx = 0;
    static uint8_t preview_map_type = 0;
    static uint8_t preview_print_phase = 0;
    static uint8_t preview_print_wait = 0;
    static uint8_t end_printed_flag = 0;

    navi_ctrl.navi_mode_driver = (uint8_t)vofa_mode_driver;
    navi_ctrl.navi_mode_map = (uint8_t)vofa_mode_map;
    uint8_t current_print_cmd = (uint8_t)vofa_print_pose_en;

    if (last_mode_driver != navi_ctrl.navi_mode_driver || last_mode_map != navi_ctrl.navi_mode_map) {
        if (last_mode_driver == 1 && navi_ctrl.navi_mode_driver != 1) {
#if (USE_HOST_TARGET_VELOCITY == 0)
            target_velocity = 0.0f;
#endif
            navi_speed_profile_reset();
        }

        if (navi_ctrl.navi_mode_map == 0 || navi_ctrl.navi_mode_map == 1) {
            active_nav_map_type = navi_ctrl.navi_mode_map;
        }

        if (navi_ctrl.navi_mode_map == 2 && navi_ctrl.navi_mode_driver != 1) {
            record_point_count = 0;
            navi_ctrl.origin_set_flag = 0;
            navi_ctrl.point_total_count = 0;
            navi_ctrl.point_current_idx = 0;
            memset(record_point_map, 0, sizeof(record_point_map));
            memset(point_map, 0, sizeof(point_map));
            IPC_LOG_Printf("\r\n============= >>> [地图清空] 后台记录地图已成功清空 <<< =============\r\n");
        }

        if (navi_ctrl.navi_mode_driver == 0) {
            navi_speed_profile_reset();
            if ((navi_ctrl.navi_mode_map == 0 || navi_ctrl.navi_mode_map == 1) && last_mode_map != navi_ctrl.navi_mode_map) {
                IPC_LOG_Printf("\r\n[地图选择] 当前执行地图切换为: %s\r\n", active_nav_map_type == 0 ? "静态地图" : "记录地图");
            }
            if (last_mode_driver != 0) {
                IPC_LOG_Printf(">>> [状态] 系统进入停止/地图选择模式 <<<\r\n");
            }
        } else if (navi_ctrl.navi_mode_driver == 1 && last_mode_driver != 1) {
            memset(point_map, 0, sizeof(point_map));
            navi_ctrl.point_current_idx = 0;

            if (active_nav_map_type == 0) {
                memcpy(point_map, static_point_map, sizeof(static_point_map));
                navi_ctrl.point_total_count = static_point_count;
                IPC_LOG_Printf("\r\n[导航启动] 已加载【静态地图】，共 %d 个航点\r\n", navi_ctrl.point_total_count);
            } else {
                memcpy(point_map, record_point_map, sizeof(record_point_map));
                navi_ctrl.point_total_count = record_point_count;
                IPC_LOG_Printf("\r\n[导航启动] 已加载【记录地图】，共 %d 个航点\r\n", navi_ctrl.point_total_count);
            }

#if ENABLE_PATH_INTERPOLATION
            if (navi_ctrl.point_total_count >= 2) {
                navi_path_optimize();
                IPC_LOG_Printf(" [路径加密] 已开启线性插值路径加密，当前总点数: %d\r\n", navi_ctrl.point_total_count);
            }
#endif

            Navi_Data_Set_Origin();
            navi_parse_global_path();
            end_printed_flag = 0;
            navi_speed_profile_reset();
            is_action_busy = 0;
        } else if (navi_ctrl.navi_mode_driver == 2) {
            if (navi_ctrl.navi_mode_map == 1 && (last_mode_driver != 2 || last_mode_map != 1)) {
                record_point_count = 0;
                navi_ctrl.origin_set_flag = 0;
                navi_ctrl.trigger_record = 0;
                memset(record_point_map, 0, sizeof(record_point_map));
                Navi_Data_Set_Origin();
                IPC_LOG_Printf("\r\n[状态切换] 进入手动打点模式，后台记录地图已清空，下一次打点会建立 HOME 原点\r\n");
            }
        }

        last_mode_driver = navi_ctrl.navi_mode_driver;
        last_mode_map = navi_ctrl.navi_mode_map;
    }

    if (current_print_cmd == 2 && last_print_cmd != 2) {
        preview_map_type = (navi_ctrl.navi_mode_map == 2) ? active_nav_map_type : navi_ctrl.navi_mode_map;
        is_printing_map = 1;
        print_map_idx = 0;
        preview_print_phase = 0;
        preview_print_wait = 0;
    }
    last_print_cmd = current_print_cmd;

    if (is_printing_map) {
        if (preview_print_wait > 0) {
            preview_print_wait--;
        } else {
            preview_print_wait = 5;

            if (preview_print_phase == 0) {
                IPC_LOG_Printf("\r\n================ 地图预览：%s ================\r\n",
                    preview_map_type == 0 ? "静态地图" : "记录地图");
                preview_print_phase = 1;
            } else if (preview_print_phase == 1 && ((preview_map_type == 0 && static_point_count == 0) || (preview_map_type == 1 && record_point_count == 0))) {
                IPC_LOG_Printf("[地图预览失败] 当前选择的地图为空\r\n");
                preview_print_phase = 2;
            } else if (preview_print_phase == 1) {
                if (preview_map_type == 0 && print_map_idx < static_point_count) {
                    IPC_LOG_Printf(" 航点[%02d]: X=%s%d.%02d, Y=%s%d.%02d, 航向=%s%d.%02d, 类型=%s\r\n",
                        print_map_idx,
                        F_ARG(static_point_map[print_map_idx].x),
                        F_ARG(static_point_map[print_map_idx].y),
                        F_ARG(static_point_map[print_map_idx].yaw),
                        get_enum_name(static_point_map[print_map_idx].type));
                    print_map_idx++;
                } else if (preview_map_type == 1 && print_map_idx < record_point_count) {
                    IPC_LOG_Printf(" 航点[%02d]: X=%s%d.%02d, Y=%s%d.%02d, 航向=%s%d.%02d, 类型=%s\r\n",
                        print_map_idx,
                        F_ARG(record_point_map[print_map_idx].x),
                        F_ARG(record_point_map[print_map_idx].y),
                        F_ARG(record_point_map[print_map_idx].yaw),
                        get_enum_name(record_point_map[print_map_idx].type));
                    print_map_idx++;
                } else {
                    preview_print_phase = 2;
                }
            } else {
                IPC_LOG_Printf("==================================================\r\n");
                is_printing_map = 0;
            }
        }
    }
    switch(navi_ctrl.navi_mode_driver)
    {
        case 0:
            navi_speed_profile_reset();
            Navigation_Pose_Monitor_Task();
            break;

        case 1: {
            uint8_t total_points = navi_ctrl.point_total_count;
            uint8_t curr_idx = navi_ctrl.point_current_idx;
            if (total_points == 0) {
#if (USE_HOST_TARGET_VELOCITY == 0)
                target_velocity = 0.0f;
#endif
                navi_speed_profile_reset();
                break;
            }

            uint8_t lookahead_idx = curr_idx;
            double azimuth = 0.0, distance = 0.0;
            float print_turn_angle = 0.0f;
            float smooth_turn = 0.0f;
            uint8_t nav_info_valid = navi_calcnavinfo(lookahead_idx, &azimuth, &distance);
            uint8_t reached_current = navi_isreach_target_point(curr_idx);

            if (nav_info_valid) {
                float need_to_turn = navi_limit_angle180((float)azimuth - robot_pose.yaw);
                float k = (distance < 0.5f) ? 0.0f : 0.0f;
                smooth_turn = (1.0f - k) * need_to_turn;
                print_turn_angle = smooth_turn;
            }

            Navi_Action_Manager(navi_ctrl.point_current_idx);
            if (!is_action_busy) {
                target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - smooth_turn);
#if (USE_HOST_TARGET_VELOCITY == 0)
                target_velocity = navi_update_tracking_velocity((float)distance, reached_current, nav_info_valid);
#endif
            } else {
                navi_speed_profile_reset();
            }

#if WEIZHIJIANCE
            if (vofa_print_pose_en > 0.5f && current_print_cmd != 2) {
                static uint16_t dynamic_print_delay = 0;
                float current_period = vofa_print_pose_period;
                if (current_period < 500.0f) {
                    current_period = 500.0f;
                }

                uint16_t target_delay_ticks = (uint16_t)(current_period / (ENCODER_DT * 1000.0f));
                if (target_delay_ticks == 0) {
                    target_delay_ticks = 50;
                }

                if (++dynamic_print_delay >= target_delay_ticks) {
                    dynamic_print_delay = 0;
                    IPC_LOG_Printf("[位姿监测] 车姿(%s%d.%02d, %s%d.%02d) | Yaw=%s%d.%02d | 去往前瞻点(%s%d.%02d, %s%d.%02d) | 距目标:%s%d.%02d | 需转向:%s%d.%02d度 | 类型:%s\r\n",
                        F_ARG(robot_pose.x), F_ARG(robot_pose.y), F_ARG(robot_pose.yaw),
                        F_ARG(point_map[lookahead_idx].x), F_ARG(point_map[lookahead_idx].y),
                        F_ARG((float)distance), F_ARG(print_turn_angle),
                        get_enum_name(point_map[lookahead_idx].type));
                }
            }
#endif

            if (!is_action_busy && reached_current) {
                if (curr_idx >= (total_points - 1U)) {
                    if (end_printed_flag == 0) {
                        IPC_LOG_Printf("\r\n============= >>> [到达事件] 已到达终点航点 [%d] <<< =============\r\n", curr_idx);
                        IPC_LOG_Printf("=============  >>> [事件] 路线终点已到达，导航结束！ =============\r\n");
                        end_printed_flag = 1;
                    }
#if (USE_HOST_TARGET_VELOCITY == 0)
                    target_velocity = 0.0f;
#endif
                    vofa_mode_driver = 0.0f;
                    navi_ctrl.navi_mode_driver = 0;
                    navi_speed_profile_reset();
                    break;
                }

                IPC_LOG_Printf("\r\n============= >>> [到达事件] 已到达航点 [%d] <<< =============\r\n", curr_idx);
                if (navi_switch_nexttargetpoint()) {
                    uint8_t next_idx = navi_ctrl.point_current_idx;
                    IPC_LOG_Printf(" [到达航点] 下一个航点坐标为：X=%s%d.%02d, Y=%s%d.%02d | 类型:%s | 动作指令:%d\r\n",
                        F_ARG(point_map[next_idx].x),
                        F_ARG(point_map[next_idx].y),
                        get_enum_name(point_map[next_idx].type),
                        point_map[next_idx].action_cmd);
                }
            }
            break;
        }

        case 2:
            if (navi_ctrl.navi_mode_map == 1) {
                navi_auto_record_task();
            }
            break;

        default:
            break;
    }
}

//=================================航点管理======================================

/**

 * @brief 获取指定索引的航点信息

 * @details 读取指定索引的航点数据，索引无效时返回无效航点（valid=0）

 * @param[in] index: 航点索引

 * @return GPS_WayPoint_t - 航点结构体，索引无效时返回valid=0的无效航点

 */

Navi_WayPoint_t navi_get_point(uint16_t index) {

    Navi_WayPoint_t invalid_point = {0};

    invalid_point.valid = 0;

    if(index >= NAVI_POINT_MAX) {

        return invalid_point;

    }

    return point_map[index];

}





/**

 * @brief 切换到下一个目标航点

 * @details 将当前目标航点索引自增，用于航点序列导航

 * @return uint8 - 1: 切换成功；0: 已到最后一个航点

 * @note 切换成功后current_target_idx指向新的目标航点

 */

uint8 navi_switch_nexttargetpoint(void) {

    if(navi_ctrl.point_current_idx >= navi_ctrl.point_total_count- 1)  return 0; // 已到最后一个航点
    
    navi_ctrl.point_current_idx++;

    return 1;

}



//----------------------------------------------------------------------------------------------------------------
// 计算当前位置到目标航点的导航信息   (X为北，Y东,顺时针角度加)

//  target_idx: 目标航点索引

// azimuth: 方位角指针（单位：度）

// distance: 直线距离指针（单位：米）

//----------------------------------------------------------------------------------------------------------------

uint8 navi_calcnavinfo(uint16_t target_idx, double *azimuth, double *distance) {

    if(target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid || !robot_pose.is_valid)

       return 0;

    

    double dx = point_map[target_idx].x - robot_pose.x;

    double dy = point_map[target_idx].y - robot_pose.y;

    *distance = sqrt(dx * dx + dy * dy);   

    

    // 边界处理：如果已经在目标点附近，方位角保持当前航向，防止抖动

    if (*distance < 0.01) *azimuth = robot_pose.yaw;

    else {

        *azimuth = RAD_TO_ANGLE(atan2(dy, dx));

    }

    return 1;

}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介      计算平面坐标系下两点之间的距离 (单精度优化版)
// 参数说明      x1              第一个点的 X 坐标
// 参数说明      y1              第一个点的 Y 坐标
// 参数说明      x2              第二个点的 X 坐标
// 参数说明      y2              第二个点的 Y 坐标
// 返回参数      float           返回两点直线距离
//-------------------------------------------------------------------------------------------------------------------
float navi_get_two_points_distance(float x1, float y1, float x2, float y2)
{  
    float dx = x2 - x1;
    float dy = y2 - y1;
    
    // 使用 sqrtf 替代 sqrt，避免隐式转换为 double 导致运算变慢
    float distance = sqrtf(dx * dx + dy * dy); 

    return distance;  
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介      计算平面坐标系下从第一个点到第二个点的方位角 (单精度优化版)
// 参数说明      x1              第一个点的 X 坐标
// 参数说明      y1              第一个点的 Y 坐标
// 参数说明      x2              第二个点的 X 坐标
// 参数说明      y2              第二个点的 Y 坐标
// 返回参数      float           返回方位角（0.0 至 360.0 度）
// 备注信息      适用条件：X轴正方向为正北，Y轴正方向为正东。正北为0度，顺时针增加。
//-------------------------------------------------------------------------------------------------------------------
float navi_get_two_points_azimuth(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;

    return RAD_TO_ANGLE(atan2f(dy, dx));
}



//-------------------------------------------------------------------------------------------------------------------

// 判断是否到达目标航点           0 ：   未到达               1：到达

// 基于平面坐标（x/y）计算当前位置与目标航点的距离，判断是否小于设定阈值

// arget_idx: 目标航点索引

//--------------------------------------------------------------------------------------------------------------

uint8 navi_isreach_target_point(uint16 target_idx) {

    if(target_idx >= NAVI_POINT_MAX || !point_map[target_idx].valid || !robot_pose.is_valid) {

        return 0;

    }

    double dx = point_map[target_idx].x - robot_pose.x;

    double dy = point_map[target_idx].y - robot_pose.y;

    double distance_sq = dx*dx + dy*dy;
    
    float current_threshold = DISTANCE_THRESHOLD; // 判定范围
    
    switch (point_map[target_idx].type) {
        case WP_TYPE_CONE_CONE:   // 绕锥桶：要求更精准，缩小阈值
            current_threshold *= 0.8f; 
            break;
        case WP_TYPE_NORMAL:      // 普通直线：为了高速顺滑，放大阈值提前切点
            current_threshold *= 1.0f; 
            break;
        case WP_TYPE_STOP:        // 停车点：要求极高精度
            current_threshold *= 1.0f; 
            break;
        default:
            current_threshold *= 0.5f;
            break;
    }

    return (distance_sq <= (current_threshold * current_threshold)) ? 1 : 0;

}



/**
 * @brief  导航关闭时的位姿监测打印任务
 * @param  delta_ms：多长时间打印一次
 */
void Navigation_Pose_Monitor_Task(void)
{
    static uint16_t print_timer = 0;  // 【修改1】改为 uint16_t 防止溢出 (例如3000ms需要300次中断)

    // 1. 判断是否需要跳过打印：
    // 当 循迹模式开启(>=0.5) 或者 打印未使能(<0.5) 时，不打印并退出
    if (vofa_mode_driver >= 0.5f || (uint8_t)vofa_print_pose_en != 1) {
        print_timer = 0;
        return;
    }

    // 2. 获取设定的打印周期，并限制最小值为 500ms
    float current_period = vofa_print_pose_period;
    if (current_period < 500.0f) {
        current_period = 500.0f;
    }

    // 3. 计算需要多少次中断才能触发打印 (ENCODER_DT = 10ms)
    // 例如：1000ms / 10ms = 100 次中断
    uint16_t target_ticks = (uint16_t)(current_period / (ENCODER_DT * 1000.0f));

    // 4. 非阻塞延时控制打印频率
    if (++print_timer >= target_ticks)
    {
        print_timer = 0; // 重置计时器

        // 从 IPC 共享数据或全局变量获取最新的位姿
        float cur_x = core_a_status.nav_x;
        float cur_y = core_a_status.nav_y;
        float cur_v = core_a_status.nav_v;
        float cur_yaw = core_a_status.nav_yaw;

        // 执行打印，格式：[位姿监测] 车姿(X, Y) 速度, 航向
        LOG_Printf("[位姿监测] 车姿(%s%d.%02d, %s%d.%02d) 速度:%s%d.%02d m/s 航向:%s%d.%02d°\r\n",
                   F_S(cur_x), F_I(cur_x), F_D2(cur_x),
                   F_S(cur_y), F_I(cur_y), F_D2(cur_y),
                   F_S(cur_v), F_I(cur_v), F_D2(cur_v),
                   F_S(cur_yaw), F_I(cur_yaw), F_D2(cur_yaw));
    }
}



// 将航点类型枚举转换为对应的中文字符串
const char* get_enum_name(WayPoint_Type type) {
    switch (type) {
        case WP_TYPE_NORMAL:      return "普通循迹";
        case WP_TYPE_BRIDGE:      return "单边桥";
        case WP_TYPE_JUMP:        return "跳跃台阶";
        case WP_TYPE_MINE_SWEEP:  return "定点排雷";
        case WP_TYPE_CONE_CONE:   return "绕圆锥桶";
        case WP_TYPE_SIDE_SLOPE:  return "侧倾坡道";
        case WP_TYPE_STOP:        return "终点返航";
        case WP_TYPE_HOME:        return "原点";
        default:                  return "未知类型";
    }
}
     

//=====================================================静态函数定义=======================================================================




//1.坐标原点重置时间：开机时 IMU 会有几秒钟的收敛期，必须等 IMU 稳定后再将当前位置设置为 (0,0)。
//
//2.导航算法必须放在严格定时的 10ms 中断里，并且要放在 balance_control() 之后，因为导航依赖它算出来的 now_velocity（实时线速度）。


