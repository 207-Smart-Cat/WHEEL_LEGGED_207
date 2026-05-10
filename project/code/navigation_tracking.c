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

//====================================================全局变量定义=======================================================

Navi_Controller_t navi_ctrl;

Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // 预设的“路”

float wifi_cmd_trigger = 0.0;        // 摇铃变量         0: 待机, 1: 追加一个航点, 2: 清空当前地图, 3: 立刻接管执行动作

float wifi_remote_type= 0.0;                                  //赋值航点类型,按照枚举依次从0到···

float wifi_in_action= 0.0;            //动作指令    

float vofa_trigger_record = 0.0f;

//====================================================全局变量定义=======================================================
float vofa_mode_driver = 0.0f;
float vofa_mode_map = 0.0f;
float vofa_print_pose_en = 1.0f;
float vofa_print_pose_period = 1000.0f; // 默认 1000ms 打印一次
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



//====================================================具体函数编写层=============================================

//初始化
void Navi_Tracking_Init(void) {
                      
    navi_ctrl.point_total_count = 0;

    navi_ctrl.point_current_idx = 0;
    
    navi_ctrl.navi_mode_map = 0;       //默认使用静态地图
    
    navi_load_comprehensive_test_map();

}

//-------------------------------------------------------------------------------------------------------------------

//  路径预处理：线性插值

//当两点间距 > 0.3m 时，自动插入中间点，确保前瞻逻辑平滑

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
  
    static float last_record_x = 0;
    
    static float last_record_y = 0;
    
    // ---- 新增：接入 WiFi 下发的打点指令 ----
    if (vofa_trigger_record > 0.5f && last_vofa_trigger <= 0.5f) {
        navi_ctrl.trigger_record = 1; // 触发底层打点
//        vofa_trigger_record = 0.0f;   // 不手动清零，因为外接调控只能改变值，0或1，他也不能清零
    }
    
    last_vofa_trigger = vofa_trigger_record; // 更新历史状态
    

    // 如果没有开启录制模式或位姿无效，直接退出
    if (navi_ctrl.navi_mode_driver != 2 || !robot_pose.is_valid) return;

    if (navi_ctrl.point_total_count >= NAVI_POINT_MAX) return;

    
    //自动距离触发   或者    外部强制触发（遥控器按下打特殊动作点）
    float dist_since_last = navi_get_two_points_distance(robot_pose.x, robot_pose.y, last_record_x, last_record_y);
    
    if (dist_since_last > RECORD_MIN_DIST || navi_ctrl.trigger_record || navi_ctrl.origin_set_flag == 0) {
      
        // （首次记录）原点作为第0个航点

        if(navi_ctrl.origin_set_flag == 0) {

            Navi_Data_Set_Origin();

            point_map[0].x = 0.0;

            point_map[0].y = 0.0;

            point_map[0].type = WP_TYPE_HOME; // 原点默认设为返航点

            point_map[0].valid = 1;

            navi_ctrl.point_total_count = 0;

            navi_ctrl.origin_set_flag = 1;
            
            IPC_LOG_Printf("\r\n>>> [打点成功] 坐标系原点(起始点)已自动记录！ <<<\r\n");

        }
        else{
            uint8_t idx = navi_ctrl.point_total_count;
            
            // 记录平滑后的当前位置
            point_map[idx].x = robot_pose.x;
            point_map[idx].y = robot_pose.y;
            point_map[idx].yaw = robot_pose.yaw;
            
            // 如果是手动触发，赋予特殊动作属性；否则是普通循迹点
            if (navi_ctrl.trigger_record) {
                point_map[idx].type = (WayPoint_Type)wifi_remote_type;
                navi_ctrl.trigger_record = 0; // 清除标志
                
                IPC_LOG_Printf("\r\n>>> [打点成功] VOFA远程手动打点已记录！ <<<\r\n");
            } else {
                point_map[idx].type = WP_TYPE_NORMAL;
            }
            
            point_map[idx].valid = 1;          
          
        }
        
        IPC_LOG_Printf(" 记录当前航点坐标为：X=%s%d.%02d, Y=%s%d.%02d  |  航点类型: %s  | 动作指令: %d\r\n", 
            F_ARG(point_map[navi_ctrl.point_total_count].x),
            F_ARG(point_map[navi_ctrl.point_total_count].y), 
            get_enum_name(point_map[navi_ctrl.point_total_count].type),
            point_map[navi_ctrl.point_total_count].action_cmd);
        
//        IPC_LOG_Printf("%s%d.%02d,%s%d.%02d\r\n", 
//        F_ARG(point_map[navi_ctrl.point_total_count].x),
//        F_ARG(point_map[navi_ctrl.point_total_count].y));
        
        last_record_x = robot_pose.x;
        last_record_y = robot_pose.y;
        navi_ctrl.point_total_count++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//静态地图导入 (精度测定专用)
// 目的：直接赋值坐标点，检查小车跑坐标单位1时，实际物理位移
//-------------------------------------------------------------------------------------------------------------------
void navi_load_comprehensive_test_map(void) {
    navi_ctrl.point_total_count = 0;
    Navi_Data_Set_Origin(); // 设为原点
    
    uint8_t i = 0;
    
    // 1. 起点
    point_map[i++] = (Navi_WayPoint_t){0.0,  0.0,   0.0, WP_TYPE_HOME,   0, 1};
    // 2. 直线走两块地砖 (X前进 1.2m)
    point_map[i++] = (Navi_WayPoint_t){0.6f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    point_map[i++] = (Navi_WayPoint_t){1.2f,  0.0f,   0.0f, WP_TYPE_NORMAL, 0, 1};
    // 3. 右转90度 (顺时针Yaw增加)，走一块地砖 (Y增加 0.6m)
    point_map[i++] = (Navi_WayPoint_t){1.2f,  0.6f,  90.0f, WP_TYPE_NORMAL, 0, 1};
    // 4. 右转90度，走两块地砖返回 (X减小回 0)
    point_map[i++] = (Navi_WayPoint_t){0.6f,  0.6f, 180.0f, WP_TYPE_NORMAL, 0, 1};
    point_map[i++] = (Navi_WayPoint_t){0.0f,  0.6f, 180.0f, WP_TYPE_NORMAL, 0, 1};
    
    // 5. 右转90度，走一块地砖回到起点并停车
    point_map[i++] = (Navi_WayPoint_t){0.0f,  0.0f, 270.0f, WP_TYPE_STOP,   0, 1};

    navi_ctrl.point_total_count = i;
    navi_ctrl.point_current_idx = 0;
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

        case 3: // 3: 无视航点，立刻强制执行轮腿跨越/跳跃动作 ,  直接将动作代码丢给 action.c 的状态机
            action_fsm.state = (ActionState_e)wifi_in_action;
            is_action_busy = 1; 
            break;
    }

    // 处理完毕，清空摇铃，等待下一次注入
    wifi_cmd_trigger = 0; 
}



//-------------------------------------------------------------------------------------------------------------------

//   导航模式识别         2：遥控器打点模式     1：已有航点，巡航模式           0：停止

//-------------------------------------------------------------------------------------------------------------------

void task_navigation_control(void) {

    static uint8_t last_mode_dricer = 0;
    
    static uint8_t last_mode_map = 255;
    
    // 监测地图方案切换
    if (last_mode_map != navi_ctrl.navi_mode_map) {
        IPC_LOG_Printf(">>> 地图模式切换: %d -> %d <<<\r\n", last_mode_map, navi_ctrl.navi_mode_map);
        
        // 核心逻辑：无论切到哪，先重置当前地图状态
        navi_ctrl.point_total_count = 0;
        navi_ctrl.point_current_idx = 0;
        navi_ctrl.origin_set_flag = 0;
        memset(point_map, 0, sizeof(point_map)); // 物理清空

        if (navi_ctrl.navi_mode_map == 0) {
            // 模式 0：重新加载静态测试地图
            navi_load_comprehensive_test_map();
            IPC_LOG_Printf("[SYS] 已加载静态测试地图\r\n");
        } 
        else if (navi_ctrl.navi_mode_map == 1) {
            // 模式 1：进入打点录制准备状态
            IPC_LOG_Printf("[SYS] 录制模式：坐标系已重置，等待打点...（需手动切换模式driver = 2）\r\n");
        }
        
                 // ==========================================================
         // 【调试代码】: 打印全地图坐标序列
         // =========================================================
        if (navi_ctrl.navi_mode_driver == 1 && navi_ctrl.point_total_count > 0) {
             IPC_LOG_Printf("====== 导航路线已就绪 (共 %d 个点) ======\r\n", navi_ctrl.point_total_count);
             for(int i = 0; i < navi_ctrl.point_total_count; i++) {
                IPC_LOG_Printf("航点[%02d]: X=%s%d.%2d, Y=%s%d.%2d, 类型=%s\r\n", 
                          i,
                          F_ARG(point_map[i].x), 
                          F_ARG(point_map[i].y), 
                          get_enum_name(point_map[i].type));
             }
             
             IPC_LOG_Printf("=============================================\r\n");
        }
        last_mode_map = navi_ctrl.navi_mode_map;
    }
          
   // 检测到模式切换，执行预处理

    if (last_mode_dricer !=navi_ctrl.navi_mode_driver)  {
      
        // 如果当前是停止模式(0)，更新完状态直接退出，不打印任何东西
        if (navi_ctrl.navi_mode_driver == 0) {
            last_mode_dricer = navi_ctrl.navi_mode_driver;
            last_mode_map = navi_ctrl.navi_mode_map;
            return; 
        }
            
       if(navi_ctrl.navi_mode_driver== 1 && navi_ctrl.navi_mode_map == 1) {    //循迹且打点
         
          Navi_Data_Set_Origin();             // 将当前物理位置重置为 (0,0)，并锁定当前车头为新0度
          
          navi_ctrl.point_current_idx = 0;    // 目标航点从头(第0个点)开始
          
          navi_parse_global_path();       //全局路径特殊动作点预解析 
              
//          navi_path_optimize();          // 路径坐标加密
          
          IPC_LOG_Printf("\r\n>>> [模式切换] 位姿已重新归零！将以当前位置复刻路线形状！ <<<\r\n");
          
        // ==========================================================
        // 【调试代码】: 打印全地图坐标序列
        // =========================================================
        if (navi_ctrl.point_total_count > 0) {
             IPC_LOG_Printf("====== 导航路线已就绪 (共 %d 个点) ======\r\n", navi_ctrl.point_total_count);
             for(int i = 0; i < navi_ctrl.point_total_count; i++) {
                IPC_LOG_Printf("航点[%02d]: X=%s%d.%02d, Y=%s%d.%02d, 类型=%s\r\n", 
                          i,
                          F_ARG(point_map[i].x), 
                          F_ARG(point_map[i].y), 
                          get_enum_name(point_map[i].type));
             }
             
             IPC_LOG_Printf("=============================================\r\n");
        }
              
        }  
        last_mode_dricer = navi_ctrl.navi_mode_driver;
       
    }
    
    
    switch(navi_ctrl.navi_mode_driver)
    {
        case 0:   {            //停止
                ;
                break;  
        }  
        
        
    case 1:   {                //--- 循迹模式 ---自主追踪导航                
         
                                                                    
            uint8_t total_points = navi_ctrl.point_total_count;

            uint8_t curr_idx = navi_ctrl.point_current_idx;    
            
            if (total_points == 0) {
              target_velocity = 0;
//              IPC_LOG_Printf("\r\n>>>地图坐标数目为0，地图为建立<<<\r\n");
              break; 
          }

            // --- 1. 动态前瞻搜索 ---计算当前速度自适应的前瞻距离  0.2f 是速度增益系数

            float dynamic_lookahead = DISTANCE_THRESHOLD + (RPM_TO_M_COEFF(fabsf(now_velocity) )* 0.2f); 

            dynamic_lookahead = fmaxf(DISTANCE_THRESHOLD, fminf(1.0f, dynamic_lookahead));// 限幅防止前瞻过远

            
            //寻找最近的一个前瞻点
            uint8_t lookahead_idx= total_points - 1;
            for (uint8_t i = curr_idx; i < total_points; i++) {
              
                float dist = navi_get_two_points_distance( robot_pose.x  ,robot_pose.y  ,  point_map[i].x, point_map[i].y);
                
                if (dist >= dynamic_lookahead) {
                
                    lookahead_idx = i;
                    
                    break; // 找到足够远的前瞻点
                
                }

            }


            double azimuth = 0.0, distance = 0.0;
            
            float print_turn_angle = 0.0f; // 新增：专门用于打印的转向角

            //计算到前瞻点的方位和距离

            if (navi_calcnavinfo(lookahead_idx, &azimuth, &distance)) {

                float need_to_turn = navi_limit_angle180((float)azimuth - robot_pose.yaw);
            
                float k = (distance < 0.5f) ? 0.0f : 0.0f; 
                
                float smooth_turn = (1.0f - k) * need_to_turn;
                
                print_turn_angle = smooth_turn; // 保存下来用于下面的打印
                
                // 【核心坐标系转换】计算赋给底层的 target_angle！
                // 推导过程：
                // 导航系偏航角 = -(IMU偏航角 - 初始偏移)  -->  导航系增量 = -IMU增量
                // 因此，导航系要求转 smooth_turn 度，对底层 IMU 来说就是转 -smooth_turn 度。
                target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - smooth_turn);

                if (!is_action_busy) {       // 判断是否在动作接管期              
                  
                  target_velocity = 1000.0f;
                
                }            
            
            }
            
            // 打印导航信息
            if (vofa_print_pose_en > 0.5f) {
                static int8_t dynamic_print_delay = 0;
                // 需要的进入次数
                int8_t target_delay_ticks = (int8_t)(vofa_print_pose_period / (ENCODER_DT * 1000));
                if (target_delay_ticks == 0) target_delay_ticks = 5; // 防止除0或周期太小导致异常
                
                if (++dynamic_print_delay >= target_delay_ticks) {
                    dynamic_print_delay = 0;

#if WEIZHIJIANCE
                    // 打印包含坐标、偏航角、当前模式，方便你调试打点准确性
                    IPC_LOG_Printf("[位姿监测] 车姿(%s%d.%02d, %s%d.%02d)  | Yaw=%s%d.%02d |"
                                   "去往前瞻点(%s%d.%02d, %s%d.%02d) 类型：%s | "
                                    "距目标: %s%d.%02d | 需转向: %s%d.%02d度\r\n",
                          F_ARG(robot_pose.x), F_ARG(robot_pose.y),F_ARG(robot_pose.yaw), 
                          F_ARG(point_map[lookahead_idx].x), F_ARG(point_map[lookahead_idx].y), get_enum_name(point_map[lookahead_idx].type),
                          F_ARG((float)distance),F_ARG(print_turn_angle));
                    
                    IPC_LOG_Printf("检测navi_mode_driver值为：%s%d.%02d\r\n",F_ARG(navi_ctrl.navi_mode_driver));
                    
#endif

                }

            }
            
            
           // 到达判定：如果进入 10cm 范围内，切换下一点，并执行该点动作
           if (navi_isreach_target_point(curr_idx)) {
             
             //action.c接管动作                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
             
             
             navi_switch_nexttargetpoint();
             
               // ==========================================================
               // 【调试代码 】: 到达航点并切换的事件触发打印
               // ==========================================================
               uint8_t next_idx = navi_ctrl.point_current_idx;
               
               if (next_idx < navi_ctrl.point_total_count && next_idx != curr_idx) {
                 IPC_LOG_Printf(" [到达航点] 下一个航点坐标为：X=%s%d.%02d, Y=%s%d.%02d  |  航点类型: %s  | 动作指令: %d\r\n", 
                               F_ARG(point_map[curr_idx+1].x),
                               F_ARG(point_map[curr_idx+1].y), 
                               get_enum_name(point_map[curr_idx+1].type),
                               point_map[curr_idx+1].action_cmd);
                   
               } else {
                   IPC_LOG_Printf("=============  >>> [事件] 终点已到达，导航结束！ =============\r\n");

               }

           }  
//           else if (curr_idx < navi_ctrl.point_total_count - 1) {
//                // 防切角死锁：判断是否离下一个点更近
//                float dist_to_curr = navi_get_two_points_distance(robot_pose.x, robot_pose.y, point_map[curr_idx].x, point_map[curr_idx].y);
//                float dist_to_next = navi_get_two_points_distance(robot_pose.x, robot_pose.y, point_map[curr_idx+1].x, point_map[curr_idx+1].y);
//                
//                // 如果离下一个点更近，说明已经越过了当前点所在切面，强制切走！
//                if (dist_to_next < dist_to_curr) {
//                    navi_switch_nexttargetpoint();
//                }
//           }
            
            //“到达终点”后,最大速度导航回到初始位置（或者初始位置的横向轴就行）                  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            if (navi_ctrl.point_current_idx >= navi_ctrl.point_total_count - 1) {

                if (navi_isreach_target_point(navi_ctrl.point_current_idx)) {
                  
                  //回到起始点或者起始轴线就行

                }

            } 
                               
          break;
        }

        case 2:     
          {        // --- 录制模式 ---        // 周期性检测 trigger_record 标志位来保存当前 robot_pose
          
            if(navi_ctrl.navi_mode_map == 0)  {
              
                navi_load_comprehensive_test_map();           //静态地图
                
//                IPC_LOG_Printf("\r\n静态测试地图\r\n");

              
            }
            
           if(navi_ctrl.navi_mode_map == 1) {
              
                navi_auto_record_task();                  //打点画图  
                
//                IPC_LOG_Printf("\r\n正在进行动态打点画图\r\n");

              
            }    
              
            if(navi_ctrl.navi_mode_map == 2) {
              
                 navi_wifi_remote_cmd();                                                     //WiFi动态传地图
              
//                IPC_LOG_Printf("\r\n正在进行wifi远程画图\r\n");
            }
          break;
          } 
        
        default:  // 所有case都不满足时执行 
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
    
    float current_threshold = DISTANCE_THRESHOLD; // 默认 0.2m
    
    switch (point_map[target_idx].type) {
        case WP_TYPE_CONE_CONE:   // 绕锥桶：要求更精准，缩小阈值
            current_threshold *= 0.8f; 
            break;
        case WP_TYPE_NORMAL:      // 普通直线：为了高速顺滑，放大阈值提前切点
            current_threshold *= 1.0f; 
            break;
        case WP_TYPE_STOP:        // 停车点：要求极高精度
            current_threshold *= 0.5f; 
            break;
        default:
            current_threshold *= 0.5f;
            break;
    }

    return (distance_sq <= (current_threshold * current_threshold)) ? 1 : 0;

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

