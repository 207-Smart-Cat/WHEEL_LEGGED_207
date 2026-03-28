/*********************************************************************************************************************

* 打滑代码使用了加速度acc_x  ，具体方向需要根据实际更改             

* 坐标x轴正北，y轴正东。顺时针时yaw增加    

*需要确认 IMU_data.filter_result.yaw 读出来确实是 0~360 或180$ 且顺时针增加。

*yaw_filter 的范围：-180 到 180





，EKF 内部的所有的状态量、控制量、观测量，必须全部统一为 国际标准单位（SI：米 $m$、秒 $s$、弧度 $rad$）。

*********************************************************************************************************************/



#include "navigation_tracking.h"

#include "navigation_data_handling.h"

#include "navigation_action.h"

#include "kalman_rm.h"



//====================================================全局变量定义=======================================================

Navi_Controller_t navi_ctrl;

Navi_WayPoint_t     point_map[NAVI_POINT_MAX];               // 预设的“路”

uint8_t remote_type_cmd;                                  //赋值航点类型

/*腿部姿态设置*/


static uint8_t  record_sample_cnt = 0;
static double   sum_x = 0, sum_y = 0;
static float    sum_yaw = 0;

extern float x_current,y_current;

extern int Bridge_position;





//====================================================变量声明=======================================================

extern RobotState_t robot_pose;                                  // 全局实时位姿，供外部只读访问

extern Navi_Sensor_Data_t       filter_data;                       //基础滤波+处理后数据

extern IMU_t IMU_data;

extern uint8_t  is_action_busy ;       // 0:循迹控制 1:动作接管





// 引用control.c 中的控制接口

extern float now_velocity;       //当前速度(线速度)

extern float target_velocity;    // 目标速度 (线速度)

extern float target_motor_angle; // 目标角度 (用于转向)

extern float target_engine_high; // 目标腿部高度

extern float Turn_Pwm; // 转向PWM值



extern float Turn_target(float target_angle);        //将目标限制在-180-+180

extern void leg_roll_control(float leg_target, float angle_error);

extern float Turn(float gyro, float target_angle);

extern bool is_airborne(void); 

extern float leg_error;         //联动leg_adaptive.c：记录此时的侧倾补偿量

extern void engine_jump(void);



//====================================================函数声明=============================================


//====================================================具体函数编写层=============================================

//初始化

void Navi_Tracking_Init(void) {
                      
    navi_ctrl.point_total_count = 0;

    navi_ctrl.point_current_idx = 0;

    navi_ctrl.navi_mode = 0;

}



//-------------------------------------------------------------------------------------------------------------------

// 记录当前Navi位置为航点          task_navigation_control函数中已调用   ，随其一起在周期中断了

// 首次调用时设置原点，后续调用记录指定类型的航点

// type: 航点类型（WP_TYPE_HOME/WP_TYPE_NORMAL/WP_TYPE_STOP/WP_TYPE_OBSTACLE）

//-------------------------------------------------------------------------------------------------------------------

void navi_record_current_point(void) {  

    // 1. 校验前置条件

    if(!robot_pose.is_valid || navi_ctrl.point_total_count >= NAVI_POINT_MAX|| !navi_ctrl.trigger_record ) {

        return; // 数据无效或航点满

    }
    
    // 开始累加采样 (连续采样 10 次)
   if (record_sample_cnt < 10) {
        sum_x += robot_pose.x;
        sum_y += robot_pose.y;
        sum_yaw += robot_pose.yaw;
        record_sample_cnt++;
        return; // 继续等待下一周期采样
    }
        
    // 2. 采样完成，计算平均值并打点
    float avg_x = (float)(sum_x / 10.0);
    float avg_y = (float)(sum_y / 10.0);
    float avg_yaw = sum_yaw / 10.0f;
    
    WayPoint_Type type =  (WayPoint_Type)remote_type_cmd;

    // 2. 设置原点（首次记录）原点作为第0个航点

    if(navi_ctrl.origin_set_flag == 0) {

        Navi_Data_Set_Origin();

        point_map[0].x = 0.0;

        point_map[0].y = 0.0;

        point_map[0].type = WP_TYPE_HOME; // 原点默认设为返航点

        point_map[0].valid = 1;

        navi_ctrl.origin_set_flag = 1;


        navi_ctrl.point_total_count = 1;

    }
    else {

        uint8_t idx = navi_ctrl.point_total_count++;

        point_map[idx].x = avg_x;

        point_map[idx].y = avg_y;

        point_map[idx].yaw      = avg_yaw; // 抓取当前朝向

        point_map[idx].type = type;

        point_map[idx].valid = 1;
    }
        
    // 3. 重置计数器与触发标志
    record_sample_cnt = 0;
    sum_x = 0; sum_y = 0; sum_yaw = 0;
    navi_ctrl.trigger_record = 0;

}




 // ==================== 导航计算优化 ====================

//-------------------------------------------------------------------------------------------------------------------

//  路径预处理：线性插值

//当两点间距 > 0.3m 时，自动插入中间点，确保前瞻逻辑平滑

//-------------------------------------------------------------------------------------------------------------------

void navi_path_optimize(void) {

    uint8_t count = navi_ctrl.point_total_count;

    if (count < 2) return;



    for (uint8_t i = 0; i < count - 1; i++) {

        double dx = point_map[i+1].x - point_map[i].x;

        double dy = point_map[i+1].y - point_map[i].y;

        float dist = sqrtf(dx*dx + dy*dy);



        // 如果间距超过 0.4米 且 数组未满，插入一个中点

        if (dist > 0.4f && navi_ctrl.point_total_count < NAVI_POINT_MAX) {

            // 整体后移腾出空间

            for (uint8_t j = navi_ctrl.point_total_count; j > i + 1; j--) {

                point_map[j] = point_map[j-1];

            }
            
            // 坐标线性插值
            point_map[i+1].x = (point_map[i].x + point_map[i+2].x) / 2.0f;
            point_map[i+1].y = (point_map[i].y + point_map[i+2].y) / 2.0f;
            
            // 角度插值（处理过零点）
            float angle_diff = Turn_target(point_map[i+2].yaw - point_map[i].yaw);
            point_map[i+1].yaw = Turn_target(point_map[i].yaw + angle_diff * 0.5f);

            
            point_map[i+1].type = point_map[i].type;
            point_map[i+1].valid = 1;
            
            navi_ctrl.point_total_count++;

            count++;

            i++; // 跳过新生成的点

        }

    }

}







// ==================== 业务服务层 ====================

//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//静态地图导入 (精度测定专用)
// 目的：直接赋值坐标点，检查小车跑坐标单位1时，实际物理位移
//-------------------------------------------------------------------------------------------------------------------
void navi_load_static_calibration_map(void) {
    // 1. 清空当前地图
    navi_ctrl.point_total_count = 0;
    
    // 2. 设置（x，y）为起点，坐标原点 (0,0)
    Navi_Data_Set_Origin();
    
    // 3. 注入标准测试航点：直线跑 2.0米
    uint8_t i = 0;
    
    // 第0个点：起点
    point_map[i] = (Navi_WayPoint_t){0.0, 0.0, 0.0, WP_TYPE_HOME, 0, 1};
    i++;

    // 第1个点：目标点 (用于精度校准)
    // 检查：如果小车停止后，实地测量距离不是 2.0米，则需调整底层 RPM_TO_M_COEFF 系数
    point_map[i].x = 2.0; 
    point_map[i].y = 0.0;
    point_map[i].yaw = 0.0;
    point_map[i].type = WP_TYPE_STOP; // 跑完强制停机
    point_map[i].valid = 1;
    i++;

    navi_ctrl.point_total_count = i;
    navi_ctrl.point_current_idx = 0;
    
    // 自动切换到循迹模式开始测试
    navi_ctrl.navi_mode = 1; 
}


//-------------------------------------------------------------------------------------------------------------------

//   导航模式识别，确定是循迹，遥控器，【还是直接输入坐标】             遥控器打点模式 ：2      已有航点，巡航模式：1

//-------------------------------------------------------------------------------------------------------------------

void task_navigation_control(void) {

    static uint8_t last_mode = 0;


    // 检测到从录制切换到循迹的瞬间，执行预处理

    if (last_mode !=navi_ctrl.navi_mode && navi_ctrl.navi_mode != 2) {

        navi_path_optimize();          // 路径坐标加密

        navi_ctrl.point_current_idx = 0; // 重置进度

    }

    last_mode = navi_ctrl.navi_mode;

    

    if (navi_ctrl.navi_mode == 2) { 

        // --- 录制模式 ---

        // 此时 target_velocity 和 target_motor_angle 由遥控器/手柄直接给到 control.c

        // 周期性检测 trigger_record 标志位来保存当前 robot_pose

        navi_record_current_point(); 

    } 

    else if (navi_ctrl.navi_mode == 1) {                                                                    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        // --- 循迹模式 ---
        uint8_t total_points = navi_ctrl.point_total_count;

        uint8_t curr_idx = navi_ctrl.point_current_idx;      
        
        
        //检测是否侧倾 ，避免单侧坡带来的倾斜出现的问题（？？？？？？？？？？？？？？？？考虑是否要在下面的类型动作里面检测，而删除这里的逻辑）
        if (fabsf(IMU_data.filter_result.roll) > 5.0f) { // 如果侧倾超过 5 度，强制开启自适应

            leg_roll_control(target_engine_high, IMU_data.filter_result.roll);                       

        }


        // --- 1. 动态前瞻搜索 ---

         // 计算当前速度自适应的前瞻距离

        // 0.3m 是最小前瞻，0.2f 是速度增益系数（需根据 RPM 量程微调）

        float dynamic_lookahead = 0.3f + (RPM_TO_M_COEFF(fabsf(now_velocity) )* 0.2f); 

        dynamic_lookahead = fmaxf(0.3f, fminf(1.0f, dynamic_lookahead));// 限幅防止前瞻过远

        
        //寻找最近的一个前瞻点
        uint8_t lookahead_idx = curr_idx;
        for (uint8_t i = curr_idx; i < total_points; i++) {
          
            float dist = sqrtf(pow(point_map[i].x - robot_pose.x, 2) + pow(point_map[i].y - robot_pose.y, 2));
            
            lookahead_idx = i;

            if (dist >= dynamic_lookahead) break; // 找到足够远的前瞻点

        }


        double azimuth, distance;

        // 1. 计算到前瞻点的方位和距离

        if (navi_calcnavinfo(lookahead_idx, &azimuth, &distance)) {

            // 2. 将计算结果直接赋给队友的控制变量  ，融合系数 k (0.0 ~ 1.0)用来平滑路径 
                  //近距离时，信任当前 IMU 航向，k较大，按照计算角和位置行进，远距离时，通过路径计算来平滑路径 ，k较小
            float k = (distance < 0.5f) ? 0.3f : 0.0f; // 距离近时分配 30% 权重给 IMU 原始航向
        
            float fused_angle = (1.0f - k) * (float)azimuth + k * robot_pose.yaw;
          
            // 将融合后的角度作为最终控制目标
            target_motor_angle = Turn_target(fused_angle);       // 调用队友的函数确保角度选择最短路径旋转   

            if (!is_action_busy) {             // 判断是否在动作接管期。如果处于动作接管期，导航依然在后台推算位姿，但不输出转向指令  防止导航指令与跳跃/绕桩动作指令冲突
                // 将方位角偏差输入 PD 转向环
                Turn_Pwm = Turn(filter_data.yaw, target_motor_angle);     //不在接管期则转向                  ！！！！！！！！！！    此行修改将使 Turn_Pwm 受你的导航方位角控
            }
        }
        
       // 4. 到达判定：如果进入 10cm 范围内，切换下一点，并执行该点动作
       if (navi_isreach_target_point(curr_idx)) {
           // 触发特殊动作（如跳跃指令）
           navi_execute_integrated_action(curr_idx);

           navi_switch_nexttargetpoint();

            }        
        
        //“到达终点”后,target_velocity 设为 0。            ！！！！！！！！后期将这个分解为回归原点 HOME，或者单独设置一个回归变量，来实现结束时回归 出发点 。
        if (navi_ctrl.point_current_idx >= navi_ctrl.point_total_count - 1) {

            if (navi_isreach_target_point(navi_ctrl.point_current_idx)) {

                target_velocity = 0; // 到达最后一个点，强制停机

            }

        }            

    }

}

//-------------------------------------------------------------------------------------------------------------------
//    核心集成动作执行引擎
//@param point_idx 当前导航点索引
//-------------------------------------------------------------------------------------------------------------------
void navi_execute_integrated_action(uint8_t point_idx) {
    Navi_WayPoint_t *wp = &point_map[point_idx]; 

    switch (wp->type) {
      
        case WP_TYPE_NORMAL:
            target_velocity = 600;
            system_delay_ms(500);
            break;
        
        case WP_TYPE_BRIDGE: // --- 单边桥动作 
            is_action_busy = 1;
//            action_locked_yaw = wp->yaw; // 锁定录制时的航向直行
            target_velocity = 400;       // 阶梯第一段：稳步上桥
            system_delay_ms(500); 
            target_velocity = 500;       // 阶梯第二段：高速冲刺
            Bridge_position = 0;         // 激活底层腿部自适应标志位
            break;

        case WP_TYPE_JUMP: // --- 跳跃/台阶动作 ---
            is_action_busy = 1;
//            action_locked_yaw = robot_pose.yaw;
//            jump_process_control(&x_current,&y_current);
            system_delay_ms(100);
            break;

        case WP_TYPE_MINE_SWEEP: // // 今年新增：定点排雷运动逻辑预留
            // // 可在此添加：降低速度 -> 机械臂/特定的腿部抖动动作
            break;

        case WP_TYPE_CONE_CONE:  // // 今年新增：绕圆锥桶运动逻辑预留
            // // 可在此添加：修改转向环 PID 参数以实现更激进的过弯
            break;

        case WP_TYPE_CROSSING:   // --- 穿越障碍策略 ---
            is_action_busy = 1;
            target_engine_high = 0.035f;     // 压低重心
            break;

        case WP_TYPE_STOP: // --- 完赛强制停机 ---                         ！！！？？？可以考虑在动作这里设置惯性完赛指令，依据盲跑来完成项目后半段
            is_action_busy = 1;
            target_velocity = 0;
            system_delay_ms(800);        // 惯性冲线延时
            small_driver_set_duty(0, 0); // 强制锁死电机
            pit_all_close();             // 关闭控制环防止抖动
            break;
        default:
            is_action_busy = 0;
            target_motor_Stand = 1.6f;       // 恢复正常机械中值
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

Navi_WayPoint_t navi_get_point(uint8 index) {

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



        

//=====================================================静态函数定义=======================================================================













//1.坐标原点重置时间：开机时 IMU 会有几秒钟的收敛期，必须等 IMU 稳定后再将当前位置设置为 (0,0)。
//
//2.导航算法必须放在严格定时的 10ms 中断里，并且要放在 balance_control() 之后，因为导航依赖它算出来的 now_velocity（实时线速度）。

