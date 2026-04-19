#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"
#include "imu.h"
#include "engine.h"
#include "control.h"
#include "wifi.h"
#include "remote.h"
#include "screen_display.h"
#include "ipc_shared_data.h"
#include "param.h"
#include "navigation_data_handling.h"
#include "navigation_tracking.h"
#include "battery_monitor.h"

// **************************** 核间通信区域 ****************************
// 在 CM7_0 和 CM7_1 中都需要加入这段代码

// 将 Core A 的状态数据放在 0x28001000
#pragma location = 0x28001000
__no_init CoreA_Status_t core_a_status;

// 将 Core B 的指令数据放在 0x28001200 (往后偏移512字节，预留充足空间)
#pragma location = 0x28001200
__no_init CoreB_Command_t core_b_cmd;

// **************************** 宏定义区域 ****************************
// 中断
#define PIT_IMU (PIT_CH0)
#define PIT_Balance (PIT_CH10)
#define PIT_Remote (PIT_CH12)
#define PIT_Engine (PIT_CH13)
#define PIT_IPC (PIT_CH11)
#define LED1 (P19_0)
// **************************** 全局变量区域 ****************************

// 电机+舵机（运行）
int duty = 1;
int stop = 0;
extern float pitch1, roll1, yaw1;
float v_buchang;
/*腿部姿态设置*/
extern float x_current, y_current;
// 用于舵机（x，y）位置的调整,务必记住应该给一个符合区间的初始值，否则舵机将不在转动
int stop_flash = 0; // 完赛标志位

int Bridge_position = 1;     // 可能是过单边桥时候需要的（腿部自适应）
int yanshi_biaozhiwei = 100; // 可能是过单边桥时候需要的（腿部自适应模式）
int change_speed = 0;
extern float temp_a, temp_b;

// 外部变量引入
extern RobotState_t robot_pose;
extern bool IMU_ready;
extern int jump_stop;

extern uint32_t test_pit10_cnt;

// **************************** 封装调试部分函数区域 ****************************
// ================= 主函数 =================
int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
  debug_init();                  // 调试串口信息初始化
  // 此处编写用户代码 例如外设初始化代码等

  interrupt_global_disable(); // 初始化外设之前先关闭中断
    //=================================双核通信初始化======================
  IPC_Init_Shared_Memory();
  IPC_Check_And_Apply_Params_To_Core0();
  //=================================GPIO初始化=======================
  gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式
  //=================================IMU初始化=======================
  imu_init(LED1);
  pit_ms_init(PIT_IMU, 1);
  //========================遥控器控制初始化==========================
  Remote_Init();
  pit_ms_init(PIT_Remote, 10);//10ms更新一次目标速度
  //=================================平衡动作初始化========================
  Balance_init(); // 初始化平衡控制（设置Kalman滤波的各个参数）
  small_driver_uart_init(); // 驱动板通信初始化
  battery_monitor_init(); // 电池电压 ADC 初始化
  //=================================舵机初始化======================
  pit_ms_init(PIT_Engine, 20); // 舵机固定 20ms 周期更新
  pit_ms_init(PIT_IPC, 10); // 双核参数同步 10ms 周期检查

//  // === 1. 导航系统初始化 ===
//    navi_data_init();
//    Navi_Tracking_Init();
//
//    // 修改：将 PIT_Balance 从 3ms 改为 10ms 以匹配 ENCODER_DT (0.01f)
//    // 注意：如果是平衡控制强制要求 3ms，则需修改导航的 ENCODER_DT 为 0.03f 并在 3ms 中断分频调用
    pit_ms_init(PIT_Balance, 1); 
////    jump_stop = 1; // 在 control.c 中，jump_stop=1 会让 PID 参数全置 0
    interrupt_global_enable(0);
//    
    system_delay_ms(1000); // 额外等待1秒，确保卡尔曼完全静止收敛
//    
//    // === 3. 重置导航原点 (0,0) ===
//    Navi_Data_Set_Origin();
//    
//    // === 4. 切断电机动力，开启纯推车模式 ===
   
    
    

  interrupt_global_enable(0); // 在初始化后使能中断

  system_delay_ms(1000);
  

  while (true)
  {
    static uint8_t battery_update_div = 0;
    // 此处编写需要循环执行的代码
    battery_update_div++;
    if (battery_update_div >= 100)
    {
        battery_update_div = 0;
        battery_monitor_update();
    }
    IPC_Push_Status_From_CoreA();
//    imu_debug_div++;
//    if (imu_debug_div >= 10)
//    {
//        imu_debug_div = 0;
//        printf("%.3f,%.3f,%.3f\r\n",
//               IMU_data.filter_result.roll,
//               IMU_data.filter_result.pitch,
//               IMU_data.filter_result.yaw);
//    }
    //printf("%f,%f,%f,%f ,%f,%f,%f\n",robot_pose.x,robot_pose.y,robot_pose.yaw,robot_pose.v,robot_pose.w,filter_data.accel[0],robot_pose.bias_ax);    
   // printf("Target Angle:%f\n",target_angle);
    // printf("%f,%f",temp_a,temp_b);
    // printf("V: %d \n",motor_value.receive_left_speed_data);
    // printf("%d,%d\n",Motor_Left,Motor_Right);

    // printf("P: %f \n",Speed_p);
    // printf("stand: %f \n",target_motor_Stand);
    // printf("target_v: %f \n",target_velocity);
//    printf("cnt: %d \n",test_pit10_cnt);
    
    system_delay_ms(1);
  }
}
