#include "app_headfile.h"

volatile bool system_fully_ready = false;

// ==================== Shared IPC memory ====================
// CM7_0 and CM7_1 exchange data through the following fixed RAM areas.

// Core A status area: 0x28001000.
#pragma location = IPC_CORE_A_SHARED_ADDR
__no_init CoreA_Status_t core_a_status;

// Core A log area: 0x28001400.
#pragma location = IPC_LOG_SHARED_ADDR
__no_init IpcLogBox_t ipc_log_box;

// Core B command area: 0x28001800. Keep it separate from status and log areas.
#pragma location = IPC_CORE_B_SHARED_ADDR
__no_init CoreB_Command_t core_b_cmd;

// The camera capture pipeline writes raw pixels to this fixed SRAM address.
// Reserve it in the Core0 image so camera frames cannot overwrite control data.
#pragma location = 0x28026024
__root __no_init uint8 core0_camera_capture_reserve[188U * 120U];

// ==================== Peripheral assignments ====================
#define PIT_IMU (PIT_CH0)
#define PIT_Balance (PIT_CH10)
#define PIT_Remote (PIT_CH12)
#define PIT_Engine (PIT_CH13)
#define PIT_IPC (PIT_CH11)
#define PIT_Jump (PIT_CH14)
#define PIT_Navigation (PIT_CH15)
#define LED1 (P19_0)
#define CORE0_ODOMETRY_SERIAL_OUTPUT_ENABLE (0)
// ==================== Global state ====================

// Calibration and test state.
int duty = 1;
int stop = 0;
extern float pitch1, roll1, yaw1;
float v_buchang;
/* Leg posture parameters. */
extern float x_current, y_current;
// Navigation logging state.
int stop_flash = 0; // Stop-recording flag.

int Bridge_position = 1;     // Bridge-related test position.
int yanshi_biaozhiwei = 100; // Bridge-related test marker.
int change_speed = 0;
extern float temp_a, temp_b;

// External control state.
extern RobotState_t robot_pose;
extern bool IMU_ready;
extern volatile int jump_stop;

extern uint32_t test_pit10_cnt;

// ==================== Application entry ====================
int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // Initialize the system clock.
  debug_init();                  // Initialize the debug interface.

  interrupt_global_disable(); // Keep interrupts disabled during initialization.

  // ==================== IPC initialization ====================
  IPC_Init_Shared_Memory();
  IPC_Check_And_Apply_Params_To_Core0();
  // ==================== GPIO and IMU ====================
  gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // LED1 defaults to high level.
  imu_init(LED1);
  pit_ms_init(PIT_IMU, 1);
  // ==================== Balance control ====================
#if !NAV_HAND_PUSH_TEST_MODE
  Balance_init();
#endif
  small_driver_uart_init(); // Motor driver UART.
  navi_data_init(); // Navigation positioning init. Update is gated by runtime navigation switch.
  Navi_Tracking_Init(); // Navigation route/control state init. Driver mode defaults to 0.
  // ==================== Remote controller ====================
  // SBUS uses UART4. Initialize it after other UART users to keep UART4 config intact.
#if !NAV_HAND_PUSH_TEST_MODE
  Remote_Init();
  pit_ms_init(PIT_Remote, 10);
#endif
  battery_monitor_init(); // Battery-voltage ADC.
  // ==================== Periodic tasks ====================
  // pit_ms_init(PIT_Engine, 20); // leg_control now runs from balance_control 20ms divider
  pit_ms_init(PIT_IPC, 10); // 10 ms IPC task.
#if !NAV_HAND_PUSH_TEST_MODE
  pit_ms_init(PIT_Jump, 1); // 1 ms jump-state task.
#endif
  pit_ms_init(PIT_Navigation, 5); // Navigation period matches ENCODER_DT=0.005f in navigation_data_handling.h.

#if !NAV_HAND_PUSH_TEST_MODE
    pit_ms_init(PIT_Balance, 1);
#endif
    interrupt_global_enable(0);
//
  system_delay_ms(1000); // Wait for peripherals to stabilize.

  interrupt_global_enable(0); // Enable interrupts after initialization.

  system_delay_ms(1000);

  system_fully_ready = true;


  while (true)
  {
    static uint8_t battery_update_div = 0;
    static uint8_t ipc_status_push_div = 0;
#if CORE0_ODOMETRY_SERIAL_OUTPUT_ENABLE
    static uint8_t radius_print_div = 0;
#endif
#if !NAV_HAND_PUSH_TEST_MODE
    static uint8_t navigation_task_div = 0;
#endif
    // Background tasks that must run continuously.
#if !NAV_HAND_PUSH_TEST_MODE
    if (IPC_Consume_Motor_Zero_Request_Core0())
    {
        small_driver_zero_calibration_start();
    }
    small_driver_zero_calibration_task();
#endif
    IPC_Nav_Group_Core0_Task();
    Bumpy_Action_Log_Task();

#if !NAV_HAND_PUSH_TEST_MODE
    navigation_task_div++;
    if (navigation_task_div >= 10)
    {
        navigation_task_div = 0;
        if (IMU_ready && (g_runtime_status.module_enable_mask & RUNTIME_MODULE_BIT(RUNTIME_MODULE_NAVIGATION)))
        {
            navi_ctrl.navi_mode_driver = (uint8_t)vofa_mode_driver;
            navi_ctrl.navi_mode_map = (uint8_t)vofa_mode_map;
        }
    }
#endif
    battery_update_div++;
    if (battery_update_div >= 100)
    {
        battery_update_div = 0;
        battery_monitor_update();
    }
    ipc_status_push_div++;
    if (ipc_status_push_div >= 10)
    {
        ipc_status_push_div = 0;
        IPC_Push_Status_From_CoreA();
    }
#if CORE0_ODOMETRY_SERIAL_OUTPUT_ENABLE
    radius_print_div++;
    if (radius_print_div >= 5)
    {
        radius_print_div = 0;
        printf("%.3f,%.3f,%.3f\r\n", robot_pose.radius, (float)robot_pose.x, (float)robot_pose.y);
    }
#endif

    // Core0 main-loop debug output disabled for balance timing test.
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
