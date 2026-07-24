#include "app_headfile.h"

volatile bool system_fully_ready = false;

// **************************** �˼�ͨ������ ****************************
// �� CM7_0 �� CM7_1 �ж���Ҫ������δ���

// �� Core A ��״̬���ݷ��� 0x28001000
#pragma location = IPC_CORE_A_SHARED_ADDR
__no_init CoreA_Status_t core_a_status;

// �� Core A ����־������� 0x28001400
#pragma location = IPC_LOG_SHARED_ADDR
__no_init IpcLogBox_t ipc_log_box;

// �� Core B ��ָ�����ݷ��� 0x28001800 (������־����������־��������ص�)
#pragma location = IPC_CORE_B_SHARED_ADDR
__no_init CoreB_Command_t core_b_cmd;

// **************************** �궨������ ****************************
// �ж�
#define PIT_IMU (PIT_CH0)
#define PIT_Balance (PIT_CH10)
#define PIT_Remote (PIT_CH12)
#define PIT_Engine (PIT_CH13)
#define PIT_IPC (PIT_CH11)
#define PIT_Jump (PIT_CH14)
#define PIT_Navigation (PIT_CH15)
#define LED1 (P19_0)
#define IMU_ACC_RAW_VOFA_TEST_MODE (0)
#define WHEEL_SPEED_DEBUG_PRINT_MODE (0)
#define NAV_ACC_DEBUG_PRINT_MODE (1)
// **************************** ȫ�ֱ������� ****************************

// ���+��������У�
int duty = 1;
int stop = 0;
extern float pitch1, roll1, yaw1;
float v_buchang;
/*�Ȳ���̬����*/
extern float x_current, y_current;
// ���ڶ����x��y��λ�õĵ���,��ؼ�סӦ�ø�һ����������ĳ�ʼֵ��������������ת��
int stop_flash = 0; // ������־λ

int Bridge_position = 1;     // �����ǹ�������ʱ����Ҫ�ģ��Ȳ�����Ӧ��
int yanshi_biaozhiwei = 100; // �����ǹ�������ʱ����Ҫ�ģ��Ȳ�����Ӧģʽ��
int change_speed = 0;
extern float temp_a, temp_b;

// �ⲿ��������
extern RobotState_t robot_pose;
extern bool IMU_ready;
extern volatile int jump_stop;

extern uint32_t test_pit10_cnt;

// **************************** ��װ���Բ��ֺ������� ****************************
#if IMU_ACC_RAW_VOFA_TEST_MODE
static void imu_acc_raw_vofa_test_loop(void)
{
  enum { ACC_AVG_WINDOW = 100 };
  static float ax_buf[ACC_AVG_WINDOW] = {0.0f};
  static float ay_buf[ACC_AVG_WINDOW] = {0.0f};
  static float az_buf[ACC_AVG_WINDOW] = {0.0f};
  static float ax_sum = 0.0f;
  static float ay_sum = 0.0f;
  static float az_sum = 0.0f;
  static uint8_t index = 0;
  static uint8_t count = 0;
  float ax = (float)imu660rc_acc_x;
  float ay = (float)imu660rc_acc_y;
  float az = (float)imu660rc_acc_z;
  float avg_x;
  float avg_y;
  float avg_z;

  ax_sum -= ax_buf[index];
  ay_sum -= ay_buf[index];
  az_sum -= az_buf[index];

  ax_buf[index] = ax;
  ay_buf[index] = ay;
  az_buf[index] = az;

  ax_sum += ax;
  ay_sum += ay;
  az_sum += az;

  index++;
  if (index >= ACC_AVG_WINDOW)
  {
    index = 0;
  }

  if (count < ACC_AVG_WINDOW)
  {
    count++;
  }

  avg_x = ax_sum / (float)count;
  avg_y = ay_sum / (float)count;
  avg_z = az_sum / (float)count;

  printf("%.0f,%.0f,%.0f,%.3f,%.3f,%.3f\n", ax, ay, az, avg_x, avg_y, avg_z);
}
#endif
// ================= ������ =================
int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // ʱ�����ü�ϵͳ��ʼ��<��ر���>
  debug_init();                  // ���Դ�����Ϣ��ʼ��
  // �˴���д�û����� ���������ʼ�������

  interrupt_global_disable(); // ��ʼ������֮ǰ�ȹر��ж�

    //=================================˫��ͨ�ų�ʼ��======================
  IPC_Init_Shared_Memory();
  IPC_Check_And_Apply_Params_To_Core0();
  //=================================GPIO��ʼ��=======================
  gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ
  //=================================IMU��ʼ��=======================
  imu_init(LED1);
#if IMU_ACC_RAW_VOFA_TEST_MODE
  interrupt_global_enable(0);
  system_delay_ms(500);
  while (true)
  {
    imu_acc_raw_vofa_test_loop();
    gpio_toggle_level(LED1);
    system_delay_ms(20);
  }
#else
  pit_ms_init(PIT_IMU, 1);
#endif
  //=================================ƽ�⶯����ʼ��========================
#if !NAV_HAND_PUSH_TEST_MODE
  Balance_init(); // ��ʼ��ƽ����ƣ�����Kalman�˲��ĸ���������
#endif
  small_driver_uart_init(); // ������ͨ�ų�ʼ��
  navi_data_init(); // Navigation positioning init. Update is gated by runtime navigation switch.
  Navi_Tracking_Init(); // Navigation route/control state init. Driver mode defaults to 0.
  //========================ң�������Ƴ�ʼ��==========================
  // SBUS uses UART4. Initialize it after other UART users to keep UART4 config intact.
#if !NAV_HAND_PUSH_TEST_MODE
  Remote_Init();
  pit_ms_init(PIT_Remote, 10);//10ms????????
#endif
  battery_monitor_init(); // ��ص�ѹ ADC ��ʼ��
  //=================================�����ʼ��======================
  // pit_ms_init(PIT_Engine, 20); // leg_control now runs from balance_control 20ms divider
  pit_ms_init(PIT_IPC, 10); // ˫�˲���ͬ�� 10ms ���ڼ��
#if !NAV_HAND_PUSH_TEST_MODE
  pit_ms_init(PIT_Jump, 1); // ��Ծ����״̬�� 1ms ����
#endif
  pit_ms_init(PIT_Navigation, 10); // Navigation period matches ENCODER_DT=0.010f in navigation_data_handling.h.

//  // === 1. ����ϵͳ��ʼ�� ===
//    navi_data_init();
//    Navi_Tracking_Init();
//
//    // �޸ģ��� PIT_Balance �� 3ms ��Ϊ 10ms ��ƥ�� ENCODER_DT (0.01f)
//    // ע�⣺�����ƽ�����ǿ��Ҫ�� 3ms�������޸ĵ����� ENCODER_DT Ϊ 0.03f ���� 3ms �жϷ�Ƶ����
#if !NAV_HAND_PUSH_TEST_MODE
    pit_ms_init(PIT_Balance, 1);
#endif
////    jump_stop = 1; // �� control.c �У�jump_stop=1 ���� PID ����ȫ�� 0
    interrupt_global_enable(0);
//
    system_delay_ms(1000); // ����ȴ�1�룬ȷ����������ȫ��ֹ����

  interrupt_global_enable(0); // �ڳ�ʼ����ʹ���ж�

  system_delay_ms(1000);
  
  system_fully_ready = true; // 1�������ٷ��е�����


  while (true)
  {
    static uint8_t battery_update_div = 0;
    static uint8_t ipc_status_push_div = 0;
#if WHEEL_SPEED_DEBUG_PRINT_MODE
    static uint8_t wheel_speed_print_div = 0;
#endif
#if NAV_ACC_DEBUG_PRINT_MODE
    static uint8_t nav_acc_print_div = 0;
#endif
#if !NAV_HAND_PUSH_TEST_MODE
    static uint8_t navigation_task_div = 0;
#endif
    // �˴���д��Ҫѭ��ִ�еĴ���
#if !NAV_HAND_PUSH_TEST_MODE
    if (IPC_Consume_Motor_Zero_Request_Core0())
    {
        small_driver_zero_calibration_start();
    }
    small_driver_zero_calibration_task();
#endif

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
#if WHEEL_SPEED_DEBUG_PRINT_MODE
    wheel_speed_print_div++;
    if (wheel_speed_print_div >= 100)
    {
        wheel_speed_print_div = 0;
        printf("wheel_speed,L:%d,R:%d\r\n",
               motor_value.receive_left_speed_data,
               motor_value.receive_right_speed_data);
    }
#endif
#if NAV_ACC_DEBUG_PRINT_MODE
    nav_acc_print_div++;
    if (nav_acc_print_div >= 50)
    {
        nav_acc_print_div = 0;
        printf("%f,%f,%f\r\n",
               filter_data.accel[0],
               filter_data.accel[1],
               filter_data.accel[2]);
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
