#include "imu.h"
#include "zf_common_headfile.h"
#include "param.h"
float now_velocity_x = 0, now_velocity_y = 0, now_velocity_z = 0;
int add_ms = 0;
Attitude_3D_Kalman filter;
IMU_t IMU_data;
void imu_get(void)
{
    imu963ra_get_acc();  // 获取 IMU963RA 的加速度测量数值
    imu963ra_get_gyro(); // 获取 IMU963RA 的角速度测量数值
    imu963ra_get_mag();
}

void imu_update_raw(void)
{
    imu_get();
    cal(imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z, imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z,
        imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z);
}
// 获取滤波后姿态角（yaw，pitch，roll）
void imu_get_3D(float *yaw, float *pitch, float *roll)
{
    // 空指针校验（避免程序崩溃）
    if (yaw == NULL || pitch == NULL || roll == NULL)
    {
        return; // 返回错误码，告知调用者参数错误
    }

    *yaw = IMU_data.filter_result.yaw;
    *pitch = IMU_data.filter_result.pitch;
    *roll = IMU_data.filter_result.roll;
}
// 姿态解算
void imu_attitude(void)
{
    /********************卡尔曼滤波**********************************/
    Kalman_update(&IMU_data.filter_result, &filter, IMU_data.accel[0], IMU_data.accel[1], IMU_data.accel[2],
                  IMU_data.gyro[0], IMU_data.gyro[1], IMU_data.gyro[2],
                  IMU_data.mag[0], IMU_data.mag[1], IMU_data.mag[2]);
    // 控制链使用连续姿态输出，不再对姿态角做硬死区截断。
}

void imu_init(gpio_pin_enum pin)
{
    while (1)
    {
        if (imu963ra_init())
        {
            printf("\r\nIMU963RA init error."); // IMU963RA 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(pin); // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
     Kalman_init(&filter, 0.005f, 0.001f, 0.01f, 0.001f, 0.05f, 0.015f); // 初始化卡尔曼滤波器
   // Kalman_init(&filter, 1.0f, 0.05f, RPITCH_ROLL, 0.05f, 0.05f, QPITCH_ROLL); // 初始化卡尔曼滤波器
}
/*
//姿态解算
void imu_attitude(void)
{
    imu_get();
    // 1. 物理量转换
    cal(imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z,
        imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z,
        0, 0, 0); // 磁力计原始值传 0 即可

    // 2. 核心修改：将磁力计输入改为固定的虚拟矢量 (1.0, 0.0, 0.0)
    // 这样做是为了让 measurement[0] (Yaw测量值) 保持恒定，不干扰陀螺仪积分
    Kalman_update(&IMU_data.filter_result, &filter,
                  IMU_data.accel[0], IMU_data.accel[1], IMU_data.accel[2],
                  IMU_data.gyro[0],  IMU_data.gyro[1],  IMU_data.gyro[2],
                  1.0f, 0.0f, 0.0f); // 虚拟磁场

    yaw1 = IMU_data.filter_result.yaw;
}

*/


// 定义用于存储极值的全局变量，初始值给得极端一点以确保能被覆盖
float MagX_Max = -99999.0f;
float MagX_Min =  99999.0f;
float MagY_Max = -99999.0f;
float MagY_Min =  99999.0f;

// 地磁计校准例程
void imu_mag_calibration_routine(void)
{
    int print_count = 0;
    
    printf("\r\n========================================\r\n");
    printf("====== 地磁计校准程序已启动 ======\r\n");
    printf("请在水平地面上缓慢旋转机器人至少2-3圈...\r\n");
    printf("========================================\r\n");

    while (1)
    {
        // 1. 获取最新传感器原始数据 (该函数会更新 imu963ra_mag_x 等全局变量)
        imu_get(); 

        // 2. 转换为物理值
        // 这里借用你原本在 cal() 函数里的转换方法
        float current_mag_x = imu963ra_mag_transition(imu963ra_mag_x);
        float current_mag_y = imu963ra_mag_transition(imu963ra_mag_y);

        // 3. 比较并更新极值
        if (current_mag_x > MagX_Max) MagX_Max = current_mag_x;
        if (current_mag_x < MagX_Min) MagX_Min = current_mag_x;

        if (current_mag_y > MagY_Max) MagY_Max = current_mag_y;
        if (current_mag_y < MagY_Min) MagY_Min = current_mag_y;

        // 4. 降低打印频率，避免串口卡死 (假设循环很快，每 100 次打印一次)
        print_count++;
        if (print_count >= 100)
        {
            printf("\r\n--- 正在校准中，请保持缓慢旋转 ---\r\n");
            printf("当前 X: %.2f  |  当前 Y: %.2f\r\n", current_mag_x, current_mag_y);
            printf("记录 X_Min: %.2f  |  X_Max: %.2f\r\n", MagX_Min, MagX_Max);
            printf("记录 Y_Min: %.2f  |  Y_Max: %.2f\r\n", MagY_Min, MagY_Max);
            print_count = 0;
        }

        // 5. 稍微延时一下，具体延时函数请替换为你工程里的底层延时函数 (例如 system_delay_ms)
        // system_delay_ms(10); 
    }
}
