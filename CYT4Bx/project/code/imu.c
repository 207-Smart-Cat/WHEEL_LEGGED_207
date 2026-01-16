#include "imu.h"
#include "zf_common_headfile.h"
float pitch1,roll1,yaw1;
float now_velocity_x = 0,now_velocity_y = 0,now_velocity_z = 0;
int add_ms=0;
Attitude_3D_Kalman filter;
IMU_t IMU_data;

void imu_get(void)
{
    imu963ra_get_acc();                                                         // 获取 IMU963RA 的加速度测量数值
    imu963ra_get_gyro();                                                        // 获取 IMU963RA 的角速度测量数值
    imu963ra_get_mag();
}
//姿态解算
void imu_attitude(void)
{
    imu_get();
    /********************卡尔曼滤波**********************************/
    cal(imu963ra_acc_x,  imu963ra_acc_y,  imu963ra_acc_z, imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z,
                      imu963ra_mag_x,   imu963ra_mag_y,  imu963ra_mag_z);
    Kalman_update(&IMU_data.filter_result,&filter,IMU_data.accel[0],IMU_data.accel[1],IMU_data.accel[2],IMU_data.gyro[0],IMU_data.gyro[1],IMU_data.gyro[2],IMU_data.mag[0],IMU_data.mag[1],IMU_data.mag[2]);
    /*********截断滤波****************************/
//    if(func_abs(pitch1 - IMU_data.filter_result.pitch) <= 0.1)
//        IMU_data.filter_result.pitch = pitch1;
//    if(func_abs(roll1 - IMU_data.filter_result.roll) <= 0.1)
//       IMU_data.filter_result.roll = roll1;
//    if(func_abs(yaw1 - IMU_data.filter_result.yaw) <= 0.6)
//        IMU_data.filter_result.yaw = yaw1;
//    pitch1 = IMU_data.filter_result.pitch;
//    roll1 = IMU_data.filter_result.roll;
    yaw1 = IMU_data.filter_result.yaw;
}

void imu_init (gpio_pin_enum pin)
{
    while(1)
    {
        if(imu963ra_init())
        {
            printf("\r\nIMU963RA init error.");      // IMU963RA 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(pin); // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
    Kalman_init(&filter, 0.005f, 0.001f, 0.01f, 0.001f, 0.05f, 0.015f); // 初始化卡尔曼滤波器
}
