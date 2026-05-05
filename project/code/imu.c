#include "imu.h"
#include "zf_common_headfile.h"
#include "zf_device_imu660rc.h"
#include "process_rx.h"

float now_velocity_x = 0.0f;
float now_velocity_y = 0.0f;
float now_velocity_z = 0.0f;
int add_ms = 0;
imu_filter_params_t filter;
IMU_t IMU_data;

void imu_get_3D(float *yaw, float *pitch, float *roll)
{
    if (yaw == NULL || pitch == NULL || roll == NULL)
    {
        return;
    }

    *yaw = IMU_data.filter_result.yaw;
    *pitch = IMU_data.filter_result.pitch;
    *roll = IMU_data.filter_result.roll;
}

void imu_attitude(void)
{
    ICM_getEulerianAngles();

    IMU_data.accel[0] = icm_data.acc_x;
    IMU_data.accel[1] = icm_data.acc_y;
    IMU_data.accel[2] = icm_data.acc_z;

    IMU_data.gyro[0] = icm_data.gyro_x * RAD_TO_DEG;
    IMU_data.gyro[1] = icm_data.gyro_y * RAD_TO_DEG;
    IMU_data.gyro[2] = icm_data.gyro_z * RAD_TO_DEG;

    IMU_data.mag[0] = 0.0f;
    IMU_data.mag[1] = 0.0f;
    IMU_data.mag[2] = 0.0f;

    IMU_data.filter_result.roll = eulerAngle.roll;
    IMU_data.filter_result.pitch = eulerAngle.pitch;
    IMU_data.filter_result.yaw = eulerAngle.yaw;

    IMU_data.filter_result.unbiased_gyro_x = IMU_data.gyro[0];
    IMU_data.filter_result.unbiased_gyro_y = IMU_data.gyro[1];
    IMU_data.filter_result.unbiased_gyro_z = IMU_data.gyro[2];
}

void imu_init(gpio_pin_enum pin)
{
    while (1)
    {
        if (imu660rc_init(IMU660RC_QUARTERNION_480HZ))
        {
            printf("\r\nIMU660RC init error.");
        }
        else
        {
            break;
        }
        gpio_toggle_level(pin);
    }

    imu_rx_init();
}
