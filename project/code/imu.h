#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float unbiased_gyro_x;
    float unbiased_gyro_y;
    float unbiased_gyro_z;
} Attitude_3D_t;

typedef struct
{
    float gyro[3];
    float accel[3];
    float mag[3];
    float temp;
    Attitude_3D_t filter_result;
} IMU_t;

typedef struct
{
    float Qyaw;
    float Qpitch_roll;
    float Qgyrobias;
    float Ryaw;
    float Rpitch_roll;
} imu_filter_params_t;

extern imu_filter_params_t filter;
extern IMU_t IMU_data;
extern float now_velocity_x;
extern float now_velocity_y;
extern float now_velocity_z;

void imu_init(gpio_pin_enum pin);
void imu_attitude(void);
void imu_get_3D(float *yaw, float *pitch, float *roll);

#endif
