#ifndef CODE_IMU_H_
#define CODE_IMU_H_
#include "kalman_rm.h"

extern Attitude_3D_Kalman filter;
extern IMU_t IMU_data;
extern float now_velocity_x;
extern float now_velocity_y;
extern float now_velocity_z;

void imu_init (gpio_pin_enum pin);


void imu_attitude(void);


#endif /* CODE_IMU_H_ */
