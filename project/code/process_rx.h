#ifndef CODE_PROCESS_RX_H_
#define CODE_PROCESS_RX_H_

#include "zf_common_headfile.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t_rx;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t_rx;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float last_yaw;
    float last_roll;
    uint8 Dirchange;
} euler_param_t_rx;

extern float acc_data[3];
extern float gyro_data[3];
extern quater_param_t_rx Q_info;
extern euler_param_t_rx eulerAngle;
extern icm_param_t_rx icm_data;

void imu_update_acc(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z);
void process_rx_calc_6axis_quaternion(float acc_x, float acc_y, float acc_z,
                                      float gyro_x, float gyro_y, float gyro_z,
                                      float dt,
                                      quater_param_t_rx *q,
                                      euler_param_t_rx *euler);
void imu_rx_init(void);
void ICM_getEulerianAngles(void);

#endif
