#include "process_rx.h"

#define LPF_ALPHA               (0.6f)
#define delta_T                 (0.005f)
#define GyroOffset_Xdata        (9.0f)
#define GyroOffset_Ydata        (-6.2f)
#define GyroOffset_Zdata        (-0.4f)
#define AccOffset_Xdata         (-15.0f)
#define AccOffset_Ydata         (53.0f)
#define AccOffset_Zdata         (-30.0f)
#define AccScale_Xdata          (1.000821f)
#define AccScale_Ydata          (1.002777f)
#define AccScale_Zdata          (0.994748f)
#define param_Kp                (0.2f)
#define param_Ki                (0.0f)
#define PROCESS_RX_GYRO_INT_LIMIT   (1.0f)

float acc_data[3] = {0.0f, 0.0f, 0.0f};
float gyro_data[3] = {0.0f, 0.0f, 0.0f};
float I_ex = 0.0f;
float I_ey = 0.0f;
float I_ez = 0.0f;
quater_param_t_rx Q_info;
euler_param_t_rx eulerAngle;
icm_param_t_rx icm_data;

float process_rx_gyro_x_dps(float gyro_x_raw)
{
    return imu660rc_gyro_transition(gyro_x_raw - GyroOffset_Xdata);
}

float process_rx_gyro_y_dps(float gyro_y_raw)
{
    return imu660rc_gyro_transition(gyro_y_raw - GyroOffset_Ydata);
}

float process_rx_gyro_z_dps(float gyro_z_raw)
{
    return imu660rc_gyro_transition(gyro_z_raw - GyroOffset_Zdata);
}
static float normalize_angle_180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void imu_rx_init(void)
{
    acc_data[0] = 0.0f;
    acc_data[1] = 0.0f;
    acc_data[2] = 0.0f;
    gyro_data[0] = 0.0f;
    gyro_data[1] = 0.0f;
    gyro_data[2] = 0.0f;

    Q_info.q0 = 1.0f;
    Q_info.q1 = 0.0f;
    Q_info.q2 = 0.0f;
    Q_info.q3 = 0.0f;

    eulerAngle.pitch = 0.0f;
    eulerAngle.roll = 0.0f;
    eulerAngle.yaw = 0.0f;
    eulerAngle.last_yaw = 0.0f;
    eulerAngle.last_roll = 0.0f;
    eulerAngle.Dirchange = 0;

    I_ex = 0.0f;
    I_ey = 0.0f;
    I_ez = 0.0f;
}

void imu_update_acc(float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z)
{
    acc_data[0] = LPF_ALPHA * acc_data[0] + (1.0f - LPF_ALPHA) * acc_x;
    acc_data[1] = LPF_ALPHA * acc_data[1] + (1.0f - LPF_ALPHA) * acc_y;
    acc_data[2] = LPF_ALPHA * acc_data[2] + (1.0f - LPF_ALPHA) * acc_z;
    gyro_data[0] = LPF_ALPHA * gyro_data[0] + (1.0f - LPF_ALPHA) * gyro_x;
    gyro_data[1] = LPF_ALPHA * gyro_data[1] + (1.0f - LPF_ALPHA) * gyro_y;
    gyro_data[2] = LPF_ALPHA * gyro_data[2] + (1.0f - LPF_ALPHA) * gyro_z;
}

void process_rx_calc_6axis_quaternion(float acc_x, float acc_y, float acc_z,
                                      float gyro_x, float gyro_y, float gyro_z,
                                      float dt,
                                      quater_param_t_rx *q,
                                      euler_param_t_rx *euler)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0, q1, q2, q3;
    float q0_old, q1_old, q2_old, q3_old;
    float pitch_sin;
    float half_dt;

    if (q == NULL || euler == NULL || dt <= 0.0f)
    {
        return;
    }

    norm = sqrtf(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    if (norm < 1e-6f)
    {
        return;
    }

    acc_x /= norm;
    acc_y /= norm;
    acc_z /= norm;

    q0 = q->q0;
    q1 = q->q1;
    q2 = q->q2;
    q3 = q->q3;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = acc_y * vz - acc_z * vy;
    ey = acc_z * vx - acc_x * vz;
    ez = acc_x * vy - acc_y * vx;

    if (param_Ki > 0.0f)
    {
        I_ex += param_Ki * ex * dt;
        I_ey += param_Ki * ey * dt;
        I_ez += param_Ki * ez * dt;

        if (I_ex > PROCESS_RX_GYRO_INT_LIMIT) I_ex = PROCESS_RX_GYRO_INT_LIMIT;
        if (I_ex < -PROCESS_RX_GYRO_INT_LIMIT) I_ex = -PROCESS_RX_GYRO_INT_LIMIT;
        if (I_ey > PROCESS_RX_GYRO_INT_LIMIT) I_ey = PROCESS_RX_GYRO_INT_LIMIT;
        if (I_ey < -PROCESS_RX_GYRO_INT_LIMIT) I_ey = -PROCESS_RX_GYRO_INT_LIMIT;
        if (I_ez > PROCESS_RX_GYRO_INT_LIMIT) I_ez = PROCESS_RX_GYRO_INT_LIMIT;
        if (I_ez < -PROCESS_RX_GYRO_INT_LIMIT) I_ez = -PROCESS_RX_GYRO_INT_LIMIT;

        gyro_x += I_ex;
        gyro_y += I_ey;
        gyro_z += I_ez;
    }
    else
    {
        I_ex = 0.0f;
        I_ey = 0.0f;
        I_ez = 0.0f;
    }

    gyro_x += param_Kp * ex;
    gyro_y += param_Kp * ey;
    gyro_z += param_Kp * ez;

    half_dt = 0.5f * dt;
    q0_old = q0;
    q1_old = q1;
    q2_old = q2;
    q3_old = q3;
    q0 += (-q1_old * gyro_x - q2_old * gyro_y - q3_old * gyro_z) * half_dt;
    q1 += ( q0_old * gyro_x + q2_old * gyro_z - q3_old * gyro_y) * half_dt;
    q2 += ( q0_old * gyro_y - q1_old * gyro_z + q3_old * gyro_x) * half_dt;
    q3 += ( q0_old * gyro_z + q1_old * gyro_y - q2_old * gyro_x) * half_dt;

    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm < 1e-6f)
    {
        return;
    }

    q->q0 = q0 / norm;
    q->q1 = q1 / norm;
    q->q2 = q2 / norm;
    q->q3 = q3 / norm;

    euler->roll = atan2f(2.0f * (q->q0 * q->q1 + q->q2 * q->q3),
                         1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2)) * RAD_TO_DEG;
    pitch_sin = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (pitch_sin > 1.0f) pitch_sin = 1.0f;
    if (pitch_sin < -1.0f) pitch_sin = -1.0f;
    euler->pitch = asinf(pitch_sin) * RAD_TO_DEG;
    euler->yaw = atan2f(2.0f * (q->q0 * q->q3 + q->q1 * q->q2),
                        1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3)) * RAD_TO_DEG;
}

void ICM_getEulerianAngles(void)
{
    imu_update_acc((float)imu660rc_acc_x,
                   (float)imu660rc_acc_y,
                   (float)imu660rc_acc_z,
                   (float)imu660rc_gyro_x,
                   (float)imu660rc_gyro_y,
                   (float)imu660rc_gyro_z);

    icm_data.acc_x = imu660rc_acc_transition((acc_data[0] - AccOffset_Xdata) * AccScale_Xdata);
    icm_data.acc_y = imu660rc_acc_transition((acc_data[1] - AccOffset_Ydata) * AccScale_Ydata);
    icm_data.acc_z = imu660rc_acc_transition((acc_data[2] - AccOffset_Zdata) * AccScale_Zdata);

    icm_data.gyro_x = process_rx_gyro_x_dps(gyro_data[0]) * DEG_TO_RAD;
    icm_data.gyro_y = process_rx_gyro_y_dps(gyro_data[1]) * DEG_TO_RAD;
    icm_data.gyro_z = process_rx_gyro_z_dps(gyro_data[2]) * DEG_TO_RAD;

    Q_info.q0 = imu660rc_quarternion[3];
    Q_info.q1 = imu660rc_quarternion[0];
    Q_info.q2 = imu660rc_quarternion[1];
    Q_info.q3 = imu660rc_quarternion[2];

    eulerAngle.roll = -imu660rc_roll;
    eulerAngle.pitch = imu660rc_pitch;
    eulerAngle.yaw = normalize_angle_180(-imu660rc_yaw);

    if ((eulerAngle.yaw - eulerAngle.last_yaw) < -350.0f)
    {
        eulerAngle.Dirchange++;
    }
    else if ((eulerAngle.yaw - eulerAngle.last_yaw) > 350.0f)
    {
        eulerAngle.Dirchange--;
    }

    eulerAngle.last_roll = eulerAngle.roll;
    eulerAngle.last_yaw = eulerAngle.yaw;
}
