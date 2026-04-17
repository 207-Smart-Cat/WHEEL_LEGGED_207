#include "kalman_rm.h"
#include "zf_common_headfile.h"
extern IMU_t IMU_data;

//é‰ãƒ¦ç°®éˆî†ç…¡é”›å±½ç—‰ç’‡î…å”¬é®ä¾Šç´æµ£å—˜æ§¸é—‚î‡€î•½æ¶“å¶…ã‡
// äº’è¡¥æ»¤æ³¢å®šä¹‰

//void cal(int16 imu963ra_acc_x,  int16 imu963ra_acc_y, int16 imu963ra_acc_z,int16 imu963ra_gyro_x, int16 imu963ra_gyro_y,int16 imu963ra_gyro_z,
//        int16 imu963ra_mag_x,  int16 imu963ra_mag_y,  int16 imu963ra_mag_z)
//{
//     IMU_data.accel[0] = imu963ra_acc_transition(imu963ra_acc_x);
//     IMU_data.accel[1] = imu963ra_acc_transition(imu963ra_acc_y);
//     IMU_data.accel[2] = imu963ra_acc_transition(imu963ra_acc_z);
//     IMU_data.gyro[0] = imu963ra_gyro_transition(imu963ra_gyro_x);
//     IMU_data.gyro[1] = imu963ra_gyro_transition(imu963ra_gyro_y);
//     IMU_data.gyro[2] = imu963ra_gyro_transition(imu963ra_gyro_z);
//     IMU_data.mag[0] = imu963ra_mag_transition(imu963ra_mag_x);
//     IMU_data.mag[1] = imu963ra_mag_transition(imu963ra_mag_y);
//     IMU_data.mag[2] = imu963ra_mag_transition(imu963ra_mag_z);
//}

float mag_offset_x = -0.080f;
float mag_offset_y =  0.040f;
float mag_scale_x  =  1.000f;
float mag_scale_y  =  1.050f;

void cal(int16 imu963ra_acc_x,  int16 imu963ra_acc_y, int16 imu963ra_acc_z,
         int16 imu963ra_gyro_x, int16 imu963ra_gyro_y,int16 imu963ra_gyro_z,
         int16 imu963ra_mag_x,  int16 imu963ra_mag_y,  int16 imu963ra_mag_z)
{
     // 1. åŠ é€Ÿåº¦å’Œè§’é€Ÿåº¦ç‰©ç†é‡è½¬æ?
     IMU_data.accel[0] = imu963ra_acc_transition(imu963ra_acc_x);
     IMU_data.accel[1] = imu963ra_acc_transition(imu963ra_acc_y);
     IMU_data.accel[2] = imu963ra_acc_transition(imu963ra_acc_z);
     
     IMU_data.gyro[0] = imu963ra_gyro_transition(imu963ra_gyro_x);
     IMU_data.gyro[1] = imu963ra_gyro_transition(imu963ra_gyro_y);
     IMU_data.gyro[2] = imu963ra_gyro_transition(imu963ra_gyro_z);

     // 2. æå–ç£åŠ›è®¡åŸå§‹ç‰©ç†å€?
     float raw_mag_x = imu963ra_mag_transition(imu963ra_mag_x);
     float raw_mag_y = imu963ra_mag_transition(imu963ra_mag_y);
     float raw_mag_z = imu963ra_mag_transition(imu963ra_mag_z);

     // 3. åº”ç”¨ç¡¬ç£å’Œè½¯ç£æ ¡å‡†ï¼ˆä½¿ç”¨å…¨å±€å˜é‡ï¼Œä¸å†å†™æ­»ï¼‰
     IMU_data.mag[0] = (raw_mag_x - mag_offset_x) * mag_scale_x;
     IMU_data.mag[1] = (raw_mag_y - mag_offset_y) * mag_scale_y;
     IMU_data.mag[2] = raw_mag_z;
     
}
void Kalman_init(Attitude_3D_Kalman* filter,float Abtastzeit_s, float Qyaw, float Qpitch_roll, float Qgyrobias, float Ryaw, float Rpitch_roll)
{
    filter->Abtastzeit_s = Abtastzeit_s;
    filter->Qyaw = Qyaw;
    filter->Qpitch_roll = Qpitch_roll;
    filter->Qgyrobias = Qgyrobias;
    filter->Ryaw = Ryaw;
    filter->Rpitch_roll = Rpitch_roll;
    filter->P1_1 = filter->P2_2 = filter->P3_3 = filter->P4_4 = filter->P5_5 = filter->P6_6 = 1;
    filter->P3_4 = filter->P3_5 = filter->P3_6 = filter->P2_5 = filter->P2_6 = filter->P1_6 = 0;
    filter->yaw = filter->pitch = filter->roll = 0;
    filter->xbias = 0;
    filter->ybias = 0;
    filter->zbias = 0;
}

void Kalman_update(Attitude_3D_t* result, Attitude_3D_Kalman* filter, float Acc_X, float Acc_Y, float Acc_Z, float Gyro_X, float Gyro_Y, float Gyro_Z, float Mag_X, float Mag_Y, float Mag_Z)
    {
    /*********  Variablen  **********/
    //yaw pitch roll
    float measurement[3]={0,0,0};

    //Innovation, yaw pitch roll
    float y[3];

    //xé˜èˆµï¿½ä½¸ï¿½ï¿½
    float K1_1, K6_1, K2_2, K5_2, K6_2, K3_3, K4_3, K5_3, K6_3;

    /*********  å¨´å¬®å™ºéŠè‰°î…¸ç» ï¿½  **********/
    //Winkel in RAD fçœ‰r Winkelfunktionen
    float roll_rad;
    float pitch_rad;
    roll_rad = (double)filter->roll * DEG_TO_RAD;
    pitch_rad = (double)filter->pitch * DEG_TO_RAD;
    //Pitch und Roll aus Accelerometer
    float acc_xz_norm = sqrtf(Acc_X * Acc_X + Acc_Z * Acc_Z);
    float acc_yz_norm = sqrtf(Acc_Y * Acc_Y + Acc_Z * Acc_Z);
    float acc_norm = sqrtf(Acc_X * Acc_X + Acc_Y * Acc_Y + Acc_Z * Acc_Z);
    float acc_error = fabsf(acc_norm - 9.81f);
    float dynamic_Rpitch_roll = filter->Rpitch_roll;

    if(acc_xz_norm > 1e-6f && acc_yz_norm > 1e-6f)
    {
        measurement[2] = (double)atanf(Acc_Y / acc_xz_norm) * RAD_TO_DEG;
        measurement[1] = (double)atanf(-1 * Acc_X / acc_yz_norm) * RAD_TO_DEG;
    }

    // ¶¯Ì¬¹¤¿öÏÂ¼õÈõ¼ÓËÙ¶È¼ÆĞŞÕı£¬±ÜÃâÍÈ²¿¶¯×÷ºÍµØÃæ³å»÷°ÑÏß¼ÓËÙ¶Èµ±³É×ËÌ¬±ä»¯¡£
    if (acc_error > 0.15f)
    {
        float acc_scale = 1.0f + (acc_error - 0.15f) * 6.0f;
        if (acc_scale > 12.0f)
        {
            acc_scale = 12.0f;
        }
        dynamic_Rpitch_roll *= acc_scale;
    }
    //Magnetometer Messung in horizontale Ebene transformieren
    float XH;
    XH = Mag_X * cosf(pitch_rad) + Mag_Z * cosf(roll_rad) * sinf(pitch_rad) + Mag_Y * sinf(pitch_rad) * sinf(roll_rad);
    float YH;
    YH = Mag_Y * cosf(roll_rad) - Mag_Z * sinf(roll_rad);

    //Yaw mit Magnetometer berechnen
    measurement[0] = (double)atan2f(XH,YH) * RAD_TO_DEG;

    //prevent 360deg jump
    if ((measurement[0] - filter->yaw) > 180)
    {
        measurement[0] -= 360;
    }
    else if ((measurement[0] - filter->yaw) < -180)
    {
        measurement[0] += 360;
    }

    /*********  Kalman Algorithmus  **********/
    //new state estimates mit Vorwç›²rtstransitions Matrix
    filter->yaw += ((cosf(roll_rad) * (Gyro_Z - filter->zbias)) / cosf(pitch_rad) + (sinf(roll_rad) * (Gyro_Y - filter->ybias)) / cosf(pitch_rad)) * filter->Abtastzeit_s;
    filter->pitch += (cosf(roll_rad) * (Gyro_Y - filter->ybias) - sinf(roll_rad) * (Gyro_Z - filter->zbias)) * filter->Abtastzeit_s;
    filter->roll += (Gyro_X - filter->xbias + cosf(roll_rad) * tanf(pitch_rad) * (Gyro_Z - filter->zbias) + tanf(pitch_rad) * sinf(roll_rad) * (Gyro_Y - filter->ybias)) * filter->Abtastzeit_s;

    //new error covariance estimate
    float P4_4_temp = filter->P4_4;
    float P5_5_temp = filter->P5_5;
    float P6_6_temp = filter->P6_6;

    filter->P1_1 = filter->P1_1 + filter->Qyaw - (filter->P1_6 * filter->Abtastzeit_s * cosf(roll_rad)) / cosf(pitch_rad) - (filter->Abtastzeit_s * cosf(roll_rad) * (filter->P1_6 - (filter->P6_6 * filter->Abtastzeit_s * cosf(roll_rad)) / cosf(pitch_rad))) / cosf(pitch_rad);

    filter->P2_2 = filter->P2_2 + filter->Qpitch_roll - filter->Abtastzeit_s * cosf(roll_rad) * (filter->P2_5 - filter->P5_5 * filter->Abtastzeit_s * cosf(roll_rad)) + filter->Abtastzeit_s * sinf(roll_rad) * (filter->P2_6 + filter->P6_6 * filter->Abtastzeit_s * sinf(roll_rad)) - filter->P2_5 * filter->Abtastzeit_s * cosf(roll_rad) + filter->P2_6 * filter->Abtastzeit_s * sinf(roll_rad);

    filter->P3_3 = filter->P3_3 + filter->Qpitch_roll - filter->P3_4 * filter->Abtastzeit_s - filter->Abtastzeit_s * (filter->P3_4 - filter->P4_4 * filter->Abtastzeit_s) - filter->Abtastzeit_s * tanf(pitch_rad) * sinf(roll_rad) * (filter->P3_5 - filter->P5_5 * filter->Abtastzeit_s * tanf(pitch_rad) * sinf(roll_rad)) - filter->P3_6 * filter->Abtastzeit_s * cosf(roll_rad) * tanf(pitch_rad) - filter->P3_5 * filter->Abtastzeit_s * tanf(pitch_rad) * sinf(roll_rad) - filter->Abtastzeit_s * cosf(roll_rad) * tanf(pitch_rad) * (filter->P3_6 - filter->P6_6 * filter->Abtastzeit_s * cosf(roll_rad) * tanf(pitch_rad));

    filter->P4_4 += filter->Qgyrobias;
    filter->P5_5 += filter->Qgyrobias;
    filter->P6_6 += filter->Qgyrobias;

    filter->P1_6 -= (P6_6_temp * filter->Abtastzeit_s * cosf(roll_rad)) / cosf(pitch_rad);
    filter->P2_5 -= P5_5_temp * filter->Abtastzeit_s * cosf(roll_rad);
    filter->P2_6 += P6_6_temp * filter->Abtastzeit_s * sinf(roll_rad);
    filter->P3_4 -= P4_4_temp * filter->Abtastzeit_s;
    filter->P3_5 -= P5_5_temp * filter->Abtastzeit_s * tanf(pitch_rad) * sinf(roll_rad);
    filter->P3_6 -= P6_6_temp * filter->Abtastzeit_s * cosf(roll_rad) * tanf(pitch_rad);

    //Innovation berechnen
    y[0] = measurement[0] - filter->yaw;
    y[1] = measurement[1] - filter->pitch;
    y[2] = measurement[2] - filter->roll;

    //Kalman Gain berechnen
    K1_1 = filter->P1_1 / (filter->P1_1 + filter->Ryaw);
    K2_2 = filter->P2_2 / (filter->P2_2 + dynamic_Rpitch_roll);
    K3_3 = filter->P3_3 / (filter->P3_3 + dynamic_Rpitch_roll);
    K4_3 = filter->P3_4 / (filter->P3_3 + dynamic_Rpitch_roll);
    K5_2 = filter->P2_5 / (filter->P2_2 + dynamic_Rpitch_roll);
    K5_3 = filter->P3_5 / (filter->P3_3 + dynamic_Rpitch_roll);
    K6_1 = filter->P1_6 / (filter->P1_1 + filter->Ryaw);
    K6_2 = filter->P2_6 / (filter->P2_2 + dynamic_Rpitch_roll);
    K6_3 = filter->P3_6 / (filter->P3_3 + dynamic_Rpitch_roll);

    //corrected state estimate
    filter->yaw += K1_1 * y[0];
    filter->pitch += K2_2 * y[1];
    filter->roll += K3_3 * y[2];

    filter->xbias += K4_3 * y[2];
    filter->ybias += K5_2 * y[1] + K5_3 * y[2];
    filter->zbias += K6_1 * y[0] + K6_2 * y[1] + K6_3 * y[2];

    //corrected error covariance matrix
    filter->P1_1 -= filter->P1_1 * K1_1;
    filter->P2_2 -= filter->P2_2 * K2_2;
    filter->P3_3 -= filter->P3_3 * K3_3;
    filter->P4_4 -= filter->P3_4 * K4_3;
    filter->P5_5 -= (K5_2 * filter->P2_5 + K5_3 * filter->P3_5);
    filter->P6_6 -= (K6_1 * filter->P1_6 + K6_2 * filter->P2_6 + K6_3 * filter->P3_6);

    filter->P1_6 -= filter->P1_6 * K1_1;
    filter->P2_5 -= filter->P2_5 * K2_2;
    filter->P2_6 -= filter->P2_6 * K2_2;
    filter->P3_4 -= filter->P3_4 * K3_3;
    filter->P3_5 -= filter->P3_5 * K3_3;
    filter->P3_6 -= filter->P3_6 * K3_3;

    //contrain yaw to +-180deg
    if (filter->yaw > 180)
    {
        filter->yaw -= 360;
    }
    else if (filter->yaw < -180)
    {
        filter->yaw += 360;
    }

    Attitude_3D_t return_Data = {filter->yaw, filter->pitch, filter->roll, Gyro_X-filter->xbias, Gyro_Y-filter->ybias, Gyro_Z-filter->zbias};
    *result = return_Data;
}



