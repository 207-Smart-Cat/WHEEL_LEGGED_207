#include <stdio.h>
#include <math.h>
#include "FiveBarLinkageData.h"
#include "param.h"


#include <math.h>

// L1~L5 已经在外部宏定义

static int leg1_last , leg2_last;
void getJointAngles(float x_target, float y_target, float *phi1, float *phi4) {
    // ==========================================
    // 1. 计算左侧关节角 phi1
    // ==========================================
    float x_plus = x_target + L5 / 2.0f;
    float a = 2.0f * x_plus * L1;
    float b = 2.0f * y_target * L1;
    float c = (x_plus * x_plus) + (y_target * y_target) + (L1 * L1) - (L2 * L2);

    float a_sq_plus_b_sq = (a * a) + (b * b);
    float sum_val = a_sq_plus_b_sq - (c * c); // 等价于你原来的 a^2 + b^2 - c^2

    if (sum_val >= 0) {
        float psi1 = atan2(b, a);
        float alpha1 = acos(c / sqrt(a_sq_plus_b_sq));
        
        float phi1_rad = psi1 + alpha1; 
        
        *phi1 = phi1_rad * (180.0f / PI);

        // 标准化角度到 0 ~ 360 度
        while (*phi1 > 360.0f) *phi1 -= 360.0f;
        while (*phi1 < 0.0f)   *phi1 += 360.0f;
    } else {
        *phi1 = 400.0f; // 目标不可达标志(as a marker, no impact)
        // printf("PHI1,ERROR\n");
    }

    // ==========================================
    // 2. 计算右侧关节角 phi4
    // ==========================================
    float x_minus = x_target - L5 / 2.0f;
    float a1 = 2.0f * x_minus * L4;
    float b1 = 2.0f * y_target * L4;
    float c1 = (x_minus * x_minus) + (y_target * y_target) + (L4 * L4) - (L3 * L3);

    float a1_sq_plus_b1_sq = (a1 * a1) + (b1 * b1);
    float sum_val1 = a1_sq_plus_b1_sq - (c1 * c1);

    if (sum_val1 >= 0) {
        float psi4 = atan2(b1, a1);
        float alpha4 = acos(c1 / sqrt(a1_sq_plus_b1_sq));
        
        // 这里的 - 号对应原代码的 -sqrt 逻辑
        float phi4_rad = psi4 - alpha4; 
        
        *phi4 = phi4_rad * (180.0f / PI);

        // 标准化角度到 0 ~ 360 度
        while (*phi4 > 360.0f) *phi4 -= 360.0f;
        while (*phi4 < 0.0f)   *phi4 += 360.0f;
    } else {
        *phi4 = 400.0f; // 目标不可达标志
        // printf("PHI4,ERROR\n");
    }
}


void servo_control(float x, float y, int *leg1, int *leg2) {
    float phi1, phi4;

    getJointAngles(x, y, &phi1, &phi4);
    
    // 1. 如果目标点在物理上完全无法到达 (无解)
    if(phi1 == 400.0f || phi4 == 400.0f) {
        // 通常是因为过长不可达，使用1200站立
        *leg1 = leg1_last; // 根据你的舵机中值替换为安全的PWM值
        *leg2 = leg2_last; 
        printf("Overloaded!\n");
        return;
    }

    // 2. 对求出的角度进行强制软限幅 (保护舵机，且防止进入无赋值分支)
    if(phi1 < 99.0f)  phi1 = 99.0f;
    if(phi1 > 261.0f) phi1 = 261.0f;

    // phi4 的合法范围是 0~81 或 279~360
    // 如果 phi4 落在非法区间 81~279 内，将其强制拉回最近的边界
    if(phi4 > 81.0f && phi4 < 180.0f) phi4 = 81.0f;
    if(phi4 >= 180.0f && phi4 < 279.0f) phi4 = 279.0f;

    // 3. 计算最终的舵机 PWM 值 (此时角度绝对在安全范围内)
    *leg1 = (int)((phi1-90 ) / 180.0f * 1000.0f + 250.0f);
    
    if (phi4 >= 270.0f) {
        *leg2 = (int)((phi4 - 270.0f) / 180.0f * 1000.0f + 250.0f);
    } 
    else if(phi4 <= 90.0f) {
        *leg2 = (int)((90.0f + phi4) / 180.0f * 1000.0f + 250.0f);
    } 
    else {
        // 兜底赋值，防止任何意料之外的数值导致乱码
        *leg2 = 250; 
    }
    leg1_last=*leg1;
    leg2_last=*leg2;
    
}