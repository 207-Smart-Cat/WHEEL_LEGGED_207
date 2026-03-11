#include <stdio.h>
#include <math.h>
#include "FiveBarLinkageData.h"


#define MIN_X -0.05
#define MAX_X 0.05
#define X_STEP 0.001

#define MIN_Y 0.02
#define MAX_Y 0.14
#define Y_STEP 0.001
#define L1  0.06
#define L2  0.09
#define L3  0.09
#define L4  0.06
#define L5  0.038
#define PI 3.141592653589793
#include <math.h>

#define PI 3.141592653589793f
// 假设 L1~L5 已经在外部宏定义

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
        
        // 这里的 + 号对应原代码的 +sqrt 逻辑 (通常代表某种"肘部"配置)
        float phi1_rad = psi1 + alpha1; 
        
        *phi1 = phi1_rad * (180.0f / PI);

        // 标准化角度到 0 ~ 360 度
        while (*phi1 > 360.0f) *phi1 -= 360.0f;
        while (*phi1 < 0.0f)   *phi1 += 360.0f;
    } else {
        *phi1 = 400.0f; // 目标不可达标志
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
    }
}


void servo_control(float x, float y, int *leg1, int *leg2) {   //负责由目标点位计算
    float phi1, phi4;

    
    getJointAngles(x, y, &phi1, &phi4);
        if(phi1==400||phi4==400)
        {
            
        }
        else
        {
             if((phi1>=99&&phi1<=261)&&((phi4 >= 279)||(phi4 <= 81)))
            {
                *leg1 = (270 - phi1) / 180 * 1000 + 250;
                if (phi4 >= 270)
                {
                    *leg2 = (int)((phi4 - 270) / 180 * 1000 + 250);
                }
                else if(phi4 <= 90)
                {
                    *leg2 = (int)((90 + phi4) / 180 * 1000 + 250);
                }
            }
            else
            {
                
            }
        }
 }

