/*********************************************************************************************************************

********************************************************************************************************************/


#ifndef __MATRIX_H
#define __MATRIX_H

#include "zf_common_typedef.h"

//=========================参数配置============================
#define MAT_MAX_N    6

//==========================结构体定义=========================
typedef struct {
    float data[MAT_MAX_N][MAT_MAX_N];
    int rows;
    int cols;
} Matrix;

//=========================变量声明============================

//===========================函数声明============================
void mat_init(Matrix *m, int rows, int cols);        // 初始化矩阵
void mat_set(Matrix *m, int row, int col, float val);     // 矩阵赋值
void mat_add(Matrix *C, Matrix *A, Matrix *B);            // 矩阵加法 C = A + B
void mat_mul(Matrix *C, Matrix *A, Matrix *B);                    // 矩阵乘法 C = A * B
void mat_trans(Matrix *B, Matrix *A);                    // 矩阵转置 B = A^T
void mat_inv_diag(Matrix *B, Matrix *A);                  // 对角矩阵求逆
void mat_eye(Matrix *m);                                // 单位矩阵
float mat_get(Matrix *m, int row, int col); // 获取矩阵元素值
void mat_scale(Matrix *out, Matrix *mat, float scale);  //矩阵缩放（所有元素 × 系数）
void mat_zero(Matrix *m);             //矩阵置零（初始化后清空）
uint8_t mat_inv(Matrix *A_inv, Matrix *A);//通用矩阵求逆

#endif
