/*********************************************************************************************************************

********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "filter_function.h"

//====================================================变量声明=======================================================


//====================================================自定义全局变量=================================================


//====================================================自定义静态变量=================================================


//====================================================函数声明======================================================

//---------------------------------------------------卡尔曼滤波----------------------------------------------------------------
//卡尔曼滤波的使用步骤
//        (1) 选择状态量（需要输出的值）、观测量(测量得到的值)
//        (2) 构建方程
//        (3) 初始化参数
//        (4) 带入公式迭代
//        (5) 调节超参数P、Q
//
//    X：k时刻系统状态                Z：k时刻测量值
//    U：k时刻对系统控制量      H：测量系统参数
//                                                                                     方差
//    F：状态转移矩阵                  W(k)：过程噪声 ----> Q
//                                                                                     方差
//    B：控制矩阵                              V(k)：测量噪声 ----> R
//
//    离散控制系统
//    系统描述：X(k|k-1) = FX(k-1|k-1) + BU(k) + (W(k))
//    测量值：Z(k) = HX(k) + V(k)
//
//卡尔曼滤波预测和更新公式：
//1.状态预测：        X_ = F * X + B * U                                 X_:当前时刻预测值，由上一时刻最优值X和当前控制输入U计算                          F：状态矩阵，由运动方程确定
//2.协方差预测：      P_ = F * P * P(T) + Q                              U：当前时刻输入 ，由传感器得到                                                    B：控制矩阵，由运动方程确定
//3.计算卡尔曼增益:   K = P * H(T)  /  (H * P(K) * H(T)+ R)              P_：当前时刻预测值协方差矩阵，反应预测值可信度                                   H：测量矩阵，由测量方程确定
//4.状态更新：        X = X + K * (Z - H* X_)                            K：卡尔曼增益，相当于预测值和测量值的数据融合系数                                I：单位矩阵
//5.协方差更新：      P = (I - K * H ) * P_                                超参数： Q——预测噪声协方差矩阵，预测值准则大
//                                                                                    R——测量噪声协方差矩阵，测量值准，则大
//
//    1. 先验估计
//* * *公式1：X(k|k-1) = FX(k-1|k-1) + BU(k) + (W(k))
//
//
//    2. 预测协方差矩阵
//* * *公式2：P(k|k-1)=FP(k-1|k-1)A^T + Q
//
//
//    3. 建立测量方程
//
//       
//    4. 计算卡尔曼增益
//* * *公式3：Kg(k)= P(k|k-1)H^T/(HP(k|k-1)H^T+R)
//
//    5. 计算当前最优化估计值
//* * *公式4：X(k|k) = X(k|k-1) + kg(k)[z(k) - HX(k|k-1)]
//
//    6. 更新协方差矩阵
//* * *公式5：P(k|k)=[I-Kg(k)H]P(k|k-1)
//-------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化卡尔曼滤波器
// 参数说明  state_dim：状态维度，滤波状态量个数                 obs_dim：观测维度，传感器实际测量值的个数（IMU 的 roll/pitch/yaw，共 3 个观测值）            ctrl_dim：系统控制输入的个数（比如你陀螺仪三轴数据，共 3 个控制量）
// 返回参数  1; 初始化成功   
// 使用示例     
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 kalman_filter_init(KalmanFilter_Struct *kf, int state_dim, int obs_dim,int ctrl_dim) {
    // 1. 维度校验（仅支持1-6阶状态，1-4阶观测）
    if(kf == NULL || state_dim < 1 || state_dim > 6 || obs_dim < 1 || obs_dim > 4) {
        return 0; // 初始化失败
    }
    // 2. 初始化核心矩阵
    mat_init(&kf->X, state_dim, 1);         // X状态量矩阵 [n*1]
    mat_init(&kf->B, state_dim, ctrl_dim); // B：n*1
    mat_init(&kf->U, ctrl_dim, 1);         // U：1*1
    mat_init(&kf->F, state_dim, state_dim); // F状态转移矩阵 [n*n]
    mat_init(&kf->F_T, state_dim, state_dim); 
    mat_init(&kf->H, obs_dim, state_dim);   // H观测矩阵 [m*n]
    mat_init(&kf->H_T, state_dim, obs_dim);
    mat_init(&kf->P, state_dim, state_dim); // P协方差矩阵 [n*n]
    mat_init(&kf->Q, state_dim, state_dim); // Q过程噪声 [n*n]
    mat_init(&kf->R, obs_dim, obs_dim);     // R观测噪声 [m*m]
    mat_init(&kf->Z, obs_dim, 1);
    mat_init(&kf->K, state_dim, obs_dim);   // K卡尔曼增益 [n*m]

    // 3. 初始化临时矩阵（避免重复初始化）
    mat_init(&kf->temp_X, state_dim, 1);         // X状态量矩阵 [n*1] 
    mat_init(&kf->temp1, state_dim, state_dim);
    mat_init(&kf->temp2, state_dim, obs_dim);
    mat_init(&kf->temp3, obs_dim, obs_dim);
    mat_init(&kf->temp4, obs_dim, 1); 
    mat_init(&kf->temp_inv, obs_dim, obs_dim);


    // 4. 默认参数（用户可后续修改）
    kf->state_dim = state_dim;
    kf->obs_dim = obs_dim;
    mat_eye(&kf->F); // 转移矩阵默认单位矩阵
    mat_eye(&kf->P); // 协方差矩阵默认单位矩阵
    mat_scale(&kf->Q, &kf->Q, 0.01f);      //默认Q/R缩小100倍（减少噪声影响）
    mat_scale(&kf->R, &kf->R, 0.01f); 
    mat_eye(&kf->B); // 默认单位矩阵，用户后续修改
    return 1; // 初始化成功
}





//-------------------------------------------------------------------------------------------------------------------
// 函数简介    卡尔曼预测阶段（核心公式：X' = F*X + B*U；P' = F*P*F^T + Q）    
// 参数说明             
// 返回参数      
//------------------------------------------------------------------------------------------------------------
// 卡尔曼预测阶段（X' = F*X + B*U；P' = F*P*F^T + Q）
void kalman_filter_predict(KalmanFilter_Struct *kf) {
    if(kf == NULL) return;
    int n = kf->state_dim;
    int p = 3; // 控制量维度（陀螺仪三轴）

    // 维度校验（调用你的mat_get/行列数）
    if(kf->F.rows != n || kf->F.cols != n || 
       kf->B.rows != n || kf->B.cols != p ||
       kf->U.rows != p || kf->U.cols != 1) {
        return;
    }

    // 1. 状态预测：X' = F*X + B*U
    // 步骤1：计算 F*X（调用你的mat_mul）
    mat_mul(&kf->temp_X, &kf->F, &kf->X);
    
    // 步骤2：计算 B*U
    Matrix B_U;
    mat_init(&B_U, n, 1);
    mat_mul(&B_U, &kf->B, &kf->U);
    
    // 步骤3：X' = F*X + B*U（调用你的mat_add）
    mat_add(&kf->temp_X, &kf->temp_X, &B_U);
    
    // 逐元素更新X（调用你的mat_set/mat_get）
    for(int i=0; i<n; i++) {
        mat_set(&kf->X, i, 0, mat_get(&kf->temp_X, i, 0));
    }

    // 2. 协方差预测：P' = F*P*F^T + Q
    // 步骤1：temp1 = F * P
    mat_mul(&kf->temp1, &kf->F, &kf->P);
    // 步骤2：计算F的转置（调用你的mat_trans）
    mat_trans(&kf->F_T, &kf->F);
    // 步骤3：P = temp1 * F^T
    mat_mul(&kf->P, &kf->temp1, &kf->F_T);
    // 步骤4：加上过程噪声 Q
    mat_add(&kf->P, &kf->P, &kf->Q);
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介      卡尔曼更新阶段（核心公式：K = P'*H^T*(H*P'*H^T + R)^-1；X = X' + K*(Z-H*X')；P = (I-K*H)*P'）  
// 参数说明     obs_data 观测值数据        
// 返回参数      
//-------------------------------------------------------------------------------------------------------------------
// 卡尔曼更新阶段（核心公式：K = P'*H^T*(H*P'*H^T + R)^-1；X = X' + K*(Z-H*X')；P = (I-K*H)*P'）
void kalman_filter_update(KalmanFilter_Struct *kf, float *obs_data) {
    if(kf == NULL || obs_data == NULL) return;
    int n = kf->state_dim;
    int m = kf->obs_dim;

    // 维度校验
    if(kf->H.rows != m || kf->H.cols != n ||
       kf->R.rows != m || kf->R.cols != m) {
        return;
    }

    // 1. 构造观测矩阵Z [m*1]（调用你的mat_set）
    for(int i=0; i<m; i++) {
        mat_set(&kf->Z, i, 0, obs_data[i]);
    }

    // 2. 计算 H*P*H^T + R
    // 步骤1：temp2 = P * H^T
    mat_trans(&kf->H_T, &kf->H);
    mat_mul(&kf->temp2, &kf->P, &kf->H_T);
    
    // 步骤2：temp3 = H * temp2
    mat_mul(&kf->temp3, &kf->H, &kf->temp2);
    
    // 步骤3：temp3 = temp3 + R
    mat_add(&kf->temp3, &kf->temp3, &kf->R);

    // 3. 求temp3的逆（调用新增的mat_inv）
    if(!mat_inv(&kf->temp_inv, &kf->temp3)) {
        return; // 矩阵不可逆，更新失败
    }

    // 4. 计算卡尔曼增益：K = P*H^T * temp_inv
    mat_mul(&kf->K, &kf->temp2, &kf->temp_inv);

    // 5. 计算观测残差：residual = Z - H*X
    mat_mul(&kf->temp4, &kf->H, &kf->X);
    for(int i=0; i<m; i++) {
        float val = mat_get(&kf->Z, i, 0) - mat_get(&kf->temp4, i, 0);
        mat_set(&kf->temp4, i, 0, val);
    }

    // 6. 更新状态：X = X' + K*residual
    mat_mul(&kf->temp_X, &kf->K, &kf->temp4);
    mat_add(&kf->X, &kf->X, &kf->temp_X);

    // 7. 更新协方差：P = (I - K*H) * P
    // 步骤1：计算 K*H（复用temp1）
    mat_mul(&kf->temp1, &kf->K, &kf->H);
    
    // 步骤2：计算 I - K*H（复用temp_inv作为单位矩阵）
    mat_eye(&kf->temp_inv); 
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            float val = mat_get(&kf->temp_inv, i, j) - mat_get(&kf->temp1, i, j);
            mat_set(&kf->temp_inv, i, j, val);
        }
    }
    
    // 步骤3：更新P = (I-KH) * P
    mat_mul(&kf->temp1, &kf->temp_inv, &kf->P);
    
    // 逐元素拷贝P矩阵（避免指针重叠）
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            mat_set(&kf->P, i, j, mat_get(&kf->temp1, i, j));
        }
    }
}


//---------------------------------------------------一阶低通滤波-----------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化一阶低通滤波器
// 参数说明  lpf：滤波器结构体指针；alpha：滤波系数（0<alpha<1）
// 返回参数  1=成功，0=失败
// 备注信息  alpha越小，滤波越平滑，但响应越慢
//-------------------------------------------------------------------------------------------------------------------
uint8 low_pass_filter_init(LowPassFilter_Struct *lpf, float alpha) {
    if(lpf == NULL || alpha <= 0 || alpha >= 1) {
        return 0;
    }
    lpf->alpha = alpha;
    lpf->prev_value = 0.0f;
    lpf->is_init = 1;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  一阶低通滤波更新
// 参数说明  lpf：滤波器结构体指针；input：当前输入值
// 返回参数  滤波后结果
// 备注信息  公式：out = alpha*input + (1-alpha)*prev_out
//-------------------------------------------------------------------------------------------------------------------
float low_pass_filter_update(LowPassFilter_Struct *lpf, float input) {
    if(lpf == NULL || !lpf->is_init) {
        return input; // 未初始化直接返回输入
    }
    // 一阶低通核心公式
    lpf->prev_value = lpf->alpha * input + (1 - lpf->alpha) * lpf->prev_value;
    return lpf->prev_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化滑动平均滤波器
// 参数说明  maf：滤波器结构体指针；buf_size：滑动窗口大小（建议3/5/10）
// 返回参数  1=成功，0=失败
// 使用示例  moving_avg_filter_init(&maf, 5);
// 备注信息  动态申请缓冲区，需调用deinit释放
//-------------------------------------------------------------------------------------------------------------------
uint8 moving_avg_filter_init(MovingAvgFilter_Struct *maf, int buf_size) {
    if(maf == NULL || buf_size < 2 || buf_size > 128) {
        return 0;
    }
    // 动态申请缓冲区（嵌入式需确保内存足够）
    maf->buf = (float *)malloc(buf_size * sizeof(float));
    if(maf->buf == NULL) {
        return 0;
    }
    // 初始化参数
    maf->buf_size = buf_size;
    maf->index = 0;
    maf->count = 0;
    maf->sum = 0.0f;
    maf->is_init = 1;
    // 缓冲区清零
    for(int i=0; i<buf_size; i++) {
        maf->buf[i] = 0.0f;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  滑动平均滤波更新
// 参数说明  maf：滤波器结构体指针；input：当前输入值
// 返回参数  滤波后结果
// 使用示例  float res = moving_avg_filter_update(&maf, imu_data);
// 备注信息  窗口未满时返回均值，窗口满后滑动更新
//-------------------------------------------------------------------------------------------------------------------
float moving_avg_filter_update(MovingAvgFilter_Struct *maf, float input) {
    if(maf == NULL || !maf->is_init) {
        return input;
    }
    // 1. 减去即将被覆盖的旧值（窗口满时）
    if(maf->count >= maf->buf_size) {
        maf->sum -= maf->buf[maf->index];
    }
    // 2. 存入新值，更新和
    maf->buf[maf->index] = input;
    maf->sum += input;
    // 3. 更新索引
    maf->index = (maf->index + 1) % maf->buf_size;
    // 4. 更新计数
    if(maf->count < maf->buf_size) {
        maf->count++;
    }
    // 5. 计算均值
    return maf->sum / maf->count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  释放滑动平均滤波器内存
// 参数说明  maf：滤波器结构体指针
// 返回参数  无
// 使用示例  moving_avg_filter_deinit(&maf);
// 备注信息  嵌入式场景需手动释放，避免内存泄漏
//-------------------------------------------------------------------------------------------------------------------
void moving_avg_filter_deinit(MovingAvgFilter_Struct *maf) {
    if(maf == NULL || !maf->is_init) {
        return;
    }
    if(maf->buf != NULL) {
        free(maf->buf);
        maf->buf = NULL;
    }
    maf->is_init = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化中值滤波器
// 参数说明  mf：滤波器结构体指针；buf_size：缓冲区大小（建议奇数，如3/5/7）
// 返回参数  1=成功，0=失败
// 使用示例  median_filter_init(&mf, 5);
// 备注信息  动态申请缓冲区，需调用deinit释放
//-------------------------------------------------------------------------------------------------------------------
uint8 median_filter_init(MedianFilter_Struct *mf, int buf_size) {
    if(mf == NULL || buf_size < 3 || buf_size % 2 == 0 || buf_size > 128) {
        return 0; // 建议至少3个数据，且为奇数
    }
    mf->buf = (float *)malloc(buf_size * sizeof(float));
    if(mf->buf == NULL) {
        return 0;
    }
    mf->buf_size = buf_size;
    mf->index = 0;
    mf->count = 0;
    mf->is_init = 1;
    // 缓冲区清零
    for(int i=0; i<buf_size; i++) {
        mf->buf[i] = 0.0f;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  中值滤波更新（排序取中值）
// 参数说明  mf：滤波器结构体指针；input：当前输入值
// 返回参数  滤波后结果
// 使用示例  float res = median_filter_update(&mf, encoder_data);
// 备注信息  抗脉冲干扰能力强，适合剔除异常值
//-------------------------------------------------------------------------------------------------------------------
float median_filter_update(MedianFilter_Struct *mf, float input) {
    if(mf == NULL || !mf->is_init) { 
        return input;
    }
    // 1. 存入新值
    mf->buf[mf->index] = input;
    mf->index = (mf->index + 1) % mf->buf_size;
    // 2. 更新计数
    if(mf->count < mf->buf_size) {
        mf->count++;
    }
    // 3. 缓冲区未满时返回输入（或均值，按需调整）
    if(mf->count < mf->buf_size) {
        return input;
    }
    // 4. 拷贝数据并排序
    float temp[mf->buf_size];
    for(int i=0; i<mf->buf_size; i++) {
        temp[i] = mf->buf[i];
    }
    // 简单冒泡排序
    for(int i=0; i<mf->buf_size-1; i++) {
        for(int j=0; j<mf->buf_size-i-1; j++) {
            if(temp[j] > temp[j+1]) {
                float t = temp[j];
                temp[j] = temp[j+1];
                temp[j+1] = t;
            }
        }
    }
    // 5. 返回中值（奇数长度，取中间值）
    return temp[mf->buf_size / 2];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  释放中值滤波器内存
// 参数说明  mf：滤波器结构体指针
// 返回参数  无
// 使用示例  median_filter_deinit(&mf);
// 备注信息  嵌入式场景需手动释放
//-------------------------------------------------------------------------------------------------------------------
void median_filter_deinit(MedianFilter_Struct *mf) {
    if(mf == NULL || !mf->is_init) {
        return;
    }
    if(mf->buf != NULL) {
        free(mf->buf);
        mf->buf = NULL;
    }
    mf->is_init = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化均值滤波器（静态窗口）
// 参数说明  af：滤波器结构体指针；buf_size：缓冲区大小
// 返回参数  1=成功，0=失败
// 使用示例  avg_filter_init(&af, 10);
// 备注信息  与滑动平均的区别：窗口填满后才返回均值，适合批量数据处理
//-------------------------------------------------------------------------------------------------------------------
uint8 avg_filter_init(AvgFilter_Struct *af, int buf_size) {
    if(af == NULL || buf_size < 2) {
        return 0;
    }
    af->buf = (float *)malloc(buf_size * sizeof(float));
    if(af->buf == NULL) {
        return 0;
    }
    af->buf_size = buf_size;
    af->index = 0;
    af->count = 0;
    af->sum = 0.0f;
    af->is_init = 1;
    // 缓冲区清零
    for(int i=0; i<buf_size; i++) {
        af->buf[i] = 0.0f;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  均值滤波更新
// 参数说明  af：滤波器结构体指针；input：当前输入值
// 返回参数  滤波后结果（窗口未满返回输入）
// 使用示例  float res = avg_filter_update(&af, adc_data);
// 备注信息  适合静态数据采集，不适合实时动态数据
//-------------------------------------------------------------------------------------------------------------------
float avg_filter_update(AvgFilter_Struct *af, float input) {
    if(af == NULL || !af->is_init) {
        return input;
    }
    // 1. 存入新值，更新和
    af->buf[af->index] = input;
    af->sum += input;
    af->index++;
    af->count++;
    // 2. 窗口未满返回输入
    if(af->count < af->buf_size) {
        return input;
    }
    // 3. 窗口填满后返回均值，重置参数（静态窗口）
    float avg = af->sum / af->buf_size;
    af->index = 0;
    af->count = 0;
    af->sum = 0.0f;
    // ====== 新增：缓冲区清零（可选，增强鲁棒性） ======
    for(int i=0; i<af->buf_size; i++) {
        af->buf[i] = 0.0f;
    }
    return avg;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  释放均值滤波器内存
// 参数说明  af：滤波器结构体指针
// 返回参数  无
// 使用示例  avg_filter_deinit(&af);
// 备注信息  嵌入式场景需手动释放
//-------------------------------------------------------------------------------------------------------------------
void avg_filter_deinit(AvgFilter_Struct *af) {
    if(af == NULL || !af->is_init) {
        return;
    }
    if(af->buf != NULL) {
        free(af->buf);
        af->buf = NULL;
    }
    af->is_init = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化互补滤波器
// 参数说明  cf：滤波器结构体指针；alpha：互补系数（0<alpha<1）
// 返回参数  1=成功，0=失败
// 备注信息  alpha越大，越信任input1（如陀螺仪），越小越信任input2（如加速度计）           需要用时间函数来计算时间差
//-------------------------------------------------------------------------------------------------------------------
uint8 complementary_filter_init(ComplementaryFilter_Struct *cf, float alpha) {
    if(cf == NULL || alpha <= 0 || alpha >= 1) {
        return 0;
    }
    cf->alpha = alpha;
    cf->prev_value = 0.0f;
    cf->prev_time = timer_get(TIMER_FILTER_CH); // 逐飞库获取当前时间（us）
    cf->is_init = 1;
    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介  互补滤波更新（陀螺仪角速度+加速度计角度融合）
// 参数说明  cf：滤波器结构体指针；gyro_angular：陀螺仪角速度（°/s）；accel_angle：加速度计角度（°）
// 返回参数  融合后角度（°）
// 使用示例  float angle = complementary_filter_update(&cf, gyro_z, accel_angle);
// 备注信息  核心公式：out = α*(out + 角速度*dt) + (1-α)*加速度计角度
//-------------------------------------------------------------------------------------------------------------------
float complementary_filter_update(ComplementaryFilter_Struct *cf, float gyro_angular, float accel_angle) {
    if(cf == NULL || !cf->is_init) {
        return accel_angle; // 未初始化返回加速度计角度（更稳定）
    }

    // 1. 计算时间差dt（单位：秒，s）
    uint32 current_time = timer_get(TIMER_FILTER_CH);
    uint32 time_diff;
    if(current_time >= cf->prev_time) {
        time_diff = current_time - cf->prev_time;
    } else {
        // 溢出后：差值 = 最大值 - 上一次值 + 当前值 + 1
        time_diff = (0xFFFFFFFF - cf->prev_time) + current_time + 1;
    }
    float dt = time_diff / 1000000.0f;
    // =================================
    cf->prev_time = current_time;


    // 防护：dt过大（如传感器卡顿），限制最大dt
    if(dt > 0.1f || dt <= 0) {
        dt = 0.001f; // 限制为1ms，避免积分溢出
    }

    // 步骤1：陀螺仪积分预测角度 = 上一次融合角度 + 角速度*dt
    float gyro_predict_angle = cf->prev_value + gyro_angular * dt;
    // 步骤2：融合（α信任陀螺仪积分，1-α信任加速度计）
    cf->prev_value = cf->alpha * gyro_predict_angle + (1 - cf->alpha) * accel_angle;

    return cf->prev_value;
}
