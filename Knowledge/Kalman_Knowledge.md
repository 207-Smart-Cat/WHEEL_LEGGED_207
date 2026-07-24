# 当前导航 Kalman 状态估计说明

本文档解释当前代码中的 IMU + 轮速计 Kalman 融合逻辑，包括输入、输出、状态方程、观测方程、里程计如何使用 Kalman 结果、当前更相信哪个传感器，以及可以重点检查的潜在 bug。

相关代码：

- `project/code/process_rx.c`：IMU 原始数据低通、offset/scale 校准、单位转换。
- `project/code/imu.c`：把 IMU 解算结果封装到 `IMU_data`。
- `project/code/navigation_data_handling.c`：导航坐标映射、重力扣除、轮速转换、EKF 配置和更新、里程计积分。
- `project/code/filter_function.c`：通用 Kalman predict/update 实现。

---

## 1. 当前 Kalman 不是完整位姿 EKF

当前 Kalman 只估计车辆的速度状态，不直接估计 `x/y/yaw`。

Kalman 状态向量是：

```text
X = [v, w, bias_ax, bias_w]^T
```

各状态含义：

| 状态 | 单位 | 含义 |
|---|---|---|
| `v` | `m/s` | 车体前向线速度 |
| `w` | `rad/s` | yaw 角速度，也就是绕竖直 Z 轴旋转的角速度 |
| `bias_ax` | `m/s^2` | IMU 前向加速度零偏 |
| `bias_w` | `rad/s` | IMU yaw gyro 零偏 |

最终 `robot_pose.x/y` 是在 Kalman 外部通过航迹推算积分出来的。`robot_pose.yaw` 不是由 Kalman 的 `w` 积分得到，而是直接取 IMU yaw：

```c
robot_pose.yaw = navi_limit_angle180(filter_data.yaw);
```

所以当前结构可以概括为：

```text
Kalman 输出 v/w -> 用于积分 x/y
IMU yaw         -> 直接作为 robot_pose.yaw
```

---

## 2. IMU 数据如何进入导航

### 2.1 IMU 原始数据到 g / rad/s

`process_rx.c` 中 `ICM_getEulerianAngles()` 做 IMU 原始数据处理。

加速度处理：

```c
icm_data.acc_x = imu660rc_acc_transition((acc_data[0] - AccOffset_Xdata) * AccScale_Xdata);
icm_data.acc_y = imu660rc_acc_transition((acc_data[1] - AccOffset_Ydata) * AccScale_Ydata);
icm_data.acc_z = imu660rc_acc_transition((acc_data[2] - AccOffset_Zdata) * AccScale_Zdata);
```

其中：

```c
#define AccOffset_Xdata         (-15.0f)
#define AccOffset_Ydata         (53.0f)
#define AccOffset_Zdata         (-30.0f)
#define AccScale_Xdata          (1.000821f)
#define AccScale_Ydata          (1.002777f)
#define AccScale_Zdata          (0.994748f)
```

这些参数是写死的 offset/scale 标定值。代码里没有自动标定流程，也没有记录它们如何测得。它们的作用是在原始 LSB 转 g 之前修正零偏和比例。

IMU660RC 当前加速度量程为 `+-8g`，转换因子约为：

```text
4098.36 LSB/g
```

所以 `icm_data.acc_x/y/z` 的单位是：

```text
g
```

陀螺仪处理：

```c
icm_data.gyro_x = process_rx_gyro_x_dps(gyro_data[0]) * DEG_TO_RAD;
icm_data.gyro_y = process_rx_gyro_y_dps(gyro_data[1]) * DEG_TO_RAD;
icm_data.gyro_z = process_rx_gyro_z_dps(gyro_data[2]) * DEG_TO_RAD;
```

`icm_data.gyro_x/y/z` 的单位是：

```text
rad/s
```

### 2.2 IMU_data 封装

`imu.c` 中 `imu_attitude()` 将 `icm_data` 写入全局 `IMU_data`。

```c
IMU_data.accel[0] = icm_data.acc_x;
IMU_data.accel[1] = icm_data.acc_y;
IMU_data.accel[2] = icm_data.acc_z;
```

所以：

```text
IMU_data.accel[] 单位仍然是 g
```

gyro 在这里被转成 `deg/s`：

```c
IMU_data.gyro[0] = icm_data.gyro_x * RAD_TO_DEG;
IMU_data.gyro[1] = icm_data.gyro_y * RAD_TO_DEG;
IMU_data.gyro[2] = icm_data.gyro_z * RAD_TO_DEG;
```

所以：

```text
IMU_data.gyro[] 单位是 deg/s
```

---

## 3. 导航坐标系映射和重力扣除

`navigation_data_handling.c` 的 `navi_parse_data()` 将 IMU 数据映射到导航坐标系。

当前姿态映射：

```c
raw_data.yaw   = -relative_yaw;
raw_data.pitch = -IMU_data.filter_result.roll;
raw_data.roll  = -IMU_data.filter_result.pitch;
```

当前加速度映射：

```c
raw_data.accel[0] = -IMU_data.accel[1];
raw_data.accel[1] = -IMU_data.accel[0];
raw_data.accel[2] =  IMU_data.accel[2];
```

因此：

| 导航变量 | 代码意图 | 单位 |
|---|---|---|
| `raw_data.accel[0]` | 车体前向 X 轴加速度，含重力 | `g` |
| `raw_data.accel[1]` | 车体右向 Y 轴加速度，含重力 | `g` |
| `raw_data.accel[2]` | 车体竖直 Z 轴加速度，含重力 | `g` |

重力扣除在 `Navi_Remove_Gravity()` 中完成：

```c
g_comp_x = sinf(pitch_rad);
g_comp_y = sinf(roll_rad) * cosf(pitch_rad);
g_comp_z = cosf(roll_rad) * cosf(pitch_rad);

*p_ax = (raw_data.accel[0] - g_comp_x) * GRAVITY;
*p_ay = (raw_data.accel[1] - g_comp_y) * GRAVITY;
*p_az = (raw_data.accel[2] - g_comp_z) * GRAVITY;
```

扣重力后的结果：

```text
filter_data.accel[] 单位是 m/s^2
```

当前 Kalman 只使用：

```c
filter_data.accel[0]
```

它必须表示：

```text
车体前向净线加速度 ax，单位 m/s^2
```

它不是俯仰角，也不是绕 X 轴旋转角速度。

静止检查期望：

| 状态 | `raw_data.accel[]` 期望 | `filter_data.accel[]` 期望 |
|---|---|---|
| 水平静止 | `[0, 0, +1] g` | `[0, 0, 0] m/s^2` |
| 车头抬起静止 | X 轴出现重力分量 | 扣除后仍接近 0 |
| 左右倾斜静止 | Y/Z 轴重力分量变化 | 扣除后仍接近 0 |

如果静止俯仰时 `filter_data.accel[0]` 出现 `+-19 m/s^2`，说明重力扣除方向可能反了，或姿态角映射和加速度轴映射不一致。`19.6 m/s^2` 约等于 `2g`，常见原因是本该相减的重力分量被反向相加。

---

## 4. 轮速计如何进入 Kalman

驱动板 UART2 解析结果写入：

```c
motor_value.receive_left_speed_data
motor_value.receive_right_speed_data
```

导航层做符号修正：

```c
raw_data.left_rpm  = (int16_t)(-motor_value.receive_left_speed_data);
raw_data.right_rmp = (int16_t)(-motor_value.receive_right_speed_data);
```

rpm 转线速度：

```c
raw_data.left_mps  = RPM_TO_M_COEFF((float)raw_data.left_rpm);
raw_data.right_mps = RPM_TO_M_COEFF((float)raw_data.right_rmp);
```

转换公式：

```text
m/s = rpm * (WHEEL_DIAMETER * pi / 60)
```

当前参数：

```c
WHEEL_DIAMETER = 0.045 m
WHEEL_DISRANCE = 0.190 m
```

轮速低通后得到：

```c
filter_data.left_mps
filter_data.right_mps
```

Kalman 使用两个轮速观测：

```c
v_obs_mps = Navi_Get_Forward_Mps();
w_obs_enc = Navi_Get_YawRate_Enc();
```

对应函数：

```c
return (filter_data.left_mps - filter_data.right_mps) * 0.5f;
```

```c
return -(filter_data.left_mps + filter_data.right_mps) / WHEEL_DISRANCE;
```

含义：

| 观测 | 单位 | 含义 |
|---|---|---|
| `v_obs_mps` | `m/s` | 轮速估计的前向速度 |
| `w_obs_enc` | `rad/s` | 左右轮差速估计的 yaw 角速度 |

由于当前左右轮在直行时符号相反，所以前向速度使用 `(left - right) / 2`，角速度使用 `-(left + right) / axle_distance`。

---

## 5. Kalman 的输入和输出

### 5.1 控制输入 U

```c
float a_input = filter_data.accel[0];
```

单位：

```text
m/s^2
```

含义：

```text
IMU 扣重力后的车体前向净线加速度
```

这用于预测阶段，不是观测阶段。

### 5.2 观测输入 Z

```c
float obs_z[3] = { v_obs_mps, w_obs_enc, gyro_z_obs };
```

其中：

| 观测 | 单位 | 来源 |
|---|---|---|
| `v_obs_mps` | `m/s` | 左右轮速合成 |
| `w_obs_enc` | `rad/s` | 左右轮速差速合成 |
| `gyro_z_obs` | `rad/s` | IMU yaw gyro |

当前代码使用：

```c
float gyro_z_obs = raw_data.unbiased_gyro[2];
```

注意：虽然 `navi_parse_data()` 对 `filter_data.unbiased_gyro[2]` 做了低通，但 EKF 当前观测使用的是 `raw_data.unbiased_gyro[2]`，不是低通后的 gyro。

### 5.3 输出

Kalman 更新后：

```c
float opt_v = mat_get(&nav_ekf.X, 0, 0);
float opt_w = mat_get(&nav_ekf.X, 1, 0);
```

输出含义：

| 输出 | 单位 | 含义 |
|---|---|---|
| `opt_v` | `m/s` | 融合后的前向速度 |
| `opt_w` | `rad/s` | 融合后的 yaw 角速度 |
| `robot_pose.bias_ax` | `m/s^2` | Kalman 估计的前向加速度零偏 |
| `robot_pose.bias_w` | `rad/s` | Kalman 估计的 gyro_z 零偏 |

---

## 6. 状态方程：预测阶段如何计算

`navi_update_F_B_U()` 配置预测矩阵。

当前采样时间：

```c
ENCODER_DT = 0.010f
```

即：

```text
dt = 10 ms
```

状态预测等价于：

```text
v_k       = v_{k-1} + (a_x - bias_ax) * dt
w_k       = w_{k-1}
bias_ax_k = bias_ax_{k-1}
bias_w_k  = bias_w_{k-1}
```

矩阵形式：

```text
X_k = F X_{k-1} + B U
```

其中：

```text
F =
[1 0 -dt 0]
[0 1  0  0]
[0 0  1  0]
[0 0  0  1]

B =
[dt]
[0 ]
[0 ]
[0 ]

U = a_x
```

因此第一行展开为：

```text
v_k = v_{k-1} - bias_ax * dt + a_x * dt
```

如果 `a_x = filter_data.accel[0]` 错误地包含重力残差，例如 `19.6 m/s^2`，每 10ms 会给速度预测增加：

```text
19.6 * 0.010 = 0.196 m/s
```

这对里程计是严重错误输入。

---

## 7. 观测方程：更新阶段如何融合

`navi_ekf_config()` 配置 H 矩阵：

```c
mat_set(&nav_ekf.H, 0, 0, 1.0f); // v_enc = v
mat_set(&nav_ekf.H, 1, 1, 1.0f); // w_enc = w
mat_set(&nav_ekf.H, 2, 1, 1.0f); // gyro = w + bias_w
mat_set(&nav_ekf.H, 2, 3, 1.0f);
```

观测方程：

```text
Z = H X + noise
```

展开：

```text
v_enc  = v
w_enc  = w
gyro_z = w + bias_w
```

矩阵形式：

```text
[v_enc ]   [1 0 0 0] [v      ]
[w_enc ] = [0 1 0 0] [w      ]
[gyro_z]   [0 1 0 1] [bias_ax]
                     [bias_w ]
```

直观理解：

- 轮速给出前向速度 `v`。
- 左右轮差速给出 yaw 角速度 `w`。
- IMU gyro_z 给出 `w + gyro_z 零偏`。
- Kalman 通过轮速 `w_enc` 和 IMU `gyro_z` 的差异估计 `bias_w`。

---

## 8. Kalman 算法步骤

通用实现在 `filter_function.c`。

### 8.1 预测

```text
X_pred = F X + B U
P_pred = F P F^T + Q
```

其中：

- `X` 是状态估计。
- `P` 是状态协方差，表示当前估计的不确定度。
- `Q` 是过程噪声，表示对模型预测的不信任程度。

### 8.2 更新

计算预测观测：

```text
Z_pred = H X_pred
```

计算残差：

```text
y = Z - Z_pred
```

计算 Kalman 增益：

```text
K = P H^T (H P H^T + R)^-1
```

状态更新：

```text
X = X_pred + K y
```

协方差更新：

```text
P = (I - K H) P
```

这里 `R` 是观测噪声，表示对传感器测量的不信任程度。

---

## 9. 当前更相信 IMU 还是轮速计

当前代码在 `USE_WIFI_TUNE = 1` 时使用运行时变量：

```c
nav_r_v_normal = 0.1f;
nav_r_w_normal = 0.1f;
nav_r_gyro     = 0.01f;
```

R 矩阵含义：

| R 项 | 对应观测 | 默认值 | 越小表示 |
|---|---|---:|---|
| `NAV_R_V_NORMAL` | 轮速线速度 `v_obs_mps` | `0.1` | 越相信轮速线速度 |
| `NAV_R_W_NORMAL` | 轮速角速度 `w_obs_enc` | `0.1` | 越相信轮速差速角速度 |
| `NAV_R_GYRO` | IMU `gyro_z_obs` | `0.01` | 越相信 IMU yaw gyro |

因此在正常情况下：

```text
gyro_z 观测噪声 0.01
轮速 v/w 观测噪声 0.1
```

当前配置更相信：

```text
IMU gyro_z > 轮速角速度 > 轮速线速度
```

但要注意：

- `v` 的观测只有轮速，没有 IMU 速度观测。
- IMU 加速度不是观测，而是预测输入。
- 所以速度 `v` 最终仍主要被轮速拉住，只是预测阶段受 IMU 前向加速度影响。

打滑时：

```c
current_R_v = NAV_R_V_SLIP;
current_R_w = NAV_R_W_SLIP;
```

默认：

```text
NAV_R_V_SLIP = 10.0
NAV_R_W_SLIP = 10.0
```

这表示打滑时大幅降低对轮速 `v/w` 的信任。

静止时 ZUPT：

```c
if (fabsf(v_obs_mps) < 0.01f && fabsf(w_obs_enc) < 0.05f && !airborne_flag) {
    a_input = 0.0f;
    current_R_v = 0.0001f;
    current_R_w = 0.0001f;
    w_obs_enc = 0.0f;
}
```

这表示静止时强制相信轮速为 0，用轮速零速约束压住速度漂移。

---

## 10. 里程计如何使用 Kalman 数据

Kalman 更新后，代码读取：

```c
float opt_v = mat_get(&nav_ekf.X, 0, 0);
float opt_w = mat_get(&nav_ekf.X, 1, 0);
```

然后使用 `opt_v` 和 `opt_w` 做航迹积分。

积分使用：

```c
float dt = ENCODER_DT;
float yaw_rad = ANGLE_TO_RAD(filter_data.yaw);
float dtheta = opt_w * dt;
```

如果 `opt_w` 很小，使用中点积分：

```c
dx = opt_v * cosf(yaw_rad + half_dtheta) * dt;
dy = opt_v * sinf(yaw_rad + half_dtheta) * dt;
```

如果 `opt_w` 不小，使用圆弧解析积分：

```c
radius = opt_v / opt_w;
dx = radius * (sinf(yaw_rad + dtheta) - sinf(yaw_rad));
dy = radius * (cosf(yaw_rad) - cosf(yaw_rad + dtheta));
```

最后：

```c
robot_pose.x += dx;
robot_pose.y += dy;
```

并写回：

```c
robot_pose.v       = opt_v;
robot_pose.w       = opt_w;
robot_pose.bias_ax = X[2];
robot_pose.bias_w  = X[3];
robot_pose.yaw     = filter_data.yaw;
```

所以里程计使用 Kalman 的方式是：

| 里程计变量 | 是否来自 Kalman |
|---|---|
| `robot_pose.v` | 是，来自 `X[0]` |
| `robot_pose.w` | 是，来自 `X[1]` |
| `robot_pose.x` | 间接使用 Kalman，通过 `opt_v/opt_w` 积分 |
| `robot_pose.y` | 间接使用 Kalman，通过 `opt_v/opt_w` 积分 |
| `robot_pose.yaw` | 否，直接来自 IMU yaw |

---

## 11. 当前代码中值得重点检查的 bug / 风险点

### 11.1 `filter_data.accel[0]` 的物理含义可能被误解

当前 Kalman 要求：

```text
filter_data.accel[0] = 前向净线加速度 ax，单位 m/s^2
```

它不是：

```text
俯仰角
绕 X 轴旋转角速度
车身姿态变化量
```

如果静止俯仰时 `filter_data.accel[0]` 随俯仰变化到 `+-19 m/s^2`，说明该值仍然包含重力分量，不符合当前 EKF 输入要求。

检查方法：

```text
水平静止：filter_data.accel[0] ~= 0
车头抬起静止：filter_data.accel[0] 仍应 ~= 0
车头下压静止：filter_data.accel[0] 仍应 ~= 0
```

如果做不到，应先修正 IMU 轴映射、姿态角映射或重力扣除符号，不应优先调 Q/R。

### 11.2 姿态角和加速度轴映射可能不一致

当前代码：

```c
raw_data.pitch = -IMU_data.filter_result.roll;
raw_data.roll  = -IMU_data.filter_result.pitch;

raw_data.accel[0] = -IMU_data.accel[1];
raw_data.accel[1] = -IMU_data.accel[0];
raw_data.accel[2] =  IMU_data.accel[2];
```

重力扣除要求姿态角定义和加速度轴定义严格一致。否则会出现：

```text
raw_accel 和 g_comp 本应同号，实际反号
filter = raw_accel - g_comp 变成近似 2g 残差
```

这是目前最需要实测验证的部分。

### 11.3 EKF 使用 raw gyro_z，而不是低通后的 gyro_z

`navi_parse_data()` 中有：

```c
filter_data.unbiased_gyro[2] = low_pass_filter_update(&lpf_w, raw_data.unbiased_gyro[2]);
```

但 `navi_ekf_update()` 中实际使用：

```c
float gyro_z_obs = raw_data.unbiased_gyro[2];
```

这可能是有意为之，因为注释说“取出完全原始的 gyro_z”。但如果 gyro_z 噪声较大，需要确认这是设计选择还是遗留问题。

### 11.4 腾空时仍执行 Kalman update，可能让 P 虚假变小

腾空时，代码把观测值设置成预测值：

```c
v_obs_mps  = X[0];
w_obs_enc  = X[1];
gyro_z_obs = X[1] + X[3];
```

这样残差为 0，状态不会变。但随后仍调用：

```c
kalman_filter_update(&nav_ekf, obs_z);
```

即使残差为 0，协方差 `P` 仍会因为 update 而降低，相当于系统“以为自己得到了可靠观测”。更严谨的处理通常是：

```text
腾空时跳过 update
或把 R 设置得很大
或只保留 IMU gyro_z 更新，切断轮速 v/w 更新
```

当前实现可能导致腾空期间状态不确定度被低估。

### 11.5 P 更新不是 Joseph 形式，数值稳定性一般

当前协方差更新：

```text
P = (I - K H) P
```

这在理论推导中常见，但数值上不如 Joseph 形式稳定：

```text
P = (I - K H) P (I - K H)^T + K R K^T
```

在 MCU 浮点和长期运行中，简化形式可能导致 `P` 逐渐非对称或非正定。当前矩阵规模较小，短时间可能没有明显问题，但它是一个风险点。

### 11.6 注释中 Q/R 信任度描述容易误导

正确理解：

```text
Q 越大：越不相信预测模型，允许状态更快变化
R 越大：越不相信传感器观测
```

如果注释写成“预测值准则 Q 大”或“测量值准则 R 大”，容易导致调参方向反了。

### 11.7 轮速观测 v/w 来自同一组左右轮，但 R 假设独立

当前观测：

```text
v_obs_mps 来自左右轮
w_obs_enc 也来自左右轮
```

但 R 是对角矩阵，假设观测噪声互相独立。工程上可以接受，但当轮子打滑、悬空、轮径误差或轴距误差存在时，`v_obs` 和 `w_obs` 往往会同时偏。这是模型简化，不一定是 bug，但调参时要知道这个假设。

### 11.8 `bias_ax` 可观测性依赖轮速和加速度一致性

`bias_ax` 没有直接观测，只能通过：

```text
IMU 加速度积分预测出的 v
与轮速观测 v_obs 的长期差异
```

间接估计。如果轮速本身错误、打滑频繁，或者 IMU 前向加速度扣重力错误，`bias_ax` 会被迫吸收错误，数值可能失去物理意义。

---

## 12. 推荐的调试输出顺序

为了查 bug，建议按以下顺序输出，而不是一开始只看 Kalman 结果。

### 12.1 IMU 静态检查

输出：

```text
IMU_data.accel[0], IMU_data.accel[1], IMU_data.accel[2]
raw_data.accel[0], raw_data.accel[1], raw_data.accel[2]
raw_data.pitch, raw_data.roll, raw_data.yaw
filter_data.accel[0], filter_data.accel[1], filter_data.accel[2]
```

静止任意姿态期望：

```text
filter_data.accel[0] ~= 0 m/s^2
filter_data.accel[1] ~= 0 m/s^2
filter_data.accel[2] ~= 0 m/s^2
```

尤其是 `filter_data.accel[0]`，它直接进入 Kalman 预测。

### 12.2 轮速检查

输出：

```text
motor_value.receive_left_speed_data
motor_value.receive_right_speed_data
raw_data.left_rpm
raw_data.right_rmp
filter_data.left_mps
filter_data.right_mps
v_obs_mps
w_obs_enc
```

静止期望：

```text
v_obs_mps ~= 0
w_obs_enc ~= 0
```

直行期望：

```text
v_obs_mps 为正
w_obs_enc 接近 0
```

原地顺时针旋转期望：

```text
w_obs_enc 与 IMU gyro_z_obs 同号
```

### 12.3 Kalman 检查

输出：

```text
a_input
v_obs_mps
w_obs_enc
gyro_z_obs
robot_pose.v
robot_pose.w
robot_pose.bias_ax
robot_pose.bias_w
```

静止期望：

```text
a_input ~= 0
v_obs_mps ~= 0
w_obs_enc ~= 0
gyro_z_obs ~= 0
robot_pose.v ~= 0
robot_pose.w ~= 0
```

如果 `a_input` 错误但 `v_obs_mps` 为 0，Kalman 会表现为“预测想跑，轮速更新又拉回来”，这通常会造成速度抖动和 bias 异常。

---

## 13. 一句话总结

当前 Kalman 融合的本质是：

```text
用 IMU 前向净加速度预测 v；
用轮速线速度修正 v；
用轮速差速和 IMU gyro_z 共同修正 yaw 角速度 w 和 gyro 零偏；
再用融合后的 v/w 加 IMU yaw 积分 robot_pose.x/y。
```

当前更相信 IMU gyro_z，因为 `NAV_R_GYRO = 0.01` 小于轮速角速度的 `NAV_R_W_NORMAL = 0.1`。但前向速度 `v` 的观测主要来自轮速，IMU 加速度只参与预测。

因此，`filter_data.accel[0]` 必须是准确的前向净线加速度。如果它在静止俯仰时出现 `+-19 m/s^2`，这不是正常姿态反馈，而是重力扣除/坐标映射错误，会直接破坏 Kalman 预测和里程计积分。
