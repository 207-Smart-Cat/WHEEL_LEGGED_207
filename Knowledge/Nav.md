# 导航总结

## 小车如何定位

小车使用本地二维坐标系。导航准备完成后，`Navi_Data_Set_Origin()` 将当前位置设为 `(x=0, y=0)`，并把当前一段时间平均后的 IMU yaw 作为本地航向零点。在该坐标系中，X 正方向为小车初始车头方向，Y 正方向为右侧/东方向，yaw 顺时针增加。

每 10 ms，`pit0_ch15_isr()` 调用一次 `navi_ekf_update()`。导航模块先解析 IMU 姿态、角速度、加速度，以及左右电机速度反馈。IMU 轴向和符号被映射到导航坐标系，加速度会扣除重力分量，轮速 RPM 会转换为车轮线速度。

定位估计由 EKF 完成，状态量为 `[v, w, bias_ax, bias_w]`，分别表示前进速度、yaw 角速度、加速度零偏和陀螺仪零偏。编码器提供前进速度观测，左右轮差速提供 yaw 角速度观测，IMU 陀螺仪提供另一组 yaw 角速度观测。融合后的速度和 yaw 再通过航迹推算累加到全局位置：

- `x += dx`
- `y += dy`
- `yaw = 滤波后的 IMU yaw`

直线运动时使用中点积分；转弯运动时使用基于 `v / w` 的圆弧模型。最终得到全局位姿 `robot_pose`，其中包含 `x`、`y`、`yaw`、`v` 和 `w`。

系统还加入了鲁棒性处理：通过比较编码器加速度和 IMU 加速度判断打滑。打滑时会增大编码器观测噪声，使 EKF 降低对轮速的信任。跳跃或腾空阶段会抑制无效编码器观测，避免空转轮速污染定位结果。

## 小车如何导航到目标点

目标路径保存在航点数组 `point_map[]` 中，每个航点包含 `x`、`y`、`yaw`、类型和有效标志。航点可以来自静态测试地图，也可以来自遥控打点记录。在循迹模式下，`task_navigation_control()` 通过 `navi_ctrl.point_current_idx` 选择当前目标航点。

针对当前航点，`navi_calcnavinfo()` 计算：

- `distance = sqrt((target_x - car_x)^2 + (target_y - car_y)^2)`
- `azimuth = atan2(target_y - car_y, target_x - car_x)`

航向误差为：

```c
need_to_turn = navi_limit_angle180(azimuth - robot_pose.yaw);
```

该误差会转换为底层转向目标角：

```c
target_angle = navi_limit_angle180(IMU_data.filter_result.yaw - smooth_turn);
```

如果没有特殊动作接管，小车会使用该 `target_angle` 和 `DEFAULT_TRACKING_VELOCITY` 朝目标航点行驶。当小车进入航点距离阈值后，`navi_isreach_target_point()` 判定到达，随后 `navi_switch_nexttargetpoint()` 切换到下一个航点。若航点类型为跳跃、单边桥、绕锥桶、侧坡或停车等特殊类型，则由 `Navi_Action_Manager()` 临时接管速度、腿部和车身姿态控制，动作完成后再交还循迹控制。

## 如何使用导航系统

导航运行前需要满足几个条件：IMU 已完成初始化并稳定，系统 `system_fully_ready = true`，运行模块中的 `RUNTIME_MODULE_NAVIGATION` 已开启，电机驱动板能够正常回传左右轮速度。导航中断 `pit0_ch15_isr()` 会在这些条件满足后每 10 ms 自动更新定位和循迹控制。

导航系统需要的核心输入值如下：

- IMU 数据：`yaw`、`pitch`、`roll`、三轴角速度、三轴加速度。
- 电机速度反馈：左/右轮 RPM，用于换算线速度和差速角速度。
- 车辆结构参数：车轮直径 `WHEEL_DIAMETER = 0.045 m`，轮距 `WHEEL_DISRANCE = 0.190 m`。
- 导航参数：EKF 的 `Nav_Q_*`、`Nav_R_*`，以及到点阈值 `DISTANCE_THRESHOLD`。
- 路径数据：`point_map[]` 中的航点坐标、航点类型和有效标志。
- 控制输出：导航最终写入 `target_angle`，并在默认配置下写入 `DEFAULT_TRACKING_VELOCITY` 作为循迹速度。

常用导航模式值：

| 参数 | ID | HEX | 常用值 | 作用 |
|---|---:|---:|---|---|
| `Navi_ModeDriver` | 51 | `0x33` | `0` 停止，`1` 循迹，`2` 打点记录 | 控制导航工作模式 |
| `Navi_ModeMap` | 52 | `0x34` | `0` 静态地图，`1` 打点地图，`2` WiFi 动态地图 | 选择路径来源 |
| `Navi_TrigRecord` | 53 | `0x35` | `0 -> 1` | 触发记录当前点 |
| `Navi_PrintPoseEn` | 54 | `0x36` | `1` | 打开位姿/目标点打印 |
| `Navi_PrintPeriod` | 55 | `0x37` | `1000` | 打印周期，单位 ms |
| `Navi_WifiType` | 57 | `0x39` | 见航点类型枚举 | 手动打点时指定航点类型 |
| `Navi_Speed_Kp` | 59 | `0x3B` | `220` | 距离到目标速度的比例项 |
| `Navi_Speed_Ki` | 60 | `0x3C` | `0` | 距离速度决策积分项，默认关闭 |
| `Navi_Speed_Kd` | 61 | `0x3D` | `20` | 距离变化微分项，用于抑制速度突变 |
| `Navi_Speed_Max` | 62 | `0x3E` | `400` | 导航普通循迹最大目标速度 |
| `Navi_Speed_MaxStep` | 63 | `0x3F` | `12` | 每 10 ms 目标速度最大变化量 |

航点类型 `WayPoint_Type`：

| 值 | 类型 |
|---:|---|
| `0` | 普通循迹点 |
| `1` | 单边桥 |
| `2` | 跳跃台阶 |
| `3` | 定点排雷 |
| `4` | 绕圆锥桶 |
| `5` | 侧倾坡道 |
| `6` | 终点停车 |
| `7` | 原点 |

## 上位机如何交互

上位机通过 VOFA/WiFi 下发参数。单参数写入格式为：

```text
AA C2 参数ID float小端序
```

常用 float 小端值：

```text
0.0    = 00 00 00 00
1.0    = 00 00 80 3F
2.0    = 00 00 00 40
6.0    = 00 00 C0 40
1000.0 = 00 00 7A 44
```

如果需要确认导航模块开启，可发送运行模块指令，其中导航模块 ID 为 `4`：

```text
AA E2 01 04 01
```

开启静态地图循迹的典型流程：

```text
AA C2 34 00 00 00 00    // Navi_ModeMap = 0，选择静态测试地图
AA C2 36 00 00 80 3F    // Navi_PrintPoseEn = 1，打开位姿打印
AA C2 37 00 00 7A 44    // Navi_PrintPeriod = 1000 ms
AA C2 33 00 00 80 3F    // Navi_ModeDriver = 1，进入循迹模式
```

停止导航：

```text
AA C2 33 00 00 00 00    // Navi_ModeDriver = 0
```

使用打点地图的典型流程：

```text
AA C2 34 00 00 80 3F    // Navi_ModeMap = 1，选择打点地图
AA C2 33 00 00 00 40    // Navi_ModeDriver = 2，进入记录模式
```

记录普通航点时，让小车移动到需要记录的位置，然后触发一次 `0 -> 1` 边沿：

```text
AA C2 39 00 00 00 00    // Navi_WifiType = 0，普通循迹点
AA C2 35 00 00 00 00    // Navi_TrigRecord = 0，先清零
AA C2 35 00 00 80 3F    // Navi_TrigRecord = 1，触发打点
```

记录特殊航点时，先设置 `Navi_WifiType`，再触发打点。例如记录终点停车点：

```text
AA C2 39 00 00 C0 40    // Navi_WifiType = 6，终点停车
AA C2 35 00 00 00 00    // Navi_TrigRecord = 0
AA C2 35 00 00 80 3F    // Navi_TrigRecord = 1
```

打点完成后切换到循迹模式：

```text
AA C2 33 00 00 80 3F    // Navi_ModeDriver = 1
```

导航速度不再使用固定 `DEFAULT_TRACKING_VELOCITY`，而是在普通循迹且无动作接管时，根据当前点距离动态生成 `target_velocity`：

```text
distance
   |
   v
增量式 PID: Navi_Speed_Kp / Ki / Kd
   |
   v
限幅: 0 ~ Navi_Speed_Max
   |
   v
加速度限制: 每周期变化不超过 Navi_Speed_MaxStep
   |
   v
target_velocity
```

进入到点阈值、导航信息无效、动作接管或模式切换时，导航速度 PID 会清零，普通循迹速度输出为 0 或停止覆盖动作模块的速度。

观察导航状态有两种方式：一种是打开 `Navi_PrintPoseEn`，系统会按周期打印当前位置、目标点、距离和需转向角；另一种是使用 Wave 输出查看变量，导航相关通道为 `NavX`、`NavY`、`NavV`、`NavW`、`NavYaw`、`NavOK`。

注意：当前 `Navi_WifiCmd` 的 WiFi 动态追加航点逻辑尚未完整实现，可靠使用方式是静态地图或打点地图。

## 导航 Workflow 和数据流

导航整体 Workflow：

1. 系统初始化  
   `main_cm7_0.c` 中先初始化 IMU、平衡控制、电机驱动通信，然后调用 `navi_data_init()` 初始化导航滤波器和 EKF，再调用 `Navi_Tracking_Init()` 初始化航点控制器和默认地图。

2. 等待系统就绪  
   导航中断 `pit0_ch15_isr()` 会检查 `system_fully_ready`、`IMU_ready` 和 `RUNTIME_MODULE_NAVIGATION`。未满足时直接退出，不更新定位。

3. 设置导航原点  
   第一次进入导航中断时调用 `Navi_Data_Set_Origin()`，把当前位置设为 `(0,0)`，并把当前平均 yaw 作为本地坐标系零方向。

4. 周期定位更新  
   每 10 ms 调用 `navi_ekf_update()`：读取 IMU 和轮速，完成坐标映射、滤波、去重力、打滑检测、EKF 融合，再用融合速度和 yaw 积分更新 `robot_pose`。

5. 周期导航控制  
   定位完成后调用 `task_navigation_control()`。该函数根据 `Navi_ModeDriver` 选择停止、循迹或打点模式。

6. 航点跟踪  
   在循迹模式下，从 `point_map[]` 取当前目标点，计算当前位置到目标点的距离和方位角，再计算航向误差，生成 `target_angle` 和目标速度。

7. 到点与切点  
   若当前位置进入目标点阈值，`navi_isreach_target_point()` 判定到达，`navi_switch_nexttargetpoint()` 切换到下一个航点。

8. 特殊动作接管  
   若目标点类型是跳跃、单边桥、绕锥桶、侧坡或停车，`Navi_Action_Manager()` 会临时接管速度、腿长和车身姿态，动作完成后再交还循迹控制。

核心数据流：

```text
IMU 原始姿态/角速度/加速度
        +
左右电机 RPM
        |
        v
navi_parse_data()
轴向映射、符号统一、轮速换算、低通滤波、去重力
        |
        v
Navi_Slip_Detection()
判断轮速加速度和 IMU 加速度是否矛盾
        |
        v
EKF
预测：使用 IMU 前向加速度
更新：融合编码器线速度、编码器角速度、IMU gyro_z
        |
        v
opt_v / opt_w
融合后的前进速度和 yaw 角速度
        |
        v
航迹积分
直行用中点积分，转弯用圆弧模型积分
        |
        v
robot_pose
x, y, yaw, v, w, bias_ax, bias_w
```

循迹控制数据流：

```text
robot_pose 当前位姿
        +
point_map[current_idx] 当前目标航点
        |
        v
navi_calcnavinfo()
计算 distance 和 azimuth
        |
        v
need_to_turn = azimuth - robot_pose.yaw
        |
        v
target_angle
写入底层方向控制目标
        |
        v
距离 PID 速度决策
根据 distance 生成 target_velocity
        |
        v
底层平衡/速度/方向控制
驱动左右电机差速，让小车转向目标点
        |
        v
navi_isreach_target_point()
到达后切换下一个航点
```

上位机数据流：

```text
VOFA/WiFi 参数指令
        |
        v
VOFA_Protocol_Parse()
解析 AA C2 / AA E2 等指令
        |
        v
IPC 参数同步
写入 vofa_mode_driver、vofa_mode_map、vofa_trigger_record 等变量
        |
        v
Core0 主循环
同步到 navi_ctrl.navi_mode_driver / navi_ctrl.navi_mode_map
        |
        v
task_navigation_control()
根据模式执行停止、循迹或打点
```

最终输出数据流：

```text
导航模块输出 target_angle / target_velocity
        |
        v
方向环、速度环、平衡环
        |
        v
左右电机 PWM / 腿部姿态命令
        |
        v
小车运动
        |
        v
新的 IMU 和编码器反馈再次进入导航闭环
```
