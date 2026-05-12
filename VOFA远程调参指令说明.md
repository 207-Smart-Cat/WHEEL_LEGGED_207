# VOFA 远程调参与 WiFi 指令说明

本文档对应当前项目 project/code/vofa_protocol.c 与 project/code/param_registry.def 的实现。

## 基本约定

- 所有 HEX 字节用空格分隔。
- float 使用 IEEE754 小端序。例如 1.0f = 00 00 80 3F，0.0f = 00 00 00 00，1000.0f = 00 00 7A 44。
- 参数 ID 从 1 开始，对应 param_registry.def 的顺序。
- 当前单参数写入协议是 AA C2，不是 AA C1。

## 常用指令

| 功能 | 指令格式 | 说明 |
|---|---|---|
| 切换 WiFi 模式 | AA EE mode | mode=0 Silent, 1 Wave, 2 Image, 3 Log |
| 切换通道显示 | AA FF ch | ch=1..5，翻转显示开关 |
| 参数日志简洁模式 | AA E1 00 | 只输出简洁调参日志 |
| 参数日志详细模式 | AA E1 01 | 输出来源、命令等详细信息 |
| 参数日志模式切换 | AA E1 02 | 简洁/详细之间切换 |
| 查询运行状态 | AA E2 00 | 打印 runtime 模块状态 |
| 设置运行模块 | AA E2 01 id enable | id=0..5，enable=0/1 |
| 设置小车模式 | AA E2 02 mode | mode=0..3 |
| 翻转运行模块 | AA E2 03 id | id=0..5 |
| 急停 | AA C1 01 00 00 6C | 动作控制协议，不是调参协议 |
| 单参数写入 | AA C2 id float4 | 不带校验；共 7 字节 |
| 保存参数到 Flash | AA C3 88 55 | 电机 PWM 较大时会拒绝保存 |
| 批量写入参数 | AA C4 [PARAM_COUNT个float] checksum | checksum 为所有 float 数据字节累加低 8 位 |
| 读取当前参数 | AA C5 88 55 | 返回 AA C4 参数包，并打印参数摘要 |

## 单参数写入示例

写入 Navi_ModeDriver = 1.0：

~~~text
AA C2 33 00 00 80 3F
~~~

写入 Navi_ModeDriver = 0.0：

~~~text
AA C2 33 00 00 00 00
~~~

写入 Navi_PrintPeriod = 1000.0：

~~~text
AA C2 37 00 00 7A 44
~~~

## 导航观察模式常用参数

| 参数 | ID | HEX ID | 常用值 | 作用 |
|---|---:|---:|---|---|
| Navi_ModeDriver | 51 | 0x33 | 0=停用, 1=已有地图计算, 2=记录/远程模式 | 当前代码中 1 只观察打印，不接管小车 |
| Navi_ModeMap | 52 | 0x34 | 0=静态地图, 1=打点地图, 2=WiFi动态地图 | 选择地图来源 |
| Navi_TrigRecord | 53 | 0x35 | 0/1 | 打点触发，通常需要 0->1 边沿 |
| Navi_PrintPoseEn | 54 | 0x36 | 1 | 允许导航位姿/目标点打印 |
| Navi_PrintPeriod | 55 | 0x37 | 1000 | 导航打印周期，单位 ms |
| Navi_WifiCmd | 56 | 0x38 | 0/1/2/3 | WiFi 动态地图/动作命令入口 |
| Navi_WifiType | 57 | 0x39 | 航点类型枚举 | 动态航点类型 |
| Navi_WifiAction | 58 | 0x3A | 动作状态枚举 | 动态动作指令 |

打开静态地图观察打印：

~~~text
AA C2 34 00 00 00 00
AA C2 36 00 00 80 3F
AA C2 37 00 00 7A 44
AA C2 33 00 00 80 3F
~~~

关闭导航观察计算：

~~~text
AA C2 33 00 00 00 00
~~~

## Wave 文本指令

这些是 ASCII 文本，不是 HEX：

~~~text
WAVE?
WAVE OFF
WAVE 1 4 6
WAVE 1,4,6
~~~

- WAVE?：打印 Wave 变量 ID 对照表。
- WAVE OFF：进入 Silent。
- WAVE 1 4 6：选择最多 6 个变量并进入 Wave 输出。

## Wave 变量 ID

| ID | 变量名 |
|---:|---|
| 1 | Roll |
| 2 | Pitch |
| 3 | Yaw |
| 4 | L_Spd |
| 5 | R_Spd |
| 6 | L_PWM |
| 7 | R_PWM |
| 8 | SpdO_L |
| 9 | SpdO_R |
| 10 | AngO_L |
| 11 | AngO_R |
| 12 | GyrO_L |
| 13 | GyrO_R |
| 14 | TurnO |
| 15 | LegO |
| 16 | LegTilt |
| 17 | LegXOff |
| 18 | LegXTar |
| 19 | LegTick |
| 20 | Battery |
| 21 | NavX |
| 22 | NavY |
| 23 | NavV |
| 24 | NavW |
| 25 | NavYaw |
| 26 | NavOK |
| 27 | LegXGain |
| 28 | LegXLim |
| 29 | LegXStep |
| 30 | LegXHit |
| 31 | ZeroSt |
| 32 | ZeroMs |
| 33 | ZeroRx |
| 34 | ZeroSpd |
| 35 | ZeroStart |
| 36 | ZeroTx |
| 37 | ZeroTask |
| 38 | ZeroRxCnt |

## 参数 ID 对照表

| ID | HEX ID | 参数名 | 作用 |
|---:|---:|---|---|
| 1 | 0x01 | Q_yaw | 旧姿态滤波 yaw 过程噪声参数；当前 660RC 四元数方案下基本保留兼容。 |
| 2 | 0x02 | Q_pr | 旧姿态滤波 pitch/roll 过程噪声参数；当前 660RC 四元数方案下基本保留兼容。 |
| 3 | 0x03 | Q_bias | 旧姿态滤波陀螺零偏过程噪声；当前 660RC 四元数方案下基本保留兼容。 |
| 4 | 0x04 | R_yaw | 旧姿态滤波 yaw 观测噪声；当前 660RC 四元数方案下基本保留兼容。 |
| 5 | 0x05 | R_pr | 旧姿态滤波 pitch/roll 观测噪声；当前 660RC 四元数方案下基本保留兼容。 |
| 6 | 0x06 | Speed_P | 速度外环 P，目标速度与实际速度误差输出给速度环结果。 |
| 7 | 0x07 | Speed_I | 速度外环 I，用于消除长期位置/速度偏差，过大会慢性发散或来回跑。 |
| 8 | 0x08 | Speed_D | 速度外环 D，目前一般不重点使用，用于速度变化阻尼。 |
| 9 | 0x09 | Angle_P | 角度环 P，姿态角误差输出角速度目标/扰动量。 |
| 10 | 0x0A | Angle_I | 角度环 I，用于修正平衡点附近长期单边跑。 |
| 11 | 0x0B | Angle_D | 角度环 D，用于抑制角度变化和中扰动超调。 |
| 12 | 0x0C | Gyro_P | 角速度内环 P，直接决定抑制角速度的力度。 |
| 13 | 0x0D | Gyro_I | 角速度内环 I，通常谨慎使用，用于角速度零偏残差补偿。 |
| 14 | 0x0E | Gyro_D | 角速度内环 D，用于抑制高频角速度变化，过大易抖。 |
| 15 | 0x0F | Target_Velocity | 目标速度，遥控/WiFi/导航可能修改。当前腿部前后倾斜也会参考速度环结果。 |
| 16 | 0x10 | Target_Angle | 目标航向角/转向目标，方向环使用。 |
| 17 | 0x11 | Target_Motor_Stand | 车身平衡机械零点/站立角补偿。 |
| 18 | 0x12 | Leg_Kp | 腿长/腿部高度控制 PID 的 P，当前具体作用取决于腿控是否启用。 |
| 19 | 0x13 | Leg_Ki | 腿长/腿部高度控制 PID 的 I。 |
| 20 | 0x14 | Leg_Kd | 腿长/腿部高度控制 PID 的 D。 |
| 21 | 0x15 | X_Current | 腿部五连杆目标 X 位置基础值，影响前后方向腿姿态。 |
| 22 | 0x16 | Y_Current | 腿部五连杆目标 Y 位置基础值，影响车身高度/腿长。 |
| 23 | 0x17 | Air_Roll_P | 空中/横滚辅助控制 P，主要用于跳跃/腾空相关姿态辅助。 |
| 24 | 0x18 | Air_Roll_I | 空中/横滚辅助控制 I。 |
| 25 | 0x19 | Air_Roll_D | 空中/横滚辅助控制 D。 |
| 26 | 0x1A | Direction_P | 方向/航向环 P，用于根据目标航向生成差速转向。 |
| 27 | 0x1B | Direction_I | 方向/航向环 I，用于消除长期航向偏差。 |
| 28 | 0x1C | Direction_D | 方向/航向环 D，用于提供转向阻尼。 |
| 29 | 0x1D | Nav_Q_V | 惯导 EKF 线速度过程噪声，越大越不信模型预测速度。 |
| 30 | 0x1E | Nav_Q_W | 惯导 EKF 角速度过程噪声，越大越不信模型预测角速度。 |
| 31 | 0x1F | Nav_Q_Bias_Ax | 惯导 EKF 加速度零偏游走噪声，影响加速度偏置估计速度。 |
| 32 | 0x20 | Nav_Q_Bias_W | 惯导 EKF 角速度零偏游走噪声，影响陀螺偏置估计速度。 |
| 33 | 0x21 | Nav_R_V_Normal | 惯导 EKF 正常状态下轮速线速度观测噪声，越小越信轮速。 |
| 34 | 0x22 | Nav_R_V_Slip | 惯导 EKF 打滑状态下线速度观测噪声，越大越不信轮速。 |
| 35 | 0x23 | Nav_R_W_Normal | 惯导 EKF 正常状态下角速度观测噪声，越小越信观测角速度。 |
| 36 | 0x24 | Nav_R_W_Slip | 惯导 EKF 打滑状态下角速度观测噪声，越大越不信轮速推导角速度。 |
| 37 | 0x25 | Nav_R_Gyro | 惯导 EKF 陀螺仪角速度观测噪声，越小越信 IMU gyro。 |
| 38 | 0x26 | Mag_Offset_X | 磁力计 X 偏置参数；当前更换 IMU 后基本保留兼容，暂不作为主导航依据。 |
| 39 | 0x27 | Mag_Offset_Y | 磁力计 Y 偏置参数；当前更换 IMU 后基本保留兼容。 |
| 40 | 0x28 | Mag_Scale_X | 磁力计 X 比例校准；当前基本保留兼容。 |
| 41 | 0x29 | Mag_Scale_Y | 磁力计 Y 比例校准；当前基本保留兼容。 |
| 42 | 0x2A | Leg_X_Gain | 速度环输出映射到腿部 X 前后倾斜的增益。 |
| 43 | 0x2B | Leg_X_Limit | 速度环映射到腿部 X 偏移的最大限幅。 |
| 44 | 0x2C | Leg_X_MinStep | 腿部 X 偏移最小有效变化步长/死区，用于避免小抖动。 |
| 45 | 0x2D | Leg_X_StepLimit | 腿部 X 偏移每周期最大变化量，用于限制舵机响应速度。 |
| 46 | 0x2E | Jump_Burst_PWM | 跳跃爆发阶段舵机目标 PWM/力度参数。 |
| 47 | 0x2F | Jump_Burst_MS | 跳跃爆发阶段持续时间，单位 ms。 |
| 48 | 0x30 | Jump_AirRetract_Y | 跳跃腾空/收腿阶段目标 Y，用于空中腿长。 |
| 49 | 0x31 | Jump_Buffer_Y | 跳跃落地缓冲阶段目标 Y，用于落地缓冲腿长。 |
| 50 | 0x32 | Jump_LandingMax_MS | 跳跃等待落地的最大时间，单位 ms，超时进入恢复。 |
| 51 | 0x33 | Navi_ModeDriver | 导航驱动模式：0停用，1已有地图计算，2记录/远程模式。当前 1 只观察打印不接管。 |
| 52 | 0x34 | Navi_ModeMap | 导航地图模式：0静态测试地图，1打点地图，2WiFi动态地图。 |
| 53 | 0x35 | Navi_TrigRecord | 导航打点触发变量，通常从 0 变 1 触发一次记录。 |
| 54 | 0x36 | Navi_PrintPoseEn | 导航位姿/目标点/距离/需转向角打印开关，1开启。 |
| 55 | 0x37 | Navi_PrintPeriod | 导航打印周期，单位 ms。 |
| 56 | 0x38 | Navi_WifiCmd | 导航 WiFi 动态地图命令入口：0待机，1追加点，2清空地图，3触发动作。 |
| 57 | 0x39 | Navi_WifiType | 导航 WiFi 动态航点类型，对应 WayPoint_Type 枚举。 |
| 58 | 0x3A | Navi_WifiAction | 导航 WiFi 动态动作指令，对应动作状态/动作命令。 |

## 注意事项

- AA C1 不是调参，当前主要用于急停/动作控制。
- 新增参数后，如果 Flash 里保存的是旧参数表，新参数会使用 param.c 中的默认值。
- 如果修改了参数但重启后丢失，需要发送 AA C3 88 55 保存到 Flash。
- AA C5 88 55 可以读取当前 Core0 实际使用的参数，并返回一帧 AA C4 参数包。
- 当前导航已改成观察模式：Navi_ModeDriver=1 只计算和打印，不写 target_angle / target_velocity。