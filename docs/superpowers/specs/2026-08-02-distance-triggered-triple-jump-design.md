# 距离触发三级跳设计文档

## 目标

基于最新 `p3` 分支中已经验证过的 PWM 跳跃动作，实现一个由屏幕控制的三级跳流程。操作者配置三段地面距离和一个前进速度，按下 `Go` 后，车辆锁定启动瞬间的 yaw，依次完成三次跳跃。腾空期间轮子空转不得计入里程。第三次可靠落地、发生故障或操作者主动停止后，车辆必须停车并返回 `Standby`。

## 范围

本次改动包含：

- 一个可复用的异步单跳动作接口；
- 一个可复用的 Z 轴落地检测器；
- 一个距离触发的三级跳编排器；
- 每次从起跳到落地期间的里程冻结机制；
- 一个带参数编辑和持久化功能的独立 Jump 页面；
- 用于配置、命令、状态和故障信息的双核 IPC；
- 将导航跳跃中的舵机动作阶段迁移到共享单跳接口。

原导航动作中的探边、后退、助跑、航点触发、遥控触发以及科目三路线逻辑仍属于导航流程，不进入新的 Jump 页面。进入 Jump 页面时绝不能直接触发原先的导航跳跃流程。

## 已确认的用户需求

- `Go` 会清零第一段距离，并立即以设定速度前进。
- `Go` 会锁存一次 `IMU_data.filter_result.yaw`；行驶、起跳、腾空和落地阶段始终使用这个 yaw 作为目标方向。
- 第一次跳跃在车辆距离 `Go` 起点达到 `x1` 米时触发。
- 第二次跳跃在第一次可靠落地后，地面净前向位移达到 `x2` 米时触发。
- 第三次跳跃在第二次可靠落地后，地面净前向位移达到 `x3` 米时触发。
- 每次起跳触发后立即冻结里程，直到可靠确认落地；编码器数据接收和轮速闭环仍继续运行。
- 屏幕显示可靠落地次数，范围为 `0/3` 到 `3/3`。
- 落地等待超时属于故障，绝不能当作成功落地。
- 第三次可靠落地时立即把前进速度设为零；舵机恢复完成后返回 `Standby`。
- UI 只提供五个可选行：`Go/Standby`、`x1`、`x2`、`x3` 和速度。
- `x1` 范围为 `0.00..1.00 m`；`x2`、`x3` 范围为 `0.00..0.20 m`；速度范围为 `0..300`。
- 距离调节步长为 `0.01 m`，速度调节步长为 `10`。
- 在 `Standby` 状态长按 `OK`，将四个参数统一写入 Flash。

## 总体架构

### 共享单跳动作内核

`jump_action.c/.h` 负责完整的非阻塞舵机动作序列，对外接口如下：

```c
typedef enum {
    JUMP_ACTION_PROFILE_FIRST = 0,
    JUMP_ACTION_PROFILE_FOLLOWUP
} JumpActionProfile_e;

typedef enum {
    JUMP_ACTION_IDLE = 0,
    JUMP_ACTION_PREPARE,
    JUMP_ACTION_TAKEOFF,
    JUMP_ACTION_AIRBORNE,
    JUMP_ACTION_RECOVER,
    JUMP_ACTION_DONE,
    JUMP_ACTION_FAULT
} JumpActionState_e;

typedef enum {
    JUMP_ACTION_RESULT_NONE = 0,
    JUMP_ACTION_RESULT_LANDED,
    JUMP_ACTION_RESULT_TIMEOUT,
    JUMP_ACTION_RESULT_ABORTED
} JumpActionResult_e;

uint8 JumpAction_Start(JumpActionProfile_e profile);
void JumpAction_Task5ms(float accel_z_g);
void JumpAction_Abort(void);
uint8 JumpAction_IsActive(void);
uint8 JumpAction_IsAirborne(void);
JumpActionState_e JumpAction_GetState(void);
JumpActionResult_e JumpAction_GetResult(void);
JumpActionResult_e JumpAction_ConsumeResult(void);
```

`JumpAction_Start()` 是业务层需要的“一次调用即可触发完整动作”的入口。系统 PIT 每 5 ms 调用一次任务函数推进动作，调用方不再自行编写离散的 PWM 和定时时序。

初始动作配置严格沿用最新 `p3/navigation_action.c` 中的参数：

| 阶段 | 第一跳 | 后续跳跃 |
| --- | ---: | ---: |
| 预压目标 | 370 PWM | 370 PWM |
| 预压时间 | 200 ms | 100 ms |
| 爆发起跳 | 1300 PWM，持续 180 ms | 1300 PWM，持续 180 ms |
| 空中收腿 | 420 PWM，动作 80 ms，再保持 50 ms | 相同 |
| 落地缓冲 | 450 PWM | 相同 |
| 落地恢复 | 400 PWM，持续 50 ms | 相同 |

所有对称舵机输出都通过同一个限幅函数，保持左右 PWM 之和为 `1500`，单侧 PWM 限制在跳跃专用的 `150..1350` 范围内。

跳跃舵机时序和落地状态转换只能由 `jump_action.c` 实现。原 `jump_control.c` 中的独立动作序列应删除或改为很薄的兼容封装，不能继续保留第二套实现。

### 落地检测器

`landing_detector.c/.h` 是一个确定性的可复用状态机，每 5 ms 接收一次 Z 轴加速度：

```c
typedef enum {
    LANDING_DETECTOR_WAIT_AIRBORNE = 0,
    LANDING_DETECTOR_WAIT_IMPACT,
    LANDING_DETECTOR_WAIT_SETTLE,
    LANDING_DETECTOR_LANDED,
    LANDING_DETECTOR_TIMEOUT
} LandingDetectorState_e;

void LandingDetector_Reset(float baseline_z_g);
LandingDetectorState_e LandingDetector_Update(float accel_z_g);
```

车辆处于地面状态时，检测器通过最近 100 ms 的滚动采样确定重力方向和 Z 轴基准值。如果基准绝对值不在 `0.60..1.40 g` 内，`Go` 将被拒绝并报告传感器故障。

落地检测严格按照以下顺序进行：

1. 起跳触发时复位检测器，但预压和爆发阶段不允许产生落地事件。
2. 进入腾空阶段后，连续四个样本中至少三个低于 `0.65 * baseline`，才确认出现腾空特征并使能落地判断。
3. 硬落地路径：四个样本窗口内至少两个样本超过 `baseline + 0.50 g`，随后五个样本中至少四个回到 `baseline +/- 0.35 g` 的稳定区间。
4. 软落地路径：确认腾空至少 80 ms 后，如果连续六个样本回到 `baseline +/- 0.25 g`，同样确认落地。
5. 从进入腾空阶段开始计时，如果 800 ms 内两条路径都没有确认落地，则报告 `TIMEOUT`。

检测器根据起跳前基准自动统一 Z 轴符号。起跳冲击、孤立噪声峰值以及未出现腾空特征便回到重力区间，都不能被计为落地。

### 三级跳编排器

`triple_jump.c/.h` 专门负责屏幕三级跳流程：

```c
typedef struct {
    float x1_m;
    float x2_m;
    float x3_m;
    float speed;
} TripleJumpConfig_t;

typedef enum {
    TRIPLE_JUMP_STANDBY = 0,
    TRIPLE_JUMP_DRIVING,
    TRIPLE_JUMP_EXECUTING,
    TRIPLE_JUMP_RECOVERING,
    TRIPLE_JUMP_FAULT
} TripleJumpState_e;

uint8 TripleJump_Start(const TripleJumpConfig_t *config, float current_yaw_deg);
void TripleJump_Task5ms(void);
void TripleJump_Stop(void);
TripleJumpState_e TripleJump_GetState(void);
float TripleJump_GetSegmentDistance(void);
uint8 TripleJump_GetLandingCount(void);
uint8 TripleJump_GetFault(void);
```

一次运行期间，编排器持有设定速度和锁存 yaw。每个 5 ms 周期都把锁存值写入 `target_angle`，确保除紧急安全流程外的其他逻辑不能改变运行方向。

状态流程如下：

```text
STANDBY
  -> Go：检查条件、锁存 yaw、清零分段起点
DRIVING
  -> 分段距离 >= x[n]：冻结里程、启动对应单跳配置
EXECUTING
  -> 确认落地：落地次数加一、重设分段起点、解除里程冻结
RECOVERING
  -> 舵机恢复期间允许累计真实地面距离，但不允许启动下一跳
  -> 落地次数 < 3：进入 DRIVING
  -> 落地次数 == 3：速度清零，进入 STANDBY
FAULT
  -> 速度清零、中止舵机动作、解除里程冻结，记录故障后进入 STANDBY
```

如果某一段配置距离为零，则在该段第一个允许触发的 5 ms 周期立即起跳。上一跳仍处于恢复阶段时，即使距离已经达到阈值，也不能提前启动下一跳。

### 里程处理

跳跃期间必须继续接收轮端遥测数据。若丢弃编码器 UART 帧，轮速闭环会使用陈旧数据，因此不能使用 `jump_should_suspend_encoder()` 停止数据接收。

改为在 `navigation_data_handling.c/.h` 中提供明确的里程冻结接口。冻结期间：

- IMU 姿态和 yaw 继续更新；
- 编码器采样继续供电机控制使用；
- 导航 EKF 不融合空中轮速；
- `robot_pose.x`、`robot_pose.y` 不进行积分；
- 三级跳当前分段距离保持不变。

可靠确认落地后，编排器立即以当前冻结位置建立新的分段起点，然后解除冻结。此时舵机恢复可以继续，但从物理落地点开始产生的真实地面运动会被计入下一段距离。分段距离使用锁存 yaw 方向上的净投影位移，并且最小限制为零；倒退会抵消之前的前进位移，而不是增加总行程。

操作者停止、落地超时、上层安全管理中止和紧急停止等所有退出路径，都必须且只能正确释放一次里程冻结。

### 与导航动作集成

现有导航外层状态继续负责：

- 探索台阶边缘；
- 触边后的稳定等待；
- 后退和助跑；
- 航点与遥控触发；
- 第二、第三跳的接近距离；
- 路线完成处理。

当导航外层状态决定起跳后，舵机时序和落地检测统一交给 `JumpAction_Start()` 与 `JumpAction_Task5ms()`。导航逻辑读取共享动作的阶段和结果，用于设置前进速度、腾空标志以及下一个外层状态。这样既保持现有导航路线行为，又能让全部跳跃动作只保留一个实现来源。

Jump 页面绝不调用 `IPC_Request_Nav_Jump()` 或 `Navi_Jump_Start()`。

## UI 与按键交互

在主页选中 `Jump` 后进入 `UI_SCREEN_JUMP`，不再立即触发任何动作。

页面包含五个可选行和三个只读状态行：

```text
------ TRIPLE JUMP ------
Mode       Standby / Go
X1         0.50 m
X2         0.15 m
X3         0.15 m
Speed      0
Landings   0 / 3
Distance   0.00 m
Status     READY
```

- `UP/DOWN`：选择行。
- 在模式行按 `OK`：切换 `Standby/Go`。
- 在参数行按 `OK`：进入或确认参数编辑。
- 编辑时使用 `UP/DOWN`：距离按 `0.01 m`、速度按 `10` 调节，并严格限制范围。
- 编辑状态按 `BACK`：取消本次编辑；非编辑状态按 `BACK`：如果正在运行，先停止并返回 `Standby`，然后回到主页。
- 在 `Standby` 长按 `OK`：保存全部四个参数。
- 运行期间禁止编辑参数和写入 Flash。

状态栏至少能够区分：`READY`、`RUNNING`、`AIRBORNE`、`RECOVER`、`DONE`、`IMU ERROR`、`BUSY`、`TIMEOUT`、`ABORTED`、`SAVED` 和 `SAVE ERROR`。

## Flash 持久化配置

现有通用参数表已经包含 63 个参数，并使用 64 位更新掩码。新增四个三级跳参数不能继续追加到该表中，因此采用独立的版本化配置结构。

Flash 页面分配如下：

- `0..79` 页：导航点组；
- `94` 页：三级跳配置；
- `95` 页：现有通用参数。

第 94 页记录包含魔数、格式版本、`TripleJumpConfig_t` 和 CRC32。只有魔数、版本、CRC 均正确，并且四个参数都是有限数值且处于规定范围内时，配置才有效；否则自动使用以下安全默认值：

```text
x1 = 0.50 m
x2 = 0.15 m
x3 = 0.15 m
speed = 0
mode = Standby
```

Core 1 启动时、主 UI 显示前完成配置加载。运行模式和跳跃进度不写入 Flash。

## IPC 通信约定

共享命令区增加独立的三级跳配置，以及单调递增的 `start_seq` 和 `stop_seq`。Core 0 保存最后一次已消费的序号，从而允许周期轮询且不会漏掉布尔标志的短暂跳变。

Core 0 向屏幕回传：

- 三级跳编排器状态；
- 已确认落地次数；
- 当前分段距离；
- 锁存 yaw；
- 当前单跳动作阶段；
- 故障码；
- 最近确认的启动和停止序号。

增加字段后，必须继续通过共享内存大小的编译期断言。

## 安全与动作仲裁

只有满足以下条件时才接受 `Go`：

- 系统和 IMU 已完成初始化；
- Z 轴重力基准有效；
- 电机、平衡和舵机模块已启用；`Go` 会在清零起点前启用导航/里程模块；
- 紧急停止未激活；
- 当前没有独立跳跃、导航跳跃、颠簸动作或科目三动作占用执行器；
- 四个配置值均为有限数值且位于规定范围内。

启动检查失败时，流程保持 `Standby`、速度设为零，并显示明确的故障原因。紧急停止和车辆安全管理复位必须中止动作、释放运动控制权并解除里程冻结。第三次落地时立即把速度设为零，舵机安全恢复完成后再正式进入 `Standby`。

## 测试策略

主机侧确定性测试通过注入加速度、时间、位姿和动作结果进行。必须覆盖：

- 第一跳和后续跳跃的 PWM 阶段边界；
- 对称 PWM 限幅；
- 起跳冲击不能被判为落地；
- 孤立加速度峰值不能被判为落地；
- 硬冲击后稳定的落地路径；
- 经过腾空确认后的软落地路径；
- 超时不能产生虚假落地次数；
- `Go` 只锁存一次 yaw，并清零第一段距离；
- `x1`、`x2`、`x3` 按顺序触发，包括距离为零的情况；
- 腾空期间分段距离保持不变；
- 倒退位移抵消前进位移；
- 每次可靠落地只增加一次计数；
- 第三次落地后，舵机恢复完成并返回 `Standby`；
- 停止、故障和紧急退出路径都会解除里程冻结；
- 参数范围、调节步长、默认值回退、CRC 拒绝和 Flash 读写闭环；
- 从主页进入 Jump 页面不会调用旧导航跳跃；
- 共享单跳动作迁移后，现有导航动作测试仍然通过。

最终验证还包括仓库现有的全部主机测试；若当前环境具备对应工具链，还需要完成 IAR 工程构建。

## 成功标准

- 调用方只需调用一次 `JumpAction_Start()`，即可启动完整舵机跳跃动作。
- Jump 页面路径和导航路径之间不存在重复的跳跃舵机时序。
- 空中轮子空转不会改变导航位置和三级跳分段距离。
- 只有经过腾空确认和稳定确认的 Z 轴落地事件才增加屏幕计数。
- 三段触发距离分别以 `Go` 起点、第一次落地点和第二次落地点为零点。
- 一次运行期间锁存 yaw 始终不变。
- 第三次落地、落地超时、操作者停止和紧急停止最终都会使车辆停车、舵机处于安全状态、里程解除冻结。
- 保存的参数重启后仍可读取；Flash 数据损坏时自动使用安全默认值。
