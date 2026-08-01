# 独立距离触发三级跳实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Jump 页面中实现一个与现有导航状态机隔离、支持 Flash 调参、固定 yaw、空中暂停计距以及三次可靠落地后自动回到 Standby 的三级跳模块。

**Architecture:** `landing_detector` 负责纯 Z 轴落地判定，`jump_control` 负责一次完整 PWM 跳跃，`triple_jump` 负责纯三级跳状态和独立编码器分段里程，`triple_jump_runtime` 负责 Core 0 硬件接线。Core 1 通过 IPC 发送配置和 Go/Standby 请求并显示 Core 0 状态；第 94 页 Flash 独立保存四个参数。现有 `navigation_action.c/.h` 不修改。

**Tech Stack:** C11 主机测试、IAR Embedded Workbench C、TRAVEO T2G 双核共享内存、IPS200 四键 UI、片内 Work Flash。

## 全局约束

- `navigation_action.c/.h` 不产生代码改动。
- `x1` 范围 `0.00..1.00 m`；`x2/x3` 范围 `0.00..0.20 m`；速度范围 `0..300`。
- 距离调节步长 `0.01 m`；速度调节步长 `10`。
- `Go` 时锁存 `IMU_data.filter_result.yaw`，直到第三跳结束或中止都不得更新锁存值。
- 起跳触发到可靠落地期间，独立分段里程保持不变，但编码器 UART 接收继续。
- 第三次可靠落地立即将速度设为零，舵机恢复结束后进入 `Standby`。
- 落地超时属于故障，不增加落地次数。
- Flash 第 `94` 页保存三级跳配置；页面 `0..79` 和 `95` 的现有用途保持不变。
- 所有新增生产行为先通过失败测试定义，再写最小实现。

---

### Task 1: 纯 Z 轴落地检测器

**Files:**
- Create: `project/code/landing_detector.h`
- Create: `project/code/landing_detector.c`
- Create: `tests/test_landing_detector.c`

**Interfaces:**
- Consumes: 每 5 ms 一个单位为 g 的 Z 轴加速度样本，以及起跳前 100 ms 基准值。
- Produces: `LandingDetector_Init`、`LandingDetector_BeginAirborne`、`LandingDetector_Update`、`LandingDetector_GetState`，供 `jump_control.c` 使用。

- [ ] **Step 1: 写入失败测试**

```c
int main(void)
{
    LandingDetector_t detector;
    uint32_t i;
    LandingDetector_Init(&detector, 1.0f);
    assert(LandingDetector_Update(&detector, 1.8f) == LANDING_DETECTOR_WAIT_AIRBORNE);
    LandingDetector_BeginAirborne(&detector);
    for (i = 0U; i < 4U; ++i) LandingDetector_Update(&detector, 0.2f);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_WAIT_IMPACT);
    LandingDetector_Update(&detector, 1.7f);
    LandingDetector_Update(&detector, 1.7f);
    for (i = 0U; i < 5U; ++i) LandingDetector_Update(&detector, 1.0f);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_LANDED);

    LandingDetector_Init(&detector, 1.0f);
    LandingDetector_BeginAirborne(&detector);
    for (i = 0U; i < 4U; ++i) LandingDetector_Update(&detector, 0.2f);
    for (i = 0U; i < 160U; ++i) LandingDetector_Update(&detector, 0.2f);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_TIMEOUT);

    LandingDetector_Init(&detector, 1.0f);
    LandingDetector_BeginAirborne(&detector);
    for (i = 0U; i < 20U; ++i) LandingDetector_Update(&detector, 0.2f);
    for (i = 0U; i < 6U; ++i) LandingDetector_Update(&detector, 1.0f);
    assert(LandingDetector_GetState(&detector) == LANDING_DETECTOR_LANDED);
    return 0;
}
```

- [ ] **Step 2: 运行测试并确认因缺少接口而失败**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_landing_detector.c project/code/landing_detector.c -o tests/test_landing_detector.exe`

Expected: FAIL，提示 `landing_detector.h` 或接口不存在。

- [ ] **Step 3: 实现最小检测状态机**

实现以下常量与顺序：100 ms 基准；基准绝对值 `0.60..1.40 g`；四样本中三个低于 `0.65 * baseline` 才使能；硬落地为四样本中两次超过 `baseline + 0.50 g` 后五样本中四次稳定；软落地为腾空至少 80 ms 后连续六次进入 `baseline +/- 0.25 g`；800 ms 超时。

- [ ] **Step 4: 运行并确认检测器测试通过**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_landing_detector.c project/code/landing_detector.c -o tests/test_landing_detector.exe; .\tests\test_landing_detector.exe`

Expected: exit 0，无警告。

- [ ] **Step 5: 提交检测器**

```bash
git add project/code/landing_detector.h project/code/landing_detector.c tests/test_landing_detector.c
git commit -m "feat: add robust landing detector"
```

### Task 2: 独立三级跳状态与编码器里程逻辑

**Files:**
- Create: `project/code/jump_action_types.h`
- Create: `project/code/triple_jump.h`
- Create: `project/code/triple_jump.c`
- Create: `tests/test_triple_jump.c`

**Interfaces:**
- Consumes: `TripleJumpConfig_t`、启动 yaw、左右轮 RPM、单跳动作阶段/结果和 5 ms tick。
- Produces: 标准整数类型的 `JumpActionProfile_e/JumpActionResult_e`、`TripleJump_ConfigIsValid`、纯 `TripleJumpContext_t` 状态机，以及 `TripleJumpOutput_t` 中的目标速度、目标 yaw、起跳请求、动作配置和停止请求。

- [ ] **Step 1: 写入失败测试**

```c
int main(void)
{
    TripleJumpContext_t ctx;
    TripleJumpOutput_t out = {0};
    TripleJumpInput_t in = {0};
    uint32_t i;
    TripleJumpConfig_t cfg = {0.10f, 0.05f, 0.05f, 200.0f};
    TripleJump_Init(&ctx);
    assert(TripleJump_Start(&ctx, &cfg, 37.0f));
    in.left_rpm = -120.0f;
    in.right_rpm = 120.0f;
    for (i = 0U; i < 500U && !out.start_jump; ++i)
        TripleJump_Update5ms(&ctx, &in, &out);
    assert(out.start_jump && out.profile == JUMP_ACTION_PROFILE_FIRST);
    assert(TripleJump_GetSegmentDistance(&ctx) >= 0.10f);
    {
        float frozen_m = TripleJump_GetSegmentDistance(&ctx);
        in.left_rpm = -3000.0f;
        in.right_rpm = 3000.0f;
        for (i = 0U; i < 100U; ++i) TripleJump_Update5ms(&ctx, &in, &out);
        assert(fabsf(TripleJump_GetSegmentDistance(&ctx) - frozen_m) < 0.0001f);
    }
    in.action_result = JUMP_ACTION_RESULT_LANDED;
    TripleJump_Update5ms(&ctx, &in, &out);
    assert(TripleJump_GetLandingCount(&ctx) == 1U);
    assert(fabsf(TripleJump_GetSegmentDistance(&ctx)) < 0.0001f);
    return 0;
}
```

- [ ] **Step 2: 运行并确认因接口缺失失败**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_triple_jump.c project/code/triple_jump.c -lm -o tests/test_triple_jump.exe`

Expected: FAIL，提示 `triple_jump.h` 或接口不存在。

- [ ] **Step 3: 实现配置校验、独立计距和状态转换**

前向 RPM 必须沿用控制层符号：`((-left_rpm) - (-right_rpm)) * 0.5f`。转换为 m/s 后按 `0.005f` 积分，并用独立一阶低通滤波；只有 `DRIVING/RECOVERING` 且没有计距暂停时累计。每次落地事件只消费一次；第三次落地先输出速度零，收到单跳恢复完成后进入 `Standby`。

- [ ] **Step 4: 增加边界测试**

覆盖 `x1/x2/x3=0`、配置越界/NaN、倒退抵消、空中 RPM 极大值不计数、yaw 在更新输入变化时仍保持启动值、超时进入故障且落地次数不增加、Stop 清理计距暂停。

- [ ] **Step 5: 运行并确认全部逻辑测试通过**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_triple_jump.c project/code/triple_jump.c -lm -o tests/test_triple_jump.exe; .\tests\test_triple_jump.exe`

Expected: exit 0，无警告。

- [ ] **Step 6: 提交三级跳纯逻辑**

```bash
git add project/code/jump_action_types.h project/code/triple_jump.h project/code/triple_jump.c tests/test_triple_jump.c
git commit -m "feat: add distance-triggered triple jump logic"
```

### Task 3: 将 P3 PWM 动作封装进 jump_control

**Files:**
- Create: `project/code/jump_action_profile.h`
- Create: `project/code/jump_action_profile.c`
- Modify: `project/code/jump_control.h`
- Modify: `project/code/jump_control.c`
- Modify: `project/user/cm7_0_isr.c`
- Create: `tests/test_jump_action_profile.c`

**Interfaces:**
- Consumes: Task 1 的 `LandingDetector_t` 与 `JumpActionProfile_e`。
- Produces: `JumpAction_Start`、`JumpAction_Task5ms`、`JumpAction_Abort`、动作状态查询和一次性结果消费；保留 `jump_drive_symmetric_pwm`、`jump_calc_prepare_pwm` 供现有代码编译。

- [ ] **Step 1: 写动作配置失败测试**

测试第一跳预压 `200 ms`、后续跳预压 `100 ms`、爆发 `1300/180 ms`、收腿 `420/(80+50) ms`、缓冲 `450`、恢复 `400/50 ms`，以及 PWM 限制 `150..1350` 和总和 `1500`。

- [ ] **Step 2: 编译并确认旧实现不满足新配置**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_jump_action_profile.c project/code/jump_action_profile.c -o tests/test_jump_action_profile.exe`

Expected: FAIL，提示动作配置接口不存在或数值不匹配。

- [ ] **Step 3: 重构 jump_control 状态机**

`JumpAction_Task5ms` 每次只增加 5 ms；进入 `AIRBORNE` 时调用 `LandingDetector_BeginAirborne`；只有 `LANDED` 才产生 `JUMP_ACTION_RESULT_LANDED`，`TIMEOUT` 产生故障并进入安全恢复。`jump_start()`、`jump_process_control()`、`jump_abort()` 作为兼容包装，不再拥有第二套时序。

- [ ] **Step 4: 将 PIT_CH14 从 1 ms 状态推进改为 5 ms**

保留 1 ms PIT 配置，但在 ISR 内用 `static uint8 divider` 每五次调用一次 `TripleJumpRuntime_Task5ms()`；紧急停止时调用统一 Stop/Abort。

- [ ] **Step 5: 运行动作配置与前两项测试**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_jump_action_profile.c project/code/jump_action_profile.c -o tests/test_jump_action_profile.exe; .\tests\test_jump_action_profile.exe`

Expected: exit 0。

- [ ] **Step 6: 提交单跳动作内核**

```bash
git add project/code/jump_action_profile.h project/code/jump_action_profile.c project/code/jump_control.h project/code/jump_control.c project/user/cm7_0_isr.c tests/test_jump_action_profile.c
git commit -m "refactor: encapsulate p3 PWM jump action"
```

### Task 4: Core 0 运行时适配与双核 IPC

**Files:**
- Create: `project/code/triple_jump_runtime.h`
- Create: `project/code/triple_jump_runtime.c`
- Modify: `project/code/app_headfile.h`
- Modify: `project/code/ipc_shared_data.h`
- Modify: `project/code/ipc_shared_data.c`
- Modify: `project/code/status_registry.def`
- Modify: `project/code/vehicle_supervisor.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Create: `project/code/triple_jump_integration_test.ps1`

**Interfaces:**
- Consumes: Task 2 输出和 Task 3 单跳入口；`motor_value`、`IMU_data`、`target_velocity`、`target_angle`、运行模块状态。
- Produces: `TripleJumpRuntime_RequestStart/Stop/Task5ms`，IPC 的 `start_seq/stop_seq/config`，以及屏幕可读状态。

- [ ] **Step 1: 写静态集成失败测试**

脚本必须断言：Jump 请求不再写 `nav_jump_request`；Core B 配置包含四项和序号；Core A 状态包含 state/landings/distance/yaw/fault；PIT_CH14 调用独立任务；`navigation_action.c` 的 Git diff 为空。

- [ ] **Step 2: 运行并确认脚本失败**

Run: `powershell -ExecutionPolicy Bypass -File project/code/triple_jump_integration_test.ps1`

Expected: FAIL，提示三级跳 IPC 或运行时接口缺失。

- [ ] **Step 3: 实现 IPC 和 Core 0 适配**

Core 1 更新完整配置后递增 `start_seq`；Core 0 只消费大于已确认序号的新请求。Go 时锁存当前 IMU yaw、将导航模块关闭、启动纯状态机；每 5 ms 写目标速度/yaw并驱动单跳。Stop、监督器复位和紧急停止统一清零速度、Abort 单跳并回 Standby。

- [ ] **Step 4: 更新共享内存断言和 IAR 工程文件**

新增结构后保留 `sizeof(CoreA_Status_t) <= IPC_CORE_A_SHARED_SIZE` 与 `sizeof(CoreB_Command_t) <= IPC_CORE_B_SHARED_SIZE`。将 `landing_detector.c`、`jump_action_profile.c`、`triple_jump.c`、`triple_jump_runtime.c` 及头文件加入两个 IAR 工程，不增加 `navigation_action.c` 改动。

- [ ] **Step 5: 运行静态集成测试**

Run: `powershell -ExecutionPolicy Bypass -File project/code/triple_jump_integration_test.ps1`

Expected: `triple jump integration checks passed`。

- [ ] **Step 6: 提交运行时与 IPC**

```bash
git add project/code/triple_jump_runtime.* project/code/app_headfile.h project/code/ipc_shared_data.* project/code/status_registry.def project/code/vehicle_supervisor.c project/user/cm7_0_isr.c project/iar/project_config/*.ewp project/code/triple_jump_integration_test.ps1
git commit -m "feat: wire standalone triple jump runtime"
```

### Task 5: Flash 配置与 Jump 页面

**Files:**
- Create: `project/code/triple_jump_config.h`
- Create: `project/code/triple_jump_config.c`
- Create: `tests/test_triple_jump_config.c`
- Modify: `project/code/screen_display.c`
- Modify: `project/user/main_cm7_1.c`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_0.ewp`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp`
- Create: `project/code/triple_jump_ui_test.ps1`

**Interfaces:**
- Consumes: `TripleJumpConfig_t`、Flash 页 94、Task 4 IPC 状态与请求接口。
- Produces: `TripleJumpConfig_Default`、`TripleJumpConfig_Encode`、`TripleJumpConfig_Decode`、CRC32、Flash 读写、`UI_SCREEN_JUMP` 绘制和按键处理。

- [ ] **Step 1: 写配置失败测试**

```c
int main(void)
{
    TripleJumpConfig_t cfg;
    TripleJumpConfig_t decoded;
    TripleJumpConfigRecord_t record;
    TripleJumpConfig_Default(&cfg);
    assert(cfg.x1_m == 0.50f && cfg.x2_m == 0.15f && cfg.x3_m == 0.15f);
    assert(cfg.speed == 0.0f);
    assert(TripleJump_ConfigIsValid(&cfg));
    cfg.x2_m = 0.21f;
    assert(!TripleJump_ConfigIsValid(&cfg));
    TripleJumpConfig_Default(&cfg);
    TripleJumpConfig_Encode(&cfg, &record);
    assert(TripleJumpConfig_Decode(&record, &decoded));
    record.config.x2_m += 0.01f;
    assert(!TripleJumpConfig_Decode(&record, &decoded));
    return 0;
}
```

- [ ] **Step 2: 编译并确认配置接口缺失失败**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_triple_jump_config.c project/code/triple_jump_config.c project/code/triple_jump.c -lm -o tests/test_triple_jump_config.exe`

Expected: FAIL，提示配置接口不存在。

- [ ] **Step 3: 实现纯配置、CRC 与页 94 存储**

记录格式固定为魔数、版本、四项 float、CRC32。读取失败、NaN、Inf 或越界均回退 `0.50/0.15/0.15/0`。只允许 Standby 写 Flash；保存后读回校验并返回成功/失败。

- [ ] **Step 4: 写 UI 静态失败测试**

脚本断言主页 Jump 进入 `UI_SCREEN_JUMP` 而不是调用 `IPC_Request_Nav_Jump`；页面包含 Mode/X1/X2/X3/Speed/Landings/Distance/Status；运行中拒绝编辑和保存；长按 OK 调用独立保存接口；BACK 先 Stop 再回主页。

- [ ] **Step 5: 实现 Jump 页面与启动加载**

页面使用现有四键事件框架。五个可选行、三个只读行；距离步进 `0.01`，速度步进 `10`；Go/Standby 使用 Task 4 IPC；Core 1 启动时在通用参数加载后读取页 94 并显示该值。

- [ ] **Step 6: 运行配置和 UI 测试**

Run: `gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_triple_jump_config.c project/code/triple_jump_config.c project/code/triple_jump.c -lm -o tests/test_triple_jump_config.exe; .\tests\test_triple_jump_config.exe; powershell -ExecutionPolicy Bypass -File project/code/triple_jump_ui_test.ps1`

Expected: 两项均 exit 0，UI 脚本打印 `triple jump UI checks passed`。

- [ ] **Step 7: 提交 Flash 与 UI**

```bash
git add project/code/triple_jump_config.* tests/test_triple_jump_config.c project/code/screen_display.c project/user/main_cm7_1.c project/iar/project_config/*.ewp project/code/triple_jump_ui_test.ps1
git commit -m "feat: add persistent Jump tuning screen"
```

### Task 6: 全量回归、构建检查与远端交付

**Files:**
- Modify only if verification exposes a defect in files already listed above.

**Interfaces:**
- Consumes: Tasks 1–5 的完整实现。
- Produces: 无告警主机测试、静态集成检查、可用的 IAR 工程配置和远端 `jump` 提交。

- [ ] **Step 1: 运行全部新增主机测试**

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_landing_detector.c project/code/landing_detector.c -o tests/test_landing_detector.exe
.\tests\test_landing_detector.exe
gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_triple_jump.c project/code/triple_jump.c -lm -o tests/test_triple_jump.exe
.\tests\test_triple_jump.exe
gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_jump_action_profile.c project/code/jump_action_profile.c -o tests/test_jump_action_profile.exe
.\tests\test_jump_action_profile.exe
gcc -std=c11 -Wall -Wextra -Werror -Iproject/code tests/test_triple_jump_config.c project/code/triple_jump_config.c project/code/triple_jump.c -lm -o tests/test_triple_jump_config.exe
.\tests\test_triple_jump_config.exe
```

- [ ] **Step 2: 运行仓库既有主机测试和 PowerShell 回归脚本**

运行 `tests/test_*.c` 对应的现有 gcc 命令，以及 `project/code/*_test.ps1`。所有命令必须 exit 0。

- [ ] **Step 3: 检查导航文件未修改和代码格式**

Run: `git diff origin/p3 -- project/code/navigation_action.c project/code/navigation_action.h; git diff --check`

Expected: 第一个命令无输出，第二个命令 exit 0。

- [ ] **Step 4: 尝试 IAR 构建**

若 `IarBuild.exe` 可用，分别构建 `project/iar/project_config/cyt4bb7_cm_7_0.ewp` 与 `cyt4bb7_cm_7_1.ewp` 的 Debug 配置；若工具链不存在，记录为环境限制，不用其他编译器假装完成固件构建。

- [ ] **Step 5: 最终提交并推送**

```bash
git add project/code/landing_detector.c project/code/landing_detector.h project/code/jump_action_types.h project/code/jump_action_profile.c project/code/jump_action_profile.h project/code/jump_control.c project/code/jump_control.h project/code/triple_jump.c project/code/triple_jump.h project/code/triple_jump_runtime.c project/code/triple_jump_runtime.h project/code/triple_jump_config.c project/code/triple_jump_config.h project/code/ipc_shared_data.c project/code/ipc_shared_data.h project/code/screen_display.c project/code/app_headfile.h project/code/status_registry.def project/code/vehicle_supervisor.c project/user/cm7_0_isr.c project/user/main_cm7_1.c project/iar/project_config/cyt4bb7_cm_7_0.ewp project/iar/project_config/cyt4bb7_cm_7_1.ewp tests/test_landing_detector.c tests/test_triple_jump.c tests/test_jump_action_profile.c tests/test_triple_jump_config.c project/code/triple_jump_integration_test.ps1 project/code/triple_jump_ui_test.ps1
git commit -m "test: verify standalone triple jump workflow"
git push origin jump
```

远端 `refs/heads/jump` 必须与本地 `HEAD` 哈希一致，工作区必须干净。
