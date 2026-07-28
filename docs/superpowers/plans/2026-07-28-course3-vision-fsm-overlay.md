# 科目三 Vision FSM 叠加显示 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在科目三桥/台阶流程中自动显示 Vision 画面，并用红色大字号显示真实 FSM 阶段，结束后恢复原页面。

**Architecture:** Core0 通过 `status_registry.def` 将显示状态写入共享 `CoreA_Status_t`；Core1 通过现有 IPC 拉取该字段，驱动 Vision 页面。相机画面保持不变，只叠加阶段文本。

**Tech Stack:** C11、共享 SRAM IPC、IPS200 RGB565 显示。

## Global Constraints

- 自动切换仅限 `VEHICLE_MODE_COURSE_3` 且状态非 `IDLE`。
- Core1 不直接读取 `action_fsm`。
- 不改变控制参数、视觉输出或 FSM 转换条件。
- 普通点不触发自动 Vision 页面。
- 文案为红色 `IPS200_16X16_FONT`。

### Task 1: 发布真实 FSM 显示状态

**Files:** `project/code/course3_display_state.c`, `project/code/course3_display_state.h`, `project/code/navigation_action.c`, `project/code/navigation_action.h`, `project/code/status_registry.def`, `tests/test_course3_display_state.c`.

**Interfaces:** `Course3DisplayState_t`; `uint8_t Course3DisplayState_FromActionState(ActionState_e state)`; `uint8_t Navi_Action_Get_Course3_Display_State(void)`.

- [ ] 写测试：断言 `FSM_COURSE3_TRACK_ALIGN/ACTION/DONE/FSM_IDLE` 分别映射为 `TRACK_ALIGN/ACTION/DONE/IDLE`。
- [ ] 运行 `gcc -std=c11 -Wall -Wextra -Iproject/code tests/test_course3_display_state.c -o tests/test_course3_display_state.exe`，确认 API 缺失失败。
- [ ] 实现纯映射模块、导航访问器，并在 `status_registry.def` 添加 `STATUS_ITEM(uint8, course3_display_state, Navi_Action_Get_Course3_Display_State())`。
- [ ] 运行测试，确认映射通过；提交 `feat: publish course3 display fsm state`。

### Task 2: 绘制红色大字号状态

**Files:** `project/code/camera_test_display.c`, `project/code/camera_test_display.h`, `tests/test_course3_display_state.c`.

**Interfaces:** `const char *CameraTestDisplay_Course3StateText(uint8_t state)` 返回 `TRACK ALIGN`、`ACTION`、`DONE` 或 `NULL`；`void CameraTestDisplay_DrawCourse3FsmOverlay(void)`。

- [ ] 扩展测试：断言四个显示状态对应的文本映射正确。
- [ ] 运行测试，确认显示文本 API 缺失失败。
- [ ] 实现映射；相机画面刷新后，红色黑底、`IPS200_16X16_FONT` 在 `(8,164)` 绘制文本，随后恢复白色黑底、`IPS200_8X16_FONT`。
- [ ] 运行纯映射测试；若主机缺嵌入式 SDK 头，仅记录限制而不将其视为功能失败。提交 `feat: overlay course3 fsm on vision page`。

### Task 3: 自动进入与恢复 Vision 页面

**Files:** `project/code/screen_display.c`, `project/code/course3_display_state.c`, `project/code/course3_display_state.h`, `tests/test_course3_display_state.c`.

**Interfaces:** `uint8_t Course3Vision_ShouldEnter(uint8_t vehicle_mode, uint8_t state, uint8_t already_active)`；`uint8_t Course3Vision_ShouldRestore(uint8_t state, uint8_t auto_vision_active)`。

- [ ] 写测试：仅 Course3 的非 IDLE 状态进入；IDLE 且自动显示激活时恢复。
- [ ] 运行测试，确认 helper 缺失失败。
- [ ] 实现 helper；在 `screen_display_process()` 拉取 IPC 后，首次进入时保存非 Vision 页面并切入 Vision，回到 IDLE 时恢复保存页面。
- [ ] 运行新映射测试、桥峰值、科目三对齐、视觉控制测试及 `git diff --check`。
- [ ] 提交 `feat: auto show vision during course3 actions`。

## Plan Self-Review

- Task 1 提供真实跨核状态，Task 2 保留画面并叠加红字，Task 3 实现科目三自动进入和恢复。
- 所有状态名和接口在三个任务中一致。
