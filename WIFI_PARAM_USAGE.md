# WiFi 和参数使用注意事项

本文记录当前项目的 WiFi 使用方式、VOFA 调参协议、增加参数时需要修改的位置，以及参数系统的限制。

## WiFi 使用方式

### 基本配置位置

- WiFi 配置：`project/code/wifi.h`
- WiFi 实现：`project/code/wifi.c`
- Core 1 主循环：`project/user/main_cm7_1.c`

常用配置在 `wifi.h`：

```c
#define WIFI_PROTOCOL_MODE  1
#define WIFI_TARGET_PORT    "8086"
#define WIFI_LOCAL_PORT     "6666"
#define WIFI_SSID_TEST      "test207"
#define WIFI_PASSWORD_TEST  "12345678"
```

`WIFI_PROTOCOL_MODE`：

- `0`：TCP
- `1`：UDP，当前默认

### 当前默认模式

当前默认：

```c
wifi_mode_t current_wifi_mode = WIFI_MODE_SILENT;
```

含义：

- 默认不周期发送波形数据。
- 不影响 WiFi 接收 VOFA 指令。
- 不影响 `LOG_Printf()` 输出日志。
- 不影响参数读取、参数下发、参数保存。

注意：`WIFI_MODE_SILENT` 只关闭 `wifi_report_task()` 的周期数据，不代表 WiFi 完全静默。`LOG_Printf()` 在 WiFi 已连接时仍可能走 WiFi 发送。

### Core 1 主循环任务

```c
wifi_process_loop();
wifi_report_task();
wifi_health_check_task();
wifi_auto_reconnect_task();
IPC_Update_Wifi_Status_From_CoreB(wifi_is_connected);
```

说明：

- `wifi_process_loop()`：接收 WiFi 数据并交给 `VOFA_Protocol_Parse()`。
- `wifi_report_task()`：按当前 WiFi 模式发送波形、图像或日志。
- `wifi_health_check_task()`：当前不主动探测，避免误判 WiFi-SPI 正常忙状态为断线。
- `wifi_auto_reconnect_task()`：仅在 `wifi_is_connected == 0` 后尝试重连。
- `IPC_Update_Wifi_Status_From_CoreB()`：把 WiFi 状态同步给 Core 0，Core 0 据此决定是否允许电机输出。

### WiFi 重连注意事项

当前重连主要依赖发送失败：

```text
WiFi 发送失败累计达到阈值
    -> wifi_is_connected = 0
    -> wifi_auto_reconnect_task() 尝试重新初始化 WiFi
```

注意：

- 默认静默模式下没有周期发送，所以物理拔掉 WiFi 模块不一定会立刻被发现。
- 不建议在小车运行时热插拔 WiFi 模块。
- 如果 WiFi 模块异常，优先复位小车或复位 WiFi 模块。
- 不要简单用 `WIFI_SPI_INT_PIN` 低电平判断断线，该引脚在模块忙或正常通信时也可能为低，容易误判。

## VOFA / WiFi 指令

所有调参指令最终进入：

```c
VOFA_Protocol_Parse(uint8 *rx_buffer, uint32 data_length)
```

### 模式切换：`AA EE`

格式：

```text
AA EE mode
```

`mode`：

- `00`：`WIFI_MODE_SILENT`
- `01`：`WIFI_MODE_WAVE`
- `02`：`WIFI_MODE_IMAGE`
- `03`：`WIFI_MODE_LOG`

如果当前已经在 `WIFI_MODE_WAVE`，再次下发 `AA EE 01` 会切换 `wave_format`。

### 通道显示切换：`AA FF`

格式：

```text
AA FF channel
```

`channel` 范围是 `1` 到 `5`，对应 `channel_show[0]` 到 `channel_show[4]`。

### 单参数下发：`AA C2`

格式：

```text
AA C2 param_id float_little_endian
```

说明：

- `param_id` 从 `1` 开始，不是从 `0` 开始。
- `param_id = 1` 对应 `param_registry.def` 第一行。
- float 使用小端字节序。
- 当前 `AA C2` 单参数下发没有校验和。

代码转换关系：

```c
index = param_id - 1;
core_b_cmd.params[index] = value;
core_b_cmd.update_mask |= (1ULL << index);
core_b_cmd.param_update_flag = 1;
```

### 保存参数到 Flash：`AA C3`

格式：

```text
AA C3 88 55
```

保存前会检查电机输出：

```c
abs(core_a_status.left_pwm_duty) > 1000 ||
abs(core_a_status.right_pwm_duty) > 1000
```

如果电机还在明显输出，则拒绝保存。

### 批量下发所有参数：`AA C4`

格式：

```text
AA C4 [PARAM_COUNT 个 float] checksum
```

说明：

- checksum 是 float 数据区所有字节的 8 位累加和。
- 当前 `PARAM_COUNT = 63`。
- 当前 payload 长度是 `63 * 4 = 252` 字节。
- 当前整帧长度是 `252 + 3 = 255` 字节。

### 读取当前参数：`AA C5`

格式：

```text
AA C5 88 55
```

功能：

- 从 Core 0 拉取当前真实参数。
- 通过 WiFi 返回一帧 `AA C4` 格式的参数包。
- 同时通过日志打印当前参数和可复制的 HEX 参数帧。

## 增加参数时必须修改的位置

参数唯一主表是：

```text
project/code/param_registry.def
```

所有可 WiFi/VOFA 调节、可 IPC 同步、可 Flash 保存的参数，都必须进入这个文件。

### 推荐添加步骤

1. 在 `project/code/param.c` 第一段添加运行时变量。

```c
float new_param = 0.0f;
```

2. 在 `project/code/param.c` 第二段添加默认值。

```c
const float New_Param_init = 0.0f;
```

3. 在 `project/code/param.h` 声明运行时变量和默认值。

```c
extern float new_param;
extern const float New_Param_init;
```

4. 在 `project/code/param_registry.def` 最后追加一行。

```c
PARAM_ITEM(P_NEW_PARAM, new_param, New_Param_init, "New_Param")
```

重要：只能追加到最后，不要插入中间，不要重排旧参数。

5. 如果需要在屏幕显示，修改 `screen_display.c`。

6. 如果需要在 Flash 加载日志里显示，修改 `ipc_shared_data.c` 的 `FLASH LOAD RESULT` 打印部分。

7. 如果需要在 VOFA 上位机里调节，更新上位机的参数 ID 对照表。

### 参数 ID 规则

`param_registry.def` 的行号决定 VOFA 下发 ID：

```text
VOFA param_id = param_registry.def 中的顺序号，从 1 开始
```

例如当前：

- 第 1 行：`P_Q_YAW`，VOFA ID = `1`
- 第 6 行：`P_SPEED_P`，VOFA ID = `6`
- 第 42 行：`P_LEG_X_GAIN`，VOFA ID = `42`

不要在中间插入参数，否则旧的 VOFA ID、Flash 参数、HEX 帧都会错位。

## 当前参数系统限制

### 当前参数数量

```text
PARAM_COUNT = 63
```

### VOFA 回传缓冲限制

`vofa_protocol.c` 当前使用统一参数帧缓冲：

```c
#define VOFA_PARAM_FRAME_BUF_SIZE   (256)
#define VOFA_PARAM_MAX_COUNT        (63)
```

一帧参数长度：

```text
PARAM_COUNT * 4 + 3
```

当前 256 字节缓冲最多支持：

```text
floor((256 - 3) / 4) = 63 个参数
```

代码中已经加入 `VOFA_PARAM_MAX_COUNT (63)` 编译期保护。超过 63 个参数前，需要同时扩大参数帧缓冲并重做 `CoreB_Command_t.update_mask`。

### UART / WiFi 接收缓冲限制

`vofa_protocol.c`：

```c
static uint8 fifo_get_data[256];
```

`wifi.c`：

```c
static uint8 wifi_parse_buffer[512];
```

串口批量参数帧最多受 256 字节限制：

```text
PARAM_COUNT * 4 + 3 <= 256
PARAM_COUNT <= 63
```

### update_mask 限制

`CoreB_Command_t` 里使用：

```c
uint64_t update_mask;
```

所以最多只能可靠表示 64 个参数。当前代码按最多 63 个参数设计，因为 `param_id` 从 1 开始，且避免 `1ULL << 64` 这类风险。

如果 `PARAM_COUNT >= 64`，继续增加参数会有位移风险，必须改成多段 mask 或全量更新机制。

### Flash 保存格式

当前 Flash 保存格式：

```text
header: 0x55AA55AA
PARAM_COUNT 个 float
tail:   0x11223344
```

保存长度：

```c
PARAM_COUNT + 2
```

注意：

- `PARAM_DATA_LENGTH (19)` 已删除。
- 当前 Flash 没有参数版本号，也没有保存 `PARAM_COUNT`。

### 改参数表后的 Flash 风险

安全操作：

- 只在 `param_registry.def` 最后追加参数。
- 追加后第一次上电如果 Flash 校验失败，使用默认值是正常现象。
- 调好后重新保存 Flash。

危险操作：

- 不要在中间插入参数。
- 不要重排参数。
- 不要在参数数量不变的情况下替换某一行含义。

这些操作可能导致旧 Flash 校验仍然通过，但参数含义已经错位。

## 当前检查发现的问题

### 1. Flash 格式缺少版本号

当前只靠头尾判断 Flash 是否有效。

结论：短期可用，但长期建议加入参数版本号或参数数量字段。

### 2. 参数回传缓冲已扩展

当前 63 个参数，256 字节回传缓冲最多支持 63 个参数。

结论：当前已经达到 63 个参数上限；继续追加参数前必须重做缓冲和 `update_mask`。

## 建议后续优化顺序

1. 给 Flash 参数格式加入版本号和 `PARAM_COUNT`。
2. 当参数接近 64 个时，重做 `update_mask`。
3. 按功能继续清理旧调试字段和无效参数。
