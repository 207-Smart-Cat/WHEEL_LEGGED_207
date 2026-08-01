# AssiBurst / Anti-Stall Assist Logic

本文档记录当前代码中的防卡滞助推逻辑。当前实现本质是 `anti_stall` 积分 PWM assist，不是单轮独立 burst；它会把同一个 `assist_pwm` 同向叠加到左右轮。

## 1. 代码位置

- 主逻辑文件：`project/code/control.c`
- 对外调试变量声明：`project/code/control.h`
- 状态同步注册：`project/code/status_registry.def`
- 屏幕菜单与 WiFi 波形选择：`project/code/screen_display.c`
- WiFi 波形变量定义与取值：`project/code/wifi.h`、`project/code/wifi.c`
- 运行时开关枚举：`project/code/runtime_status.h`

核心函数：

```c
static float anti_stall_update(uint8 enabled,
                               float target_velocity_cmd,
                               float measured_velocity);

static void Motor_Output_Apply(float gyro_pwm,
                               float assist_pwm,
                               float turn_pwm,
                               uint8 turn_enabled);
```

## 2. 开关逻辑

防卡滞助推由运行时模块开关控制：

```c
RUNTIME_MODULE_ANTI_STALL
```

默认状态为关闭。默认模块掩码 `RUNTIME_DEFAULT_MODULE_MASK` 未包含 `RUNTIME_MODULE_ANTI_STALL`，所以必须在屏幕模块菜单或通信命令中手动开启后才会输出助推 PWM。

屏幕模块菜单中显示为：

```c
"Anti Stall Assist"
```

短标签显示为：

```c
"Assist"
```

## 3. 速度来源与单位

当前助推使用的速度不是 m/s，而是电机反馈的原始速度量级。

在 `balance_control()` 中：

```c
Encoder_Left = -motor_value.receive_left_speed_data;
Encoder_Right = -motor_value.receive_right_speed_data;
now_velocity = (Encoder_Left - Encoder_Right) / 2.0f;
```

随后助推使用：

```c
assist_pwm = anti_stall_update(
    Runtime_Is_Module_Enabled(RUNTIME_MODULE_ANTI_STALL),
    target_velocity,
    now_velocity
);
```

因此速度误差为：

```c
speed_error = target_velocity - now_velocity;
```

注意：这里的 `target_velocity` 与 `now_velocity` 是控制环内部速度量，不是导航层转换后的 m/s。导航层另有 `RPM_TO_M_COEFF` 将轮速换算为 m/s，但当前 assist 没有使用该换算。

## 4. 积分公式

当前积分没有显式乘以 `dt`。每次 `balance_control()` 调用都会在满足条件时累加一次速度误差：

```c
g_anti_stall_assist.integral += speed_error;
g_anti_stall_assist.integral = constrain_float(
    g_anti_stall_assist.integral,
    0.0f,
    ANTI_STALL_INTEGRAL_LIMIT
);

g_anti_stall_assist.assist_pwm = constrain_float(
    ANTI_STALL_PWM_GAIN * g_anti_stall_assist.integral,
    0.0f,
    ANTI_STALL_PWM_LIMIT
);
```

当前参数：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `ANTI_STALL_TARGET_MIN` | `40.0f` | 目标前进速度必须大于该值才允许助推 |
| `ANTI_STALL_ERROR_START` | `60.0f` | 速度误差小于该值时不累积 |
| `ANTI_STALL_RECOVER_ERROR` | `20.0f` | 误差回到该范围内时清零 |
| `ANTI_STALL_RECOVER_RATIO` | `0.85f` | 实际速度达到目标速度 85% 时清零 |
| `ANTI_STALL_INTEGRAL_LIMIT` | `50000.0f` | 积分状态限幅 |
| `ANTI_STALL_PWM_GAIN` | `0.04f` | 积分到 PWM 的比例 |
| `ANTI_STALL_PWM_LIMIT` | `6000.0f` | assist PWM 输出限幅 |

等价公式：

```text
error[k] = target_velocity - now_velocity

integral[k] = clamp(integral[k-1] + error[k], 0, 50000)

assist_pwm[k] = clamp(0.04 * integral[k], 0, 6000)
```

因为没有乘以 `dt`，积分速度取决于 `balance_control()` 的实际调用周期。若 `error = 400`，则每周期 PWM 增量约为：

```text
delta_pwm = 0.04 * 400 = 16 PWM / control cycle
```

## 5. 触发条件

只有以下条件全部满足时，assist 才会累积和输出：

- `RUNTIME_MODULE_ANTI_STALL` 已开启。
- `target_velocity > 40.0f`，即只针对前进目标速度。
- 电机模块 `RUNTIME_MODULE_MOTOR` 已开启。
- 当前没有跳跃接管：`!jump_is_active()`。
- 当前没有导航舵机接管：`!Navi_Action_Servo_Takeover_Active()`。
- `now_velocity < target_velocity * 0.85f`。
- `target_velocity - now_velocity > 60.0f`。

当前逻辑不对后退目标速度输出 assist。

## 6. 清零条件

清零函数：

```c
static void anti_stall_reset(uint8 reason)
```

会清零：

```c
g_anti_stall_assist.integral = 0.0f;
g_anti_stall_assist.assist_pwm = 0.0f;
anti_stall_dbg_integral = 0.0f;
anti_stall_dbg_pwm = 0.0f;
anti_stall_dbg_clear_reason = (float)reason;
```

当前清零原因码：

| 原因码 | 名称 | 触发条件 |
|---:|---|---|
| `0` | `ANTI_STALL_CLEAR_NONE` | 正在输出或保留 assist |
| `1` | `ANTI_STALL_CLEAR_DISABLED` | assist 开关关闭 |
| `2` | `ANTI_STALL_CLEAR_NO_TARGET` | 目标速度不大于 `40.0f` |
| `3` | `ANTI_STALL_CLEAR_RECOVERED` | 轮速恢复或误差过小 |
| `4` | `ANTI_STALL_CLEAR_SAFETY` | 电机关闭、跳跃、舵机接管、急停/WiFi 断开等安全路径 |

在 `balance_control()` 中，急停、WiFi 断开、跳跃暂停都会走 `Motor_Output_Stop()`，其内部也会调用：

```c
anti_stall_reset(ANTI_STALL_CLEAR_SAFETY);
```

## 7. PWM 混合方式

当前 `assist_pwm` 是左右轮对称同向叠加，不参与转向差速。

```c
if (turn_enabled)
{
    logical_left = (int)(gyro_pwm + assist_pwm + turn_pwm);
    logical_right = (int)(gyro_pwm + assist_pwm - turn_pwm);
}
else
{
    logical_left = (int)(gyro_pwm + assist_pwm);
    logical_right = (int)(gyro_pwm + assist_pwm);
}
```

含义：

```text
左轮逻辑 PWM = Gyro_Pwm + Assist_Pwm + Turn_Pwm
右轮逻辑 PWM = Gyro_Pwm + Assist_Pwm - Turn_Pwm
```

所以当前 assist 会同时增加左右轮的前进驱动量。它不会判断哪一个轮子被障碍物卡住，也不会只给单侧轮子输出 burst。

## 8. 总 PWM 限幅

assist 单项限幅为：

```c
ANTI_STALL_PWM_LIMIT = 6000.0f
```

最终电机输出还会经过总限幅：

```c
limited_left = MOTOR_LEFT_LIMIT_SIGN * cuu(logical_left);
limited_right = MOTOR_RIGHT_LIMIT_SIGN * cuu(logical_right);
```

`cuu()` 使用：

```c
MAX_DUTY * (PWM_DUTY_MAX / 100)
```

当前：

```c
MAX_DUTY = 45
PWM_DUTY_MAX = 10000
```

所以总输出限幅为：

```text
45 * (10000 / 100) = 4500 PWM
```

驱动层 `small_driver_set_duty()` 里还有一次 `MOTOR_DUTY_MAX_ABS` 限幅，值同样是 4500。

## 9. WiFi / 屏幕观测

状态注册在 `status_registry.def`：

```c
STATUS_ITEM(float, anti_stall_enabled, anti_stall_dbg_enabled)
STATUS_ITEM(float, anti_stall_integral, anti_stall_dbg_integral)
STATUS_ITEM(float, anti_stall_pwm, anti_stall_dbg_pwm)
STATUS_ITEM(float, anti_stall_clear_reason, anti_stall_dbg_clear_reason)
```

WiFi 波形变量名：

```c
"AstEn", "AstInt", "Assist", "AstClr"
```

取值路径与 `L_PWM`、`R_PWM` 相同，都是通过 `wifi_wave_get_value()` 从 `core_a_status` 读取：

```c
case WIFI_WAVE_VAR_LEFT_PWM:
    return (float)core_a_status.left_pwm_duty;

case WIFI_WAVE_VAR_RIGHT_PWM:
    return (float)core_a_status.right_pwm_duty;

case WIFI_WAVE_VAR_ASSIST_PWM:
    return core_a_status.anti_stall_pwm;
```

屏幕 WiFi 波形的 `motor` 分组当前包含：

```c
static const wifi_wave_var_t k_wave_group_motor[] = {
    WIFI_WAVE_VAR_LEFT_SPEED,
    WIFI_WAVE_VAR_RIGHT_SPEED,
    WIFI_WAVE_VAR_LEFT_PWM,
    WIFI_WAVE_VAR_RIGHT_PWM,
    WIFI_WAVE_VAR_ASSIST_PWM,
    WIFI_WAVE_VAR_BATTERY
};
```

因此在屏幕 `WIFI -> wave -> motor` 页面选择 `Assist` 后，WiFi 回传中会包含当前 assist PWM 输出值。

## 10. 当前局限

当前实现只输出一个标量 `assist_pwm`：

```c
typedef struct
{
    float integral;
    float assist_pwm;
    uint8 clear_reason;
} anti_stall_assist_state_t;
```

这意味着：

- 不能区分左轮或右轮被绊住。
- 不能只给单侧轮子 burst。
- 单侧卡住时，左右轮都会同时加同样的 assist PWM。
- 若要实现“判断哪一个轮子被绊住，然后只给对应轮子加入 burst”，需要把状态改成左右独立，例如 `left_integral/right_integral`、`left_assist_pwm/right_assist_pwm`，并把 `Motor_Output_Apply()` 改为接收左右两个 assist 输入。
