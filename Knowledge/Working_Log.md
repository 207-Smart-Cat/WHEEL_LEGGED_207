# Working Log

## 2026-05-15 导航速度 PID 改造

### 修改目标

原循迹模式下，导航速度使用固定值 `DEFAULT_TRACKING_VELOCITY`。本次改为根据“当前位置到目标航点的距离”动态生成 `target_velocity`，使距离越远速度越高，接近目标点时自动减速，并通过限幅和每周期最大变化量限制加速度。

### 代码修改

- `project/code/navigation_tracking.c`
  - 新增 `navi_update_tracking_velocity(distance, reached, nav_valid)`。
  - 使用增量式 PID：输入为目标点距离，输出为 `target_velocity`。
  - 加入速度限幅：`0 ~ Navi_Speed_Max`。
  - 加入速度变化率限制：每 10 ms 最大变化 `Navi_Speed_MaxStep`。
  - 到点、导航信息无效、无航点、停止模式、模式切换、动作接管时重置速度 PID。
  - 保留动作接管逻辑：`is_action_busy == 1` 时导航不覆盖动作模块写入的速度。

- `project/code/navigation_tracking.h`
  - 更新注释：`USE_HOST_TARGET_VELOCITY == 0` 时使用导航距离 PID 生成速度。

- `project/code/param.c`
  - 新增导航速度 PID 运行时参数和默认值。

- `project/code/param.h`
  - 新增导航速度 PID 参数声明。

- `project/code/param_registry.def`
  - 在参数表末尾追加 5 个参数，保证旧参数 ID 不变。

### 新增 VOFA 参数

| ID | HEX | 参数名 | 默认值 | 作用 |
|---:|---:|---|---:|---|
| 59 | `0x3B` | `Navi_Speed_Kp` | `220` | 距离到速度的比例项 |
| 60 | `0x3C` | `Navi_Speed_Ki` | `0` | 积分项，默认关闭 |
| 61 | `0x3D` | `Navi_Speed_Kd` | `20` | 微分项，抑制速度突变 |
| 62 | `0x3E` | `Navi_Speed_Max` | `400` | 普通循迹目标速度上限 |
| 63 | `0x3F` | `Navi_Speed_MaxStep` | `12` | 每 10 ms 目标速度最大变化量 |

### 行为变化

旧逻辑：

```c
target_velocity = DEFAULT_TRACKING_VELOCITY;
```

新逻辑：

```text
distance
  -> 增量式 PID
  -> 速度限幅
  -> 每周期变化限幅
  -> target_velocity
```

### 注意事项

- 当前 `PARAM_COUNT = 63`，已经达到现有 IPC/VOFA 参数系统上限。继续新增参数前必须扩展参数缓冲和 `update_mask` 机制。
- `Navi_Speed_Ki` 默认保持 0，优先避免接近目标点时积分过冲。
- `Navi_Speed_MaxStep` 越小，加速越平滑，但响应越慢。
- 特殊动作点的速度仍由 `navigation_action.c` 接管，不受普通循迹速度 PID 控制。

## 2026-05-15 编码器角速度方向修正

### 修改内容

文件：`project/code/navigation_data_handling.c`

将编码器差速计算得到的 yaw 角速度加回负号：

```c
static float Navi_Get_YawRate_Enc(void)
{
    return -(filter_data.left_mps + filter_data.right_mps) / WHEEL_DISRANCE;
}
```

### 修改原因

实车验证发现，不加负号时编码器角速度和 IMU gyro 观测方向相反，EKF 融合后 `robot_pose.w` 会出现先正后负的异常。加回负号后，顺时针转动时 `robot_pose.w` 稳定为正，并与 `nav_yaw` 顺时针增加的导航坐标约定一致。
