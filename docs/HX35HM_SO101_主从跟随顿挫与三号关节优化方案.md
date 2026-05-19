# HX35HM SO101 主从跟随顿挫与三号关节优化方案

本文档用于审阅后再执行修改。当前不建议继续盲目调 `lpf_alpha`、`publish_rate_hz` 或 `stream_command_duration`，应先按数据把问题分成两类处理：

1. 三号关节 `elbow_flex` 的独立执行/映射问题。
2. leader 侧轮询回读导致的全局阶梯输入问题。

## 0. 当前结论

已验证的现象：

- 从臂异步固定频率写入后，实时性明显改善。
- alpha-beta 预测滤波在当前轮询阶梯输入下会放大抖动，不适合直接使用。
- LPF 能抑制抖动，但不能根治顿挫。
- follower command 已被平滑成小步，但 leader 输入本身每次跳变较大。
- 三号关节 `elbow_flex` 的 command 与 follower 实际反馈误差远大于其他关节。

最近一次诊断数据摘要：

```text
leader shoulder_pan  median step ~= 2.16 deg
leader shoulder_lift median step ~= 2.88 deg
leader elbow_flex    median step ~= 2.16 deg

command shoulder_pan  median step ~= 0.24 deg
command shoulder_lift median step ~= 0.29 deg
command elbow_flex    median step ~= 0.28 deg

command_vs_follower elbow_flex median_abs_error ~= 13.1 deg
command_vs_follower elbow_flex p95_abs_error    ~= 63.3 deg
```

判断：

```text
全局顿挫主要来自 leader 侧回读阶梯。
三号关节另有执行/映射/限位问题，需要单独处理。
```

## 1. 当前不建议继续做的事

不要继续无依据地做以下操作：

```text
继续提高 leader position_readback_rate_hz
继续提高 teleop publish_rate_hz
继续增大 alpha-beta 预测
继续对所有关节统一加 smoothing
直接改三号关节 joint_zero_positions
直接改三号关节限位
```

原因：

- `240Hz` leader 回读已触发大量串口 `Input/output error`，说明上限已接近。
- alpha-beta 已实测抖动明显。
- 全局 smoothing 会降低所有关节响应，但不能解决三号关节的大误差。
- 三号关节如果是 scale/zero/clip 问题，盲改会让其他姿态更差。

## 2. 执行前固定基线

先确认当前运行基线。

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 node list
ros2 param get /follower_command_relay filter_mode
ros2 param get /follower_command_relay lpf_alpha
ros2 param get /follower_command_relay publish_rate_hz
ros2 param get /follower/hx35hm_bridge stream_command_async_write
ros2 param get /follower/hx35hm_bridge stream_target_smoothing
ros2 param get /follower/hx35hm_bridge stream_max_velocity_rad_s
ros2 param get /leader/hx35hm_bridge position_readback_rate_hz
ros2 param get /leader/hx35hm_bridge state_publish_rate_hz
```

当前建议基线：

```text
filter_mode = lpf
lpf_alpha = 0.55
publish_rate_hz = 75.0
leader position_readback_rate_hz = 180.0
leader state_publish_rate_hz = 100.0
follower stream_command_async_write = true
follower stream_target_smoothing = true
follower stream_max_velocity_rad_s = 2.5
```

## 3. 第一阶段：三号关节专项诊断

目标：确认三号关节问题到底来自哪里。

需要采集：

```text
leader elbow_flex
relay command elbow_flex
follower elbow_flex
follower servo 3 raw position
是否发生 servo_pos clamp
```

### 3.1 新增三号专项采样脚本

建议新增脚本：

```text
src/so101_hx35hm_bridge/scripts/diagnose_elbow_following.py
```

功能：

- 订阅 `/leader/joint_states`
- 订阅 `/follower/forward_controller/commands`
- 订阅 `/follower/joint_states`
- 直接读取 `/dev/so101_follower` 上的 servo `id=3` raw position
- 输出 CSV 与 summary

输出文件建议：

```text
/tmp/so101_elbow_following.csv
/tmp/so101_elbow_following_summary.json
```

CSV 字段：

```text
t
leader_elbow_rad
command_elbow_rad
follower_elbow_rad
follower_servo3_pos
command_to_servo3_expected_pos
raw_pos_error
rad_error_command_minus_follower
```

### 3.2 三号诊断动作

运行主从控制后，做小范围动作：

```text
只动主臂三号关节
先慢速正向移动
再慢速反向移动
幅度不要大
保持其他关节尽量不动
```

诊断命令示例：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

python3 /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/diagnose_elbow_following.py \
  --duration 15 \
  --output-csv /tmp/so101_elbow_following.csv \
  --output-json /tmp/so101_elbow_following_summary.json
```

### 3.3 判定规则

#### 情况 A：command 正常变化，但 follower raw pos 不动

判断：

```text
从臂 servo 3 没执行命令
可能是总线通信、舵机保护、供电、机械卡滞或命令被限幅到边界
```

处理：

- 单独控制从臂 servo 3 小幅运动验证。
- 检查是否频繁 `Clipped joint 'elbow_flex' command`。
- 检查 `joint_zero_positions[2]` 是否导致命令长期接近 `servo_pos_min/max`。

#### 情况 B：raw pos 动了，但 follower joint_states 不匹配

判断：

```text
读回映射错误
```

处理：

- 检查 follower 的 `joint_directions[2]`。
- 检查 follower 的 `joint_zero_positions[2]`。
- 使用 raw pos 反算 rad，确认读回公式是否正确。

#### 情况 C：command 本身已经超出 follower 可执行范围

判断：

```text
relative mapping 对三号不适用
```

处理：

- 对三号单独设置 `joint_scales[2] < 1.0`。
- 或对三号设置独立 offset。
- 不建议先改硬件 zero。

#### 情况 D：只有三号在接近边界时异常

判断：

```text
机械姿态/rest 基准或三号零点导致可用空间太小
```

处理：

- 重新标定三号 follower zero。
- 或在 teleop 中限制三号 scale。

## 4. 第二阶段：三号关节修正

根据诊断结果选择修正方式。

### 4.1 优先修 teleop scale/offset

如果三号跟随方向正确，但幅度过大或很快顶边界，优先改：

```text
src/so101-ros-physical-ai/so101_teleop/config/teleop.yaml
```

示例：

```yaml
joint_scales: [1.0, 1.0, 0.75, 1.0, 1.0, 1.0]
joint_offsets: [0.062832, 0.0, 0.0, 0.0, 0.0, 0.0]
```

验证：

```bash
cd /home/rog/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_teleop --symlink-install
source install/setup.bash
```

重启主从后再次跑诊断。

### 4.2 只有确认读回/写入映射错误时，才改 follower zero

文件：

```text
src/so101-ros-physical-ai/so101_bringup/config/hx35hm_follower_bridge_params.yaml
```

三号对应：

```yaml
joint_zero_positions:
- 498.0
- 499.19
- -249.81   # elbow_flex
- 497.05
- 500.0
- 500.0
```

注意：

```text
改 joint_zero_positions 会影响从臂单独 MoveIt 控制。
只有确认硬件/软件角度映射错误时才改。
```

## 5. 第三阶段：处理全局顿挫

三号关节处理完后，再处理全局顿挫。

当前已知：

```text
leader 输入是阶梯信号
relay/follower 已经能把 command 平滑成小步
继续提高频率不可行
```

### 5.1 短期方案：舵机友好模式

目标：

```text
降低目标刷新频率
让舵机有足够时间完成每段小运动
减少内部反复重规划
```

建议参数实验：

```yaml
publish_rate_hz: 40.0
lpf_alpha: 0.65
```

follower：

```yaml
stream_write_rate_hz: 40.0
stream_command_duration: 0.05
stream_target_smoothing: true
stream_max_velocity_rad_s: 3.0
```

预期：

```text
响应略慢
机械动作可能更顺
```

### 5.2 中期方案：leader 发布端插值

在 `hx35hm_bridge` leader 模式中记录每个关节：

```text
last_pos
previous_pos
last_read_time
previous_read_time
estimated_velocity
```

发布 `/leader/joint_states` 时，不直接发布最后一次读数，而是使用：

```text
estimated_pos = last_pos + velocity * prediction_dt
```

限制：

```text
prediction_dt 不超过 0.02~0.03s
速度估计必须限幅
如果长时间未更新，则退回 last_pos
```

风险：

```text
可能出现轻微超前
需要按关节调速度上限
```

### 5.3 长期方案：STM32 批量状态帧

根本方案：

```text
STM32 固定周期读取 6 个舵机
STM32 组装完整状态帧
PC 一次读取完整 6 关节状态
```

目标：

```text
同步状态帧
减少 PC 到 STM32 的逐舵机请求
减少串口事务数量
减少关节间相位差
```

这是接近官方 leader 手感的方向。

## 6. 建议执行顺序

推荐顺序：

```text
1. 保留当前可运行主从基线
2. 新增三号专项诊断脚本
3. 只动三号关节采样 15 秒
4. 判断三号是 scale/zero/clip/执行问题
5. 优先改 teleop joint_scales[2] 或 joint_offsets[2]
6. 重启主从并复测三号
7. 三号稳定后，再测试舵机友好模式
8. 如果仍不满意，再设计 leader 发布端插值或 STM32 批量状态帧
```

## 7. 回退点

### 7.1 回退 teleop 参数

当前可回退基线：

```yaml
publish_rate_hz: 75.0
filter_mode: "lpf"
lpf_alpha: 0.55
joint_scales: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
joint_offsets: [0.062832, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
```

### 7.2 回退 follower stream 参数

当前可回退基线：

```yaml
position_readback_rate_hz: 5.0
stream_command_duration: 0.03
stream_command_async_write: true
stream_write_rate_hz: 75.0
stream_target_smoothing: true
stream_max_velocity_rad_s: 2.5
suspend_readback_after_stream_command_s: 0.0
```

### 7.3 紧急停止

```bash
pkill -f 'teleop_hx35hm.launch.py|follower_command_relay|hx35hm_bridge|leader_hx35hm.launch.py|follower_hx35hm_moveit.launch.py|move_group|rviz2' || true
```

如果 ROS graph 仍显示残留：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

