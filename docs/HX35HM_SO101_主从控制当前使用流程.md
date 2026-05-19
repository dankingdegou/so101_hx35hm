# HX35HM SO101 主从控制当前使用流程

本文档记录当前项目中 HX35HM SO101 主臂控制从臂的实际使用流程。它面向日常操作、测试和排查，不是理论方案。

当前日期版本：2026-05-17

## 1. 当前系统状态

当前主从控制链路：

```text
主臂 STM32 /dev/so101_leader
  -> /leader/hx35hm_bridge
  -> /leader/joint_states
  -> /follower_command_relay
  -> /follower/forward_controller/commands
  -> /follower/hx35hm_bridge
  -> 从臂 HX-35HM 舵机
```

当前模式：

```text
leader bridge: 只读，不接收控制命令，启动时卸力
teleop relay: relative mapping
follower bridge: 异步固定频率写入
```

当前已知问题：

```text
1. 主从实时性已经比早期版本明显改善。
2. 仍有机械顿挫感。
3. leader 侧舵机读数是 round_robin 轮询阶梯信号，这是全局顿挫的主要来源。
4. 三号关节 elbow_flex 仍有明显独立误差，需要专项诊断。
```

## 2. 硬件连接要求

### 2.1 串口绑定

确认串口软链接：

```bash
ls -l /dev/so101_leader /dev/so101_follower /dev/ttyACM*
```

预期：

```text
/dev/so101_leader   -> ttyACM*
/dev/so101_follower -> ttyACM*
```

查看当前绑定依据：

```bash
udevadm info --query=property --name=/dev/so101_leader | egrep 'ID_SERIAL_SHORT|DEVLINKS'
udevadm info --query=property --name=/dev/so101_follower | egrep 'ID_SERIAL_SHORT|DEVLINKS'
```

注意：

```text
不要依赖 ttyACM0/ttyACM1 的插拔顺序。
主从控制必须使用 /dev/so101_leader 和 /dev/so101_follower。
```

### 2.2 供电

从臂：

```text
需要独立 12V 供电。
```

主臂：

```text
当前方案需要主臂舵机可被手动拖动。
启动时 leader bridge 会发送卸力命令。
如果主臂仍然很紧，先执行紧急停止，再检查是否有旧进程或上力命令残留。
```

## 3. 启动前检查

### 3.1 清理旧进程

建议每次正式测试前先清理：

```bash
pkill -f 'teleop_hx35hm.launch.py|follower_command_relay|hx35hm_bridge|leader_hx35hm.launch.py|follower_hx35hm_moveit.launch.py|move_group|rviz2|robot_state_publisher' || true
```

检查：

```bash
ps -ef | rg 'teleop_hx35hm|leader_hx35hm|follower_hx35hm_moveit|hx35hm_bridge|follower_command_relay|move_group|rviz2|robot_state_publisher'
```

如果 `ros2 node list` 仍显示旧节点，但 `ps` 没有进程，重启 ROS daemon：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

### 3.2 检查串口

```bash
ls -l /dev/so101_leader /dev/so101_follower
```

如果软链接不存在，先不要启动主从控制，先修复 udev 规则或重新插拔板子。

## 4. 启动主从控制

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup teleop_hx35hm.launch.py \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false
```

启动日志中应看到：

```text
[leader.hx35hm_bridge]: Connecting Board on /dev/so101_leader, command input: <disabled/read-only>
[leader.hx35hm_bridge]: Disabling torque for servo IDs [1, 2, 3, 4, 5, 6] during startup
[leader.hx35hm_bridge]: Command subscription disabled; bridge is read-only
[follower.hx35hm_bridge]: Connecting Board on /dev/so101_follower, command input: forward_controller/commands
[follower.hx35hm_bridge]: Async stream writer enabled at 75.0 Hz
[follower.hx35hm_bridge]: Stream target smoothing enabled: max_velocity=2.500 rad/s
[follower_command_relay]: Mapping mode: relative
[follower_command_relay]: Relative teleop ready: follower targets follow leader deltas.
```

如果没有 `Relative teleop ready`，说明主臂或从臂 joint_states 没有正常发布。

## 5. 当前基线参数

### 5.1 Teleop relay

文件：

```text
src/so101-ros-physical-ai/so101_teleop/config/teleop.yaml
```

当前基线：

```yaml
mapping_mode: "relative"
publish_rate_hz: 75.0
filter_mode: "lpf"
lpf_alpha: 0.55
joint_scales: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
joint_offsets: [0.062832, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
```

说明：

```text
joint_offsets[0] = 0.062832 是第一关节 shoulder_pan 的自动标定补偿。
alpha-beta filter 已实测抖动明显，当前不推荐启用。
```

### 5.2 Leader bridge

文件：

```text
src/so101-ros-physical-ai/so101_bringup/config/hx35hm_leader_bridge_params.yaml
```

当前基线：

```yaml
enable_command_subscription: false
enable_position_readback: true
state_publish_rate_hz: 100.0
position_readback_rate_hz: 180.0
clamp_readback_to_joint_limits: false
disable_torque_on_startup: true
restore_torque_on_shutdown: false
maintain_torque_disabled: false
```

说明：

```text
240Hz readback 已触发串口 Input/output error，不建议使用。
180Hz 是当前较稳的上限附近参数。
```

### 5.3 Follower bridge

文件：

```text
src/so101-ros-physical-ai/so101_bringup/config/hx35hm_follower_bridge_params.yaml
```

当前基线：

```yaml
enable_position_readback: true
position_readback_rate_hz: 5.0
stream_command_duration: 0.03
stream_command_async_write: true
stream_write_rate_hz: 75.0
stream_target_smoothing: true
stream_max_velocity_rad_s: 2.5
suspend_readback_after_stream_command_s: 0.0
```

说明：

```text
从臂运行时主要服务写命令。
position_readback_rate_hz 降到 5Hz 是为了减少读写抢总线。
启动时仍会做 initial readback，relative teleop 可以捕获 follower baseline。
```

## 6. 启动后验证

### 6.1 节点

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 node list
```

预期：

```text
/leader/hx35hm_bridge
/follower/hx35hm_bridge
/follower_command_relay
/leader/robot_state_publisher
/follower/robot_state_publisher
```

如果出现重复 `robot_state_publisher`，先用 `ps` 判断是真进程还是 ROS daemon 缓存。

### 6.2 话题频率

```bash
ros2 topic hz /leader/joint_states
ros2 topic hz /follower/forward_controller/commands
ros2 topic hz /follower/joint_states
```

当前参考：

```text
/leader/joint_states ~= 100Hz
/follower/forward_controller/commands ~= 75Hz
/follower/joint_states 可能低于 50Hz，属于正常现象
```

### 6.3 参数确认

```bash
ros2 param get /follower_command_relay filter_mode
ros2 param get /follower_command_relay lpf_alpha
ros2 param get /follower_command_relay publish_rate_hz
ros2 param get /follower_command_relay joint_offsets

ros2 param get /leader/hx35hm_bridge position_readback_rate_hz
ros2 param get /leader/hx35hm_bridge state_publish_rate_hz

ros2 param get /follower/hx35hm_bridge stream_command_async_write
ros2 param get /follower/hx35hm_bridge stream_write_rate_hz
ros2 param get /follower/hx35hm_bridge stream_target_smoothing
ros2 param get /follower/hx35hm_bridge stream_max_velocity_rad_s
ros2 param get /follower/hx35hm_bridge stream_command_duration
```

## 7. 第一关节 offset 自动标定

脚本：

```text
src/so101_hx35hm_bridge/scripts/auto_trim_teleop_offset.py
```

适用场景：

```text
主从第一关节 shoulder_pan 存在稳定角度偏差。
```

先手动把主臂和从臂摆到你认为第一关节应对齐的位置。

Dry-run：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
cd /home/rog/ros2_ws

python3 src/so101_hx35hm_bridge/scripts/auto_trim_teleop_offset.py \
  --joint shoulder_pan \
  --mode match-leader \
  --samples 120 \
  --timeout 6
```

写入配置：

```bash
python3 src/so101_hx35hm_bridge/scripts/auto_trim_teleop_offset.py \
  --joint shoulder_pan \
  --mode match-leader \
  --samples 120 \
  --timeout 6 \
  --gain 0.6 \
  --apply
```

写入后编译：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rog/ros2_ws
colcon build --packages-select so101_teleop --symlink-install
```

然后重启主从控制。

## 8. 流畅度诊断

脚本：

```text
src/so101_hx35hm_bridge/scripts/diagnose_teleop_smoothness.py
```

运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
cd /home/rog/ros2_ws

python3 src/so101_hx35hm_bridge/scripts/diagnose_teleop_smoothness.py \
  --duration 15 \
  --output /tmp/so101_teleop_smoothness_report.json
```

采样期间需要实际拖动主臂，否则只能得到静态结果。

重点看：

```text
leader 每个关节的 median step / p95 step
command 每个关节的 median step / p95 step
command_vs_follower_latest_error
```

最近一次有效结论：

```text
leader 输入阶梯较大。
command 已经被平滑。
三号 elbow_flex 的 follower 误差异常大。
```

## 9. 三号关节当前处理建议

当前不要先改全局参数，应优先做三号专项诊断。

参考方案文档：

```text
docs/HX35HM_SO101_主从跟随顿挫与三号关节优化方案.md
```

目标：

```text
确认三号问题是 scale、offset、zero、clip、机械限位还是执行问题。
```

推荐下一步：

```text
新增并运行 diagnose_elbow_following.py
只动主臂三号关节做小范围采样
根据 raw servo3 position 与 command/follower joint_states 判断问题来源
```

## 10. 参数调整建议

### 10.1 如果跟随太慢

优先改：

```yaml
stream_max_velocity_rad_s: 3.0
```

不建议直接大幅提高 `publish_rate_hz`。

### 10.2 如果机械顿挫明显

可以尝试：

```yaml
stream_max_velocity_rad_s: 1.8
```

或舵机友好模式：

```yaml
publish_rate_hz: 40.0
stream_write_rate_hz: 40.0
stream_command_duration: 0.05
```

但这会增加延迟。

### 10.3 如果出现抖动

确认：

```yaml
filter_mode: "lpf"
```

不要启用：

```yaml
filter_mode: "alpha_beta"
```

alpha-beta 已实测会放大当前 leader 阶梯读数导致的抖动。

### 10.4 如果串口出现 Input/output error

立即降低 leader 回读频率：

```yaml
position_readback_rate_hz: 120.0
```

不要使用：

```yaml
position_readback_rate_hz: 240.0
```

## 11. 编译命令

修改 teleop 配置或代码后：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rog/ros2_ws
colcon build --packages-select so101_teleop --symlink-install
```

修改 bridge 或 bringup 配置后：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rog/ros2_ws
colcon build --packages-select so101_hx35hm_bridge so101_bringup --symlink-install
```

全部修改后：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/rog/ros2_ws
colcon build --packages-select so101_teleop so101_hx35hm_bridge so101_bringup --symlink-install
```

## 12. 停止与急停

主从控制异常时：

```bash
pkill -f 'teleop_hx35hm.launch.py|follower_command_relay|hx35hm_bridge|leader_hx35hm.launch.py|follower_hx35hm_moveit.launch.py|move_group|rviz2|robot_state_publisher' || true
```

确认进程：

```bash
ps -ef | rg 'teleop_hx35hm|leader_hx35hm|follower_hx35hm_moveit|hx35hm_bridge|follower_command_relay|move_group|rviz2|robot_state_publisher'
```

清理 ROS graph 缓存：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

## 13. 推荐日常流程

```text
1. 检查 /dev/so101_leader 和 /dev/so101_follower
2. 清理旧进程
3. 启动 teleop_hx35hm.launch.py
4. 确认 Relative teleop ready
5. 确认 leader 约 100Hz、command 约 75Hz
6. 手拖测试
7. 如果一号稳定偏差，运行 auto_trim_teleop_offset.py
8. 如果整体顿挫，运行 diagnose_teleop_smoothness.py
9. 如果三号异常，按三号专项方案处理
10. 修改后编译并重启
```

## 14. 当前优先级

当前不建议继续优先调全局流畅度。

优先级：

```text
P0: 保持主臂卸力和串口绑定稳定
P1: 三号 elbow_flex 专项诊断和修正
P2: 保留当前异步写入主从基线
P3: 评估舵机友好模式是否比高频模式更适合当前硬件
P4: 长期考虑 STM32 批量状态帧
```

