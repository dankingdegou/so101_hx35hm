# HX35HM SO101 主从控制参数调整教程

本文档用于调整 HX35HM SO101 主臂控制从臂的手感参数。当前默认链路是：

```text
/leader/joint_states
  -> /follower_command_relay
  -> /follower/arm_trajectory_controller/follow_joint_trajectory
  -> /follower/hx35hm_bridge
  -> 从臂舵机
```

也就是说，日常主从跟随默认走 `FollowJointTrajectory`，不是直接连续写
`/follower/forward_controller/commands`。

## 1. 调参前准备

每次调参前先清理旧进程：

```bash
pkill -f 'teleop_hx35hm.launch.py|follower_command_relay|hx35hm_bridge|robot_state_publisher' || true
```

加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
```

启动主从控制：

```bash
ros2 launch so101_bringup teleop_hx35hm.launch.py \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false
```

启动日志里应看到：

```text
FollowJointTrajectory action server ready
FJT action: /follower/arm_trajectory_controller/follow_joint_trajectory
Relative teleop ready
```

## 2. 主要调参文件

### 2.1 Teleop 主从映射参数

文件：

```text
src/so101-ros-physical-ai/so101_teleop/config/teleop.yaml
```

这个文件决定主臂读数怎样变成从臂目标。

当前流畅优先基线：

```yaml
publish_rate_hz: 75.0
filter_mode: "lpf"
lpf_alpha: 0.55
output_deadband_rad: 0.0
gripper_output_deadband_rad: 0.0
output_keepalive_s: 0.0
trajectory_goal_duration_s: 0.04
trajectory_goal_points: 1
joint_scales: [1.0, 1.0, -1.05, 1.0, 1.0, 1.0]
joint_offsets: [0.062832, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
```

### 2.2 Follower bridge 执行参数

文件：

```text
src/so101-ros-physical-ai/so101_bringup/config/hx35hm_follower_bridge_params.yaml
```

这个文件决定从臂 bridge 如何执行 FJT 和备用 forward stream。

当前 FJT 基线：

```yaml
enable_follow_joint_trajectory: true
position_readback_rate_hz: 3.0
stream_continuous_follow: false
trajectory_command_rate_hz: 50.0
trajectory_min_command_interval_s: 0.02
trajectory_min_segment_duration_s: 0.02
trajectory_min_total_duration_s: 0.10
trajectory_final_settle_s: 0.01
```

## 3. 参数作用说明

### 3.1 `publish_rate_hz`

作用：`follower_command_relay` 生成目标的频率。

建议：

```text
75.0：当前流畅优先基线。
50.0：更稳一些，但实时性会下降。
40.0：排查抖动时可以临时使用，不建议作为最终手感。
```

如果提高到 100Hz 以上，可能会让 action goal 过于密集，反而不稳定。

### 3.2 `lpf_alpha`

作用：低通滤波强度。

```text
越大：越跟手，但更容易把主臂细小抖动传过去。
越小：越稳，但更肉、更慢。
```

建议范围：

```text
0.55：当前流畅优先基线。
0.45：轻微压抖。
0.35：明显更稳，但手感会变钝。
0.25：抗抖优先，不适合作为流畅手感基线。
```

### 3.3 `output_deadband_rad`

作用：输出死区。目标变化小于该值时不发送新 FJT 目标。

```text
0.0：最跟手，当前基线。
0.004：轻微压静止抖动。
0.008：明显压抖，但低速小动作会丢细节。
0.012：抗抖优先，容易有不跟手感。
```

调手感时不要一上来把死区加大。先确认抖动是否真的来自主臂读数噪声。

### 3.4 `trajectory_goal_duration_s`

作用：每个 FJT goal 的时间窗口。

```text
0.04：当前流畅优先基线，延迟低。
0.06：稍稳。
0.10：明显变稳，但会有窗口感。
0.12：抗抖版用过，手感偏肉。
```

如果感觉“拖动后从臂慢半拍”，优先检查这个参数。

### 3.5 `trajectory_goal_points`

作用：每个 FJT goal 内放几个轨迹点。

```text
1：当前基线，响应直接。
3：更像小轨迹段，但频繁 cancel/send 时可能变钝。
```

主从跟随建议先保持 `1`。

### 3.6 `joint_scales`

作用：每个关节的比例和方向。

当前：

```yaml
joint_scales: [1.0, 1.0, -1.05, 1.0, 1.0, 1.0]
```

说明：

```text
第 3 轴 elbow_flex 当前使用 -1.05，是为了适配当前实物方向和比例。
不要把这个参数当成滤波参数使用。
如果某个关节方向反了，改正负号。
如果某个关节幅度总是偏大或偏小，微调倍率。
```

### 3.7 `joint_offsets`

作用：每个关节的固定补偿。

当前：

```yaml
joint_offsets: [0.062832, 0.0, 0.0, 0.0, 0.0, 0.0]
```

说明：

```text
0.062832 rad 约等于 3.6 度，用于 shoulder_pan 当前偏差补偿。
如果某个关节一直有固定角度偏差，再调 offset。
如果只是运动过程不流畅，不要先调 offset。
```

## 4. 推荐调参顺序

### 4.1 当前基线：流畅优先

先用这组确认系统能跟手：

```yaml
publish_rate_hz: 75.0
lpf_alpha: 0.55
output_deadband_rad: 0.0
trajectory_goal_duration_s: 0.04
trajectory_goal_points: 1
```

### 4.2 如果静止时轻微抖动

第一步只改：

```yaml
output_deadband_rad: 0.004
gripper_output_deadband_rad: 0.008
```

不要同时改 `lpf_alpha` 和 `trajectory_goal_duration_s`。

### 4.3 如果还是抖

第二步再改：

```yaml
lpf_alpha: 0.45
```

### 4.4 如果仍然抖，但能接受一点延迟

第三步再改：

```yaml
trajectory_goal_duration_s: 0.06
```

### 4.5 如果感觉变肉或不跟手

按相反方向退：

```yaml
trajectory_goal_duration_s: 0.04
lpf_alpha: 0.55
output_deadband_rad: 0.0
```

## 5. 修改后编译

改完参数后执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_teleop so101_bringup --symlink-install
```

重新加载环境：

```bash
source /home/rog/ros2_ws/install/setup.bash
```

确认安装后的参数已经同步：

```bash
rg -n 'publish_rate_hz|lpf_alpha|output_deadband|trajectory_goal_duration|trajectory_goal_points' \
  install/so101_teleop/share/so101_teleop/config/teleop.yaml
```

## 6. 验证命令

查看节点：

```bash
ros2 node list
```

确认 FJT action：

```bash
ros2 action list | rg follow_joint_trajectory
```

查看 teleop 参数：

```bash
ros2 param get /follower_command_relay publish_rate_hz
ros2 param get /follower_command_relay lpf_alpha
ros2 param get /follower_command_relay output_deadband_rad
ros2 param get /follower_command_relay trajectory_goal_duration_s
```

查看主臂状态频率：

```bash
ros2 topic hz /leader/joint_states
```

## 7. 快速回退参数

如果调乱了，先恢复到流畅优先基线：

```yaml
publish_rate_hz: 75.0
lpf_alpha: 0.55
output_deadband_rad: 0.0
gripper_output_deadband_rad: 0.0
output_keepalive_s: 0.0
trajectory_goal_duration_s: 0.04
trajectory_goal_points: 1
```

然后重新编译：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_teleop so101_bringup --symlink-install
source /home/rog/ros2_ws/install/setup.bash
```

## 8. 调参记录模板

每次调参建议记录：

```text
日期：
参数：
  publish_rate_hz:
  lpf_alpha:
  output_deadband_rad:
  trajectory_goal_duration_s:
  trajectory_goal_points:
现象：
  静止抖动：
  慢速跟手：
  快速拖动：
  是否有段落感：
结论：
```

## 9. 注意事项

```text
不要一次改太多参数。
不要把 joint_offsets 当成流畅性参数。
不要把 joint_scales 当成滤波参数。
如果主臂本身读数是阶梯状，单靠从臂参数不能完全消除段落感。
如果从臂明显卡顿，先确认没有旧进程和重复节点。
```
