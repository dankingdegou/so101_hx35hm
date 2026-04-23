# HX35HM + SO101 MoveIt 规划控制启动流程

适用对象：当前这套 `HX-35HM + SO101 follower`

目的：给出一套从启动、检查、规划、执行到关闭的完整 MoveIt 控制流程。

这份流程分成两种模式：

- **模式 A：HX35HM 真机规划控制**
- **模式 B：纯 MoveIt 演示 / mock 控制**

如果你现在要控制真实 HX35HM 机械臂，优先看模式 A。

---

## 1. 这套系统里 MoveIt 是怎么接控制的

MoveIt 这边真正执行规划结果时，走的是这两个 action：

- `[follower/arm_trajectory_controller/follow_joint_trajectory]`
- `[follower/gripper_controller/gripper_cmd]`

对应配置在：

- [moveit_controllers.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/moveit_controllers.yaml)

HX35HM 桥接节点会在 follower 命名空间里提供：

- `FollowJointTrajectory` action server
- `ParallelGripperCommand` action server

相关实现见：

- [bridge_node.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py)

所以你可以把它理解成：

1. `robot_state_publisher` 发布机器人模型
2. `hx35hm_bridge` 接收 MoveIt 下发的轨迹和夹爪命令
3. `move_group` 做规划
4. `rviz2` 负责可视化和交互

---

## 2. 真机模式推荐流程

### 2.1 先编译并 source

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select so101_bringup so101_hx35hm_bridge so101_moveit_config \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/ros2_ws/install/setup.bash
```

如果你只是改了 launch 或 yaml，通常也建议先重新 `source` 一次工作区。

如果你后面看到这类报错：

- `ModuleNotFoundError: No module named 'catkin_pkg'`
- `execute_process(/home/rog/.local/bin/python3.11 ...) returned error code 1`

通常不是源码有问题，而是 `build/` 里的 CMake 缓存还记着旧的 Python 解释器。上面这套命令已经把这个问题一并规避了。

---

## 3. 启动 HX35HM 真机 MoveIt 控制栈

推荐直接起这条总入口：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_cameras:=false \
  use_rviz:=true \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_red_detector:=false
```

这样会启动：

- `robot_state_publisher`
- `hx35hm_bridge`
- `move_group`
- `moveit_rviz`
- 关节状态回读和轨迹执行

说明：

- 真正控制 HX35HM 的，是 `so101_hx35hm_bridge`。
- 这个启动方式适合你现在的机械臂控制层调试。

---

## 4. 启动后要看到什么

正常日志里至少应出现：

- `Connecting Board on /dev/ros_robot_controller`
- `FollowJointTrajectory action server ready at 'arm_trajectory_controller/follow_joint_trajectory'`
- `ParallelGripperCommand action server ready at 'gripper_controller/gripper_cmd'`
- `move_group` 成功启动

如果这几项不全，先不要进 RViz 执行轨迹。

---

## 5. 启动后检查控制链路

另开一个终端：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 5.1 看节点

```bash
ros2 node list | rg 'follower|move_group|rviz|hx35hm'
```

你希望看到至少这些：

- `/follower/hx35hm_bridge`
- `/move_group`
- `/rviz2` 或 `teleop_rviz`

### 5.2 看 action

```bash
ros2 action list | rg 'follow_joint_trajectory|gripper_cmd'
```

应该看到：

- `/follower/arm_trajectory_controller/follow_joint_trajectory`
- `/follower/gripper_controller/gripper_cmd`

### 5.3 看关节状态

```bash
ros2 topic echo /follower/joint_states --once
```

确认：

- 有输出
- 关节名正常
- 当前位置不是全 0 或乱跳

---

## 6. RViz 里怎么规划和执行

RViz 会自动打开 `moveit.rviz` 配置。

你在 `MotionPlanning` 面板里通常这样做：

1. 选择 planning group `manipulator`
2. 拖动交互箭头，或者在状态列表里选 `rest`、`zero`、`extended`
3. 点 `Plan`
4. 确认轨迹合理后点 `Execute`

如果你想测试夹爪：

1. 切到 `gripper` group
2. 选 `open` 或 `closed`
3. 规划并执行

MoveIt 的语义状态定义在：

- [so101_arm.srdf](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/so101_arm.srdf)

其中你会经常用到：

- `rest`
- `zero`
- `extended`
- `open`
- `closed`

---

## 7. 建议的首次验证顺序

第一次起真机时，建议按这个顺序验证：

### 7.1 先验证静态姿态

在 RViz 里先让机器人回到：

- `rest`

确认实机姿态和 RViz 模型一致，没有明显方向错误。

### 7.2 再验证小幅运动

从 `rest` 开始做很小的规划偏移，比如：

- 只改一个关节
- 让末端稍微前伸或抬高

确认：

- 路径没有穿模
- 真机没有反向
- 末端实际运动方向和预期一致

### 7.3 最后再做大动作

确认小动作没问题后，再尝试：

- 左右平移
- 抬高 / 降低
- 末端旋转
- 夹爪开合

---

## 8. 控制层常见切换

### 8.1 只想规划，不想接相机

```bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_cameras:=false \
  use_rviz:=true \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_red_detector:=false
```

### 8.2 想带关节拖动调试

```bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_cameras:=false \
  use_rviz:=true \
  use_joint_gui:=true \
  use_aruco_detector:=false \
  use_red_detector:=false
```

### 8.3 只要纯 MoveIt 演示，不接 HX35HM

如果你只是想看 MoveIt 本体工作，不连 HX35HM 真机，可以用：

```bash
ros2 launch so101_bringup follower_moveit_demo.launch.py hardware_type:=mock
```

这个更偏“演示/验证”用途，不是你现在真实 HX35HM 控制的主入口。

---

## 9. 如果你想单独看 MoveIt 本体

MoveIt 本体相关文件是这些：

- [move_group.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/launch/move_group.launch.py)
- [moveit_rviz.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/launch/moveit_rviz.launch.py)
- [moveit_controllers.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/moveit_controllers.yaml)
- [joint_limits.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/joint_limits.yaml)
- [kinematics.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/kinematics.yaml)

如果你后面想调规划性能，优先看这几份。

---

## 10. 关闭顺序

关闭时直接按各终端：

```bash
Ctrl+C
```

推荐顺序：

1. 先关 RViz
2. 再关 MoveIt / bringup
3. 最后关桥接和硬件相关节点

如果你在做反复调试，关闭后再重新起一遍整套链路会最稳。

---

## 11. 你实际最该记住的一句

对 HX35HM 真机来说，MoveIt 规划控制的主入口就是：

```bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py
```

其余参数主要是为了让你更安全地调试和减少干扰。
