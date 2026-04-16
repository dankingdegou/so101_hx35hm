# SO101 + HX-35HM (STM32) ROS2 工作空间完整使用流程

适用环境：Ubuntu 24.04 + ROS 2 Jazzy。

本文基于你当前工作空间 `~/ros2_ws`，把现有内容串成一条从“装配前归中位 → 联调 → MoveIt → 录制/转换/推理”的可执行流程，并明确哪些环节是 **HX-35HM 分支**、哪些是 **原仓库(Feetech)分支**。

---

## 0. 你当前工作空间有什么

### 0.1 顶层（`~/ros2_ws`）

- `HX35HM_单臂集成使用说明.md`：HX-35HM 单臂 + MoveIt 的启动方式与注意事项
- `HX35HM_机械臂装配建议.md`：装配阶段如何把“机械零位”和“软件零位”对齐
- `HX35HM_舵机替换与集成方案.md`：为什么要加 bridge、系统链路怎么改
- `HX35HM_集成实现修改日志.md`：你做过哪些改动、踩过哪些坑
- `continuous_sweep.py`：裸控制板扫动测试（不走 ROS）
- `return_to_home.py`：裸控制板回中位（不走 ROS）
- `ros_robot_controller_设备命名与串口稳定性修改说明.md`：`/dev/ros_robot_controller` udev 规则与默认设备名

### 0.2 `~/ros2_ws/src` 关键包

- `so101-ros-physical-ai/`：原始 SO101 完整 ROS2 栈（teleop / MoveIt / 录制 / 推理 / 数据集转换）
- `ros_robot_controller-ros2/`：STM32 控制板 ROS2 Python SDK（`ros_robot_controller_sdk.Board`）
- `so101_hx35hm_bridge/`：你写的 HX-35HM 桥接节点（订阅命令，驱动舵机；发布 joint_states；可提供 FollowJointTrajectory action）

---

## 1. 装配前必做（HX-35HM）

### 1.1 供电与接线

- 控制板 USB 连主机，舵机电源单独供电（9.6–12.6V，电流留足余量）
- 控制板、舵机电源、主机必须共地

### 1.2 固定设备名 `/dev/ros_robot_controller`（强烈推荐）

工程自带 udev 规则文件：

- `~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/scripts/99-ros-robot-controller.rules`

安装（只需一次）：

```bash
cd ~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/scripts
sudo cp 99-ros-robot-controller.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ros_robot_controller
```

### 1.3 给 6 个舵机设置唯一 ID（建议 1~6）

脚本：

- `~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py`

建议每次只接一个舵机，避免广播误改。

---

## 2. 装配用“一键归中位”（你要的脚本）

脚本（可直接跑源码）：

- `~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py`

最常用：

```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py
```

常用参数：

```bash
# 指定设备与舵机列表
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py \
  --device /dev/ros_robot_controller --servo-ids 1 2 3 4 5 6

# 仅打印不下发
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py --dry-run
```

说明：HX-35HM 默认 `pos=500` 约等于 120°中位（0->0°，1000->240°）。

---

## 3. 构建与环境（一次性/每次开终端）

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 4. HX-35HM 运行模式 A：单臂 MoveIt（推荐用于装配后校准）

这一条链路 **不走 ros2_control**，直接用：

- `so101_hx35hm_bridge` 提供 `FollowJointTrajectory` action（给 MoveIt 执行）
- `so101_description` 发布 TF/URDF
- `so101_moveit_config` 提供 MoveIt 配置与 RViz

启动：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py use_sim_time:=false
```

装配校准建议：

1. 先用上面的“一键归中位”把所有舵机打到中位再装配（降低偏差）
2. RViz 里把关节拉到 `0 rad` 看机械姿态是否符合你的“零位定义”
3. 若某个关节方向相反或零位不对，优先在 bridge 里校准参数，而不是在上层硬凑

bridge 可调参数在：

- `~/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`

常用校准思路（通过 ROS 参数传入）：

- `joint_directions`：每关节 `1/-1`（和 `joint_names` 顺序对齐）
- `joint_zero_positions`：每关节 0 rad 对应的舵机 pos（和 `joint_names` 顺序对齐）
- `servo_zero_pos`：全局默认零位（默认 500）

---

## 5. HX-35HM 运行模式 B：leader→follower teleop（需要“混合 bringup”）

现状说明：

- `so101_bringup/launch/teleop.launch.py` 里已经加了 `use_hx35hm`，会额外启动 `hx35hm_bridge`
- 但 `teleop.launch.py` 只有一个 `hardware_type` 参数，同时影响 leader 和 follower 的 `ros2_control` bringup
- 所以如果你的 leader 还是原 Feetech 硬件、follower 改成 HX-35HM，建议按下面方式“拆开启动”

推荐启动顺序（混合：leader 真机 + follower mock + HX bridge 实际驱动）：

1) 启动 leader（真实硬件）

```bash
ros2 launch so101_bringup leader.launch.py \
  hardware_type:=real usb_port:=/dev/so101_leader use_rviz:=false
```

2) 启动 follower（mock，只提供控制器与话题接口）

```bash
ros2 launch so101_bringup follower.launch.py \
  hardware_type:=mock arm_controller:=forward_controller use_rviz:=false
```

3) 启动 HX-35HM bridge（在 follower 命名空间下发布 joint_states、订阅 commands）

```bash
ros2 run so101_hx35hm_bridge hx35hm_bridge --ros-args \
  -r __ns:=/follower \
  -p device:=/dev/ros_robot_controller \
  -p publish_joint_states_topic:=joint_states \
  -p command_topic:=forward_controller/commands
```

4) 启动 teleop 节点（只启动 teleop 本体）

```bash
ros2 launch so101_teleop teleop.launch.py \
  leader_namespace:=leader follower_namespace:=follower arm_controller:=forward_controller
```

---

## 6. 原仓库标准主流程（Feetech）：teleop → 录制 → 转数据集 → 推理

如果你仍然在用原 Feetech driver（或仿真/Mock），按 `so101-ros-physical-ai/USAGE_zh.md` 走即可：

### 6.1 Teleop

```bash
ros2 launch so101_bringup teleop.launch.py
```

### 6.2 录制 episode

终端 A：

```bash
ros2 launch so101_bringup recording_session.launch.py \
  experiment_name:=pick_and_place \
  task:="Pick up the cube and place it in the container." \
  use_rerun:=true
```

终端 B（键盘控制）：

```bash
ros2 run episode_recorder teleop_episode_keyboard
```

默认输出：`~/.ros/so101_episodes/<experiment_name>/...`

### 6.3 rosbag → LeRobot 数据集

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai
pixi run -e lerobot convert -- \
  --input-dir ~/.ros/so101_episodes/pick_and_place \
  --config ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id local/so101_test
```

### 6.4 推理（同步/异步）

参考：

- `~/ros2_ws/src/so101-ros-physical-ai/so101_inference/README.md`
- `~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/inference.launch.py`

---

## 7. 你当前 HX-35HM 分支的“缺口”（我建议你下一步补齐的）

目前你已经有：

- HX-35HM 的 bridge（命令 + joint_states + FollowJointTrajectory）
- 单臂 MoveIt launch：`so101_bringup/launch/follower_hx35hm_moveit.launch.py`

但还缺一个“HX-35HM 录制/推理 session 一键启动”的聚合 launch（类似 `recording_session.launch.py` / `inference.launch.py`），让它们不依赖 `follower.launch.py`（ros2_control）。

如果你希望我继续做，我可以新增两个 launch：

1. `follower_hx35hm_recording_session.launch.py`：HX bridge + cameras + episode_recorder
2. `follower_hx35hm_inference.launch.py`：HX bridge + cameras + pixi infer/async_infer

这样你就可以真正做到“HX-35HM 硬件上跑全流程”。

