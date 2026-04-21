# SO-101 ROS Physical AI 完整使用说明（中文）

适用环境：**Ubuntu 24.04 + ROS 2 Jazzy**。本仓库提供 SO-101 机械臂 **leader/follower** 形态的完整 ROS2 栈：`ros2_control`（Feetech STS3215）、teleop、MoveIt2、多相机、episode 录制（rosbag2/MCAP）、rosbag→LeRobot 数据集转换、LeRobot 训练、策略推理（本机/远端）、Rerun 可视化。

> 如果你只想最快跑通：先看「## 4. 三条命令跑通主流程」。

---

## 1. 仓库与包职责速览

- **`so101_bringup`**：总控 launch + 配置聚合（两臂 bringup、相机、teleop、MoveIt demo、录制 session、推理 session、TF）。
- **`so101_description`**：URDF/Xacro + meshes；包含 `ros2_control` 相关 xacro（真实硬件/Mock）。
- **`feetech_ros2_driver`（子模块）**：Feetech STS3215 的 `ros2_control` HardwareInterface（真实硬件用）。
- **`so101_teleop`**：C++ teleop 节点，订阅 leader `/joint_states`，向 follower 控制器发布命令。
- **`episode_recorder`**：C++ 生命周期录包节点（默认 MCAP），配套键盘客户端控制 start/stop/discard。
- **`rosbag_to_lerobot`**：把 rosbag episode 转成 LeRobot v3 数据集（建议在 Pixi `lerobot` 环境跑）。
- **`so101_inference`**：推理节点（同步：本机加载 ACT/SmolVLA；异步：连远端 `policy_server`）。
- **`policy_server`**：远端 GPU 侧推理服务（ZMQ/gRPC），通常不在机器人上运行。

---

## 2. 前置条件

### 2.1 系统与 ROS

- Ubuntu 24.04
- ROS 2 Jazzy（已安装并可 `source /opt/ros/jazzy/setup.bash`）
- `rosdep`、`colcon` 可用

### 2.2 硬件（上真机时）

- 两只 SO-101 臂（leader + follower），USB 连接
- 可选：腕部相机 + 顶视相机（或 RealSense）

---

## 3. 安装与构建（一次性）

### 3.1 克隆（含子模块）

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/legalaspro/so101-ros-physical-ai.git
```

### 3.2 rosdep 初始化与依赖安装

> `rosdep init` 只需要做一次；若你已做过可跳过。

```bash
sudo rosdep init
rosdep update

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3.3 colcon 构建

```bash
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

如果你曾经在这台机器上用过别的 Python 环境，或者构建日志里出现旧路径比如 `/home/rog/.local/bin/python3.11`，建议额外加一次：

```bash
colcon build --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

这样可以把 CMake 缓存里残留的旧 Python 解释器引用清掉，避免 `catkin_pkg` 之类的假缺失报错。

---

## 4. 三条命令跑通主流程（推荐）

### 4.1 Teleop（leader → follower）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup teleop.launch.py
```

常用覆盖：

```bash
# 用 FollowJointTrajectory（轨迹控制器）替代 forward_controller
ros2 launch so101_bringup teleop.launch.py arm_controller:=trajectory_controller

# 关相机（没有相机时建议关掉）
ros2 launch so101_bringup teleop.launch.py use_cameras:=false use_camera_tf:=false

# 关 RViz（只要纯控制链路）
ros2 launch so101_bringup teleop.launch.py use_teleop_rviz:=false
```

### 4.2 录制 episode（rosbag2/MCAP）

终端 A（启动录制 session）：

```bash
ros2 launch so101_bringup recording_session.launch.py \
  experiment_name:=pick_and_place \
  task:="Pick up the cube and place it in the container." \
  use_rerun:=true
```

终端 B（键盘控制录制）：

```bash
ros2 run episode_recorder teleop_episode_keyboard
```

常用键位：

- `r`：开始录制
- `s`：停止并保存
- `d` / Backspace：丢弃当前 episode
- `q`：退出

默认输出目录：`~/.ros/so101_episodes/<experiment_name>/...`

### 4.3 rosbag → LeRobot 数据集

本仓库用 [Pixi](https://pixi.sh/) 提供隔离的 Python/LeRobot 环境（强烈建议用它跑转换与推理）。

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai

pixi run -e lerobot convert -- \
  --input-dir ~/.ros/so101_episodes/pick_and_place \
  --config ~/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml \
  --repo-id local/so101_test
```

---

## 5. 上真机前：硬件准备（必须）

请先完整阅读：[`docs/hardware.md`](docs/hardware.md)。

这里给出最关键的“必须项”摘要：

### 5.1 LeRobot 电机 setup + 校准（写 EEPROM）

在用 ROS 控制真实机械臂之前，必须先按 LeRobot SO-101 指南完成两只臂的：

- setup motors（写入电机 ID / 波特率）
- calibration（写入 offsets / limits 等持久参数）

> 没完成这一步不要从 ROS 发送控制命令。

### 5.2 udev：稳定设备名（推荐）

本仓库默认使用如下设备路径：

- `/dev/so101_leader`
- `/dev/so101_follower`
- `/dev/cam_wrist`
- `/dev/cam_overhead`

安装示例规则并替换其中的设备属性：

```bash
sudo cp docs/assets/99-so101.rules.example /etc/udev/rules.d/99-so101.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

验证：

```bash
ls -l /dev/so101_leader /dev/so101_follower
ls -l /dev/cam_wrist /dev/cam_overhead 2>/dev/null || true
```

### 5.3 权限组（串口/相机）

```bash
sudo usermod -aG dialout,video $USER
# 重新登录或重启生效
```

---

## 6. 相机（可选，但录制/推理常用）

### 6.1 入口配置

- 相机总配置：`so101_bringup/config/cameras/so101_cameras.yaml`
- 各驱动参数模板：
  - `so101_bringup/config/cameras/so101_gs_cam.yaml`（gscam）
  - `so101_bringup/config/cameras/so101_usb_cam.yaml`（usb_cam）
  - `so101_bringup/config/cameras/so101_libcam_cam.yaml`（camera_ros/libcamera）
  - `so101_bringup/config/cameras/so101_v4l2_cam.yaml`（v4l2_camera）

### 6.2 没有相机时怎么跑

你可以在 launch 时禁用相机：

```bash
ros2 launch so101_bringup teleop.launch.py use_cameras:=false use_camera_tf:=false
```

或者给录制 session 传入一个空相机配置（仓库自带）：

```bash
ros2 launch so101_bringup recording_session.launch.py \
  cameras_config_file:=/home/$USER/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_no_cameras.yaml
```

---

## 7. ros2_control 与控制器

### 7.1 follower 控制器（常用）

`so101_bringup/config/ros2_control/follower_controllers.yaml`：

- `joint_state_broadcaster`
- `forward_controller`（默认推荐）
- `trajectory_controller`（可选）

### 7.2 leader 控制器

`so101_bringup/config/ros2_control/leader_controllers.yaml`：

- `joint_state_broadcaster`（leader 通常只发布 state）

---

## 8. MoveIt2（follower）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_moveit_demo.launch.py
```

---

## 9. Rerun 可视化（推荐）

本仓库通过 `pixi.toml` 提供 Rerun bridge 与 viewer。

建议设置一次环境变量（指向仓库根目录）：

```bash
export SO101_RERUN_ENV_DIR=~/ros2_ws/src/so101-ros-physical-ai
```

### 9.1 Teleop + Rerun（不启 RViz）

```bash
ros2 launch so101_bringup teleop.launch.py use_rerun:=true use_teleop_rviz:=false
```

### 9.2 独立启动 viewer + bridge

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai
pixi run viewer
pixi run bridge
```

---

## 10. 推理（LeRobot：同步/异步）

> 推理与转换强烈建议用 Pixi `lerobot` 环境（见 `pixi.toml`）。

### 10.1 先启动 follower 硬件栈 + 相机

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup inference.launch.py
```

### 10.2 同步推理（本机加载策略）

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai

pixi run -e lerobot infer -- --ros-args \
  -p repo_id:="legalaspro/act_so101_pnp_crosslane_showcase_60_50hz_v0"
```

SmolVLA（相机观测键常见是 `camera1/camera2`）：

```bash
pixi run -e lerobot infer -- --ros-args \
  -p repo_id:="legalaspro/smolvla_so101_pnp_crosslane_showcase_60_50hz_v0" \
  -p policy_type:=smolvla \
  -p fps:=50.0 \
  -p camera_top_name:=camera1 \
  -p camera_wrist_name:=camera2
```

### 10.3 异步推理（远端 GPU：policy_server）

远端 GPU 机器上运行：

```bash
policy-server --transport=zmq --host=0.0.0.0 --port=8090 --fps=50
```

机器人侧运行 async 客户端：

```bash
cd ~/ros2_ws/src/so101-ros-physical-ai

pixi run -e lerobot async_infer -- --ros-args \
  -p repo_id:="legalaspro/smolvla_so101_pnp_crosslane_showcase_60_50hz_v0" \
  -p policy_type:=smolvla \
  -p server_address:=192.168.1.100:8090 \
  -p fps:=50.0 \
  -p actions_per_chunk:=50 \
  -p chunk_size_threshold:=0.6 \
  -p camera_top_name:=camera1 \
  -p camera_wrist_name:=camera2
```

更完整的参数解释请看：[`so101_inference/README.md`](so101_inference/README.md) 与 [`policy_server/README.md`](policy_server/README.md)。

---

## 11. 训练（LeRobot）

```bash
pixi shell -e lerobot

lerobot-train \
  --dataset.repo_id=<hf-username>/so101-pick-and-place \
  --policy.type=act \
  --output_dir=outputs/train/act_so101_pick_place \
  --job_name=act_so101_pick_place \
  --policy.device=cuda
```

---

## 12. 常见问题速查

- **设备路径不稳定 / 找不到 `/dev/so101_*`**：优先用 udev；按 `docs/hardware.md` 查询 `udevadm info` 并更新规则。
- **串口/相机权限问题**：确认 `dialout`、`video` 组已加入且重新登录生效。
- **相机节点报错**：先确认 `/dev/video*` 与 `video_device` 配置一致；没有相机就用 `use_cameras:=false`。
- **录制提示缺话题**：要么没有启动相机/控制器导致 topic 不存在，要么 topic 名与录制配置不匹配；检查 `so101_bringup/config/recording/episode_recorder_so101.yaml` 的 `topics`。
- **推理找不到 `lerobot`**：推理/转换/异步客户端都应在 Pixi `lerobot` 环境里运行。
