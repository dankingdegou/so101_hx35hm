## HX‑35HM 单臂集成使用说明（ROS2 + MoveIt）

本文基于当前工作区 `~/ros2_ws`，说明如何使用 HX‑35HM 舵机 + STM32 控制板与 SO101 单臂工程集成，并通过 MoveIt 在 RViz 中进行规划和真实执行。

---

### 1. 硬件与串口准备

#### 1.1 硬件连接

- **控制板 ↔ 主机**
  - 控制板通过 USB 接到主机，系统识别为 `ttyACM*`。
- **控制板 ↔ HX‑35HM 舵机**
  - HX‑35HM 接线：`GND` / `VIN` / `SIG`。
  - 多个舵机串联接在总线接口上。
  - 舵机电源与控制板、主机必须 **共地**。

#### 1.2 固定串口设备名 `/dev/ros_robot_controller`

工程中已经自带 udev 规则文件：

- 路径：`src/ros_robot_controller-ros2/src/ros_robot_controller/scripts/99-ros-robot-controller.rules`
- 核心内容（简化）：

```text
SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", \
  SYMLINK+="ros_robot_controller", \
  MODE="0666", GROUP="dialout", \
  ENV{ID_MM_PORT_IGNORE}="1"
```

安装步骤（只需执行一次）：

```bash
cd ~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/scripts
sudo cp 99-ros-robot-controller.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插控制板后检查：

```bash
ls -l /dev/ros_robot_controller
```

应看到符号链接指向某个 `ttyACM*`。之后所有代码统一使用 `/dev/ros_robot_controller`。

---

### 2. 舵机 ID 管理与基础通信测试

#### 2.1 修改单个舵机 ID

工具脚本：

- 文件：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py`

逻辑：

- 使用 `Board(device="/dev/ros_robot_controller")` 打开控制板；
- 调用 `bus_servo_set_id(old_id, new_id)` 修改舵机 ID；
- 再通过 `bus_servo_read_id(254)` 回读确认。

使用方法（每次只接一个舵机）：

```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py
```

如需指定目标 ID，可编辑脚本末尾：

```python
change_servo_id(
    device="/dev/ros_robot_controller",
    old_id=254,   # 当前 ID 不确定时可用广播 254
    new_id=1,     # 设置的新 ID
)
```

为 6 个舵机依次设置好独立 ID（例如 1~6）。

#### 2.2 连续往复扫动测试（裸舵机）

脚本：

- 文件：`~/ros2_ws/continuous_sweep.py`

核心行为：

- 打开 `/dev/ros_robot_controller`；
- 对 `SERVO_IDS = [1, 2, 3, 4, 5, 6]` 按正弦函数做持续往复转动：
  - 中心 `pos=500`（约 120° 中位）；
  - 振幅约 `amp=300`（约 ±72°）；
  - 周期由 `math.sin(0.5 * t)` 控制。

运行：

```bash
cd ~/ros2_ws
source install/setup.bash
python3 continuous_sweep.py
```

现象：

- 启动瞬间若舵机原本不在中位，会先快速回中位（一次性短暂动作）；
- 随后所有舵机在中位附近规律往复转动。

停止：

```text
在运行脚本的终端按 Ctrl + C
```

舵机会回到中位并停止。

---

### 3. HX‑桥接节点（`so101_hx35hm_bridge`）说明

#### 3.1 包结构

- 目录：`src/so101_hx35hm_bridge/`
- 关键文件：
  - `package.xml` / `setup.py` / `setup.cfg`
  - `so101_hx35hm_bridge/bridge_node.py`
  - `launch/so101_hx35hm_bridge.launch.py`

安装与构建：

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### 3.2 功能概述

桥接节点 `hx35hm_bridge` 的主要职责：

1. **连接 STM32 控制板**

   - 使用 `Board(device="/dev/ros_robot_controller")` 打开串口；
   - `enable_reception()` 开启接收线程。

2. **命令方向：ROS 关节角 → 舵机总线位置**

   - 订阅话题：
     - `/follower/forward_controller/commands`（`std_msgs/Float64MultiArray`，单位 rad）；
   - 提供 ActionServer：
     - `/follower/arm_trajectory_controller/follow_joint_trajectory`
     - 类型：`control_msgs/action/FollowJointTrajectory`
     - 用于 MoveIt 轨迹执行。

   - 关节名与舵机 ID 映射（可按实际调整）：

     ```python
     JOINT_ID_MAP = {
         "shoulder_pan": 1,
         "shoulder_lift": 2,
         "elbow_flex": 3,
         "wrist_flex": 4,
         "wrist_roll": 5,
         "gripper": 6,
     }
     ```

   - 单位转换：

     ```python
     angle_deg = angle_rad * 180.0 / math.pi       # rad → deg
     pos = angle_deg / 240.0 * 1000.0              # 0~240° ↔ 0~1000
     pos = max(0.0, min(1000.0, pos))              # 饱和在物理范围
     board.bus_servo_set_position(duration, [[servo_id, pos], ...])
     ```

3. **状态方向：最近一次命令的关节角 → JointState**

   - 每次发送命令时更新 `current_positions`；
   - 定时发布 `sensor_msgs/JointState`：
     - 话题：参数 `publish_joint_states_topic`（在 `follower` 命名空间下通常为 `joint_states`，即 `/follower/joint_states`）；
     - MoveIt 通过 remap 使用该话题作为当前 joint state。

---

### 4. 单臂 + HX‑MoveIt 启动与使用

目标：只启用 follower 单臂，通过 MoveIt 在 RViz 中进行规划并执行到 HX‑35HM 舵机。

#### 4.1 清理旧进程（可选但推荐）

```bash
pkill -f teleop.launch.py || true
pkill -f follower_hx35hm_moveit.launch.py || true
pkill -f continuous_sweep.py || true
pkill -f ros_robot_controller || true
pkill -f hx35hm_bridge || true
pkill -f move_group || true
pkill -f rviz2 || true
```

确认 ROS 图是否干净：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 node list
```

应当为空或仅有系统节点。

#### 4.2 启动单臂 HX‑MoveIt 工程

启动命令：

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py use_sim_time:=false
```

该 launch 文件会：

- 使用 `so101_description` 中的 `so101_arm.urdf.xacro`，variant 为 `follower`；
- 启动：
  - `/follower/hx35hm_bridge` 节点（连接控制板，驱动舵机）；
  - `move_group`（MoveIt）；
  - 带 MoveIt 插件的 RViz。

检查节点与 action：

```bash
ros2 node list
ros2 action info /follower/arm_trajectory_controller/follow_joint_trajectory
```

应看到：

- Action server：`/follower/hx35hm_bridge`；
- Action client：`/moveit_simple_controller_manager`。

#### 4.3 在 RViz 中使用 MoveIt

1. 在 MoveIt 面板中选择规划组：`manipulator`。
2. 在 3D 场景里拖动末端交互 marker，设定一个新的目标位姿（注意不要太夸张，以免超出 URDF 限位）。
3. 按顺序点击：
   - `Plan`：预览绿色轨迹；
   - `Plan and Execute`：执行轨迹。
4. 观察：
   - HX‑35HM 舵机是否按预期转动；
   - 可以多次规划不同目标，第一次执行时如当前姿态与初始姿态差距很大，可能会看到一次较快的“回补”动作，后续会平滑许多。

---

### 5. 通过话题直接控制关节（绕过 MoveIt）

用于简单调试或验证桥接是否正常。

前提：`follower_hx35hm_moveit.launch.py` 或至少 `hx35hm_bridge` 在运行。

发送一组关节命令（约 30°）：

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 topic pub /follower/forward_controller/commands std_msgs/msg/Float64MultiArray "
layout:
  dim: []
  data_offset: 0
data: [0.52, 0.52, 0.52, 0.52, 0.52, 0.52]
" -1
```

查看关节状态：

```bash
ros2 topic echo /follower/joint_states
```

---

### 6. 机械装配与零位建议

在将 HX‑35HM 安装回机械臂之前，推荐流程：

1. **舵机中位定位**
   - 通过上位机软件或简单脚本，将每个舵机转到中位：
     - 例如 pos ≈ 500（对应约 120°）。
   - 此时的舵机角度即作为该关节的“机械零位”或中位参考。

2. **在中位姿态下安装连杆与舵盘**
   - 在保持舵机通电、不强扭的前提下：
     - 让机械结构（臂段/连杆）处于预期的零姿态（例如手臂垂下或伸直）；
     - 对齐舵盘与连杆基准，锁紧螺丝。
   - 如存在少量齿差，可使用舵机的 offset 参数进行小范围偏差校正。

3. **整体零位检查**
   - 所有关节装好后，通过：
     - `ros2 topic pub /follower/forward_controller/commands` 发送一组 0 rad 命令；
   - 检查机械臂在“零姿态”是否符合预期；
   - 再在 URDF / MoveIt 中确保关节限位和正方向与机械实际一致。

---

### 7. 常见排查要点

1. **舵机完全不动**
   - 检查：
     - `/dev/ros_robot_controller` 是否存在；
     - 是否有其他进程（旧的 `ros_robot_controller`、旧 bridge、旧 teleop）占用该串口；
     - `ros2 action info /follower/arm_trajectory_controller/follow_joint_trajectory` 中的 Action server 是否为 `/follower/hx35hm_bridge`。

2. **第一次执行冲击大**
   - 原因多为当前实现的“初始 joint state”和舵机当前物理角度不一致；
   - 装机后可在启动逻辑中增加“读取当前舵机位置作为初始关节状态”的步骤。

3. **角度感觉偏小**
   - 检查：
     - URDF 中的关节 `limit` 是否限制了行程；
     - HX‑35HM EEPROM 中是否设置了较窄的角度限位；
   - 如需增大幅度，可适当调大 MoveIt 中目标位姿变化，或调整 URDF/舵机限位参数。

如后续在装机或调试中遇到具体问题（例如某关节方向反了、动作幅度不对），建议先在 RViz 的“Joint Space”只调一个关节、观察对应舵机，再结合上述映射与单位关系进行有针对性的微调。\

