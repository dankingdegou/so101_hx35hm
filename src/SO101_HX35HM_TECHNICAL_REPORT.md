# SO101 + HX-35HM（STM32 Board）ROS 2 技术报告

版本：v1（基于当前 `~/ros2_ws` 工作空间现状）  
日期：2026-03-17  
适用系统：Ubuntu 24.04 + ROS 2 Jazzy

---

## 1. 摘要

本项目在 `so101-ros-physical-ai`（SO101 机械臂完整 ROS2 栈：teleop / MoveIt2 / 录制 / 数据集转换 / 推理）的基础上，将底层舵机从 Feetech STS3215（`ros2_control` 硬件插件）替换为 **HX-35HM 总线舵机 + STM32 控制板**，通过 `ros_robot_controller-ros2` 提供的 Python SDK（`Board`）直接驱动舵机，并保持上层接口尽可能不变。

核心集成策略为“桥接节点方案”：

- 上层（teleop / 推理 / 录制）依然使用 `ForwardCommandController` 的命令话题 `/follower/forward_controller/commands`（`std_msgs/Float64MultiArray`，单位 rad）。
- 新增 `so101_hx35hm_bridge` 节点把“关节角(rad)”映射为“舵机位置(pos)”并调用 `Board.bus_servo_set_position()` 下发。
- bridge 同时发布 `/follower/joint_states` 给 MoveIt/录制/推理作为观测。
- 对于 MoveIt 规划执行：bridge 提供 `FollowJointTrajectory` action（arm 部分），以简化方式逐点执行轨迹。

现状评估：

- **手臂 5 轴（不含夹爪）在 HX-35HM 模式下可实现 MoveIt 规划与执行。**
- **夹爪在 MoveIt 配置里仍被当作 `ParallelGripperCommand` 控制器，但 HX-35HM 模式当前缺少对应 action server，属于功能缺口。**
- bridge 已支持 **真实位置回读** 并用于发布 `/follower/joint_states`，使 MoveIt/录制/推理使用的状态更贴近真实机械臂；同时保留“命令回显”作为回读失败时的退化路径。

---

## 2. 项目范围与目标

### 2.1 目标

1. 在尽量少改动上层（teleop / MoveIt / 录制 / 推理）的前提下，完成 HX-35HM 的硬件替换与可用控制链路。
2. 提供装配阶段可复现的“rest 折叠姿态”定位能力，降低装配/对齐成本与撞机风险。
3. 输出完整的使用流程与装配文档，形成可交付的工程资料。

### 2.2 非目标（当前未实现或未纳入）

- 开发新的 `ros2_control` C++ HardwareInterface 插件（更正统，但工作量大）。
- 真实舵机位置/温度/电压等的持续回读闭环控制与故障保护策略（当前仅基础下发与有限告警）。
- MoveIt 夹爪的 `ParallelGripperCommand` 在 HX-35HM 下的完整动作执行（当前缺口）。

---

## 3. 工作空间结构

工作空间：`~/ros2_ws`

### 3.1 顶层文档与脚本（HX-35HM 相关）

- `HX35HM_舵机替换与集成方案.md`：总体设计与方案对比
- `HX35HM_集成实现修改日志.md`：改动记录与踩坑点
- `HX35HM_单臂集成使用说明.md`：单臂 + MoveIt 启动与注意事项
- `ros_robot_controller_设备命名与串口稳定性修改说明.md`：udev 与设备名稳定化
- `continuous_sweep.py`：裸控制板扫动测试
- `return_to_home.py`：裸控制板回中位

### 3.2 `~/ros2_ws/src` 关键包

- `so101-ros-physical-ai/`：原 SO101 完整 ROS2 栈（teleop / MoveIt / 录制 / 推理 / 数据集）
- `ros_robot_controller-ros2/`：STM32 控制板 ROS2 Python SDK（含 `ros_robot_controller_sdk.Board`）
- `so101_hx35hm_bridge/`：HX-35HM 桥接包（bridge 节点、装配脚本、配置）

---

## 4. 运行环境与依赖

### 4.1 系统与 ROS

- Ubuntu 24.04
- ROS 2 Jazzy：`source /opt/ros/jazzy/setup.bash`

### 4.2 设备命名（关键）

通过 udev 规则将 STM32 控制板稳定命名为：

- `/dev/ros_robot_controller`

规则文件参考：

- `~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/scripts/99-ros-robot-controller.rules`

### 4.3 HX-35HM 默认角度映射假设（需与你实际一致）

本工程默认采用常见默认映射：

- `pos ∈ [0, 1000]` 对应 `angle ∈ [0°, 240°]`
- 中位：`pos=500` 约等于 `120°`

如果你把舵机配置为其它量程（例如 0..1500 对应 0..360°），需要同步调整：

- bridge 节点参数：`servo_pos_min/max`、`servo_range_deg`
- 装配脚本配置：`so101_hx35hm_bridge/config/assembly_calibration.yaml`

---

## 5. 总体架构

### 5.1 原始（Feetech + ros2_control）参考链路

```text
leader HW -> /leader/joint_states
                     |
                     v
              so101_teleop
                     |
                     v
/follower/forward_controller/commands (rad)
                     |
                     v
ros2_control (Feetech HardwareInterface) -> Feetech servos
                     |
                     v
/follower/joint_states (from joint_state_broadcaster)
```

### 5.2 HX-35HM（桥接节点方案）链路

**Teleop / 推理 / 录制：**

```text
so101_teleop / so101_inference
        |
        v
/follower/forward_controller/commands (Float64MultiArray.data, rad)
        |
        v
so101_hx35hm_bridge (rad -> deg -> pos -> Board SDK)
        |
        v
STM32 Board -> HX-35HM bus servos
        |
        v
/follower/joint_states (默认: 周期性回读舵机位置并发布; 失败时退化为最近一次下发角度)
```

**MoveIt 规划执行（arm 5 轴）：**

```text
move_group + OMPL plan
        |
        v
/follower/arm_trajectory_controller/follow_joint_trajectory  (action)
        |
        v
so101_hx35hm_bridge ActionServer
        |
        v
逐点执行: send_positions -> bus_servo_set_position
```

---

## 6. 关键组件设计与接口

### 6.1 `ros_robot_controller-ros2`（STM32 Board SDK）

核心对象：`ros_robot_controller.ros_robot_controller_sdk.Board`

关键接口：

- 写入：`bus_servo_set_position(duration_s, [[id, pos], ...])`
- 读取：`bus_servo_read_position(id, timeout=...)`（bridge 周期性回读使用）

SDK 文件：

- `~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`

### 6.2 `so101_hx35hm_bridge`（桥接包）

#### 6.2.1 bridge 节点

文件：

- `~/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`

输入：

- 订阅 `command_topic`（默认 `/follower/forward_controller/commands`）
  - 类型：`std_msgs/msg/Float64MultiArray`
  - 语义：按 `joint_names` 顺序的关节目标位置（rad）

输出：

- 发布 `publish_joint_states_topic`（推荐 `/follower/joint_states`）
  - 类型：`sensor_msgs/msg/JointState`
  - 语义：当前关节位置（默认来自周期性回读；回读失败时退化为最近一次发送角度）

动作（可选开启）：

- `arm_trajectory_controller/follow_joint_trajectory`
  - 类型：`control_msgs/action/FollowJointTrajectory`
  - 运行在 `namespace=follower` 时 action 全名为：
    - `/follower/arm_trajectory_controller/follow_joint_trajectory`

关节到舵机 ID 映射：

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

角度换算（可配置）：

- `rad -> deg -> pos`
- `pos = zero_pos + direction * (deg * pos_per_deg)`
- `pos_per_deg = (servo_pos_max - servo_pos_min) / servo_range_deg`

为避免“静默夹断”导致状态不一致，bridge 在发生 clamp 时会告警，并将发布的关节角更新为 clamp 后的等效角度。

#### 6.2.2 bridge 参数（建议纳入统一配置管理）

常用参数：

- `device`：默认 `/dev/ros_robot_controller`
- `joint_names`：默认 `JOINT_ID_MAP.keys()`
- `command_topic`：默认 `/follower/forward_controller/commands`
- `publish_joint_states_topic`：默认 `/joint_states`（推荐运行时改为 `/follower/joint_states`）
- `move_duration`：默认 `0.2`（秒）
- `state_publish_rate_hz`：默认 `50.0`
- `enable_follow_joint_trajectory`：默认 `true`（teleop 模式建议关掉）
- `enable_position_readback`：默认 `true`
- `position_readback_rate_hz`：默认 `60.0`（round_robin 模式下约为每关节 ~10Hz）
- `position_readback_mode`：默认 `round_robin`（可选 `all`）
- `position_readback_timeout_s`：默认 `0.05`
- `servo_pos_min/max`、`servo_range_deg`、`servo_zero_pos`
- `joint_directions[]`、`joint_zero_positions[]`（与 `joint_names` 顺序对齐）

#### 6.2.3 启动文件

- `~/ros2_ws/src/so101_hx35hm_bridge/launch/so101_hx35hm_bridge.launch.py`

### 6.3 `so101-ros-physical-ai`（上层栈）

上层栈提供完整的“控制 + 规划 + 录制 + 数据集 + 推理”管线；在 HX-35HM 模式下，核心原则是尽量保持这些节点与配置不改或少改，让它们继续围绕同一组 ROS 接口工作。

#### 6.3.1 Teleop（leader -> follower）

- 包：`so101-ros-physical-ai/so101_teleop`
- 行为：订阅 leader `/joint_states`，向 follower 发布：
  - forward 模式：`/follower/forward_controller/commands`（`Float64MultiArray`，单位 rad）
  - trajectory 模式：`/follower/trajectory_controller/joint_trajectory`（`JointTrajectory`，单位 rad）

关键配置：

- `so101-ros-physical-ai/so101_teleop/config/teleop.yaml`：关节列表与发布频率（默认 50Hz）
- `so101-ros-physical-ai/so101_teleop/launch/teleop.launch.py`：根据命名空间拼出 topic

在 HX-35HM 模式下建议使用 **forward 模式**，因为 bridge 直接订阅 forward 命令即可跑通 teleop/推理/录制主链路。

#### 6.3.2 Bringup（launch 聚合）

- 包：`so101-ros-physical-ai/so101_bringup`
- 关键入口：
  - `launch/teleop.launch.py`：双臂 + teleop +（可选）HX-35HM bridge
  - `launch/follower_hx35hm_moveit.launch.py`：HX-35HM 单臂 + MoveIt（不走 ros2_control）
  - `launch/recording_session.launch.py`：双臂 + 相机 + 录制
  - `launch/inference.launch.py`：follower + 相机 + 推理（Pixi）

> HX-35HM 集成时的关键点：follower 如果用 HX-35HM 驱动，仍然需要 `forward_controller` 提供命令 topic，但不应启动原 Feetech 硬件接口去连接串口。因此 `teleop.launch.py` 已引入 `follower_hardware_type` 并在 `use_hx35hm:=true` 时默认让 follower 走 `mock`（保留 controller 话题接口）。见：`so101-ros-physical-ai/so101_bringup/launch/teleop.launch.py`。

#### 6.3.3 MoveIt2（规划与执行）

MoveIt 由 `so101_moveit_config` 提供配置：

- SRDF 命名姿态（含 `rest` 折叠）：`so101-ros-physical-ai/so101_moveit_config/config/so101_arm.srdf`
- 控制器配置：`so101-ros-physical-ai/so101_moveit_config/config/moveit_controllers.yaml`
- move_group 启动：`so101-ros-physical-ai/so101_moveit_config/launch/move_group.launch.py`
  - 在根命名空间启动 move_group，但 remap `joint_states` 到 `/<namespace>/joint_states`

HX-35HM 轨迹执行路径：

- move_group 期望 arm 控制器为 `FollowJointTrajectory`
- `follower_hx35hm_moveit.launch.py` 启动 bridge 的 action server：
  - `/follower/arm_trajectory_controller/follow_joint_trajectory`

当前差异与缺口：

- `moveit_controllers.yaml` 还定义了 `follower/gripper_controller`（`ParallelGripperCommand`），但 HX-35HM 模式没有提供 `/follower/gripper_controller/gripper_cmd` action server，因此夹爪执行属于缺口（arm 5 轴可执行）。

#### 6.3.4 Episode 录制与数据集

录制：

- 包：`episode_recorder`
- 默认录制 topics 配置：`so101-ros-physical-ai/so101_bringup/config/recording/episode_recorder_so101.yaml`
  - 其中包含：
    - `/follower/joint_states`
    - `/follower/forward_controller/commands`
    - 相机话题

转换为 LeRobot 数据集：

- 包：`rosbag_to_lerobot`
- 配置：`so101-ros-physical-ai/rosbag_to_lerobot/config/so101.yaml`
  - `observation.state` 来自 `/follower/joint_states`
  - `action` 来自 `/follower/forward_controller/commands`

结论：只要 HX bridge 正确发布 `/follower/joint_states`，录制与数据集链路无需改动即可工作。

#### 6.3.5 推理（同步/异步）

包：`so101_inference`

- 默认参数：
  - `fwd_topic=/follower/forward_controller/commands`
  - `joints_topic=/follower/joint_states`

因此 HX-35HM 模式只要满足上述两个 topic 语义（rad），推理链路可直接复用。

---

## 7. 话题、动作与数据语义（总表）

### 7.1 核心话题

| 名称 | 类型 | 单位/语义 | 发布者 | 订阅者 |
|---|---|---|---|---|
| `/follower/forward_controller/commands` | `std_msgs/Float64MultiArray` | `data[i]` 为关节目标位置（rad），顺序与 joint 列表一致 | `so101_teleop` / `so101_inference` | `so101_hx35hm_bridge` |
| `/follower/joint_states` | `sensor_msgs/JointState` | 当前关节角（rad），`name[]` 与模型一致 | `so101_hx35hm_bridge`（HX 模式）或 `joint_state_broadcaster`（Feetech 模式） | MoveIt / 录制 / 推理 / 可视化 |

### 7.2 MoveIt 动作

| 名称 | 类型 | 语义 | 提供者 |
|---|---|---|---|
| `/follower/arm_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt 轨迹执行（arm 5 轴） | `so101_hx35hm_bridge`（HX 模式）或 `trajectory_controller`（Feetech 模式） |

### 7.3 夹爪动作（当前缺口）

| 名称 | 类型 | 语义 | 状态 |
|---|---|---|---|
| `/follower/gripper_controller/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | MoveIt 夹爪开合 | HX 模式缺失（需要实现或禁用） |

---

## 8. 装配与标定（以 `rest` 折叠姿态为基准）

### 8.1 参考姿态定义（软件侧）

`rest` 姿态来自 SRDF：

- `so101-ros-physical-ai/so101_moveit_config/config/so101_arm.srdf`

关节角（rad）：

- `shoulder_pan = 0`
- `shoulder_lift = -1.57`
- `elbow_flex = 1.57`
- `wrist_flex = 0.75`
- `wrist_roll = 0`

### 8.2 装配专用定位工具

配置文件（关节 -> 舵机 ID/方向/零位）：

- `~/ros2_ws/src/so101_hx35hm_bridge/config/assembly_calibration.yaml`

装配脚本（两种模式：单舵机 / 命名姿态）：

- `~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py`

典型用法：

```bash
# 预览 rest 将下发的舵机 pos（不发硬件）
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest --dry-run --yes

# 实际下发 rest（会要求你输入 YES 二次确认）
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest

# 单个舵机定位（装某个关节时用）
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --servo-id 2 --pos 500
```

### 8.3 标定参数的含义与落点

本工程把“物理装配差异”抽象为三个可控量：

1. `servo_id`：物理舵机 ID
2. `direction`：关节正方向（+1/-1）
3. `zero_pos`：关节 0 rad 对应的舵机位置值（pos）

这些标定值建议统一维护在 `assembly_calibration.yaml`，并同步写入：

- bridge 节点的 `joint_directions`、`joint_zero_positions`
- 装配脚本的 `joints.*` 段

---

## 9. 构建与运行

### 9.1 构建

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 9.2 HX-35HM 单臂 + MoveIt（规划执行）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py use_sim_time:=false
```

说明：

- 该 launch 不启 ros2_control（避免硬件插件干扰）
- 使用 bridge 提供 FollowJointTrajectory action 执行 arm 轨迹

### 9.3 双臂 teleop（follower 用 HX-35HM）

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup teleop.launch.py use_hx35hm:=true
```

该 launch 会：

- leader 走 `hardware_type`（默认 real）
- follower 默认在 `use_hx35hm:=true` 时走 `mock`（保留控制器话题接口）
- 启动 `hx35hm_bridge` 订阅 `/follower/forward_controller/commands` 并发布 `/follower/joint_states`

---

## 10. 测试与验收清单

### 10.1 硬件/设备名

```bash
ls -l /dev/ros_robot_controller
```

### 10.2 单舵机通信

- `change_servo_id.py` 能读写 ID
- `continuous_sweep.py` 能小幅扫动（建议只连一个舵机测试）

### 10.3 装配定位

```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest --dry-run --yes
```

确认输出表格与目标一致后再实发。

### 10.4 ROS 图检查（运行后）

```bash
ros2 topic echo /follower/joint_states -n 1
ros2 topic echo /follower/forward_controller/commands -n 1
ros2 action list | rg follow_joint_trajectory
```

---

## 11. 已知问题与风险

1. **夹爪控制缺口（MoveIt）**：HX 模式缺少 `/follower/gripper_controller/gripper_cmd` action server。
2. **回读负载与一致性风险**：position 回读为逐舵机请求-应答，会增加总线负载；若回读频率过高或总线拥塞，可能出现超时/抖动（当前通过 `timeout` 与 `round_robin` 默认策略降低阻塞风险）。
3. **轨迹执行简化**：FollowJointTrajectory 按点 sleep，未做插值与速度/加速度约束严格跟踪，复杂轨迹可能抖动或偏离期望。
4. **参数一致性风险**：如果 HX-35HM 的量程配置（pos/deg）与工程假设不一致，会造成角度映射错误，需要统一调整 `servo_range_deg/pos_max`。
5. **串口独占**：同一时间只能由一个进程打开 `/dev/ros_robot_controller`；已在 bringup 中避免同时启动 `ros_robot_controller` 节点与 bridge。

---

## 12. 迭代路线（建议优先级）

P0（强烈建议尽快做）：

- 为 HX-35HM 实现夹爪 action：
  - 要么在 `so101_hx35hm_bridge` 内新增 `ParallelGripperCommand` ActionServer
  - 要么在 HX 模式的 MoveIt 配置中禁用 gripper controller

P1（提升一致性与安全）：

- bridge 已支持周期性回读 `bus_servo_read_position()`，后续可补充温度/电压/过载等诊断与保护
- 增加温度/电压/过载保护策略（诊断 + 自动停机）

P2（提升轨迹质量）：

- FollowJointTrajectory 插值执行（按时间采样而非逐点 sleep）
- 与 MoveIt 执行监控更一致（可选反馈）

P3（工程化）：

- 统一配置源：bridge 与装配脚本共享同一份 YAML（避免两处维护）
- 增加“HX-35HM 录制 session / 推理 session”一键 launch（不依赖 ros2_control）

---

## 13. 附录：关键文件索引

### 13.1 HX-35HM bridge

- bridge 节点：`~/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`
- bridge launch：`~/ros2_ws/src/so101_hx35hm_bridge/launch/so101_hx35hm_bridge.launch.py`
- 装配配置：`~/ros2_ws/src/so101_hx35hm_bridge/config/assembly_calibration.yaml`
- 装配脚本：`~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py`
- 全中位脚本：`~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py`

### 13.2 MoveIt 与 rest 姿态

- SRDF：`~/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/so101_arm.srdf`
- 控制器配置：`~/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/moveit_controllers.yaml`
- HX MoveIt bringup：`~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py`

### 13.3 Bringup（teleop）

- `~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/teleop.launch.py`
