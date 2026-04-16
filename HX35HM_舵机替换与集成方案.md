## 背景与目标

- **现有工程**：`so101-ros-physical-ai`，机械臂完整 ROS2 栈（teleop / MoveIt / 录制 / 推理），底层通过 `ros2_control` + `feetech_ros2_driver/FeetechHardwareInterface` 控制 Feetech STS3215 总线舵机。
- **新硬件**：
  - 舵机：幻尔 HX-35HM 总线舵机（半双工 UART，总线协议详见《HX-35HM 总线舵机使用说明》）。
  - 控制板：STM32 总线舵机控制板，官方 ROS2 Python SDK：`ros_robot_controller-ros2`。
- **目标**：在尽量少改动 SO101 上层结构（teleop / MoveIt / 推理）的前提下，用 HX-35HM + STM32 控制板替换原来的 Feetech 舵机与驱动。

## 关键现状梳理

### 1. SO101 工程控制链路

- **URDF / ros2_control 配置**
  - 文件：`so101_description/urdf/ros2_control/so101_ros2_control.xacro`
  - 关节与舵机 ID 映射（示例）：
    - `shoulder_pan` → ID 1
    - `shoulder_lift` → ID 2
    - `elbow_flex` → ID 3
    - `wrist_flex` → ID 4
    - `wrist_roll` → ID 5
    - `gripper` → ID 6
  - 硬件插件：
    - `hardware_type == 'mock'`：`mock_components/GenericSystem`
    - `hardware_type == 'real'`：`feetech_ros2_driver/FeetechHardwareInterface`
- **控制器配置**
  - 文件：`so101_bringup/config/ros2_control/follower_controllers.yaml`
  - follower 控制器：
    - `joint_state_broadcaster`
    - `forward_controller`（`forward_command_controller/ForwardCommandController`）
    - `trajectory_controller`（`joint_trajectory_controller/JointTrajectoryController`）
  - 接口：
    - 命令接口：`position`
    - 状态接口：`position`, `velocity`
- **上层节点（只依赖 ROS 接口）**
  - `so101_teleop`：订阅 leader `/joint_states`，发布 follower 的关节命令（通过 `forward_controller` / `trajectory_controller`）。
  - MoveIt：通过 `trajectory_controller` 暴露的 `FollowJointTrajectory` action 控制机械臂。
  - 录制 / 推理：订阅 `joint_states` + 控制话题 / action，不关心底层具体舵机与协议。

> 结论：**上层都是“关节空间 + ros2_control”思路。只要继续提供同名关节的 position 命令接口和 joint_states，上层可以基本保持不变。**

### 2. 新控制板 ROS2 SDK 结构（`ros_robot_controller-ros2`）

- 包结构（主要 Python 包）：
  - `ros_robot_controller`（Python 节点包）
    - `ros_robot_controller_node.py`：创建 ROS2 节点 `RosRobotController`。
    - `ros_robot_controller_sdk.py`：底层 Python SDK，直接通过串口与 STM32 通信。
    - `launch/ros_robot_controller.launch.xml`：启动节点。
  - `ros_robot_controller_msgs`（消息/服务定义）
    - 消息：`BusServoState`, `SetBusServoState`, `MotorsState`, `PWMServoState` 等。
    - 服务：`GetBusServoState`, `GetPWMServoState`。

- **节点 `RosRobotController` 的主要接口**
  - 底层对象：`self.board = Board()`（来自 `ros_robot_controller_sdk`）。
  - 话题发布（相对 `~` 命名空间）：
    - `~/imu_raw` (`sensor_msgs/Imu`)
    - `~/joy` (`sensor_msgs/Joy`)
    - `~/sbus` (`ros_robot_controller_msgs/Sbus`)
    - `~/button` (`ButtonState`)
    - `~/battery` (`std_msgs/UInt16`)
  - 订阅 / 服务（与总线舵机相关）：
    - 订阅：`~/bus_servo/set_state` (`SetBusServoState`)
    - 服务：`~/bus_servo/get_state` (`GetBusServoState`)
    - 订阅：`~/pwm_servo/set_state` (`SetPWMServoState`)
    - 服务：`~/pwm_servo/get_state` (`GetPWMServoState`)

- **底层 SDK `Board` 与舵机相关的关键接口**
  - 写入控制：
    - `bus_servo_set_position(duration, positions)`  
      `positions` 为 `[[id, position], ...]`，`duration` 为秒。
    - `bus_servo_set_angle_limit(servo_id, [min, max])`
    - `bus_servo_set_vin_limit(servo_id, [min_v, max_v])`
    - `bus_servo_set_temp_limit(servo_id, limit)` 等。
  - 读取状态：
    - `bus_servo_read_position(servo_id)` → 当前舵机位置值。
    - `bus_servo_read_angle_limit(servo_id)` → 角度限位。
    - `bus_servo_read_vin(servo_id)` / `bus_servo_read_vin_limit(servo_id)`
    - `bus_servo_read_temp(servo_id)` / `bus_servo_read_temp_limit(servo_id)`
    - `bus_servo_read_torque_state(servo_id)` 等。

> 结论：**新控制板 + SDK 完全具备总线舵机控制所需功能，只是没有直接实现 ros2_control 的硬件插件，而是通过 Topic / Service 提供控制接口。**

### 3. HX-35HM 舵机特性（基于说明书）

- 通信与电气：
  - 半双工 UART 总线，波特率 115200。
  - 接口：`GND` / `VIN` / `SIG`（信号端为半双工 UART）。
  - 支持多机串联，ID 范围 0–253。
  - 工作电压：9.6–12.6 V，堵转电流可达 3 A。
- 角度 / 位置映射（舵机模式）：
  - 0–1000 对应 0°–240°（手册示例）。
  - 中位：500 → 120°。
  - 可配置限位：例如将最大角度限为 180°，则对应位置约为 750（根据 240/1000 比例）。
- 安全特性：
  - 堵转保护、过温保护。
  - 可配置：角度限位、电压限位、温度限位、偏差校正、速度控制等。

> 结论：**HX-35HM 完全满足机械臂关节舵机的基本与安全需求，需要在软硬件上合理设置 ID、限位、偏差、速度参数。**

## 总体集成思路选择

### 方案对比概述

1. **方案 A：写新的 ros2_control 硬件插件（C++），直接使用串口协议与 STM32 / 舵机通信**
   - 优点：架构“最正统”，完全集成在 ros2_control 生态。
   - 缺点：需要 C++ 插件开发，重写串口协议，工作量和调试成本较高。

2. **方案 B：保留 ros2_control + 控制器结构，增加一个 Python “桥接节点”使用 `Board` SDK 驱动舵机（推荐）**
   - 思路：控制链路变为  
     `teleop/MoveIt/推理 → ros2_control controllers → 桥接节点 → Board SDK → STM32 → HX-35HM`
   - 优点：上层基本不动；底层改动集中在桥接层，全部使用 Python，易于调试与迭代。
   - 缺点：bridge 相当于“用户自定义硬件接口”，理论上较 C++ 插件略“绕”，但是对实际使用影响不大。

3. **方案 C：完全绕开 ros2_control，直接让 teleop/MoveIt 使用 ros_robot_controller_msgs 的 Topic/Service**
   - 缺点：需要大幅重构 teleop、MoveIt 配置与上层逻辑，不推荐。

> 推荐采用 **方案 B：桥接节点方案**，先尽快跑通，后续如有必要再重构为正式 C++ 硬件插件。

## 方案 B：桥接节点整体设计

### 1. 总体结构

- 新建一个 ROS2 Python 包（建议名称）：`so101_hx35hm_bridge`
- 核心节点（Python）：`Hx35hmBridgeNode`
  - 依赖：
    - `sensor_msgs/JointState`
    - `control_msgs/JointTrajectory`（如果需要直接对接 trajectory_controller 的 action，可选）
    - `ros_robot_controller.ros_robot_controller_sdk.Board`（或通过 `ros_robot_controller_msgs` 的 Topic/Service 调用）
  - 主要功能：
    - **订阅**：`forward_controller` 的命令话题（或 trajectory_controller 的命令/action）。
    - **发布**：`/joint_states`（或写入 controller_manager 期望的 state 接口）。
    - **内部调用**：`Board.bus_servo_set_position` / `bus_servo_read_position` 等，完成与 HX-35HM 的协议交互。

### 2. 关节与舵机 ID 映射设计

- 在桥接节点内维护一份关节与舵机 ID 的映射表，例如：

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

- 这份映射需要与实际机械臂装配、舵机 ID 设定保持一致（通过上位机 / 调试板软件设置）。
- 如果需要支持 leader/follower 双臂，可增加命名空间或前缀区分不同控制板 / 总线。

### 3. 关节角度到舵机位置值的转换

- 根据 HX-35HM 说明书（以 0–1000 对应 0°–240° 为例）：
  - 角度（deg）转舵机位置值（pos）：
    - `pos = angle_deg / 240.0 * 1000.0`
  - 对应地，如果需要支持 0–360° 范围（0–1500）时：
    - `pos = angle_deg / 360.0 * 1500.0`
- 由于 SO101 的 URDF 与控制通常使用 **弧度（rad）**，建议在桥接层做转换：
  - `angle_deg = angle_rad * 180.0 / π`
  - 再按上式转换为舵机位置值。
- 不同关节的物理零位 / 机械限位可能不同，需要结合装配姿态，给每个关节定义：
  - 软件零位偏移（offset）
  - 软件限位（软件夹在舵机本身限位内，避免硬撞）。

### 4. 舵机状态回读与 `joint_states` 发布

- 周期性（例如 50 Hz）调用：
  - `Board.bus_servo_read_position(servo_id)` 获取每个关节当前舵机位置值。
  - 将 pos 值反推为角度（deg / rad）：
    - `angle_deg = pos / 1000.0 * 240.0`
    - `angle_rad = angle_deg * π / 180.0`
  - 组合成 `sensor_msgs/JointState`：
    - `name`：`["shoulder_pan", "shoulder_lift", ...]`
    - `position`：对应的弧度数组。
    - 可选：通过历史差分估算 `velocity`。
- 发布到系统的 `joint_states` 话题，或以命名空间形式挂到对应机械臂名下（例如 `/so101_follower/joint_states`）。

### 5. 与 ros2_control 控制器的衔接

有两种常见接法，可按实际需求选择：

1. **方式 1：桥接节点直接订阅 ForwardCommandController 的命令话题（简单）**
   - 配置 `forward_controller`，使其发布到某个命名空间下的命令话题，例如：
     - `/follower/forward_controller/commands`（`std_msgs/Float64MultiArray` 或类似类型）。
   - 桥接节点订阅该话题：
     - 收到的命令为关节目标位置（rad）。
     - 将其转换为舵机 pos 值，调用 `bus_servo_set_position(duration, ...)`。
   - 优点：实现简单，适合 teleop 与简单控制。
   - 缺点：如果想严格遵守 `ros2_control` 的接口规范，稍微“非标准”，但对你当前目标影响不大。

2. **方式 2：桥接节点对接 TrajectoryController 的 FollowJointTrajectory（更标准）**
   - 订阅 / 作为 client 调用 `trajectory_controller` 提供的 `FollowJointTrajectory` action。
   - 在 `goal` 中解析轨迹点，并以合适频率 / 插值方式调用 `bus_servo_set_position`。
   - 优点：与 MoveIt 集成更自然。
   - 缺点：轨迹解算逻辑略复杂，开发工作量比方式 1 大。

> 初期建议：**先实现方式 1，跑通 teleop 与简单位置控制；后期若需要高质量轨迹，可再扩展方式 2。**

### 6. 安全参数与保护逻辑

桥接节点中建议在启动流程或单独的初始化脚本中利用 SDK 设置舵机的安全参数：

- 电压限位：
  - `bus_servo_set_vin_limit(servo_id, [min_v, max_v])`
- 温度限位：
  - `bus_servo_set_temp_limit(servo_id, temp_limit)`
- 角度限位：
  - `bus_servo_set_angle_limit(servo_id, [min_pos, max_pos])`
- 扭矩开关：
  - `bus_servo_enable_torque(servo_id, 1/0)`

同时在桥接节点中增加基本的保护逻辑：

- 若读取到的温度、电压、位置异常（超出安全范围），可以：
  - 停止下发命令。
  - 记录错误 / 输出日志。
  - 视情况发布诊断信息（`diagnostic_msgs`）。

## 实施步骤（简要清单）

### 步骤 0：硬件侧准备

1. 使用厂家的调试板 + 上位机软件：
   - 单独连接每一个 HX-35HM 舵机，完成：
     - **ID 设置**（与 JOINT_ID_MAP 对应）。
     - **角度限位**（避免超出机械结构允许范围）。
     - **偏差调节**（让关键姿态的零位一致）。
     - **基本速度设置**（避免太快引起大冲击）。
2. 根据说明书设计/确认电源与电流：
   - 确保电源电压 9.6–12.6 V，电流能力足够（按每个舵机 1 A 及堵转情况预留安全余量）。
   - 使用足够粗的电源线和合适的电容/保险丝，减少压降和浪涌问题。

### 步骤 1：单独验证 `ros_robot_controller-ros2`

1. 构建并安装：
   - 确认 `~/ros2_ws/src/ros_robot_controller-ros2` 已加入工作区。
   - 在 `~/ros2_ws` 运行：
     - `colcon build --symlink-install`
2. 运行官方节点：
   - `ros2 launch ros_robot_controller ros_robot_controller.launch.xml`
3. 编写一个简单的测试节点（或用 `ros2 topic pub` / `ros2 service call`）：
   - 调用 `~/bus_servo/set_state`，让一个/多个舵机按指定位置转动。
   - 调用 `~/bus_servo/get_state`，确认能正确读到位置 / 电压 / 温度等。

### 步骤 2：设计与实现桥接包骨架

1. 新建 ROS2 Python 包（示例名称）：`so101_hx35hm_bridge`
   - 使用 `ros2 pkg create --build-type ament_python so101_hx35hm_bridge` 或手动创建。
2. 在 `setup.py` 中添加依赖：
   - `rclpy`
   - `sensor_msgs`
   - `std_msgs`
   - `ros_robot_controller`（若直接使用 `Board` 类）
3. 在包内创建节点文件（例如 `so101_hx35hm_bridge/bridge_node.py`），实现：
   - 初始化 `Board(device="/dev/ttyACM0")`，`enable_reception()`。
   - 订阅 `forward_controller` 的命令话题。
   - 定时器读取舵机位置并发布 `JointState`。
   - 使用 `JOINT_ID_MAP` 进行关节-舵机映射和单位转换。
4. 创建对应的 `launch` 文件（例如 `so101_hx35hm_bridge.launch.py`），用于：
   - 启动 `ros_robot_controller` 节点。
   - 启动 `so101_hx35hm_bridge` 节点。

### 步骤 3：与 SO101 工程集成（软连接）

1. 在 `so101_description`/`so101_bringup` 相关的 `launch` 文件中：
   - 增加选项参数（例如 `use_hx35hm:=true/false`），控制是否启用新桥接方案。
   - 当 `use_hx35hm:=true` 时：
     - 可以选择 **不加载 Feetech 硬件插件**（或使用 `mock_components/GenericSystem` 仅用于上层控制器）。
     - 启动 `ros_robot_controller` + `so101_hx35hm_bridge`。
2. 确认 `follower` 端的 `forward_controller` 输出与桥接节点订阅的话题名称一致。
3. 确认 `joint_states` 话题名与 teleop / MoveIt / 录制配置中使用的一致，或在配置中适配。

### 步骤 4：联调与验证

1. 在只启用机械臂、不启相机/推理的情况下启动：
   - 类似：
     - `ros2 launch so101_bringup teleop.launch.py use_cameras:=false use_camera_tf:=false use_hx35hm:=true`
2. 检查：
   - `ros2 topic echo /joint_states`：确认关节名与角度变化正常。
   - 使用 teleop（leader/follower）检查关节实际响应是否符合预期。
3. 按顺序逐步打开：
   - 录制节点。
   - MoveIt demo。
   - 推理节点。
   - 每一步只改动必要配置，确保系统稳定。

### 步骤 5：优化与长期演进（可选）

1. 若桥接节点方案稳定，可考虑：
   - 把 `Board` SDK 中的协议逻辑移植成 C++ 实现，写一个正式的 `hardware_interface::SystemInterface` 插件。
   - 在 URDF 中直接用新的插件替代 Feetech 插件，使结构完全统一。
2. 增加更多安全保护：
   - 通过诊断话题发布温度、电压、错误状态。
   - 在极端情况下（过温、过压）自动解除扭矩或停机。

## 总结

- **替换可行性**：基于当前代码与文档，使用 HX-35HM 舵机 + STM32 控制板完全可以替换原 Feetech 驱动，只需在 PC/ROS2 侧完成协议与接口的桥接。
- **推荐路线**：优先采用“保留 ros2_control + 新建 Python 桥接节点”的方案，上层 teleop / MoveIt / 推理改动最小，风险可控，迭代成本低。
- **关键工作点**：
  - 舵机 ID / 限位 / 偏差的正确标定。
  - 关节-舵机映射与单位转换（rad/deg/pos）。
  - 舵机状态回读与 `joint_states` 的一致性。

在真正动手修改代码前，可以以此文档为 checklist，从硬件 → SDK → 桥接包 → SO101 集成，逐步完成与验证。

