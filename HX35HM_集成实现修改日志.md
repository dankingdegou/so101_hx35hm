## HX-35HM 舵机集成实现修改日志

> 用于记录从 Feetech 舵机到 HX-35HM + STM32 控制板集成过程中，所有关键代码与配置改动。  
> 日期：2026-03-13 起

---

### 1. 方案与总体设计文档

- **新增文件**
  - `HX35HM_舵机替换与集成方案.md`
    - 内容：现有 SO101 工程控制链路分析、新控制板 SDK 能力、HX-35HM 舵机特性。
    - 明确推荐集成方案：保留 `ros2_control` 结构，新增 Python 桥接节点（方案 B）。

---

### 2. 控制板 SDK 辅助脚本与串口稳定性（ros_robot_controller-ros2）

- **新增脚本：舵机 ID 修改工具**
  - 文件：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py`
  - 功能：
    - 使用 `Board` SDK 通过控制板串口修改单个 HX-35HM 总线舵机 ID，并回读验证。
    - 默认仅连接一个舵机时使用，避免广播命令误改多舵机 ID。

- **新增 udev 规则：固定控制板设备名**
  - 文件：`src/ros_robot_controller-ros2/src/ros_robot_controller/scripts/99-ros-robot-controller.rules`
  - 作用：
    - 对 `idVendor == 1a86`、`idProduct == 55d4` 的串口设备创建稳定符号链接：
      - `/dev/ros_robot_controller -> /dev/ttyACM*`
    - 设置权限与组：
      - `MODE="0666"`, `GROUP="dialout"`, `ENV{ID_MM_PORT_IGNORE}="1"`

- **修改 Board 默认串口设备名**
  - 文件：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`
  - 改动：
    - `Board.__init__` 的默认参数：
      - 从：`device="/dev/ttyACM0"`
      - 改为：`device="/dev/ros_robot_controller"`
  - 影响：
    - 所有未显式指定 `device` 的 `Board()` 调用，统一通过稳定设备名访问控制板。

- **更新改 ID 脚本默认设备名**
  - 文件：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py`
  - 改动：
    - 函数默认参数 `device` 和 `__main__` 中示例调用均改为 `/dev/ros_robot_controller`。

- **补充说明文档**
  - 新增：`ros_robot_controller_设备命名与串口稳定性修改说明.md`
    - 记录：
      - udev 规则内容与安装步骤。
      - `Board` 默认参数与脚本的修改原因与影响评估。

---

### 3. HX-35HM 桥接包实现（so101_hx35hm_bridge）

- **新增 ROS2 Python 包**
  - 目录：`src/so101_hx35hm_bridge/`

- **包元数据与安装配置**
  - 新增：
    - `src/so101_hx35hm_bridge/package.xml`
    - `src/so101_hx35hm_bridge/setup.py`
    - `src/so101_hx35hm_bridge/setup.cfg`
    - `src/so101_hx35hm_bridge/resource/so101_hx35hm_bridge`
    - `src/so101_hx35hm_bridge/so101_hx35hm_bridge/__init__.py`
  - 主要内容：
    - 包名：`so101_hx35hm_bridge`
    - 构建类型：`ament_python`
    - 依赖：`rclpy`, `sensor_msgs`, `std_msgs`, `builtin_interfaces`, `ros_robot_controller`。
    - console_scripts：
      - `hx35hm_bridge = so101_hx35hm_bridge.bridge_node:main`

- **桥接节点实现**
  - 文件：`src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`
  - 主要功能：
    - 使用 `Board(device="/dev/ros_robot_controller")` 连接 STM32 控制板。
    - 订阅话题（可参数化，默认）：
      - `command_topic = "/follower/forward_controller/commands"`  
        类型：`std_msgs/Float64MultiArray`，表示每个关节的目标角度（rad）。
    - 关节与舵机 ID 映射（初始）：
      - `shoulder_pan` → 1
      - `shoulder_lift` → 2
      - `elbow_flex` → 3
      - `wrist_flex` → 4
      - `wrist_roll` → 5
      - `gripper` → 6
    - 命令转换逻辑：
      - rad → deg：`angle_deg = angle_rad * 180 / π`
      - deg → 舵机位置值：`pos = angle_deg / 240.0 * 1000.0`，并裁剪到 `[0, 1000]`。
      - 调用：`board.bus_servo_set_position(move_duration, [[servo_id, pos], ...])`
    - 状态回读与 `JointState` 发布：
      - 周期性调用 `board.bus_servo_read_position(servo_id)`。
      - 位置值 → deg：`angle_deg = pos / 1000.0 * 240.0` → rad。
      - 发布到（可参数化，默认）：`/joint_states`。
    - 可配置参数（ROS 参数）：
      - `device`：默认 `/dev/ros_robot_controller`
      - `joint_names`：关节名列表（默认使用 `JOINT_ID_MAP` 的 key）
      - `command_topic`：默认 `/follower/forward_controller/commands`
      - `move_duration`：单次动作时间（秒），默认 `0.2`
      - `publish_joint_states_topic`：默认 `/joint_states`
      - `state_publish_rate_hz`：默认 `50.0`

- **桥接 launch 文件**
  - 文件：`src/so101_hx35hm_bridge/launch/so101_hx35hm_bridge.launch.py`
  - 作用：
    - 启动 `hx35hm_bridge` 节点。
    - 提供 launch 参数：
      - `device`（默认 `/dev/ros_robot_controller`）
      - `command_topic`（默认 `/follower/forward_controller/commands`）
      - `state_topic`（默认 `/joint_states`，映射为节点参数 `publish_joint_states_topic`）

---

### 4. 与 SO101 teleop 启动流程的初步集成

- **修改文件**
  - `src/so101-ros-physical-ai/so101_bringup/launch/teleop.launch.py`

- 新增 Launch 参数
  - `use_hx35hm`（默认 `"false"`）：
    - 说明：控制是否在 teleop 流程中启用 HX-35HM 桥接节点和 STM32 控制板节点。

- 新增节点（受 `use_hx35hm` 控制）
  - `ros_robot_controller` 节点：
    - 包：`ros_robot_controller`
    - 可执行：`ros_robot_controller`
    - 仅在 `use_hx35hm == true` 时启动。
  - `hx35hm_bridge` 节点：
    - 包：`so101_hx35hm_bridge`
    - 可执行：`hx35hm_bridge`
    - 参数：
      - `device: "/dev/ros_robot_controller"`
      - `command_topic: "/follower/forward_controller/commands"`
      - `publish_joint_states_topic: "/joint_states"`
    - 仅在 `use_hx35hm == true` 时启动。

- 当前约束与使用建议
  - teleop 启动命令示例（启用 HX-35HM）：

    ```bash
    ros2 launch so101_bringup teleop.launch.py \
      use_hx35hm:=true \
      hardware_type:=mock
    ```

  - 说明：
    - `hardware_type:=mock` 建议在使用 HX-35HM 时手动传入，以避免原 Feetech 硬件插件尝试连接旧串口导致 ros2_control 启动失败。
    - 后续可以在 `follower.launch.py` 与 URDF xacro 中进一步自动化处理 `hardware_type` 与 HX-35HM 模式的联动。

---

### 5. 构建与运行测试中发现并修复的问题

- **2026-03-13：首次 `colcon build` 失败**
  - 现象：
    - `ros_robot_controller_msgs` 构建失败，错误为：
      - “failed to create symbolic link ... existing path cannot be removed: Is a directory”
  - 处理：
    - 删除旧的构建与安装输出：
      - `rm -rf build/ros_robot_controller_msgs install/ros_robot_controller_msgs`
    - 重新执行：
      - `colcon build --symlink-install`
    - 结果：
      - 全工作区构建成功，仅有 `so101_hx35hm_bridge` 的 setuptools 警告（dash-separated 选项），不影响运行。

- **2026-03-13：`teleop.launch` + `use_hx35hm:=true` 启动测试**
  - 启动命令：

    ```bash
    ros2 launch so101_bringup teleop.launch.py \
      use_hx35hm:=true \
      hardware_type:=mock \
      use_cameras:=false use_camera_tf:=false use_teleop_rviz:=false
    ```

  - 问题 1：串口同时被两个节点占用
    - 改动前：
      - `teleop.launch.py` 在 `use_hx35hm:=true` 时同时启动：
        - `ros_robot_controller` 节点
        - `hx35hm_bridge` 节点（内部也创建 `Board(device="/dev/ros_robot_controller")`）
      - 实际结果：
        - 两个节点都尝试打开 `/dev/ros_robot_controller`，`ros_robot_controller` 的接收线程报错：
          - `SerialException: device reports readiness to read but returned no data (device disconnected or multiple access on port?)`
    - 修复：
      - 修改 `teleop.launch.py`，在 `use_hx35hm` 分支 **仅启动 `hx35hm_bridge` 节点**，不再在此处自动启动 `ros_robot_controller`。
      - 说明：
        - 当前桥接节点 `hx35hm_bridge` 直接通过 `Board` SDK 驱动 HX-35HM 舵机，独占串口设备。
        - 如果后续需要 IMU / 手柄等功能，可在单独设计的流程中启动 `ros_robot_controller`，避免和桥接节点抢占串口。

  - 问题 2：controller spawner 的警告/错误（与 HX-35HM 改动关系不大）
    - 运行过程中看到若干控制器相关日志，例如：
      - `Controller already loaded, skipping load_controller`
      - `Could not configure controller with name 'joint_state_broadcaster' because no controller with this name exists`
      - `Failed to configure controller` 等。
    - 现状评估：
      - 这些日志集中在 `joint_state_broadcaster` 和 `forward_controller` 的加载/配置流程。
      - 表现为 spawner 节点在控制器已加载/未正确记录状态时重复尝试配置，引发警告或错误退出。
      - 与 HX-35HM 集成的串口/桥接逻辑无直接关系，teleop 主体和 bridge 本身仍可正常运行。
    - 后续处理计划：
      - 在需要精简日志与提升控制器状态管理可靠性时，再对 `so101_bringup` 中 controller spawner 的调用顺序与条件做细致梳理与优化。

- **2026-03-13：`so101_hx35hm_bridge` 构建时的 setuptools 警告**
  - 现象：
    - `so101_hx35hm_bridge` 在构建时打印：
      - `SetuptoolsDeprecationWarning: Invalid dash-separated options`，提示 `script-dir` / `install-scripts` 将在未来版本弃用。
  - 分析：
    - 来自 `setup.cfg` 中的旧写法：
      - `script-dir=$base/lib/so101_hx35hm_bridge`
      - `install-scripts=$base/lib/so101_hx35hm_bridge`
    - 这只是 setuptools 的“弃用提示”，**不会影响当前构建与运行**。
  - 处理：
    - 暂时仅记录为已知警告，后续如需消除，可将其改写为新版字段：
      - `script_dir` / `install_scripts`。

---

### 6. 后续计划（尚未实施的修改方向）

> 以下为 `HX35HM_舵机替换与集成方案.md` 中已规划但尚未在代码中完成的部分，后续每次实现后应在本日志追加条目。

- 在 `so101_bringup` 中进一步完善 HX-35HM 集成：
  - 已完成：
    - 在 `teleop.launch.py` 中增加 `use_hx35hm` 参数。
    - 条件性启动 `so101_hx35hm_bridge` 与 `ros_robot_controller`。
  - 待完成：
    - 在 `follower.launch.py` / URDF xacro 中联动 `hardware_type` 与 `use_hx35hm`（自动选择 `mock` 或 `real`）。
- 调整 `joint_states` 流向：
  - 使上层 teleop / MoveIt / 录制 / 推理优先使用桥接节点发布的真实关节状态，避免与 `joint_state_broadcaster` 重复冲突。
- 安全策略增强：
  - 在桥接节点中补充对舵机温度 / 电压 / 位置异常的检测与保护动作（停机 / 报警等）。

后续每一次改动完成后，请在本文件中新增小节，记录：

- 修改日期
- 受影响的文件列表
- 行为/逻辑上的变化摘要

