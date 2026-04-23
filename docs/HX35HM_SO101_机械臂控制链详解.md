# HX35HM SO101 机械臂控制链详解

## 1. 文档目的

这份文档用来完整解释你这套机械臂项目里，从“上层任务节点发出抓取意图”到“底层控制板给总线舵机发命令”的整条控制链。

重点回答几个问题：

- 你的机械臂控制链到底有哪些层
- 每一层分别负责什么
- 哪些 ROS 话题、服务、action 在起作用
- MoveIt、IK、bridge、控制板之间是什么关系
- 为什么这套系统可以做到较高精度，但平滑度会受限
- 以后排查问题应该优先看哪一层

这份文档偏“架构解释”，不是日常启动手册。

相关文档：

- `HX35HM_SO101_MoveIt规划控制启动流程.md`
- `HX35HM_SO101_红球抓取完整执行步骤.md`
- `HX35HM_SO101_红球抓取调试日志.md`
- `HX35HM_SO101_手动调参指南.md`
- `HX35HM_SO101_环境清理与进程管理.md`

---

## 2. 整体控制链总览

你这套系统可以粗略分成 6 层：

1. 任务层
2. 视觉层
3. 运动规划 / IK 层
4. ROS 控制接口层
5. `hx35hm_bridge` 适配层
6. 控制板 / 总线舵机层

如果把整条链路写成一句话，就是：

**视觉给出目标 -> 抓取节点生成动作阶段 -> IK 或 MoveIt 生成关节目标 -> ROS 控制接口发布命令 -> `hx35hm_bridge` 翻译成控制板协议 -> STM32 板子给 HX-35HM 舵机发位置命令。**

---

## 3. 第一层：任务层

任务层的核心节点是：

- `so101_grasping` 包里的 `so101_visual_grasp`

对应启动文件：

- [so101_visual_grasp.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping/launch/so101_visual_grasp.launch.py)

它负责做的事情不是“直接控制舵机”，而是：

- 订阅视觉目标位姿
- 采样并过滤目标 pose
- 生成抓取阶段：
  - `hover_high`
  - `pregrasp`
  - `grasp`
  - `retreat`
  - `rest`
- 处理抓取前后的夹爪逻辑
- 管理重试逻辑
- 处理桌面安全高度和桌面碰撞体

这层更像“任务编排器”。

### 3.1 它最重要的输入

- `pose_topic`
  默认是 `/vision/red_block/pose_base`
- `joint_states`
  从 `/follower/joint_states` 获取当前机械臂状态

### 3.2 它最重要的输出

它自己不直接往舵机发命令，而是：

- 调 `/go_to_pose`
- 调 `/go_to_joints`
- 或调用 MoveIt named pose / pose planning
- 调 gripper action

也就是说，这一层是“决定动作目标和阶段”的，不是“执行器”。

---

## 4. 第二层：视觉层

视觉层在你的项目里主要有两个节点：

- `red_circle_detector`
- `table_plane_estimator`

它们由主 bringup 启动：

- [follower_hx35hm_moveit.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py)

### 4.1 红球检测节点

红球检测节点输出：

- `/vision/red_block/pose_base`

这个 pose 是抓取任务的直接输入。

它的作用是：

- 从 RGB + depth 数据中找红球
- 输出目标在 `base_link` 坐标系下的位置

### 4.2 桌面估计节点

桌面估计节点输出：

- `/vision/table/top_z`
- `/vision/table/status`

它的作用是：

- 通过深度相机估计桌面真实高度
- 给抓取逻辑和碰撞体使用

这一步非常关键，因为：

- 视觉抓取点
- 抓取安全高度
- MoveIt 桌面碰撞盒

如果不共享同一套桌面几何信息，就很容易互相打架。

---

## 5. 第三层：运动规划 / IK 层

这是整条控制链最容易混淆的一层，因为你项目里实际上存在两套运动生成方式：

1. MoveIt
2. `so101_kinematics` 的 IK / 轨迹服务

### 5.1 MoveIt 负责什么

MoveIt 负责：

- named pose 回位
- pose target 规划
- collision scene
- 轨迹生成和 controller action 对接

相关配置文件：

- [moveit_controllers.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/moveit_controllers.yaml)
- [joint_limits.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/joint_limits.yaml)

MoveIt 最终不是直接控制舵机，而是通过：

- `FollowJointTrajectory`
- `ParallelGripperCommand`

去调用底层控制接口。

### 5.2 `so101_kinematics` 负责什么

`so101_kinematics` 这条链是你项目里非常关键的一层，主要由：

- `cartesian_motion_node.py`
- `motion_planner.py`
- `trajectory_executor.py`

组成。

它提供两个核心服务：

- `/go_to_pose`
- `/go_to_joints`

#### `/go_to_pose`

它的职责是：

- 接收笛卡尔目标 pose
- 根据策略生成关节轨迹
- 再把这条轨迹执行出去

可用策略包括：

- `cartesian`
- `joint_quintic`

#### `/go_to_joints`

它的职责是：

- 直接接收关节目标
- 生成一条 joint-space quintic 轨迹

#### `motion_planner.py`

这个模块负责：

- `plan_pose_move()`
- `plan_cartesian_segment()`
- `plan_joint_quintic()`
- `plan_joint_move()`

也就是说，它是轨迹生成器。

#### `trajectory_executor.py`

这个模块负责：

- 保存当前激活轨迹
- 根据当前墙钟时间采样轨迹
- 给 `cartesian_motion_node` 提供下一时刻的关节命令

它不是 IK 求解器，也不是 ROS 控制器，它更像一个“时间驱动的轨迹采样器”。

---

## 6. 第四层：ROS 控制接口层

这一层是 ROS 语义里的“机器人控制接口”。

你的系统里主要有三类接口：

### 6.1 `/follower/forward_controller/commands`

消息类型：

- `std_msgs/Float64MultiArray`

用途：

- `cartesian_motion_node` 用它流式发布关节角目标

这是一个很轻量的命令通道，不带完整的 `JointTrajectory` 语义，只是：

- 当前时刻这几个关节应该去哪里

### 6.2 `/follower/arm_trajectory_controller/follow_joint_trajectory`

action 类型：

- `control_msgs/action/FollowJointTrajectory`

用途：

- MoveIt 把完整手臂轨迹交给这个 action 执行

这条链适合：

- named pose
- MoveIt 规划出来的整轨动作

### 6.3 `/follower/gripper_controller/gripper_cmd`

action 类型：

- `control_msgs/action/ParallelGripperCommand`

用途：

- 专门控制夹爪开合

这条链已经被你现在的项目固定成：

- 夹爪只由 gripper action 控制
- arm 轨迹不再覆盖 gripper

这是我们之前调试里一个很重要的结果。

---

## 7. 第五层：`hx35hm_bridge` 适配层

这一层是你的 ROS 世界和 STM32 控制板世界之间的翻译器。

核心文件：

- [bridge_node.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py)

它做的事情非常多，主要包括：

- 订阅 `/follower/forward_controller/commands`
- 提供 `FollowJointTrajectory` action server
- 提供 `ParallelGripperCommand` action server
- 把关节角映射成 HX-35HM 舵机位置值
- 处理总线舵机回读
- 发布 `joint_states`

你可以把它理解成：

**ROS 控制接口到硬件协议之间的唯一翻译层。**

### 7.1 它为什么这么关键

因为上层发送的是：

- 关节角（rad）
- ROS action / topic 语义

而控制板需要的是：

- 舵机 ID
- 整数位置值
- 一个 duration

这中间全部靠 `bridge_node.py` 做转换。

### 7.2 `send_positions()` 做了什么

在 `bridge_node.py` 里，`send_positions()` 是最核心的方法之一。

它负责：

1. 把关节角 rad 转成 deg
2. 根据 `joint_directions` 和 `joint_zero_positions` 做每个关节的映射
3. 把结果映射到 `0..1000` 的舵机位置刻度
4. 调控制板 SDK 的 `bus_servo_set_position()`

也就是说：

- 上层世界是“关节空间”
- 到这里变成了“总线舵机目标刻度”

### 7.3 它同时支持两种手臂执行方式

#### 方式 A：流式命令

来源：

- `/follower/forward_controller/commands`

执行入口：

- `command_callback()`

特点：

- 每次收到一组关节目标，就按 `stream_command_duration` 下发一次
- 这主要服务于 `/go_to_pose` / `/go_to_joints` 那类上层自定义轨迹执行

#### 方式 B：完整轨迹

来源：

- `FollowJointTrajectory`

执行入口：

- `execute_trajectory_callback()`

特点：

- 接收一整条 `JointTrajectory`
- 再由 bridge 自己重采样
- 最终逐点翻译成底层命令

这主要服务于 MoveIt。

### 7.4 `joint_states` 是怎么来的

`hx35hm_bridge` 并不是直接把“上次命令值”原样发布出去，而是：

- 支持读取总线舵机回读位置
- 再把读回值反推回 joint angle
- 发布到 `/follower/joint_states`

这一步很重要，因为：

- MoveIt
- `cartesian_motion_node`
- `so101_visual_grasp`

都依赖当前机械臂状态。

如果这里不准，上层全都会被误导。

---

## 8. 第六层：控制板 / 协议 / 总线舵机层

最底层的核心文件是：

- [ros_robot_controller_sdk.py](/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py)

这个文件本质上是 STM32 控制板的 Python SDK。

### 8.1 这块板子在协议层到底做什么

它通过串口和板子通信，板子再去控制总线舵机。

对于总线舵机，最核心的方法是：

- `bus_servo_set_position(duration, positions)`

也就是说，对底层板子来说，一条最重要的手臂控制命令其实是：

- 在 `duration` 时间内
- 把这些 `servo_id`
- 移动到这些整数位置

这就是这套系统底层控制的真实语义。

### 8.2 这件事为什么重要

因为这说明：

- 这不是工业控制器
- 也不是支持轨迹缓存、速度前馈、加速度前馈的高级伺服总线

它更像：

**一次一次发“去这个位置，用这么长时间”的命令。**

所以如果上层不停高频发新位置，那机械臂观感很容易变成：

- 一格一格在追
- 机械感强
- 平滑度受限

这也是为什么你现在会感觉：

- 精度已经很好
- 但整体观感还是不够丝滑

不是单纯因为参数没调好，而是底层协议本身就更偏“离散目标控制”。

---

## 9. 你的抓取链在一次完整任务中是怎么流动的

下面按一次典型红球抓取来串这条链。

### 9.1 启动 bringup

主启动文件：

- [follower_hx35hm_moveit.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py)

它会拉起：

- `robot_state_publisher`
- `hx35hm_bridge`
- `move_group`
- `cartesian_motion_node`
- 相机链
- 红球检测
- 桌面估计

### 9.2 视觉发布红球 pose

红球检测节点发布：

- `/vision/red_block/pose_base`

### 9.3 抓取节点读取目标并生成阶段动作

`so101_visual_grasp`：

- 采样 pose
- 根据桌面高度和 offset 生成：
  - `hover_high`
  - `pregrasp`
  - `grasp`
  - `retreat`

### 9.4 抓取前半段通常怎么执行

当前你项目里，抓取前半段大多会优先走：

- `/go_to_pose`

也就是：

1. `so101_visual_grasp` 调用 `go_to_pose`
2. `cartesian_motion_node` 规划轨迹
3. `TrajectoryExecutor` 按时间采样
4. 每 `20ms` 发布一包 `/follower/forward_controller/commands`
5. `hx35hm_bridge` 把这包命令翻译成舵机位置 + `stream_command_duration`
6. SDK 调用 `bus_servo_set_position()`
7. STM32 控制板下发给 HX-35HM 总线舵机

### 9.5 夹爪怎么执行

夹爪不是走 `/forward_controller/commands`，而是：

1. `so101_visual_grasp` 发送 gripper action
2. `hx35hm_bridge` 的 `ParallelGripperCommand` action server 接收
3. `send_positions(["gripper"], ...)`
4. SDK 调 `bus_servo_set_position()`

### 9.6 回到 `rest` 怎么执行

当前主路线里，抓后回 `rest` 默认走：

- MoveIt named target

也就是：

1. `so101_visual_grasp` 调 MoveIt
2. MoveIt 规划完整 `JointTrajectory`
3. 发到 `/follower/arm_trajectory_controller/follow_joint_trajectory`
4. `hx35hm_bridge` 的 FJT action server 接收
5. bridge 自己重采样并插值
6. 最终变成一串 `bus_servo_set_position()` 命令

---

## 10. 为什么这套链路精度可以高，但平滑度还不满意

这是你现在最关心的问题，我单独写清楚。

### 10.1 精度为什么能高

因为我们已经把系统调成了：

- 目标更贴近真实读回状态
- `stream_command_duration` 不再大得离谱
- 前半段接近动作更连续
- 夹爪和 arm 的控制权已经拆清楚
- `grasp` 末端不再被旧值拖着走

所以它更“跟手”，精度自然会上来。

### 10.2 为什么观感还是不够丝滑

因为底层执行语义没有变：

- `cartesian_motion_node` 是 `50Hz` 离散发布关节目标
- `TrajectoryExecutor` 是按墙钟线性采样
- `bridge` 最终还是把每个采样点翻译成“位置 + duration”
- 控制板并不知道下一拍要保持速度连续

所以这条链更像：

- 高频离散重定目标

而不是：

- 真正连续的低 jerk 轨迹控制

这就是为什么你会觉得：

- 现在抓得准
- 但加速度感、机械感还是明显

### 10.3 当前最大的结构矛盾

你项目里手臂执行实际上有两种风格：

1. `/go_to_pose` 走流式关节命令
2. MoveIt 走 FJT 完整轨迹

这两条链在观感上天然就不完全一样。

再加上任务层又把动作分成多个阶段，所以整体会更容易出现“段落感”。

---

## 11. 以后怎么判断问题出在哪一层

下面这套排查顺序最实用。

### 11.1 如果抓取点偏了

优先看：

- 视觉目标是否准
- 桌面高度是否合理
- `grasp_x/y/z_offset`
- `After grasp target delta`

这通常先看任务层和视觉层。

### 11.2 如果夹爪状态不对

优先看：

- `/follower/gripper_controller/gripper_cmd`
- `bridge_node.py` gripper 日志
- `joint_states` 里 gripper 读回

这通常先看 gripper action 链。

### 11.3 如果动作卡顿、不顺

优先看：

- `/go_to_pose` 是否在高频流式重定目标
- `stream_command_duration`
- `TrajectoryExecutor` 的采样方式
- `bridge` 的采样和下发节奏

这通常先看 IK 执行链和 bridge。

### 11.4 如果回 `rest` 失败

优先看：

- MoveIt start state
- `allowed_start_tolerance`
- `tabletop_guard` 是否误碰撞
- `post_grasp_return_retry_count`

这通常先看 MoveIt 链。

---

## 12. 当前这条控制链最重要的几个参数

### `follower_hx35hm_moveit.launch.py`

- `move_duration = 0.8`
- `stream_command_duration = 0.05`

含义：

- 单次手动命令慢一点
- 流式控制短一点，避免明显拖尾

### `so101_visual_grasp.launch.py`

- `ik_pregrasp_duration_s = 1.7`
- `ik_grasp_duration_s = 1.1`
- `ik_retreat_duration_s = 1.1`
- `post_grasp_return_vel_scaling = 0.12`
- `post_grasp_return_acc_scaling = 0.12`

含义：

- 任务层的动作节奏

### `bridge_node.py`

- `trajectory_command_rate_hz = 50.0`
- `trajectory_min_command_interval_s = 0.015`
- `trajectory_min_segment_duration_s = 0.02`
- `trajectory_min_total_duration_s = 0.60`
- `trajectory_final_settle_s = 0.05`

含义：

- MoveIt / FJT 轨迹如何在 bridge 中被重采样和下发

---

## 13. 当前控制链的优点和短板

### 13.1 优点

- 视觉抓取闭环已经比较完整
- 夹爪控制权已经理顺
- `go_to_pose`、MoveIt、bridge、控制板的关系已经清楚
- 精度已经明显可用
- 有桌面估计、有碰撞保护、有重试逻辑

### 13.2 短板

- 仍然存在两套执行风格：
  - IK 流式执行
  - MoveIt/FJT 执行
- 底层板子协议只支持“位置 + duration”
- `TrajectoryExecutor` 不是低 jerk 执行器
- 整体观感仍容易有段落感和机械感

---

## 14. 如果以后继续升级，这条链最值得改哪里

如果以后要继续提升平滑度，而不是只调参数，我认为最值得动的是：

1. 让 `/go_to_pose` 不再直接走 `50Hz /forward_controller/commands`
2. 改成生成一整条 `JointTrajectory`
3. 然后统一交给 `FollowJointTrajectory`
4. 让手臂尽量只保留一个执行后端

这样能明显减少：

- 两套执行风格混用
- 高频离散重定目标的机械感
- 任务分段之间的风格割裂

这一步会比继续只调 `duration` 更有价值。

---

## 15. 总结

你的机械臂控制链可以概括成一句话：

**上层任务节点决定“做什么”，IK 或 MoveIt 决定“怎么走”，`hx35hm_bridge` 决定“怎么翻译成硬件命令”，STM32 控制板最终决定“怎么把这些命令发给总线舵机”。**

当前它已经做到：

- 抓取精度高
- 执行链闭合
- 夹爪和手臂控制不再互相干扰

但它仍然不够丝滑的根本原因也很明确：

- 上层还是偏离散地在驱动
- 底层协议也偏离散
- 中间没有真正统一、连续、低 jerk 的执行后端

所以你以后看这套系统时，最重要的理解不是“哪一个参数还没调好”，而是：

**这是一条多层翻译链，任何一层的语义不匹配，都会直接体现成你肉眼看到的机械感。**

