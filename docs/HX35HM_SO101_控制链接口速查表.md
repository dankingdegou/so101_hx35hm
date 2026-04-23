# HX35HM SO101 控制链接口速查表

## 1. 文档目的

这份文档是给日常调试用的“接口地图”。

用途：

- 快速确认某个节点应该看哪个 topic / service / action
- 出问题时快速定位在视觉层、任务层、IK 层、MoveIt 层还是 bridge 层
- 避免每次都去翻源码

配套文档：

- `HX35HM_SO101_机械臂控制链详解.md`
- `HX35HM_SO101_红球抓取完整执行步骤.md`
- `HX35HM_SO101_环境清理与进程管理.md`
- `HX35HM_SO101_手动调参指南.md`

---

## 2. 一张图先记住

最常用的主链路可以先记成：

1. 相机发布图像
2. 视觉节点发布目标 pose
3. 抓取节点读取目标 pose
4. 抓取节点调用 `/go_to_pose` 或 MoveIt
5. `cartesian_motion_node` 或 MoveIt 输出控制命令
6. `hx35hm_bridge` 转成控制板协议
7. 控制板给 HX-35HM 舵机下发位置命令
8. `hx35hm_bridge` 回读位置并发布 `joint_states`

---

## 3. 节点速查

### 3.1 任务层

#### `so101_visual_grasp`

包：

- `so101_grasping`

作用：

- 执行完整视觉抓取任务
- 生成 `hover_high / pregrasp / grasp / retreat / rest`
- 调用 IK 服务、MoveIt、gripper action

相关文件：

- [so101_visual_grasp.cpp](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping/src/so101_visual_grasp.cpp)
- [so101_visual_grasp.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping/launch/so101_visual_grasp.launch.py)

#### `so101_simple_pick`

包：

- `so101_grasping`

作用：

- 更简单的抓取逻辑或基础抓取验证

相关文件：

- [so101_simple_pick.cpp](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping/src/so101_simple_pick.cpp)

---

### 3.2 视觉层

#### `red_circle_detector`

包：

- `so101_hx35hm_bridge`

作用：

- 检测红球
- 发布目标在 `base_link` 下的 pose

#### `table_plane_estimator`

包：

- `so101_hx35hm_bridge`

作用：

- 从深度相机估计桌面高度
- 发布桌面 top z

---

### 3.3 运动 / IK 层

#### `cartesian_motion_node`

包：

- `so101_kinematics`

作用：

- 提供 `/go_to_pose`
- 提供 `/go_to_joints`
- 在 IK 轨迹执行时向 `/follower/forward_controller/commands` 发布关节目标

#### `move_group`

包：

- `so101_moveit_config`

作用：

- MoveIt 主规划节点
- 负责 named target、规划场景、碰撞检测、轨迹生成

---

### 3.4 硬件适配层

#### `hx35hm_bridge`

包：

- `so101_hx35hm_bridge`

作用：

- 接收 ROS 控制接口命令
- 提供 FJT 和 gripper action
- 把关节角映射成总线舵机位置
- 通过控制板 SDK 发串口命令
- 回读舵机位置并发布 `joint_states`

核心文件：

- [bridge_node.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py)

---

### 3.5 控制板 SDK 层

#### `ros_robot_controller_sdk.py`

包：

- `ros_robot_controller`

作用：

- Python 侧控制板 SDK
- 串口协议封装
- 总线舵机读写

核心文件：

- [ros_robot_controller_sdk.py](/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py)

---

## 4. 最常用 Topic 速查

### 4.1 视觉输入

#### `/static_camera/image_raw`

类型：

- `sensor_msgs/msg/Image`

用途：

- RGB 图像

常看场景：

- 红球检测异常
- ArUco 检测异常

#### `/static_camera/depth/image_raw`

类型：

- `sensor_msgs/msg/Image`

用途：

- 深度图

常看场景：

- 桌面高度不准
- 3D 点漂移

#### `/static_camera/camera_info`

类型：

- `sensor_msgs/msg/CameraInfo`

用途：

- RGB 相机内参

#### `/static_camera/depth/camera_info`

类型：

- `sensor_msgs/msg/CameraInfo`

用途：

- 深度相机内参

---

### 4.2 视觉输出

#### `/vision/red_block/pose_base`

类型：

- `geometry_msgs/msg/PoseStamped`

发布者：

- `red_circle_detector`

用途：

- 红球抓取目标

最常看场景：

- 目标点明显偏
- 抓取点位置不合理

#### `/vision/table/top_z`

类型：

- `std_msgs/msg/Float64`

发布者：

- `table_plane_estimator`

用途：

- 桌面 top z

最常看场景：

- 桌面碰撞盒不合理
- 抓取安全高度不合理

#### `/vision/table/status`

类型：

- `std_msgs/msg/String`

用途：

- 桌面估计状态信息

---

### 4.3 关节状态与控制命令

#### `/follower/joint_states`

类型：

- `sensor_msgs/msg/JointState`

发布者：

- `hx35hm_bridge`

用途：

- 当前机械臂状态

最常看场景：

- MoveIt 起点状态不对
- `go_to_pose` 误差大
- 夹爪状态是否真实变化

#### `/follower/forward_controller/commands`

类型：

- `std_msgs/msg/Float64MultiArray`

发布者：

- `cartesian_motion_node`

订阅者：

- `hx35hm_bridge`

用途：

- IK / 自定义轨迹的流式手臂关节命令

最常看场景：

- 机械臂不够平滑
- `go_to_pose` 是否真的在持续输出

---

## 5. 最常用 Service 速查

### 5.1 `/go_to_pose`

类型：

- `so101_kinematics_msgs/srv/GoToPose`

服务端：

- `cartesian_motion_node`

用途：

- 把末端移动到一个目标 pose

主要被谁调用：

- `so101_visual_grasp`

适合排查的问题：

- `hover_high / pregrasp / grasp` 阶段为什么不顺
- 为什么 IK 阶段成功但末端没到位

### 5.2 `/go_to_joints`

类型：

- `so101_kinematics_msgs/srv/GoToJoints`

服务端：

- `cartesian_motion_node`

用途：

- 直接执行一条 joint-space quintic 轨迹

适合排查的问题：

- 某个固定关节姿态是否能稳定到达
- 不想掺视觉时单独测执行链

---

## 6. 最常用 Action 速查

### 6.1 `/follower/arm_trajectory_controller/follow_joint_trajectory`

类型：

- `control_msgs/action/FollowJointTrajectory`

服务端：

- `hx35hm_bridge`

主要调用方：

- MoveIt

用途：

- 执行 MoveIt 生成的整条关节轨迹

适合排查的问题：

- named pose 回位失败
- MoveIt 规划成功但执行失败

常用检查命令：

```bash
ros2 action info /follower/arm_trajectory_controller/follow_joint_trajectory
```

### 6.2 `/follower/gripper_controller/gripper_cmd`

类型：

- `control_msgs/action/ParallelGripperCommand`

服务端：

- `hx35hm_bridge`

主要调用方：

- `so101_visual_grasp`
- MoveIt gripper controller

用途：

- 控制夹爪张开 / 闭合

适合排查的问题：

- 夹爪不闭合
- 夹爪闭合后又被打开
- 夹爪是否真的执行了动作

常用检查命令：

```bash
ros2 action info /follower/gripper_controller/gripper_cmd
```

---

## 7. 每条主链路怎么查

### 7.1 查“视觉有没有正常工作”

建议先查：

```bash
ros2 topic echo /vision/red_block/pose_base
ros2 topic echo /vision/table/top_z
ros2 topic echo /vision/table/status
```

如果这里不正常，不要先怀疑控制层。

### 7.2 查“抓取节点有没有正常发动作”

建议看：

- `so101_visual_grasp` 终端日志

重点日志关键词：

- `Moving to hover_high`
- `Moving to pregrasp`
- `Moving to grasp`
- `Closing gripper`
- `Retreating`
- `Returning to post-grasp pose`

### 7.3 查“IK 服务有没有正常执行”

建议看：

```bash
ros2 service list | grep go_to
```

重点是确认：

- `/go_to_pose`
- `/go_to_joints`

是否在线。

### 7.4 查“流式关节命令有没有真正发出来”

建议看：

```bash
ros2 topic echo /follower/forward_controller/commands
```

如果 `go_to_pose` 调用了，但这里没东西，先查 `cartesian_motion_node`。

### 7.5 查“MoveIt action 链有没有在线”

建议看：

```bash
ros2 action info /follower/arm_trajectory_controller/follow_joint_trajectory
ros2 action info /follower/gripper_controller/gripper_cmd
```

### 7.6 查“机械臂实际状态是不是被正确回读了”

建议看：

```bash
ros2 topic echo /follower/joint_states
```

如果机械臂明明动了，但 `joint_states` 完全不对：

- MoveIt 会误判起始状态
- `go_to_pose` 会误以为自己已经接近目标

---

## 8. 最值得记住的几条对应关系

### 8.1 抓取前半段主要走哪条链

大多数情况下：

- `so101_visual_grasp`
- `-> /go_to_pose`
- `-> cartesian_motion_node`
- `-> /follower/forward_controller/commands`
- `-> hx35hm_bridge`
- `-> Board.bus_servo_set_position()`

### 8.2 回 `rest` 主要走哪条链

大多数情况下：

- `so101_visual_grasp`
- `-> MoveIt named target`
- `-> /follower/arm_trajectory_controller/follow_joint_trajectory`
- `-> hx35hm_bridge`
- `-> Board.bus_servo_set_position()`

### 8.3 夹爪走哪条链

夹爪现在固定走：

- `so101_visual_grasp`
- `-> /follower/gripper_controller/gripper_cmd`
- `-> hx35hm_bridge`
- `-> Board.bus_servo_set_position()`

---

## 9. 如果某一层出问题，最可能是什么现象

### 视觉层出问题

常见现象：

- 机械臂去错位置
- 目标点忽高忽低
- 桌面高度不对

### 任务层出问题

常见现象：

- 阶段顺序不对
- 明明抓完了却没 retreat
- 回 `rest` 路线不合理

### IK 层出问题

常见现象：

- `/go_to_pose` 成功但末端没到位
- 前半段机械感很重
- 轨迹点数很多但观感很差

### MoveIt 层出问题

常见现象：

- `rest` 规划失败
- 执行前 start tolerance 不满足
- 起始状态碰撞

### bridge 层出问题

常见现象：

- 机械臂明显抖动
- 夹爪状态被覆盖
- 流式命令与 FJT 行为不一致

### 控制板 / 串口层出问题

常见现象：

- 完全不动
- 偶发卡死
- 回读超时
- 多进程抢串口导致严重抖动

---

## 10. 常用命令速查

### 看抓取目标

```bash
ros2 topic echo /vision/red_block/pose_base
```

### 看桌面高度

```bash
ros2 topic echo /vision/table/top_z
ros2 topic echo /vision/table/status
```

### 看关节状态

```bash
ros2 topic echo /follower/joint_states
```

### 看 IK 服务

```bash
ros2 service list | grep go_to
```

### 看 action

```bash
ros2 action info /follower/gripper_controller/gripper_cmd
ros2 action info /follower/arm_trajectory_controller/follow_joint_trajectory
```

### 看节点

```bash
ros2 node list
```

### 看话题

```bash
ros2 topic list
```

---

## 11. 总结

这份速查表最重要的价值不是“列接口”，而是帮你快速建立一个排查顺序：

1. 先看视觉输出是否正常
2. 再看抓取节点是否真的发起动作
3. 再看 `/go_to_pose` 或 MoveIt 是否真的执行
4. 再看 `hx35hm_bridge` 有没有收到并翻译命令
5. 最后看控制板和舵机是否真正响应

如果按这个顺序查，你基本不会再陷入“明明机械臂不顺，但不知道到底该看哪个节点”的状态。

