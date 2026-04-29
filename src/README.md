# SO101 HX35HM ROS2 Workspace

这是一个基于 ROS 2 Jazzy 的机械臂工作区，目标是把 `SO-101` 机械臂软件栈适配到 `HX-35HM` 总线舵机控制链路上，并保留上游项目在遥操作、MoveIt、数据采集与策略推理方面的能力。

仓库当前由三部分组成：

## 1. `so101_hx35hm_bridge`

这是本仓库最核心的自定义包，负责把 ROS 2 控制层和 HX-35HM 硬件控制板连接起来。

- `so101_hx35hm_bridge/bridge_node.py`
  - 订阅 `forward_controller` 的关节命令
  - 将关节角转换为 HX-35HM 舵机位置并下发到底层控制板
  - 发布 `joint_states`
  - 提供 `FollowJointTrajectory` 与 `ParallelGripperCommand` 动作接口，便于 MoveIt 执行轨迹
  - 支持舵机位置回读、串口独占锁、关节方向与零位映射配置
- `launch/so101_hx35hm_bridge.launch.py`
  - 启动桥接节点，暴露串口设备、命令话题、状态话题等参数
- `config/assembly_calibration.yaml`
  - 装配/标定相关配置
- `scripts/`
  - 包含装配姿态与回中辅助脚本
- `aruco_detector_node.py` / `red_circle_detector_node.py` / `table_plane_estimator_node.py`
  - 视觉辅助节点，用于装配、目标定位或平面估计

## 2. `so101-ros-physical-ai`

这是上游 SO-101 ROS 2 项目，提供完整的软件栈能力：

- `so101_bringup`：整机启动、控制器、相机、录制流程
- `so101_description`：URDF/Xacro 与模型资源
- `so101_moveit_config`：MoveIt 2 规划配置
- `so101_teleop`：主从遥操作
- `episode_recorder`：示教数据录制
- `rosbag_to_lerobot`：数据集转换
- `so101_inference` / `policy_server`：策略推理与远程推理服务

这个目录保留了上游能力，你的 `so101_hx35hm_bridge` 则负责把其中的控制输出接到 HX-35HM 实际硬件。

## 3. `ros_robot_controller-ros2`

这是底层控制板 SDK/ROS 2 封装，`so101_hx35hm_bridge` 会调用其中的 `ros_robot_controller_sdk.Board` 与总线舵机板通信。

## 代码调用关系

整体控制链路可以理解为：

```text
MoveIt / Teleop / Forward Controller
        ↓
so101_hx35hm_bridge/bridge_node.py
        ↓
ros_robot_controller_sdk.Board
        ↓
HX-35HM 总线舵机控制板
        ↓
机械臂关节
```

如果你主要关注“机械臂能不能动起来”，优先看这几个位置：

1. `so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`
2. `so101_hx35hm_bridge/launch/so101_hx35hm_bridge.launch.py`
3. `so101-ros-physical-ai/so101_bringup`
4. `so101-ros-physical-ai/so101_moveit_config`
5. `ros_robot_controller-ros2/src/ros_robot_controller`

## 当前仓库用途

这个仓库更像一个“完整工作区快照”，而不是单一 ROS 包仓库。它同时包含：

- 上游 SO-101 项目
- 你的 HX-35HM 适配层
- 底层控制板驱动
- HX-35HM 相关 PDF 资料

适合直接在 `~/ros2_ws/src` 下开发、编译和联调。
