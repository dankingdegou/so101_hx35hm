# SO101 HX35HM ROS 2 Workspace

这是一个面向 `SO101 follower + HX-35HM + MoveIt 2 + 顶视 RGBD 相机` 的完整 ROS 2 工作区仓库。

仓库里包含：

- 真实机械臂桥接与轨迹执行
- MoveIt 2 规划与 RViz 控制
- 红球视觉抓取链路
- 顶视相机与桌面估计
- 手眼标定、相机内参标定工具
- 一套偏工程化的中文操作文档

如果你现在的目标是把机械臂跑起来、能规划、能抓红球，这个仓库就是完整工作区，不是单一功能包。

## 系统概览

当前主链路大致是：

1. `so101_hx35hm_bridge`
   把 ROS 轨迹和夹爪命令转换成 HX-35HM 真机控制。
2. `so101_moveit_config` + `so101_bringup`
   提供 MoveIt、相机、TF、bridge、RViz 的总装配入口。
3. `so101_kinematics`
   提供 `/go_to_pose`、`/go_to_joints` 等 IK/轨迹服务。
4. `so101_grasping`
   负责从视觉目标生成 `hover_high -> pregrasp -> grasp -> retreat -> rest` 抓取序列。
5. 顶视视觉链路
   包括 ArUco、红球检测、桌面高度估计、手眼标定工具。

## 环境

推荐环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12 系统环境
- `colcon`
- 真机模式下需要：
  - HX-35HM 控制板串口
  - 顶视 RGB 或 RGBD 相机

## 目录结构

顶层关键内容：

- [`src/`](/home/rog/ros2_ws/src)
  ROS 2 源码与各功能包
- [`docs/`](/home/rog/ros2_ws/docs)
  中文工程文档与调试记录
- [`calibration/handeye/`](/home/rog/ros2_ws/calibration/handeye)
  手眼标定结果与原始样本
- [`tools/hardware_debug/`](/home/rog/ros2_ws/tools/hardware_debug)
  裸硬件调试脚本
- [`src/so101-ros-physical-ai/`](/home/rog/ros2_ws/src/so101-ros-physical-ai)
  主体 ROS 2 工程，包含 bringup、MoveIt、运动学、抓取、相机、工具
- [`src/so101_hx35hm_bridge/`](/home/rog/ros2_ws/src/so101_hx35hm_bridge)
  HX-35HM 桥接、红球检测、ArUco 检测、桌面估计
- [`calibration/handeye/aruco_handeye_result.json`](/home/rog/ros2_ws/calibration/handeye/aruco_handeye_result.json)
  当前一套手眼标定结果
- [`calibration/handeye/aruco_handeye_result_v2.json`](/home/rog/ros2_ws/calibration/handeye/aruco_handeye_result_v2.json)
  另一套手眼标定结果
- [`tools/hardware_debug/continuous_sweep.py`](/home/rog/ros2_ws/tools/hardware_debug/continuous_sweep.py)
  舵机/机械臂扫动辅助脚本
- [`tools/hardware_debug/return_to_home.py`](/home/rog/ros2_ws/tools/hardware_debug/return_to_home.py)
  简单回位脚本

归档目录：

- `archive/`
  不参与当前主工作区构建，只用于保存历史备份或非主工程内容

主要 ROS 包：

- [`so101_bringup`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup)
- [`so101_moveit_config`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config)
- [`so101_grasping`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping)
- [`so101_kinematics`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_kinematics)
- [`so101_hx35hm_bridge`](/home/rog/ros2_ws/src/so101_hx35hm_bridge)
- [`so101_openni2_camera`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_openni2_camera)

## 构建

建议统一使用系统 Python，并清理旧缓存影响：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
colcon build \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/ros2_ws/install/setup.bash
```

如果你只改了抓取相关：

```bash
colcon build --packages-select so101_grasping so101_bringup so101_kinematics so101_hx35hm_bridge \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

如果构建日志里出现旧 Python 路径，例如 `/home/rog/.local/bin/python3.11`，通常是旧 CMake 缓存，不一定是源码本身的问题。

## 快速开始

### 1. 启动真机 MoveIt 控制栈

这是当前较稳定的真机总入口：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_red_detector:=true \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_vision_debug_rviz:=false \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml
```

这会拉起：

- `hx35hm_bridge`
- `move_group`
- 顶视相机
- 红球检测 `red_circle_detector`
- 桌面估计 `table_plane_estimator`
- `cartesian_motion_node`

### 2. 执行一次红球抓取

另开终端：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=true \
  pose_topic:=/vision/red_block/pose_base \
  add_table_collision:=true \
  use_tabletop_z_topic:=true \
  min_grasp_clearance_m:=0.015 \
  min_pregrasp_clearance_m:=0.080 \
  return_to_named_pose_after_grasp:=true \
  post_grasp_named_pose:=rest \
  post_grasp_use_ik_joints:=false \
  grasp_retry_count:=1 \
  post_grasp_return_retry_count:=3 \
  open_gripper_after_return:=true
```

当前推荐抓取流程是：

- 打开夹爪
- 采样稳定红球位姿
- 读取实时桌面高度 `/vision/table/top_z`
- `hover_high -> pregrasp -> grasp`
- 闭合夹爪
- retreat
- MoveIt 规划回 `rest`
- 打开夹爪放球

### 3. 快速健康检查

```bash
ros2 node list
ros2 action list
ros2 topic list | rg '/vision|/static_camera|/joint_states'
```

你通常应该能看到：

- `/move_group`
- `/follower/hx35hm_bridge`
- `/cartesian_motion_node`
- `/red_circle_detector`
- `/vision/red_block/pose_base`
- `/vision/table/top_z`

## 标定与文档入口

如果你第一次接这套系统，建议按下面顺序看文档：

- 抓取流程：
  [`HX35HM_SO101_红球抓取完整执行步骤.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_红球抓取完整执行步骤.md)
- MoveIt 启动与规划控制：
  [`HX35HM_SO101_MoveIt规划控制启动流程.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_MoveIt规划控制启动流程.md)
- 相机内参标定：
  [`HX35HM_SO101_相机内参标定完整步骤.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_相机内参标定完整步骤.md)
- 装配姿态与归零建议：
  [`HX35HM_SO101_装配姿态与归零建议.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_装配姿态与归零建议.md)

中文工程文档导航：

- 抓取调试日志：
  [`HX35HM_SO101_红球抓取调试日志.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_红球抓取调试日志.md)
- 机械臂控制链详解：
  [`HX35HM_SO101_机械臂控制链详解.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_机械臂控制链详解.md)
- 控制链接口速查表：
  [`HX35HM_SO101_控制链接口速查表.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_控制链接口速查表.md)
- 手动调参指南：
  [`HX35HM_SO101_手动调参指南.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_手动调参指南.md)
- 环境清理与进程管理：
  [`HX35HM_SO101_环境清理与进程管理.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_环境清理与进程管理.md)
- 摄像头调位与可视化执行步骤：
  [`HX35HM_SO101_摄像头调位与可视化执行步骤.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_摄像头调位与可视化执行步骤.md)
- 超详细装配流程：
  [`HX35HM_SO101_超详细装配流程.md`](/home/rog/ros2_ws/docs/HX35HM_SO101_超详细装配流程.md)

手眼标定相关工具在：

- [`tools/handeye/collect_aruco_handeye_samples.py`](/home/rog/ros2_ws/src/so101-ros-physical-ai/tools/handeye/collect_aruco_handeye_samples.py)
- [`tools/handeye/auto_motion_aruco_sampler.py`](/home/rog/ros2_ws/src/so101-ros-physical-ai/tools/handeye/auto_motion_aruco_sampler.py)
- [`tools/handeye/solve_aruco_handeye.py`](/home/rog/ros2_ws/src/so101-ros-physical-ai/tools/handeye/solve_aruco_handeye.py)

相机内参标定工具在：

- [`tools/camera_intrinsics/calibrate_ros_camera_intrinsics.py`](/home/rog/ros2_ws/src/so101-ros-physical-ai/tools/camera_intrinsics/calibrate_ros_camera_intrinsics.py)

## 当前仓库的一些工程约定

- `build/`、`install/`、`log/` 已忽略，不进仓库
- 相机标定文件优先使用 `package://` 路径，不再依赖本机固定 `file:///home/...`
- hand-eye 工具默认输出已改成相对路径，更适合迁移到其他机器
- `src/so101-ros-physical-ai` 现在已经作为普通源码目录纳入仓库，不再依赖外部子仓库指针
- 中文工程文档已经统一整理到 `docs/`，避免和 ROS 包源码混放

## 常见注意事项

- 真机调试时，一次只保留一套 bringup 在运行，避免多个 `move_group`、多个 detector 同时存在。
- 如果抓取逻辑“看起来在执行，但动作奇怪”，先检查 ROS 图里是不是有重复节点。
- 如果 MoveIt 回 `rest` 偶发失败，优先检查：
  - 桌面碰撞体参数
  - 当前关节状态回读是否漂移
  - 是否使用了当前推荐的 `post_grasp_return_retry_count`
- 如果红球位置偏差明显，优先检查：
  - 相机内参
  - 手眼标定结果
  - 相机 TF
  - 视觉偏置参数，而不是先怀疑规划器

## 仓库状态

这是一个偏工程实践导向的工作区仓库，不是只保留最小功能示例。

所以仓库里除了运行必需的 ROS 包，也保留了：

- 中文操作文档
- 标定结果与标定原始样本（`calibration/handeye/`）
- 裸硬件调试脚本（`tools/hardware_debug/`）
- 部分相机配置
- 机械臂描述与模型资源

同时，已经明确归档、不参与当前主链路的内容会放到：

- `archive/`

如果你只是想找某个功能入口，最常用的路径通常是：

- 真机总入口：
  [`follower_hx35hm_moveit.launch.py`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py)
- 红球抓取：
  [`so101_visual_grasp.cpp`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping/src/so101_visual_grasp.cpp)
- HX35HM 桥接：
  [`bridge_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py)
- 红球检测：
  [`red_circle_detector_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/red_circle_detector_node.py)
- 桌面估计：
  [`table_plane_estimator_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/table_plane_estimator_node.py)
