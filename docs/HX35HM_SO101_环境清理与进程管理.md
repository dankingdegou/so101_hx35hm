# HX35HM + SO101 环境清理与进程管理

目标：在每次测试前后快速清理 ROS 进程，避免“重复节点、设备占用、命令互相干扰”。

---

## 1. 为什么要清理

你的项目最常见的干扰来自：

- 重复启动多套 `follower_hx35hm_moveit.launch.py`
- 上次的 `so101_visual_grasp` 没退出干净
- `gscam/openni2` 占着相机设备（`Resource busy`）
- ROS daemon 缓存导致 `node list` 看起来不干净

---

## 2. 日常快速清理（推荐）

### 2.1 只停抓取节点（保留主栈）

```bash
pkill -f 'so101_visual_grasp' || true
```

### 2.2 停整套主栈（抓取 + bringup + move_group + 相机）

```bash
pkill -f 'follower_hx35hm_moveit.launch.py|so101_visual_grasp|move_group|cartesian_motion_node|red_circle_detector|table_plane_estimator|gscam_node|openni2_camera_node|hx35hm_bridge' || true
```

### 2.3 重启 ROS daemon（清图缓存）

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

---

## 3. 深度清理（设备被占用时）

如果你看到：

- `openni2_camera_node ... Resource busy`
- `gscam` 启动后立即退出

先查占用：

```bash
lsof /dev/video0 /dev/video1 /dev/video2 /dev/video3 2>/dev/null
```

强制清理相机相关进程：

```bash
pkill -9 -f 'gscam_node|openni2_camera_node|cam_overhead|depth_overhead' || true
```

再查一次：

```bash
lsof /dev/video0 /dev/video1 /dev/video2 /dev/video3 2>/dev/null
```

无输出表示设备释放成功。

---

## 4. 启动前检查清单（30 秒）

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 4.1 看关键进程是否重复

```bash
ps -ef | rg 'follower_hx35hm_moveit.launch.py|hx35hm_bridge|cartesian_motion_node|move_group|red_circle_detector|table_plane_estimator|so101_visual_grasp'
```

原则：

- 同类关键进程只保留一套

### 4.2 看 ROS 图

```bash
ros2 node list
```

不应长期看到重复同名节点警告。

### 4.3 看关键服务

```bash
ros2 service list | rg '/go_to_pose|/go_to_joints'
```

这两个服务都在，抓取链才完整。

---

## 5. 一键“测试前清场 + 重启主栈”

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

pkill -f 'follower_hx35hm_moveit.launch.py|so101_visual_grasp|move_group|cartesian_motion_node|red_circle_detector|table_plane_estimator|gscam_node|openni2_camera_node|hx35hm_bridge' || true
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_red_detector:=true \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_vision_debug_rviz:=false \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml
```

---

## 6. 常见现象与处理

### 现象 A：抓取节点在跑，但机械臂不按预期动

处理：

1. 检查是否存在第二套 `move_group` 或 `hx35hm_bridge`
2. 清理全部主栈进程后重启

### 现象 B：相机有图但红球位姿没输出

处理：

1. 查 `red_circle_detector` 是否在
2. 查 `/vision/red_block/pose_base` 是否在发布
3. 必要时重启主栈

### 现象 C：`Resource busy`

处理：

1. `lsof /dev/video*`
2. `pkill -9 -f 'gscam_node|openni2_camera_node|cam_overhead|depth_overhead'`
3. 重启 bringup

