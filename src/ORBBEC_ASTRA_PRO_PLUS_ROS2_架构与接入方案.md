# 奥比中光 Astra Pro Plus（OpenNI2）接入 SO101 ROS2 工程的架构方案

日期：2026-03-17  
适用：Ubuntu 24.04 + ROS 2 Jazzy + Astra Pro Plus（OpenNI2 2.x）  

你当前这台相机的现状（从你的配置日志）可以总结为一句话：

- **深度/红外走 OpenNI2（`libOpenNI2.so + liborbbec.so`）**
- **RGB 走标准 UVC（`/dev/video*`，用 `gscam/usb_cam/v4l2_camera` 都能出图）**

因此最稳的工程化集成方式不是强行把它当成“一个 RGB-D 驱动”，而是把它拆成两条链路，然后在 ROS2 层统一接口。

---

## 1. 目标：接入后你希望得到什么

在 SO101 工程里，建议把“相机输出”统一成这套最小接口（每个相机一个 namespace）：

1. RGB：
   - `/<ns>/image_raw`
   - `/<ns>/camera_info`
2. Depth：
   - `/<ns>/depth/image_raw`（`16UC1`，单位毫米，OpenNI2 原生就是 mm）
   - `/<ns>/depth/camera_info`
3. 点云（给 MoveIt/避障/建图）：
   - `/<ns>/<depth_node>/points`（`PointCloud2`，XYZ 单位米）
4. TF：
   - `world -> static_camera/cam_overhead`（顶视相机外参）
   - `follower/<ee_link> -> follower/cam_wrist`（腕部相机外参）

这样做的直接收益是：

- 你原工程里 `so101_inference/episode_recorder` 默认订阅的 RGB 话题（`/static_camera/image_raw`、`/follower/image_raw`）完全不用改。
- 深度新增的 `/.../depth/*` 和 `/.../points` 可以独立给 MoveIt 或其它感知模块用，不会干扰现有链路。

---

## 2. 推荐总体架构（最稳的“分层”）

### 2.1 Driver 层（设备接入）

1. **RGB（UVC）**
   - 用你工程里已经在用的 `gscam`（或 `usb_cam`/`v4l2_camera`）发布 `/<ns>/image_raw`。
2. **Depth/PointCloud（OpenNI2）**
   - 用 OpenNI2 C++ API 直接读 `SENSOR_DEPTH`，发布：
     - `/<ns>/depth/image_raw`
     - `/<ns>/points`

你日志里提到 `ColorReaderPoll` 会报 “Couldn't start the depth stream”，这类冲突在 Astra 上很常见（OpenNI2 的彩色流和深度流并发容易出幺蛾子）。  
所以工程上建议：**RGB 走 UVC，深度走 OpenNI2，彼此解耦。**

### 2.2 Interface/Normalization 层（统一话题接口）

SO101 工程里已经有统一的 “camera bringup”：

- `so101_bringup/launch/cameras.launch.py` 从 `so101_bringup/config/cameras/*.yaml` 读取相机列表并启动。

我已经在你的工程里补了一种新的 `camera_type`：

- `so101_openni2_camera`：启动自研的 OpenNI2 深度节点（下面第 3 节）

这样你只要换一份 cameras 配置，就能把 Astra 纳入工程的统一 bringup。

### 2.3 TF/Calibration 层（外参最重要）

你的工程已经有 `so101_bringup/launch/camera_tf.launch.py` 用静态 TF 给了相机挂载点。

建议原则：

- **先保证 TF 链路连通**（点云能正确落到 `world` 或 `follower/base`）。
- 精度需求上来后，再做：
  - 深度相机相对挂载点的精确外参（毫米级）
  - RGB 与 Depth 的内外参标定与对齐（如果你确实要“像素级 RGB-D 对齐”）

---

## 3. 你工程里已经补好的实现（可直接用）

### 3.1 新增 ROS2 包：`so101_openni2_camera`

位置：

- `so101-ros-physical-ai/so101_openni2_camera`

可执行文件：

- `openni2_camera_node`

发布话题（在所在 namespace 下）：

- `depth/image_raw`：`sensor_msgs/Image`，`encoding=16UC1`，单位 mm
- `depth/camera_info`：`sensor_msgs/CameraInfo`（由 OpenNI2 的 FOV 估算出的 pinhole 模型，够用但不等价于精确标定）
- `points`：`sensor_msgs/PointCloud2`，XYZ 单位 m（点的坐标用 OpenNI2 的 `convertDepthToWorld` 生成，避免手写内参）。在本工程默认配置下点云话题是 `/static_camera/depth_overhead/points`。

参数文件（bringup 侧）：

- `so101-ros-physical-ai/so101_bringup/config/cameras/so101_openni2.yaml`

### 3.2 新增 cameras 列表：顶视 Astra（RGB + Depth + 点云）

- `so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml`

它会启动两个节点：

1. `static_camera/cam_overhead`（RGB，`gscam`）
2. `static_camera/depth_overhead`（Depth+点云，OpenNI2）

---

## 4. 你该怎么启动（最小可用流程）

1. 编译（你已经装了 `libopenni2-dev`，所以能直接编）

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_openni2_camera
source install/setup.bash
```

2. 启动顶视相机 RGB + Depth

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml
```

### 4.1 如果你想要“更稳的 30Hz”（USB2 带宽受限时强烈建议）

Astra Pro Plus 枚举为 USB2（480M），在 **RGB 640x480@30 + Depth 640x480@30** 同时跑时，实际有效帧率很容易被总线限制到 10-20Hz。

我给你准备了一套低带宽配置，把 RGB 和 Depth 都改为 320x240@30，同时保留 10Hz 点云：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd_lowbw.yaml
```

对应文件：

- `so101_cameras_astra_overhead_rgbd_lowbw.yaml`
- `so101_gs_cam_astra_overhead_320.yaml`
- `so101_openni2_320.yaml`

3. 检查话题是否出来（预期）

- RGB：
  - `/static_camera/image_raw`
- Depth：
  - `/static_camera/depth/image_raw`
  - `/static_camera/depth_overhead/points`

---

## 5. 频率与性能建议（主流做法）

1. Depth 图像：
   - 640x480@30Hz：大多数场景够用，CPU 压力不大
2. 点云：
   - 建议 5-10Hz
   - `pointcloud_stride=4`（默认）：点数直接降到 1/16，实用且省
3. MoveIt 环境更新（如果你后面要做 PlanningScene/Octomap）：
   - 2-10Hz 都常见
   - 先从 5Hz 开始，稳定后再加

---

## 6. 你日志里提到的“彩色与深度冲突”，工程上怎么处理

你已经验证：

- OpenNI2 深度/红外/点云：OK
- RGB 用 UVC：OK
- OpenNI2 同时跑彩色+深度：有冲突风险

所以推荐的长期稳定方案就是：

1. RGB 永远用 UVC（`gscam/usb_cam/v4l2_camera`）
2. 深度/点云永远用 OpenNI2
3. 如果你确实需要“像素对齐的 RGB-D”：
   - 后续再做一次系统化的 RGB-Depth 对齐标定和同步策略

---

## 7. 后续可选增强（按你需求选）

1. 如果你要把点云接入 MoveIt 做场景闭环：
   - 我可以在 `so101_moveit_config` 里补一份 `sensors_3d.yaml`（pointcloud->octomap），并在 move_group launch 里挂上。
2. 如果你要两路相机（顶视 + 腕部）都换成 Astra：
   - 我可以再补一份 `so101_cameras_astra_duo_rgbd.yaml`，并把 `device_uri` 做成可配置（按序列号/URI 绑定）。
