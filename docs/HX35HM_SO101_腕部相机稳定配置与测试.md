# HX35HM SO101 腕部相机稳定配置与测试

本文档用于在不影响顶视相机的前提下，单独启动和验证腕部 UVC 相机。

## 1. 当前目标

保持顶视相机链路不变：

```text
/static_camera/image_raw
/static_camera/camera_info
cam_overhead
```

腕部相机单独接入 follower 命名空间：

```text
/follower/image_raw
/follower/camera_info
cam_wrist
```

## 2. 当前设备绑定

腕部相机使用稳定 by-id 路径：

```text
/dev/v4l/by-id/usb-USB_CAMERA_USB_CAMERA-video-index0
```

顶视相机继续使用原有稳定 by-id 路径：

```text
/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_HD_UVC_WebCam-video-index0
```

先检查设备：

```bash
ls -l /dev/v4l/by-id /dev/v4l/by-path
```

如果后续同时接入两个同型号 `USB CAMERA`，不要继续只依赖 by-id；应改用 by-path 固定腕部相机所在 USB 口。

## 3. 只启动腕部相机

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_hx35hm_wrist_only.yaml
```

检查话题：

```bash
ros2 topic list | rg '/follower/(image_raw|camera_info)'
ros2 topic echo /follower/camera_info --once
ros2 topic echo /follower/image_raw --once --field header
```

看画面：

```bash
rqt_image_view /follower/image_raw
```

## 4. 启动双 RGB 相机

正式示教采集时继续使用双相机配置：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_hx35hm_dual_rgb.yaml
```

应同时看到：

```text
/static_camera/image_raw
/static_camera/camera_info
/follower/image_raw
/follower/camera_info
```

## 5. 内参策略

当前腕部相机先使用占位内参：

```text
cam_wrist_calib_placeholder.yaml
```

这只用于稳定出图和录制示教数据。后续如果要做腕部视觉伺服、精确位姿估计或手眼标定，应单独标定腕部相机，输出真实：

```text
cam_wrist_calib.yaml
```

然后只修改腕部相机参数文件，不修改顶视相机配置。

## 6. 示教录制话题

HX35HM SO101 示教录制应包含：

```text
/static_camera/image_raw
/follower/image_raw
/follower/joint_states
/follower/learning_action
```

其中 `/follower/image_raw` 是腕部视角，`/static_camera/image_raw` 是顶视视角。
