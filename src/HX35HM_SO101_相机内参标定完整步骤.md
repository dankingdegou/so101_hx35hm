# HX35HM + SO101 相机内参标定完整步骤

适用对象：当前这套 `HX-35HM + SO101 follower + overhead RGB 相机`

目的：完成 overhead RGB 相机的内参标定，生成真实 `camera_info`，替换当前占位参数。

这份流程对应我刚补好的本地标定脚本：

- [calibrate_ros_camera_intrinsics.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/tools/camera_intrinsics/calibrate_ros_camera_intrinsics.py)

标定完成后，输出会直接写到：

- [cam_overhead_calib.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml)

当前 overhead RGB 相机配置已经改成读取这个文件：

- [so101_gs_cam_astra_overhead.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_gs_cam_astra_overhead.yaml)

---

## 1. 先准备标定板

推荐先用最稳妥的棋盘格方案：

- 棋盘格内部角点：`9 x 6`
- 每个小方格边长：`20mm`
- 打印在平整硬纸板或泡沫板上
- 整块板不能弯、不能反光、不能卷边

注意：

- 脚本里的 `--cols 9 --rows 6 --square-size-mm 20` 指的是内部角点和单格尺寸
- 如果你打印的不是 `20mm`，命令里的 `--square-size-mm` 必须同步改

---

## 2. 先确认当前相机配置

这次标定的是 overhead RGB，不是 depth。

当前图像话题默认是：

- `/static_camera/image_raw`

当前真实内参输出文件：

- [cam_overhead_calib.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml)

这意味着你标定完成后，不需要再改 `camera_info_url`，只需要重启相机节点即可生效。

---

## 3. 编译并 source

先开一个终端：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_bringup --symlink-install
source ~/ros2_ws/src/install/setup.bash
```

---

## 4. 启动 overhead RGB 相机

建议先只启动相机，不开额外 detector：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_red_detector:=false
```

如果你只想看相机，不关心机械臂 bringup，也可以直接起 cameras：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml
```

---

## 5. 先确认图像话题正常

另开一个终端：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash
```

检查：

```bash
ros2 topic hz /static_camera/image_raw
ros2 topic echo /static_camera/camera_info --once
```

只要 RGB 图像在稳定发布，就可以继续。

---

## 6. 启动内参标定脚本

再开一个终端，执行：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/tools/camera_intrinsics/calibrate_ros_camera_intrinsics.py \
  --image-topic /static_camera/image_raw \
  --camera-name cam_overhead \
  --cols 9 \
  --rows 6 \
  --square-size-mm 20 \
  --min-samples 18 \
  --output ~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml
```

脚本窗口打开后你会看到实时图像和状态文字。

热键：

- `S`：采集一组样本
- `C`：开始标定并保存
- `U`：标定后切换去畸变预览
- `Q`：退出

---

## 7. 采样时怎么拿棋盘格

这是成败关键。

你不要只在画面中间正对着拍一堆几乎一样的图，而是要故意做“丰富分布”的姿态。

至少覆盖这些情况：

- 棋盘格在画面中央
- 棋盘格在画面左上、右上、左下、右下
- 棋盘格离相机近一些、远一些
- 棋盘格有轻微俯仰、轻微旋转
- 棋盘格占画面面积有大有小

采样原则：

- 每采一张都换一个明显不同的位置或角度
- 不要连续采几乎一样的姿态
- 棋盘格必须完整可见
- 不要严重模糊
- 不要反光过曝

经验上：

- 至少采 `18` 到 `25` 组
- 如果你画面边缘畸变较明显，尽量多采一些边角位置

---

## 8. 什么时候按 `S`

当你看到：

- 棋盘格角点已经被稳定识别
- 棋盘格完整出现在画面里
- 当前姿态和前一张差异明显

这时再按 `S`。

如果脚本提示：

- `没有检测到棋盘格`

说明当前姿态不合格。

如果提示：

- `和上一帧视角太像了`

说明你采样重复了，需要把棋盘格移动到明显不同的位置后再采。

---

## 9. 完成标定并保存

当你已经采够足够多的样本后，在脚本窗口按：

```text
C
```

脚本会：

- 调用 OpenCV `calibrateCamera`
- 计算相机矩阵和畸变参数
- 输出 RMS 和平均重投影误差
- 保存到：
  [cam_overhead_calib.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml)

如果结果正常，终端里会打印类似：

- `RMS=... px`
- `mean_reproj=... px`
- `已保存到 .../cam_overhead_calib.yaml`

---

## 10. 怎么判断这次标定结果够不够好

先看两个指标：

- `RMS`
- `mean_reproj`

经验判断：

- `< 0.5 px`：很好
- `0.5 ~ 1.0 px`：通常可用
- `> 1.0 px`：建议重新采样再做一次

如果误差偏大，常见原因是：

- 棋盘格打印不准
- 棋盘格板子不平
- 采样姿态太重复
- 图像模糊
- 画面边缘样本太少

---

## 11. 用去畸变预览做快速复查

标定完成后，在脚本窗口按：

```text
U
```

如果去畸变后的画面看起来：

- 桌面边线更直
- 边缘拉伸感变小
- 整体视觉更自然

说明这次结果通常是合理的。

---

## 12. 重启相机让新内参生效

保存完成后，关闭当前相机 launch，然后重新启动相机。

因为当前配置已经指向：

- [cam_overhead_calib.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml)

所以重启后新的 `camera_info` 就会被读进去。

---

## 13. 生效后怎么验证

重启相机后，再开一个终端检查：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 topic echo /static_camera/camera_info --once
```

你要确认：

- `k` 不再是占位值
- `d` 不再全是 0
- 宽高和当前图像一致

如果你看到的还是老参数，通常是：

- 没重启相机节点
- 启动的不是这份 overhead 配置文件
- 输出文件没写到正确路径

---

## 14. 标定完成后的正确顺序

推荐接下来这样走：

1. 固定住相机，不要再动
2. 确认新内参已经生效
3. 再做 `ArUco` 手眼标定
4. 最后重新验证红球抓取

不要在完成手眼标定后又去挪相机位置。

---

## 15. 常见问题

### 15.1 没检测到棋盘格

优先检查：

- 棋盘格尺寸参数是否填对
- 棋盘格是不是完整进入画面
- 光照是否太暗或反光
- 是否有运动模糊

### 15.2 标定误差很大

优先重做这几件事：

- 增加边缘和角落样本
- 增加近距离和远距离样本
- 去掉模糊样本
- 不要拿弯曲的纸直接标定

### 15.3 结果保存了，但启动后没变化

检查：

- [so101_gs_cam_astra_overhead.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_gs_cam_astra_overhead.yaml)
- [cam_overhead_calib.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml)

确认 `camera_info_url` 指向的就是新输出文件。
