# HX35HM + SO101 摄像头调位与可视化执行步骤

适用对象：当前这套 `HX-35HM + SO101 follower + overhead Astra RGBD 相机`

目的：在正式做手眼标定或红球抓取前，先把 overhead 摄像头的位置调到合适状态，并且用可视化窗口实时观察：

- RGB 画面
- 深度画面
- 点云
- 机器人本体是否遮挡视野
- 工作区是否完整落在画面内

---

## 1. 先明确这次要观察什么

调摄像头位置时，重点不是先抓球，而是先看下面几件事：

- 桌面抓取区域是否完整进入画面
- 机械臂在左、中、右三个典型位置时是否经常遮挡目标区
- 末端 `ArUco` 码在大多数姿态下是否容易被看见
- 深度图在桌面区域是否连续、没有大面积空洞
- 点云是否大致落在桌面上方，而不是明显漂浮或塌陷

如果这一步没调好，后面的手眼标定和抓球都会反复出问题。

---

## 2. 推荐使用的可视化组合

这份流程默认同时打开 3 类窗口：

- `RViz`：看三维点云、TF、机器人模型
- `rqt_image_view` 第 1 个窗口：看 RGB
- `rqt_image_view` 第 2 个窗口：看深度

其中：

- `RViz` 适合判断视野、遮挡、相机 TF 和点云是否合理
- `RGB` 窗口适合调相机朝向和画面范围
- `Depth` 窗口适合看桌面是否有大面积测不到

---

## 3. 相关话题和配置

你当前这套 overhead 相机默认使用：

- RGB：`/static_camera/image_raw`
- RGB 相机内参：`/static_camera/camera_info`
- Depth：`/static_camera/depth/image_raw`
- Depth 相机内参：`/static_camera/depth/camera_info`
- 点云：`/static_camera/points`

当前 RGBD 相机组合配置文件：

- [so101_cameras_astra_overhead_rgbd.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml)

当前 overhead RGB 相机参数文件：

- [so101_gs_cam_astra_overhead.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_gs_cam_astra_overhead.yaml)

现成的 overhead 调试 RViz 配置：

- [vision_overhead_debug.rviz](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/rviz/vision_overhead_debug.rviz)

---

## 4. 编译并 source

先开一个终端，执行：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_bringup so101_hx35hm_bridge so101_openni2_camera --symlink-install
source ~/ros2_ws/src/install/setup.bash
```

如果你这次没有改代码，理论上可以不重新编译，但现场执行时还是建议先 `source` 一次工作区。

---

## 5. 启动推荐模式

推荐使用“完整调试模式”，这样 `RViz` 里能同时看到：

- 机器人模型
- TF
- RGB 图像
- 深度图像
- 点云

### 5.1 终端 1：启动机器人 + 相机，不开额外 detector

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

正常情况下你应该能看到：

- `FollowJointTrajectory action server ready`
- `ParallelGripperCommand action server ready`
- `move_group` 启动成功
- `static_camera` 下的 RGB / depth 节点启动成功

这一步启动后不要关，后面的所有窗口都依赖它。

---

## 6. 检查相机话题是否正常

### 6.1 终端 2：检查话题

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 topic list | rg '/static_camera|/follower'
```

至少应看到：

- `/static_camera/image_raw`
- `/static_camera/camera_info`
- `/static_camera/depth/image_raw`
- `/static_camera/depth/camera_info`
- `/static_camera/points`

### 6.2 看图像频率

```bash
ros2 topic hz /static_camera/image_raw
ros2 topic hz /static_camera/depth/image_raw
```

只要能持续输出，频率不必绝对一致。

### 6.3 看消息头是否正常

```bash
ros2 topic echo /static_camera/camera_info --once
ros2 topic echo /static_camera/depth/camera_info --once
```

你主要确认：

- `frame_id` 不为空
- 宽高符合当前相机设置
- 消息能正常收到

---

## 7. 打开可视化窗口

### 7.1 终端 3：打开 RViz

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

rviz2 -d ~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/rviz/vision_overhead_debug.rviz
```

这个 RViz 配置默认会显示：

- `/static_camera/image_raw`
- `/static_camera/depth/image_raw`
- `/static_camera/points`
- `TF`
- `/follower/robot_description`

如果一切正常，你应该能在一个窗口里同时看到图像、点云和机器人模型。

### 7.2 终端 4：打开 RGB 图像窗口

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

rqt_image_view /static_camera/image_raw
```

### 7.3 终端 5：打开 Depth 图像窗口

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

rqt_image_view /static_camera/depth/image_raw
```

如果 `rqt_image_view` 打开后没有自动选中话题，就在窗口左上角的 topic 下拉框里手动选：

- RGB 选 `/static_camera/image_raw`
- Depth 选 `/static_camera/depth/image_raw`

---

## 8. 调机位时要怎么观察

调整摄像头物理位置时，建议一边看 `RGB` 窗口，一边看 `RViz`。

### 8.1 RGB 画面合格标准

- 抓取工作区完整出现在画面里
- 工作区尽量落在画面中间区域，不要贴边
- 桌面边缘不要切掉关键区域
- 机械臂在常用姿态下不要长期挡住工作区中心
- 光照均匀，没有严重反光和过曝

### 8.2 Depth 画面合格标准

- 桌面区域连续可见
- 目标区没有大块黑洞或空白
- 机器人前方的桌面不是大片无深度
- 机械臂运动时深度不会频繁整片闪烁

### 8.3 RViz 合格标准

- 点云大致落在桌面位置
- 点云不是整体倾斜得很夸张
- 机器人模型与真实机械臂朝向大致一致
- 相机视野覆盖你准备抓取的主要区域

---

## 9. 推荐的实际调位动作

调位时不要只看机械臂静止在一个姿态。

建议你在现场至少观察这 5 种状态：

- 机械臂收回 `rest` 位
- 机械臂伸向工作区中央
- 机械臂偏左工作位
- 机械臂偏右工作位
- 末端抬高并带一点 wrist 旋转

你要确认：

- 这几个姿态下相机都还能看清桌面关键区域
- 末端未来贴上 `ArUco` 后不会频繁被挡住
- 抓球区域不会总被机械臂本体盖住

如果你想边看边动机械臂，可以额外开 joint GUI：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/scripts/so101_joint_gui.py \
  --command-topic /follower/forward_controller/commands \
  --joint-state-topic /joint_states
```

注意：

- 手动拖动时动作要慢
- 不要一边大幅运动一边直接掰摄像头支架
- 调完摄像头再做手眼标定

---

## 10. 推荐机位判断标准

如果你只是为了尽快进入下一步，可以先按这个标准判断机位是否“够用”：

- 相机距桌面大约 `0.7m` 到 `0.9m`
- 光轴大体朝下
- 工作区落在画面中心 `60%` 到 `80%` 区域
- 机械臂从左到右运动时不会长时间遮挡工作区中心
- 末端未来装 `30mm` ArUco 后，大多数姿态都还能看清

只要满足这几个条件，就可以继续做内参检查和手眼标定。

---

## 11. 可选：保存一张当前 RGB / Depth 截图

如果你想留一份当前机位记录，可以用下面的方法各保存一张图。

### 11.1 保存 RGB 图

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class Saver(Node):
    def __init__(self):
        super().__init__('save_rgb_once')
        self.done = False
        self.sub = self.create_subscription(Image, '/static_camera/image_raw', self.cb, 10)
    def cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/tmp/static_camera_rgb.png', img)
        self.done = True

rclpy.init()
node = Saver()
while rclpy.ok() and not node.done:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
print('/tmp/static_camera_rgb.png')
PY
```

### 11.2 保存 Depth 图

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

class Saver(Node):
    def __init__(self):
        super().__init__('save_depth_once')
        self.done = False
        self.sub = self.create_subscription(Image, '/static_camera/depth/image_raw', self.cb, 10)
    def cb(self, msg):
        depth = bridge.imgmsg_to_cv2(msg)
        depth = depth.astype(np.float32)
        valid = depth > 0
        if np.any(valid):
            mn = np.min(depth[valid])
            mx = np.max(depth[valid])
            norm = np.zeros_like(depth, dtype=np.uint8)
            if mx > mn:
                norm[valid] = ((depth[valid] - mn) / (mx - mn) * 255.0).astype(np.uint8)
            color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
        else:
            color = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
        cv2.imwrite('/tmp/static_camera_depth.png', color)
        self.done = True

rclpy.init()
node = Saver()
while rclpy.ok() and not node.done:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
print('/tmp/static_camera_depth.png')
PY
```

查看：

```bash
xdg-open /tmp/static_camera_rgb.png
xdg-open /tmp/static_camera_depth.png
```

---

## 12. 常见问题排查

### 12.1 `rqt_image_view` 打开了但没图

先确认话题是否真的在发：

```bash
ros2 topic hz /static_camera/image_raw
ros2 topic hz /static_camera/depth/image_raw
```

再确认窗口里选中了正确 topic。

### 12.2 RViz 里没有点云

检查：

```bash
ros2 topic hz /static_camera/points
```

如果没有输出，说明 depth / OpenNI2 这一路没有正常起来。

### 12.3 只有 RGB，没有 Depth

优先检查：

- Astra 深度设备是否被系统识别
- `openni2_camera_node` 是否启动成功
- USB 带宽和供电是否稳定

### 12.4 画面有，但标定后还是不准

这通常不是“相机画面没起来”的问题，而是：

- RGB 内参还没换成真实标定结果
- 相机位置调完后又被动过
- 手眼标定样本分布不够好
- RGB 和 depth 本来就不是严格同一成像链路

---

## 13. 调位完成后的下一步

顺序建议是：

1. 先把摄像头位置固定死
2. 再确认 RGB 内参是正确的
3. 然后做 `ArUco` 手眼标定
4. 最后再回到红球抓取调试

不要在手眼标定做完之后再去挪摄像头位置，否则外参就失效了。

---

## 14. 收尾关闭

关闭顺序没有硬要求，通常直接在各终端里按：

```bash
Ctrl+C
```

如果你只是想暂时结束可视化，先关：

- 两个 `rqt_image_view`
- `RViz`

最后再关启动相机和机器人 bringup 的那个主终端。
