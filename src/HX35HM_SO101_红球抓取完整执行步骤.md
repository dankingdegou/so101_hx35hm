# HX35HM + SO101 红球抓取完整执行步骤

适用对象：当前这套 `HX-35HM + SO101 follower + overhead RGBD 相机 + MoveIt`

目标：按下面步骤，完整执行一次“识别红球 -> 规划 -> 抓取 -> 抬起”

---

## 1. 先明确当前流程

当前抓取链路是：

1. `so101_bringup/follower_hx35hm_moveit.launch.py`
2. 启动机械臂桥接 `hx35hm_bridge`
3. 启动 MoveIt `move_group`
4. 启动 overhead 相机
5. 启动红球检测 `red_circle_detector`
6. `so101_grasping/so101_visual_grasp`
7. 抓取节点先回 `rest`
8. 等视觉稳定后重新取红球位姿
9. 执行 `pregrasp -> grasp -> close gripper -> retreat`

注意：

- 现在已经修过一个关键问题：不是一开始就拿旧目标抓，而是先回 `rest` 后再重新取目标。
- 如果最终仍有整体偏差，优先看相机标定和相机 TF，不要先怀疑路径规划。

---

## 2. 编译

在新终端执行：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_grasping so101_bringup so101_hx35hm_bridge --symlink-install
source ~/ros2_ws/src/install/setup.bash
```

如果你只改了抓取节点：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_grasping --symlink-install
source ~/ros2_ws/src/install/setup.bash
```

---

## 3. 启动整套抓取系统

建议先不开 RViz，减少干扰：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_red_detector:=true \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false
```

正常情况下你会看到这些关键信息：

- `FollowJointTrajectory action server ready`
- `ParallelGripperCommand action server ready`
- `Red detector started`
- `You can start planning now!`

---

## 4. 启动后先做在线检查

另开一个终端：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash
```

### 4.1 检查节点

```bash
ros2 node list
```

至少应看到：

- `/follower/robot_state_publisher`
- `/follower/hx35hm_bridge` 或桥接相关节点
- `/move_group`
- `/red_circle_detector`

### 4.2 检查话题

```bash
ros2 topic list | rg '/vision|/static_camera|/joint_states'
```

至少应看到：

- `/static_camera/image_raw`
- `/static_camera/depth/image_raw`
- `/static_camera/depth/camera_info`
- `/vision/red_block/debug_image`
- `/vision/red_block/pose_base`

### 4.3 检查 action

```bash
ros2 action list
```

至少应看到：

- `/follower/arm_trajectory_controller/follow_joint_trajectory`
- `/follower/gripper_controller/gripper_cmd`
- `/move_action`

---

## 5. 抓取前先确认“视觉看到的是不是红球”

### 5.1 看红球位姿是否在持续发布

```bash
ros2 topic echo /vision/red_block/pose_base
```

正常现象：

- 有持续输出
- `frame_id` 是 `base_link`
- `z` 大致在桌面高度附近

异常现象：

- 完全没有输出
- 偶尔输出但很跳
- `z` 明显离谱

### 5.2 存一张调试图

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
        super().__init__('save_red_debug_once')
        self.done = False
        self.sub = self.create_subscription(Image, '/vision/red_block/debug_image', self.cb, 10)
    def cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/tmp/red_block_debug.png', img)
        self.done = True

rclpy.init()
node = Saver()
while rclpy.ok() and not node.done:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
print('/tmp/red_block_debug.png')
PY
```

然后查看图片：

```bash
xdg-open /tmp/red_block_debug.png
```

你要重点看：

- 绿色圆圈是不是套在红球上
- 十字标记是不是落在红球中心
- 有没有把机械臂本体的橙色部件当成目标

---

## 6. 执行一次实际抓取

确认红球摆放好、机械臂周围无碰撞风险后，再执行：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_grasping so101_visual_grasp.launch.py execute:=true
```

当前默认行为：

1. 打开夹爪
2. 机械臂回 `rest`
3. 等待视觉稳定
4. 重新采样红球位姿
5. 走到 `pregrasp`
6. 下探到 `grasp`
7. 闭合夹爪
8. 回到 `pregrasp`

正常日志关键字：

- `Opening gripper...`
- `Moving to rest pose first...`
- `Waiting 1.00s for vision to settle after rest...`
- `Using N pose samples`
- `Target pose: ...`
- `Moving to pregrasp...`
- `Moving to grasp...`
- `Closing gripper...`
- `Retreating...`
- `Visual grasp sequence completed.`

---

## 7. 如果你只想试规划，不想真的动

```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py execute:=false
```

这会只做规划，不执行机械臂和夹爪动作。

---

## 8. 当前默认参数

当前这版默认参数大意如下：

- `pose_topic=/vision/red_block/pose_base`
- `pose_sample_window_s=0.35`
- `pose_sample_count=5`
- `max_pose_spread_m=0.03`
- `post_rest_settle_s=1.0`
- `pregrasp_offset_m=0.08`
- `grasp_z_offset_m=-0.015`

如果要临时试更低一点的抓取高度：

```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=true \
  grasp_z_offset_m:=-0.020
```

如果要临时试更高一点的预抓：

```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=true \
  pregrasp_offset_m:=0.10
```

如果要临时试平面补偿：

```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=true \
  grasp_x_offset_m:=0.010 \
  grasp_y_offset_m:=-0.005
```

---

## 9. 抓偏时按这个顺序排查

### 9.1 先看是不是视觉目标错了

如果调试图里圈到的不是红球，而是机械臂本体或别的红色物体：

- 先把机械臂移回 `rest`
- 再看 `/vision/red_block/debug_image`
- 必要时进一步收紧 HSV 阈值

### 9.2 再看是不是相机标定误差

当前 overhead RGB 相机还在用占位内参文件：

- `so101_bringup/config/cameras/cam_overhead_calib_placeholder.yaml`

这意味着：

- 视觉位置可以大致用
- 但绝对抓取精度不能保证

### 9.3 再看是不是相机 TF 偏了

当前 camera 到 `base_link` 的外参是手工估值，在：

- `so101_bringup/launch/camera_tf_moveit.launch.py`

默认值：

- `camera_x=-0.15`
- `camera_y=0.0`
- `camera_z=0.80`

如果抓取整体总是向某个固定方向偏，优先调这里。

示例：

```bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_red_detector:=true \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  camera_x:=-0.13 \
  camera_y:=0.01 \
  camera_z:=0.78
```

### 9.4 最后才看规划

只有在下面情况，才优先怀疑规划：

- 日志里有 `Planning failed`
- 机械臂绕路明显异常
- 到不了 `pregrasp`
- 到不了 `grasp`

如果日志是 `Target pose` 本身就不对，但规划一路成功，那不是规划问题。

---

## 10. 推荐的实际操作顺序

每次开机后，建议固定按这个顺序：

1. 编译并 `source`
2. 启动 `follower_hx35hm_moveit.launch.py`
3. 检查节点、话题、action
4. 看 `/vision/red_block/debug_image`
5. 看 `/vision/red_block/pose_base`
6. 先 `execute:=false` 做一次规划验证
7. 再 `execute:=true` 做真实抓取

---

## 11. 一键执行版

如果当前系统已经稳定，最常用就是这两条：

终端 1：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_red_detector:=true \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false
```

终端 2：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_grasping so101_visual_grasp.launch.py execute:=true
```

---

## 12. 结束后关闭

停止抓取节点：

```bash
Ctrl+C
```

停止 bringup：

```bash
Ctrl+C
```

如果 ROS 图还残留，重启 daemon：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

---

## 13. 当前结论

当前这版已经修复：

- 旧位姿直接拿来抓的问题
- 当前机械臂姿态干扰红球检测的问题
- 单帧位姿抖动直接触发抓取的问题

当前仍然可能影响最终精度的主要因素：

1. overhead RGB 相机还不是实标定内参
2. `cam_overhead -> base_link` 仍是手调 TF

所以：

- 如果是“抓取流程乱走”，先看日志
- 如果是“总是整体偏一点”，优先做相机标定和 TF 标定

