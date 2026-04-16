# HX35HM + SO101 重新手眼标定步骤

适用对象：当前这套 `HX-35HM + SO101 follower + overhead 相机`

目标：重新得到更可靠的 `base_link -> cam_overhead` 外参，替换当前手工估值

当前仓库已经有一套现成工具链，推荐直接用：

- `tools/handeye/collect_aruco_handeye_samples.py`
- `tools/handeye/auto_motion_aruco_sampler.py`
- `tools/handeye/solve_aruco_handeye.py`

---

## 1. 先明确这次标定的模型

当前这套脚本做的是：

- **固定 overhead 相机**
- **ArUco 码固定在夹爪/末端上**
- 机器人在多个姿态下运动
- 相机持续观测 ArUco
- 求解 `base_link -> cam_overhead`

也就是说，这不是“看桌面标定板”的流程，而是：

1. 把 ArUco 固定在机械臂末端
2. 机械臂换很多姿态
3. 每个姿态都记录：
   - `marker_in_camera`
   - `tool_in_base`
4. 最后解出相机外参

---

## 2. 标定前准备

### 2.1 机械要求

- 末端固定一张 **ArUco 码**
- 固定必须刚性，不能晃
- 码面尽量平整，不要卷曲
- 码不要被夹爪结构挡住

### 2.2 推荐做法

- 先用一张 `30mm` 边长的 ArUco
- 贴在夹爪正前方或侧前方
- 保证 overhead 相机从大多数姿态都能看到
- 如果你改了打印尺寸，检测参数里的 `marker_size_m` 也必须同步修改

### 2.3 环境要求

- 光照稳定
- 桌面不要有反光
- 相机固定牢，不要中途动
- 标定过程中不要再挪动机械臂底座

---

## 3. 编译并 source

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
colcon build --packages-select so101_grasping so101_bringup so101_hx35hm_bridge --symlink-install
source ~/ros2_ws/src/install/setup.bash
```

---

## 4. 启动用于手眼标定的系统

建议只开 ArUco 检测，不开红球检测：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_aruco_detector:=true \
  use_red_detector:=false \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false
```

正常情况下你应看到：

- `FollowJointTrajectory action server ready`
- `ParallelGripperCommand action server ready`
- `Red detector` 不启动
- ArUco detector 启动
- `You can start planning now!`

---

## 5. 启动后先检查 ArUco 检测

另开一个终端：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash
```

### 5.1 看话题是否存在

```bash
ros2 topic list | rg '/vision/aruco|/static_camera'
```

至少应看到：

- `/vision/aruco/pose_camera`
- `/vision/aruco/pose_base`
- `/vision/aruco/debug_image`

### 5.2 看 ArUco 位姿是否稳定

```bash
ros2 topic echo /vision/aruco/pose_camera
```

要求：

- 持续有输出
- `frame_id` 正常
- 位姿不是乱跳

### 5.3 存一张调试图

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
        super().__init__('save_aruco_debug_once')
        self.done = False
        self.sub = self.create_subscription(Image, '/vision/aruco/debug_image', self.cb, 10)
    def cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/tmp/aruco_debug.png', img)
        self.done = True

rclpy.init()
node = Saver()
while rclpy.ok() and not node.done:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
print('/tmp/aruco_debug.png')
PY
```

看图：

```bash
xdg-open /tmp/aruco_debug.png
```

确认：

- ArUco 框选正确
- ID 正常
- 没有频繁丢失

如果这一步不稳定，不要继续采样。

---

## 6. 方案 A：手动采样

适合第一次重做，最稳。

### 6.1 启动采样脚本

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/tools/handeye/collect_aruco_handeye_samples.py \
  --pose-topic /vision/aruco/pose_camera \
  --debug-image-topic /vision/aruco/debug_image \
  --base-frame base_link \
  --tool-frame moving_jaw_so101_v1_link \
  --out ~/ros2_ws/aruco_handeye_samples.json
```

脚本界面里：

- `S`：采样
- `Q`：保存并退出

### 6.2 手动采样原则

至少采 **20 到 30 组**，越分散越好。

姿态分布建议：

- 左、中、右
- 高、中、低
- 夹爪有不同 yaw / roll
- 不同距离相机远近

不要只在一个小区域采样。

不要采这些姿态：

- ArUco 只露出一角
- 夹爪挡住大半个码
- 靠近奇异位形
- 机械臂快碰桌子

### 6.3 手动采样完成后

你会得到：

```bash
~/ros2_ws/aruco_handeye_samples.json
```

---

## 7. 方案 B：自动采样

适合在桥接和关节控制已经稳定后使用。

### 7.1 启动自动采样

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/tools/handeye/auto_motion_aruco_sampler.py \
  --command-topic /follower/forward_controller/commands \
  --joint-state-topic /follower/joint_states \
  --pose-topic /vision/aruco/pose_camera \
  --base-frame base_link \
  --tool-frame moving_jaw_so101_v1_link \
  --samples-target 30 \
  --out ~/ros2_ws/aruco_handeye_samples.json
```

### 7.2 自动采样前提

- `/follower/forward_controller/commands` 能正常驱动机械臂
- `/follower/joint_states` 正常
- `/vision/aruco/pose_camera` 稳定

### 7.3 自动采样注意

- 先盯着机械臂跑几组，确认不会撞
- 发现姿态太激进，马上 `Ctrl+C`
- 自动采样生成的是“覆盖更多姿态”的样本，不是保证每组都有效

如果自动采样效果不好，就回到手动采样。

---

## 8. 求解手眼标定结果

不管你是手动还是自动采样，求解命令都一样：

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/tools/handeye/solve_aruco_handeye.py \
  --samples ~/ros2_ws/aruco_handeye_samples.json \
  --out ~/ros2_ws/aruco_handeye_result.json \
  --base-frame base_link \
  --camera-frame cam_overhead
```

成功后会输出：

- `base->camera xyz`
- `base->camera rpy`
- `quat (xyzw)`
- `translation_rmse_m`
- `rotation_mean_deg`

结果文件在：

```bash
~/ros2_ws/aruco_handeye_result.json
```

---

## 9. 如何判断这次标定结果靠不靠谱

重点看两项：

- `translation_rmse_m`
- `rotation_mean_deg`

经验上：

- 平移 RMSE 越小越好
- 旋转平均误差越小越好

如果出现这些情况，建议重采：

- 样本数太少
- 大部分姿态都挤在一小块区域
- ArUco 经常丢失
- 解出来的相机位置明显离物理安装位置很远

---

## 10. 把结果应用到当前系统

当前系统里 `cam_overhead` 的 TF 来自：

- `so101_bringup/launch/camera_tf_moveit.launch.py`

你可以先不改源码，直接用 launch 参数覆盖：

```bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_aruco_detector:=true \
  use_red_detector:=false \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  camera_x:=<解出来的x> \
  camera_y:=<解出来的y> \
  camera_z:=<解出来的z> \
  camera_roll:=<解出来的roll_rad> \
  camera_pitch:=<解出来的pitch_rad> \
  camera_yaw:=<解出来的yaw_rad>
```

如果确认结果靠谱，再回头把默认值写进 launch 文件。

---

## 11. 标定完成后的验证

建议不要立刻上红球抓取，先做两层验证。

### 11.1 先验证 ArUco 位姿回投

启动系统后，看：

```bash
ros2 topic echo /vision/aruco/pose_base
```

然后把夹爪移动到几个位置，确认：

- 位姿变化方向符合直觉
- 不会明显漂移
- 不会整体偏一个很离谱的固定量

### 11.2 再验证红球抓取

等你确认 ArUco 外参没问题，再回到：

- `HX35HM_SO101_红球抓取完整执行步骤.md`

重新跑红球抓取。

这样可以把“手眼标定问题”和“红球检测问题”拆开看。

---

## 12. 我对你这台机器的建议

你现在最适合的顺序是：

1. 先重新做 **手动 ArUco 采样**
2. 得到 `aruco_handeye_result.json`
3. 先用 launch 参数临时覆盖新外参
4. 验证 `/vision/aruco/pose_base`
5. 再回头测红球抓取

原因很简单：

- 你当前红球抓取偏差里，最大的硬误差源还是 `cam_overhead -> base_link`
- 这个问题不先收掉，后面调 `grasp_x/y/z_offset` 都会变成“补错误”

---

## 13. 推荐的一次完整重做顺序

### 终端 1：启动系统

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_aruco_detector:=true \
  use_red_detector:=false \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false
```

### 终端 2：手动采样

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/tools/handeye/collect_aruco_handeye_samples.py \
  --out ~/ros2_ws/aruco_handeye_samples.json
```

### 终端 3：解算

```bash
cd ~/ros2_ws/src
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash

python3 ~/ros2_ws/src/so101-ros-physical-ai/tools/handeye/solve_aruco_handeye.py \
  --samples ~/ros2_ws/aruco_handeye_samples.json \
  --out ~/ros2_ws/aruco_handeye_result.json
```

---

## 14. 结束时关闭

采样脚本：

```bash
Q
```

bringup：

```bash
Ctrl+C
```

如果 ROS 图残留：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/src/install/setup.bash
ros2 daemon stop
ros2 daemon start
```
