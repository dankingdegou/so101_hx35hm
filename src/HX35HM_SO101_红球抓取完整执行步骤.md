# HX35HM + SO101 红球抓取完整执行步骤

适用对象：当前这套 `HX-35HM + SO101 follower + overhead RGB 相机 + MoveIt`

目标：按下面步骤，完整执行一次“识别红球 -> 估计桌面 -> 规划 -> 抓取 -> 抬起 -> 回到 rest -> 打开夹爪放下小球”

---

## 1. 先明确当前流程

当前抓取链路是：

1. `so101_bringup/follower_hx35hm_moveit.launch.py`
2. 启动机械臂桥接 `hx35hm_bridge`
3. 启动 MoveIt `move_group`
4. 启动 overhead RGB 相机
5. 启动红球检测 `red_circle_detector`
6. 启动桌面估计 `table_plane_estimator`
7. `so101_grasping/so101_visual_grasp`
8. 等视觉稳定后取红球位姿和桌面高度
9. 执行 `hover_high -> pregrasp -> grasp -> close gripper -> retreat -> rest`
10. 回到 `rest` 后打开夹爪，把小球放下

注意：

- 当前推荐流程不再强制先去 `extended`，而是从当前姿态直接进入 `hover_high/pregrasp`。
- 当前红球链路主要依赖 `RGB + 深度 + 桌面平面回退`。
- 当前抓取节点会优先读取 `/vision/table/top_z` 的实时桌面高度；如果没有收到新鲜数据，才回退到静态 `tabletop_z_m`。
- 如果最终仍有整体偏差，优先看相机标定、相机 TF 和 `x/y` 补偿，不要先怀疑路径规划。
- 当前已经修复：手臂 arm-only 轨迹不会再覆盖夹爪，夹爪只由 `/follower/gripper_controller/gripper_cmd` 控制。

---

## 2. 编译

在新终端执行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select so101_grasping so101_bringup so101_kinematics so101_hx35hm_bridge \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/ros2_ws/install/setup.bash
```

如果你只改了抓取节点：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select so101_grasping \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/ros2_ws/install/setup.bash
```

如果你改了 `/go_to_pose`、Cartesian 轨迹或夹爪覆盖逻辑：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select so101_kinematics so101_hx35hm_bridge \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/ros2_ws/install/setup.bash
```

如果你碰到：

- `ModuleNotFoundError: No module named 'catkin_pkg'`
- 日志里还出现 `/home/rog/.local/bin/python3.11`

那通常是 `build/` 缓存里还记着旧 Python，不是当前抓取代码本身有错。这里统一使用 `/usr/bin/python3`，就是为了把这个隐患一次性压住。

---

## 3. 启动整套抓取系统

建议先不开 RViz，也不开 joint GUI，减少干扰：

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

正常情况下你会看到这些关键信息：

- `FollowJointTrajectory action server ready`
- `ParallelGripperCommand action server ready`
- `Red detector started`
- `You can start planning now!`

非常重要：

- **一次只保留一套 `bringup` 在跑**
- 不要重复启动第二套 `follower_hx35hm_moveit.launch.py`
- 如果你叠跑两套，最常见的问题就是：
  - 两个 `move_group`
  - 两个红球 detector
  - 视觉位姿超时或抓取行为混乱

---

## 4. 启动后先做在线检查

另开一个终端：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 4.1 检查节点

```bash
ros2 node list
```

至少应看到一套：

- `/follower/robot_state_publisher`
- `/follower/hx35hm_bridge`
- `/move_group`
- `/red_circle_detector`
- `/cartesian_motion_node`

### 4.2 检查话题

```bash
ros2 topic list | rg '/vision|/static_camera|/joint_states'
```

至少应看到：

- `/static_camera/image_raw`
- `/vision/red_block/debug_image`
- `/vision/red_block/pose_base`
- `/vision/table/top_z`
- `/vision/table/status`

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
- `z` 大致在桌面高度附近，通常接近 `0`

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
- 如果圈得很准，但抓取位置还偏，优先怀疑 TF/补偿，不要先怪 detector

### 5.3 看桌面高度是否稳定

```bash
ros2 topic echo /vision/table/top_z
ros2 topic echo /vision/table/status
```

正常现象：

- `/vision/table/top_z` 持续输出一个稳定数值
- `/vision/table/status` 类似 `table z=+0.003m points=120 center=(+0.285,+0.012)`
- 如果桌面本身水平，`z` 一般接近 `0.0` 附近，上下几毫米到 1 厘米波动都算正常

如果这里不稳定，先不要急着跑抓取。优先检查：

- 深度图本身是否有大面积空洞
- 桌面 ROI 里有没有混入机械臂本体
- 相机到 `base_link` 的 TF 是否正确

---

## 6. 执行一次实际抓取

确认红球摆放好、机械臂周围无碰撞风险后，再执行。

这是当前验证过的一条更稳的真抓命令：

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

说明：
- 当前默认已经改成 `go_to_rest_before_grasp:=false`，不会再先去 `extended` 再二次规划。
- 现在默认流程是：先锁定红球位姿，再直接从当前姿态规划到 `hover_high -> pregrasp -> grasp -> retreat`，最后回 `rest` 交给 MoveIt 正常规划。
- `post_grasp_use_ik_joints:=false` 会关闭 `/go_to_joints` 绕行，最后回 `rest` 使用 MoveIt named target。
- `add_table_collision:=true` 会给 MoveIt planning scene 加桌面碰撞体，让回 `rest` 规划有机会避开桌面。
  当前默认桌面保护区已经收窄到前方工作区，避免把基座附近也误判成桌面碰撞。
- `use_tabletop_z_topic:=true` 会让抓取节点优先使用 `/vision/table/top_z` 的实时桌面高度，而不是只靠手填 `tabletop_z_m`。
- `grasp_retry_count:=1` 表示靠近红球阶段如果第一次规划/执行失败，会重新读取当前机械臂姿态和最新红球位置，再做一次二次规划。
- `post_grasp_return_retry_count:=3` 表示夹爪闭合后如果回 `rest` 失败，会重新读取当前机械臂姿态，再让 MoveIt 从当前状态重新规划回 `rest`，默认最多再试 3 次。
- `open_gripper_after_return:=true` 会在回到 `rest` 后打开夹爪，把小球放下。
- 如果后面你确实想强制先去某个命名姿态，再显式传入 `go_to_rest_before_grasp:=true staging_named_pose:=extended`。

当前默认行为：

1. 打开夹爪
2. 采样稳定红球位姿
3. 走到 `hover_high`
4. 走到 `pregrasp`
5. 下探到 `grasp`
6. 闭合夹爪
7. 回到 `pregrasp`
8. MoveIt 规划回到 `rest`
9. 打开夹爪放下小球

正常日志关键字：

- `Opening gripper...`
- `Using N pose samples`
- `Using tabletop z from topic '/vision/table/top_z': ...`
- `Target pose: ...`
- `Moving to hover_high...`
- `Moving to pregrasp...`
- `Moving to grasp...`
- `Closing gripper...`
- `Retreating...`
- `Returning to post-grasp pose 'rest'...`
- `Opening gripper after returning to 'rest'...`
- `Visual grasp sequence completed.`

---

## 7. 如果你只想试规划，不想真的动

```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=false \
  pose_topic:=/vision/red_block/pose_base \
  add_table_collision:=true \
  min_grasp_clearance_m:=0.015 \
  min_pregrasp_clearance_m:=0.080 \
  grasp_x_offset_m:=0.025 \
  return_to_named_pose_after_grasp:=true \
  post_grasp_named_pose:=rest \
  open_gripper_after_return:=true
```

这会只做规划，不执行机械臂和夹爪动作。

---

## 8. 当前默认参数

当前这版默认参数大意如下：

- `pose_topic=/vision/red_block/pose_base`
- `pose_sample_window_s=0.35`
- `pose_sample_count=5`
- `min_pose_sample_count=3`
- `max_pose_spread_m=0.03`
- `post_rest_settle_s=0.4`
- `pregrasp_offset_m=0.08`
- `hover_high_offset_m=0.04`
- `use_two_stage_pregrasp=true`
- `grasp_z_offset_m=-0.015`
- `use_tabletop_z_topic=true`
- `tabletop_z_topic=/vision/table/top_z`
- `wait_tabletop_z_timeout_s=2.0`
- `fresh_tabletop_z_slack_s=1.0`
- `go_to_rest_before_grasp=false`
- `staging_named_pose=extended`（仅在显式打开 `go_to_rest_before_grasp:=true` 时使用）
- `return_to_named_pose_after_grasp=false`
- `open_gripper_after_return=true`（只有回到 `rest` 时才会执行）
- `post_grasp_lift_before_return=false`
- `post_grasp_return_lift_z_m=0.14`
- `post_grasp_return_lift_extra_m=0.04`
- `post_grasp_return_via_rest_hover=false`
- `post_grasp_rest_hover_x_m=0.20`
- `post_grasp_rest_hover_y_m=0.0`
- `post_grasp_rest_hover_z_m=0.14`
- `post_grasp_use_ik_joints=false`
- `ik_joints_service_name=/go_to_joints`
- `ik_post_grasp_duration_s=2.4`
- `grasp_retry_count=1`
- `grasp_retry_delay_s=0.5`
- `post_grasp_return_retry_count=3`
- `post_grasp_return_retry_delay_s=1.0`

当前推荐的“安全抓取版”附加参数：

- `add_table_collision=true`
- `table_center_x_m=0.32`
- `table_center_y_m=0.0`
- `table_size_x_m=0.36`
- `table_size_y_m=0.45`
- `min_grasp_clearance_m=0.015`
- `min_pregrasp_clearance_m=0.080`
- `return_to_named_pose_after_grasp=true`
- `post_grasp_named_pose=rest`
- `post_grasp_use_ik_joints=false`
- `grasp_retry_count=1`
- `post_grasp_return_retry_count=3`
- `open_gripper_after_return=true`

如果要临时试更高一点的抓取安全高度：

```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=true \
  min_grasp_clearance_m:=0.020 \
  min_pregrasp_clearance_m:=0.090
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
  grasp_x_offset_m:=0.025 \
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

如果你已经重新做过内参和手眼外参，那这里优先看的是：

- 红球中心在 `debug_image` 里是不是准
- `pose_base` 和真实桌面点位是不是还有固定偏差

### 9.3 再看是不是相机 TF 偏了

当前 camera 到 `base_link` 的外参在：

- `so101_bringup/launch/camera_tf_moveit.launch.py`

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
- 抓完后回 `rest` 报 `Execution to named target 'rest' failed`

如果日志是 `Target pose` 本身就不对，但规划一路成功，那不是规划问题。

如果只是在最后回 `rest` 失败，优先排查：

- 是否启用了 `add_table_collision:=true`
- `tabletop_guard` 的范围是不是过大
- `joint_states` 是否稳定
- MoveIt 的 start state 是否被桌面碰撞或关节漂移卡住

当前默认主路线仍然建议保留：

```bash
post_grasp_use_ik_joints:=false
```

也就是继续让最后回 `rest` 走 MoveIt named target，而不是切回 `/go_to_joints` 绕行。

### 9.5 如果报“等待稳定红球位姿超时”

典型报错是：

- `Timed out waiting for a fresh stable pose on '/vision/red_block/pose_base'`

这通常说明：

1. 红球暂时出了视野
2. detector 没有及时恢复稳定输出
3. 你重复起了两套 `bringup`

优先排查：

1. 确保只跑一套 `bringup`
2. 确保红球放在 overhead 相机清楚可见的位置
3. 再重新抓

---

## 10. 如果旧节点或 ROS 缓存干扰脚本

如果你看到下面现象，优先清理旧节点：

- `ros2 node list` 提示有重复同名节点
- 同时出现两个 `/move_group`
- 同时出现两个 `/red_circle_detector`
- 同时出现两个 `/cartesian_motion_node`
- 抓取动作重复、上下反复、夹爪刚闭合又被打开
- `ros2 action info` 里同一个 action 有多个异常 client

### 10.1 查看是否有残留进程

```bash
pgrep -af 'follower_hx35hm_moveit|so101_visual_grasp|move_group|cartesian_motion_node|red_circle_detector|hx35hm_bridge|openni|gscam|rviz'
```

正常情况：只应该有一套 `follower_hx35hm_moveit.launch.py` 相关进程。

### 10.2 停掉旧的 ROS 抓取相关进程

这条命令只清理当前这套 SO101 抓取系统相关进程，不会删除文件：

```bash
pkill -f 'follower_hx35hm_moveit.launch.py|so101_visual_grasp|move_group|cartesian_motion_node|red_circle_detector|gscam_node|openni2_camera_node|rviz2|/home/rog/ros2_ws/install/so101_hx35hm_bridge/lib/so101_hx35hm_bridge/hx35hm_bridge' || true
```

如果你还开着对应 launch 终端，最好也在那些终端里按一次：

```bash
Ctrl+C
```

### 10.3 重启 ROS daemon 缓存

有时候进程已经停了，但 `ros2 node list` 还显示旧节点，这是 ROS daemon 的图缓存没刷新。执行：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 daemon stop
ros2 daemon start
```

### 10.4 再确认 ROS 图已经干净

```bash
pgrep -af 'follower_hx35hm_moveit|so101_visual_grasp|move_group|cartesian_motion_node|red_circle_detector|hx35hm_bridge|openni|gscam|rviz' || true

ros2 node list
```

如果你还没重新启动 bringup，`pgrep` 应该没有真实抓取相关进程。

如果已经重新启动了一套 bringup，`ros2 node list` 里应只有一套：

- `/follower/hx35hm_bridge`
- `/move_group`
- `/red_circle_detector`
- `/cartesian_motion_node`

### 10.5 检查 action 是否只有一套 server/client

```bash
ros2 action info /follower/gripper_controller/gripper_cmd
ros2 action info /follower/arm_trajectory_controller/follow_joint_trajectory
```

正常情况：

- `Action servers: 1`
- `Action clients: 1`

### 10.6 如果 Python/colcon 缓存干扰构建

如果日志里反复出现旧 Python，例如 `/home/rog/.local/bin/python3.11`，或者出现 `catkin_pkg` 相关错误，可以清理相关包的构建缓存后重编：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3

rm -rf build/so101_grasping install/so101_grasping log/latest
rm -rf build/so101_kinematics install/so101_kinematics
rm -rf build/so101_hx35hm_bridge install/so101_hx35hm_bridge

colcon build --packages-select so101_grasping so101_kinematics so101_hx35hm_bridge \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source ~/ros2_ws/install/setup.bash
```

注意：

- 不要删除 `src/`，源码都在里面。
- 不要用 `git reset --hard` 清理问题，除非你明确知道会丢掉哪些改动。
- 如果只是 ROS 图缓存问题，优先用 `ros2 daemon stop/start`，不需要删 build。

---

## 11. 推荐的实际操作顺序

每次开机后，建议固定按这个顺序：

1. 编译并 `source`
2. 启动 `follower_hx35hm_moveit.launch.py`
3. 检查节点、话题、action
4. 看 `/vision/red_block/debug_image`
5. 看 `/vision/red_block/pose_base`
6. 先 `execute:=false` 做一次规划验证
7. 再 `execute:=true` 做真实抓取
8. 抓完确认是否回到 `rest`
9. 确认回到 `rest` 后夹爪已经打开并放下小球

---

## 12. 一键执行版

如果当前系统已经稳定，最常用就是这两条：

终端 1：

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

终端 2：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch so101_grasping so101_visual_grasp.launch.py \
  execute:=true \
  pose_topic:=/vision/red_block/pose_base \
  add_table_collision:=true \
  min_grasp_clearance_m:=0.015 \
  min_pregrasp_clearance_m:=0.080 \
  return_to_named_pose_after_grasp:=true \
  post_grasp_named_pose:=rest \
  post_grasp_use_ik_joints:=false \
  open_gripper_after_return:=true
```

---

## 13. 结束后关闭

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
source ~/ros2_ws/install/setup.bash
ros2 daemon stop
ros2 daemon start
```

如果仍然看到旧节点，按第 10 节执行完整清理。

---

## 14. 当前结论

当前这版已经修复：

- 旧位姿直接拿来抓的问题
- 当前机械臂姿态干扰红球检测的问题
- 单帧位姿抖动直接触发抓取的问题
- 只动夹爪不动手臂的问题
- 抓完后自动回 `rest` 的问题
- 回到 `rest` 后自动打开夹爪放下小球
- arm-only 轨迹覆盖夹爪的问题
- 抓取后 MoveIt named target 回 `rest` 偶发执行失败的问题：现在优先用 `/go_to_joints` 回 `rest`

当前仍然可能影响最终精度的主要因素：

1. 红球高度 `z` 仍主要依赖桌面平面回退和安全高度，而不是真实球心深度
2. `x/y` 仍可能有固定方向系统偏差
3. 如果重复起多套 `bringup`，系统会变得不稳定

所以：

- 如果是“抓取流程乱走”，先看日志
- 如果是“总是整体偏一点”，优先做相机标定、TF 标定和 `x/y` 补偿
