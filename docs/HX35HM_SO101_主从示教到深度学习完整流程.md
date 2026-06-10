# HX35HM SO101 主从示教到深度学习完整流程

本文档用于规划后续“用主臂控制从臂采集示教数据，再训练深度学习策略”的完整路线。它不是单纯讲深度学习概念，而是围绕当前 HX35HM SO101 项目的真实 ROS 2 链路来组织。

当前主从控制基线：

```text
主臂 /leader/joint_states
  -> follower_command_relay
  -> /follower/arm_trajectory_controller/follow_joint_trajectory
  -> follower hx35hm_bridge
  -> 从臂舵机
```

目标路线：

```text
主从示教
  -> 录制 episode rosbag
  -> 转成 LeRobot 数据集
  -> 训练 ACT / SmolVLA 等策略
  -> 离线可视化验证
  -> 上机推理控制从臂
```

## 1. 整体阶段

建议按 6 个阶段推进：

```text
阶段 0：稳定主从控制基线
阶段 1：定义任务和采集标准
阶段 2：录制示教 episode
阶段 3：转换 LeRobot 数据集
阶段 4：训练模仿学习策略
阶段 5：离线验证和真实从臂推理
```

不要一开始就直接训练。深度学习项目的成败主要取决于数据质量，而不是模型名字。

## 2. 阶段 0：稳定主从控制基线

先确保当前主从控制可以可靠使用。

启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup teleop_hx35hm.launch.py \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false
```

需要看到：

```text
FollowJointTrajectory action server ready
FJT action: /follower/arm_trajectory_controller/follow_joint_trajectory
Relative teleop ready
```

检查：

```bash
ros2 topic hz /leader/joint_states
ros2 topic hz /follower/joint_states
ros2 action list | rg follow_joint_trajectory
```

如果主从手感还不稳定，先不要采数据。采到的数据会把抖动、延迟、段落感一起学进去。

## 3. 阶段 1：定义学习任务

每个训练任务必须先写清楚：

```text
任务名称：
任务描述：
初始物体位置范围：
目标位置范围：
成功标准：
失败标准：
每条 episode 最长时长：
需要哪些相机：
是否需要夹爪：
```

示例：

```text
任务名称：pick_red_block
任务描述：从桌面抓起红色方块并放到右侧容器中
初始物体位置范围：相机视野中心附近 20cm x 20cm
目标位置范围：右侧固定容器
成功标准：方块进入容器，机械臂回到安全姿态
失败标准：没抓到、碰倒容器、夹爪拖拽物体离开桌面
每条 episode 最长时长：20 秒
需要相机：顶视 RGB，腕部相机可选
是否需要夹爪：需要
```

第一批数据建议选择简单任务：

```text
1. 到达指定区域
2. 抓取固定位置物体
3. 抓取轻微随机位置物体
4. 搬运到固定容器
```

不要一开始就做“任意物体任意位置抓取”。

## 4. 阶段 2：确定数据内容

一个 imitation learning 数据集至少需要：

```text
observation.state：从臂关节状态
observation.images.top：顶视相机
observation.images.wrist：腕部相机
action：当时希望从臂执行的动作
task：自然语言任务描述
```

当前仓库已有：

```text
episode_recorder：录制 rosbag episode
rosbag_to_lerobot：把 rosbag 转成 LeRobot 数据集
so101_inference：加载训练好的 LeRobot policy 控制机械臂
policy_server：远程 GPU 推理服务
```

### 4.1 当前必须注意的问题

上游默认转换配置里，action 来自：

```text
/follower/forward_controller/commands
```

但当前 HX35HM 主从控制主链路是 FJT：

```text
/follower/arm_trajectory_controller/follow_joint_trajectory
```

也就是说，如果直接按上游默认 recorder 配置录制，可能出现：

```text
observation 有数据
action 没数据或不完整
```

当前 `follower_command_relay` 已提供“学习用动作镜像 topic”：

```text
/follower/learning_action
```

类型：

```text
std_msgs/msg/Float64MultiArray
```

内容：

```text
[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
```

它由 `follower_command_relay` 在发送 FJT goal 的同时发布，表示本帧主从映射后的目标关节角。这样数据集里的 action 就是“示教者希望从臂去哪里”，比只用 `/follower/joint_states` 更适合训练。

为了让 `episode_recorder` 的启动检查稳定通过，`follower_command_relay` 还会用
`learning_action_keepalive_s` 周期性重发最近的 6 维目标。这个保活只影响学习用
topic，不会额外刷新 FJT goal。

## 5. 阶段 3：录制示教数据

### 5.1 推荐录制方式

当前 HX35HM 适配建议先用手动组合方式录制，不直接套上游 `recording_session.launch.py`。

终端 1：启动主从控制。

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup teleop_hx35hm.launch.py \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false
```

终端 2：启动双 RGB 相机。当前 HX35HM 示教采集建议同时录制顶视相机和腕部相机：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_hx35hm_dual_rgb.yaml
```

该配置会发布：

```text
/static_camera/image_raw：顶视相机
/follower/image_raw：腕部相机
```

如果临时只想录顶视相机，可退回：

```bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_overhead_rgb.yaml
```

但当前 `episode_recorder_hx35hm_so101.yaml` 已要求腕部相机 topic，正式示教录制请使用双相机配置。

终端 3：启动 recorder。

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch episode_recorder recorder.launch.py \
  params_file:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/recording/episode_recorder_hx35hm_so101.yaml \
  experiment_name:=pick_red_block \
  task:="Pick up the red block and place it in the target area." \
  root_dir:=/home/rog/ros2_ws/datasets/so101_episodes
```

终端 4：启动录制键盘控制。

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 run episode_recorder teleop_episode_keyboard
```

按键：

```text
r：开始录制
s：停止并保存
d：丢弃本条
q：退出
h：帮助
```

### 5.2 录制前检查 topic

```bash
ros2 topic list | rg 'joint_states|image_raw|learning_action|forward_controller'
```

至少应有：

```text
/leader/joint_states
/follower/joint_states
/static_camera/image_raw
/follower/image_raw
/follower/learning_action
```

检查相机频率：

```bash
ros2 topic hz /static_camera/image_raw
ros2 topic hz /follower/image_raw
```

检查腕部相机 CameraInfo：

```bash
ros2 topic echo /follower/camera_info --once
```

### 5.3 每条 episode 的录制标准

每条 episode 建议满足：

```text
1. 开始时机械臂和物体都在合理初始状态。
2. 动作过程尽量自然，不要来回犹豫太多。
3. 成功后停顿 0.5 到 1 秒。
4. 失败 episode 不要混进成功数据集，除非你明确要训练失败恢复。
5. 如果中途严重碰撞或卡住，直接 discard。
```

### 5.4 数据量建议

第一阶段：

```text
20 条 episode：验证录制、转换、可视化链路。
50 条 episode：训练一个最小 ACT smoke policy。
100 到 200 条 episode：开始评估真实任务成功率。
500 条以上：再考虑更复杂随机化。
```

不要一开始采 1000 条。先用 20 条把全链路跑通。

## 6. 阶段 4：转换 LeRobot 数据集

进入上游项目根目录：

```bash
cd /home/rog/ros2_ws/src/so101-ros-physical-ai
```

本地转换：

```bash
pixi run -e lerobot convert -- \
  --input-dir  /home/rog/ros2_ws/datasets/so101_episodes/pick_red_block \
  --config     /home/rog/ros2_ws/src/so101-ros-physical-ai/rosbag_to_lerobot/config/hx35hm_so101.yaml \
  --repo-id    local/hx35hm_so101_pick_red_block \
  --overwrite
```

HX35HM 专用配置文件是：

```text
src/so101-ros-physical-ai/rosbag_to_lerobot/config/hx35hm_so101.yaml
```

其中视觉和 action 已配置为：

```yaml
reference_topic: "/follower/image_raw"

- key: "observation.images.wrist"
  topic: "/follower/image_raw"
  msg_type: "sensor_msgs/msg/Image"
  stamp_src: "bag"
  shape: [480, 640, 3]

- key: "observation.images.top"
  topic: "/static_camera/image_raw"
  msg_type: "sensor_msgs/msg/Image"
  stamp_src: "bag"
  shape: [480, 640, 3]
```

```yaml
- key: "action"
  topic: "/follower/learning_action"
  msg_type: "std_msgs/msg/Float64MultiArray"
  stamp_src: "bag"
  max_age_s: 0.05
  names: *j
```

转换后可视化：

```bash
cd /home/rog/ros2_ws/src/so101-ros-physical-ai
pixi shell -e lerobot
lerobot-dataset-viz --repo-id local/hx35hm_so101_pick_red_block --episode-index 0
```

检查重点：

```text
1. 图像是否正常。
2. observation.images.top 是否来自顶视相机。
3. observation.images.wrist 是否来自腕部相机。
4. observation.state 是否随从臂变化。
5. action 是否存在并且维度为 6。
6. action 与 observation 是否大致同步。
7. episode 开头和结尾是否干净。
```

## 7. 阶段 5：训练策略

### 7.1 ACT 作为第一选择

先用 ACT，不要一开始上大型 VLA。

```bash
cd /home/rog/ros2_ws/src/so101-ros-physical-ai
pixi shell -e lerobot

lerobot-train \
  --dataset.repo_id=local/hx35hm_so101_pick_red_block \
  --policy.type=act \
  --output_dir=outputs/train/act_hx35hm_so101_pick_red_block \
  --job_name=act_hx35hm_so101_pick_red_block \
  --policy.device=cuda
```

如果没有 GPU，临时可用 CPU 做小规模 smoke test：

```bash
lerobot-train \
  --dataset.repo_id=local/hx35hm_so101_pick_red_block \
  --policy.type=act \
  --output_dir=outputs/train/act_hx35hm_so101_smoke_cpu \
  --job_name=act_hx35hm_so101_smoke_cpu \
  --policy.device=cpu
```

CPU 只适合验证流程，不适合认真训练。

### 7.2 训练时重点看什么

不要只看 loss 降没降。还要看：

```text
1. 数据集 episode 是否真的成功。
2. action 分布是否正常。
3. 关节是否经常顶限位。
4. 图像中目标是否清楚。
5. 每条 episode 的任务描述是否一致。
```

如果训练 loss 下降但上机失败，通常是数据问题，不是模型问题。

## 8. 阶段 6：离线验证

训练完成后先做离线检查：

```text
1. 用 dataset visualizer 看原始数据。
2. 用模型在离线数据上 rollout 或预测 action。
3. 检查预测 action 是否平滑。
4. 检查 action 是否超出机械臂限位。
5. 检查夹爪动作是否在正确阶段发生。
```

不要训练完直接让真实机械臂执行。

## 9. 阶段 7：真实从臂推理

推理前先只启动从臂和相机，不启动主从 teleop。

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup inference.launch.py
```

终端 2：运行策略。

```bash
cd /home/rog/ros2_ws/src/so101-ros-physical-ai

pixi run -e lerobot infer -- --ros-args \
  -p repo_id:="local/hx35hm_so101_pick_red_block" \
  -p policy_type:=act \
  -p fps:=30.0
```

第一次上机建议：

```text
1. 降低 fps。
2. 手放在急停命令旁边。
3. 任务区域清空，只放一个轻物体。
4. 先短时间运行。
5. 一旦方向不对，立即停止。
```

紧急停止参考：

```bash
pkill -f 'infer|async_infer|lerobot_inference_node|async_ros2_inference_client|hx35hm_bridge|move_group|robot_state_publisher' || true
```

更完整急停流程看：

```text
docs/HX35HM_SO101_主从控制紧急停止命令.md
```

## 10. 推荐项目目录

建议把数据和模型放在工作区外或专门目录下：

```text
/home/rog/ros2_ws/datasets/
  so101_episodes/
    pick_red_block/
      episode_xxx/
  lerobot/
    hx35hm_so101_pick_red_block/

/home/rog/ros2_ws/models/
  act_hx35hm_so101_pick_red_block/
```

不要把大 rosbag、视频数据、模型权重直接提交进 Git 仓库。

## 11. 建议的开发里程碑

### M1：录制链路跑通

目标：

```text
录 3 条 episode，能保存 rosbag。
```

验收：

```bash
ls /home/rog/ros2_ws/datasets/so101_episodes/pick_red_block
```

### M2：数据转换跑通

目标：

```text
rosbag 能转成 local LeRobot dataset。
```

验收：

```bash
lerobot-dataset-viz --repo-id local/hx35hm_so101_pick_red_block --episode-index 0
```

### M3：确认 action 记录

目标：

```text
/follower/learning_action 正常发布，并在 hx35hm_so101.yaml 中作为 action。
```

验收：

```bash
ros2 topic echo /follower/learning_action --once
```

### M4：小数据训练 smoke test

目标：

```text
20 条 episode 训练一个 ACT smoke policy。
```

验收：

```text
训练能完整结束，输出 checkpoint。
```

### M5：真实从臂安全推理

目标：

```text
策略可以在真实从臂上运行 5 到 10 秒，不撞限位，不发散。
```

验收：

```text
从臂动作方向正确，能完成任务中的一部分动作。
```

## 12. 后续需要补的工程点

### 12.1 学习用 action mirror topic

当前已经完成：

```text
在 follower_command_relay 发送 FJT goal 的同时，发布 /follower/learning_action。
新增 so101_bringup/config/recording/episode_recorder_hx35hm_so101.yaml 录制该 topic。
```

原因：

```text
LeRobot 数据集需要 action。
当前主链路不再直接发布 /follower/forward_controller/commands。
FJT action 本身不适合作为 rosbag_to_lerobot 的简单 action 特征。
```

### 12.2 HX35HM 专用 recording launch

建议新增：

```text
so101_bringup/launch/recording_session_hx35hm.launch.py
```

它应该组合：

```text
teleop_hx35hm.launch.py
cameras.launch.py
episode_recorder
可选 rerun
```

这样后续采集数据就不用开四个终端。

### 12.3 腕部相机录制链路

当前已经新增：

```text
so101_bringup/config/cameras/so101_cameras_hx35hm_dual_rgb.yaml
so101_bringup/config/cameras/so101_gs_cam_hx35hm_wrist.yaml
```

腕部相机固定发布：

```text
/follower/image_raw
```

录制配置已包含：

```text
/static_camera/image_raw
/follower/image_raw
/follower/joint_states
/follower/learning_action
```

### 12.4 HX35HM 专用 LeRobot config

当前已经新增：

```text
rosbag_to_lerobot/config/hx35hm_so101.yaml
```

其中视觉输入使用：

```text
observation.images.wrist <- /follower/image_raw
observation.images.top   <- /static_camera/image_raw
```

其中 action 使用：

```text
/follower/learning_action
```

而不是：

```text
/follower/forward_controller/commands
```

## 13. 关键原则

```text
先跑通 3 条 episode，不要直接采大数据。
先训练 ACT，不要一开始上大 VLA。
先离线看数据，再上真实机械臂。
先保证 action 正确，再谈模型效果。
失败数据不要混进成功数据集。
主从控制不稳定时不要采数据。
```

## 14. 推荐学习顺序

如果你是边做边学，建议顺序是：

```text
1. ROS 2 topic/action/bag 基础
2. 当前主从 FJT 控制链路
3. episode_recorder 录制机制
4. rosbag_to_lerobot 数据同步逻辑
5. LeRobot dataset 格式
6. ACT 模仿学习
7. 策略推理节点如何把 action 发回从臂
8. 远程 GPU 推理和 SmolVLA
```

对应项目文件：

```text
docs/HX35HM_SO101_主从控制当前使用流程.md
docs/HX35HM_SO101_主从控制参数调整教程.md
src/so101-ros-physical-ai/episode_recorder/README.md
src/so101-ros-physical-ai/rosbag_to_lerobot/README.md
src/so101-ros-physical-ai/so101_inference/README.md
```
