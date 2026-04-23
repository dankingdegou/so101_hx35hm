# HX35HM + SO101 手动调参指南

目标：在不改算法主流程的前提下，手动调出“抓取精度稳定 + 动作不过快不过抖”的参数组合。

适用范围：

- `so101_bringup/launch/follower_hx35hm_moveit.launch.py`
- `so101_grasping/launch/so101_visual_grasp.launch.py`

---

## 1. 调参前的固定原则

每次只改一类参数，避免互相干扰：

1. 先固定视觉链路（不要边调视觉边调轨迹）
2. 再调执行平滑（速度/时长）
3. 最后微调抓取偏置（x/y/z offset）

每次测试至少跑 3 次，记录：

- 抓取是否成功
- `After grasp target delta`
- 是否触地
- 回 `rest` 是否需要重试

---

## 2. 推荐起始基线

### 2.1 bringup（执行链）

`follower_hx35hm_moveit.launch.py` 的关键参数：

- `move_duration = 0.8`
- `stream_command_duration = 0.05`

说明：

- `move_duration` 主要影响手动单次命令
- `stream_command_duration` 主要影响 `/go_to_pose` 的流式关节命令跟随

### 2.2 grasp（任务链）

`so101_visual_grasp.launch.py` 的关键参数：

- `ik_pregrasp_duration_s = 1.7`
- `ik_grasp_duration_s = 1.1`
- `ik_retreat_duration_s = 1.1`
- `post_grasp_return_vel_scaling = 0.12`
- `post_grasp_return_acc_scaling = 0.12`

---

## 3. 调参顺序（强烈建议按这个顺序）

### 步骤 A：先调 `stream_command_duration`

优先级最高，范围建议：

- `0.04 ~ 0.06`

经验：

- 太小（如 0.04）：更跟手、精度可能更高，但动作更“急”
- 太大（如 0.06）：更柔，但容易“拖”，导致前段跟随变差

建议流程：

1. 先用 `0.05` 跑 3 次
2. 想更柔，试 `0.055`
3. 若精度明显变差，退回 `0.05`

### 步骤 B：调抓取分段时长

保持 `stream_command_duration` 不变，再调：

- `ik_pregrasp_duration_s`
- `ik_grasp_duration_s`
- `ik_retreat_duration_s`

建议每次只改一个参数，步长：

- `+/- 0.1 ~ 0.2`

方向建议：

- 靠近太急：先加 `ik_pregrasp_duration_s`
- 下探太急：加 `ik_grasp_duration_s`
- 撤离太急：加 `ik_retreat_duration_s`

### 步骤 C：调回位观感

只调：

- `post_grasp_return_vel_scaling`
- `post_grasp_return_acc_scaling`

建议范围：

- `0.10 ~ 0.14`

经验：

- 越小越柔，但整体更慢
- 越大越利落，但可能显得硬

---

## 4. 安全底线参数（别越界）

建议不要超出：

- `stream_command_duration > 0.07`
- `ik_grasp_duration_s < 0.8`
- `post_grasp_return_vel_scaling > 0.20`

一旦出现以下任一情况，立即回到上一个稳定参数：

- 触地
- 连续 2 次抓取失败
- 回 `rest` 连续失败

---

## 5. 常用调参命令模板

### 5.1 启动主栈

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

### 5.2 启动抓取（带调参覆盖）

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
  open_gripper_after_return:=true \
  ik_pregrasp_duration_s:=1.7 \
  ik_grasp_duration_s:=1.1 \
  ik_retreat_duration_s:=1.1 \
  post_grasp_return_vel_scaling:=0.12 \
  post_grasp_return_acc_scaling:=0.12
```

---

## 6. 推荐记录格式

每次测试记一行：

```text
date,time,stream_cmd,pre_dur,grasp_dur,retreat_dur,ret_vel,ret_acc,success,grasp_dist,retreat_dist,rest_retry,touch_table
```

坚持记录 10~20 条后，稳定参数会非常明显。

