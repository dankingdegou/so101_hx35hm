# HX35HM SO101 RGB-D 外参调试指南

这份文档专门针对你现在这套系统：

- 顶视 RGB 相机：`/static_camera/image_raw`
- 顶视深度相机：`/static_camera/depth/image_raw`
- 红球检测节点：`red_circle_detector`
- 在线调参工具：`SO101 Depth->RGB Tuner`

目标不是做一套“学术上最完美”的标定，而是把当前 RGB-depth 配准误差压下来，让：

- `Projective RGB-D registration residual too large`
  这种告警尽量减少
- 红球 3D 点更稳
- 抓取前半段目标点更可信

---

## 1. 先理解你现在在调什么

当前系统里，红球检测会做这件事：

1. 在 RGB 图里找到红球中心像素
2. 在 depth 图里找和这个 RGB 像素对应的深度点
3. 用 `cam_overhead_depth -> cam_overhead` 的外参把 depth 点投影回 RGB
4. 算投影回来的 RGB 点和红球中心像素之间的误差

这个误差就是：

- `registration_err=...px`

如果误差太大，就会出现：

- `Projective RGB-D registration residual too large`

然后 detector 会退回近似映射，3D 点质量就会变差。

所以你现在调的不是“机械臂姿态”，而是：

**深度相机坐标系相对 RGB 相机坐标系的外参。**

---

## 2. 你现在在调哪些参数

`SO101 Depth->RGB Tuner` 窗口里这 6 个参数，就是当前在线调的 depth->rgb 外参：

- `x`
- `y`
- `z`
- `roll_deg`
- `pitch_deg`
- `yaw_deg`

它们对应 launch 参数是：

```bash
depth_to_rgb_x
depth_to_rgb_y
depth_to_rgb_z
depth_to_rgb_roll
depth_to_rgb_pitch
depth_to_rgb_yaw
```

注意：

- `roll/pitch/yaw` 在窗口里是角度
- launch 里保存时是弧度

调好以后，窗口里可以直接复制一串 launch args。

---

## 3. 调试前准备

### 3.1 先清理旧进程

避免多个 TF、多个 detector、多个相机节点同时存在：

```bash
pkill -f 'follower_hx35hm_moveit.launch.py|cartesian_motion_node|hx35hm_bridge|move_group|red_circle_detector|table_plane_estimator|gscam_node|openni2_camera_node|static_transform_publisher|so101_depth_rgb_tuner.py' || true
```

### 3.2 用 tuner 模式启动主栈

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
  use_vision_debug_rviz:=true \
  use_depth_to_rgb_tuner:=true
```

启动后你应该看到：

- 相机正常起来
- `red_circle_detector` 正常运行
- 弹出 `SO101 Depth->RGB Tuner` 窗口
- 弹出一个 RViz 调试页，里面会显示：
  - `RGB`
  - `Red Debug`
  - `Depth`
  - `Red Pose`

如果你不想在主启动命令里开 RViz，也可以单独开：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

rviz2 -d /home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/rviz/vision_overhead_debug.rviz
```

如果只是想确认红球 debug 图有没有在发，也可以直接查话题：

```bash
ros2 topic list | rg /vision/red_block/debug_image
ros2 topic hz /vision/red_block/debug_image
```

---

## 4. 调试时看什么

你主要看 3 样东西：

### 4.1 `SO101 Depth->RGB Tuner` 窗口

你真正操作的是它。

### 4.2 红球 debug 图

重点看：

- RGB 红球中心圈的位置
- depth 对应点是不是贴近红球中心
- 是否明显偏到球边缘、桌面、背景

说明：

- 这个“红球 debug 图”不是单独弹出来的独立窗口
- 它默认显示在 RViz 调试页里的 `Red Debug` 面板
- 对应话题是：

```bash
/vision/red_block/debug_image
```

### 4.3 状态文本 / 残差

重点关注：

- `registration_err=...px`
- `Projective RGB-D registration residual too large`
- `/vision/red_block/status`

理想状态是：

- `registration_err` 明显下降
- `residual too large` 告警尽量少
- `pose ok ...` 状态更稳定

---

## 5. 调试目标

不要一开始就追求“0 像素误差”，工程上先达到下面这个目标就很值：

### 第一阶段目标

- residual 明显低于当前水平
- 尽量压到 `<= 8 px`

### 更理想的目标

- residual 稳定在 `<= 5 px`

如果能做到这个量级，红球 3D 点通常会明显比现在稳。

---

## 6. 正确的调试顺序

**不要 6 个参数一起乱动。**

按下面顺序来：

1. `x`
2. `y`
3. `yaw_deg`
4. `z`
5. `roll_deg`
6. `pitch_deg`

原因：

- 现在这套系统更像是平移 + 平面内旋转误差
- 不像一开始就有很大的三维倾斜误差

所以先调：

- `x/y/yaw`

通常收益最大。

---

## 7. 每个参数怎么调

### 7.1 调 `x`

先固定其他 5 个不动，只动 `x`。

建议步长：

- 每次 `0.002 ~ 0.003 m`

不要一下跳太大。

每改一次：

1. 停 `2~3` 秒
2. 看 debug 图
3. 看 residual 是更低还是更高

如果越来越差，就反方向回去。

### 7.2 调 `y`

在 `x` 大致找到更优点之后，再调 `y`。

建议步长：

- 每次 `0.002 ~ 0.003 m`

同样只动一个变量。

### 7.3 调 `yaw_deg`

等 `x/y` 差不多后，再调 `yaw_deg`。

建议步长：

- 每次 `0.5 ~ 1.0 deg`

如果你发现：

- 对应点总是沿着某个方向打转
- 或者对齐趋势像“角度不对”

那通常就是 `yaw` 值值得动。

### 7.4 调 `z`

只有在下面情况才优先动 `z`：

- 对应点已经大致对上
- 但深度映射总有明显前后错层感

建议步长：

- 每次 `0.001 ~ 0.002 m`

### 7.5 调 `roll_deg / pitch_deg`

这两个最后再碰。

只有当你已经把：

- `x`
- `y`
- `yaw`
- 必要时 `z`

都调过了，仍然有系统性误差时，才去碰它们。

建议步长：

- 每次 `0.2 ~ 0.5 deg`

---

## 8. 一个实用的调参节奏

推荐用这种节奏：

### 第一轮：粗调

- `x` 以 `0.003` 为步长扫几步
- `y` 以 `0.003` 为步长扫几步
- `yaw_deg` 以 `1.0` 为步长扫几步

目的：

- 快速找到“明显更好”的区域

### 第二轮：细调

在第一轮找到的较优区域附近，用更小步长：

- `x/y`: `0.001`
- `yaw_deg`: `0.2 ~ 0.5`

目的：

- 把 residual 往更低压

---

## 9. 怎么判断“变好了”

不是只看一种现象，而是综合判断。

如果变好了，通常会一起出现：

- `registration_err` 数值下降
- `Projective RGB-D registration residual too large` 告警变少
- 红球 debug 图里 RGB 圆心和 depth 对应点更贴
- `/vision/red_block/status` 更稳定
- 红球 3D pose 跳动更少

如果只是某一个指标变好，但另一个明显变坏，不一定是真改善。

---

## 10. 哪些现象说明你调反了

如果出现这些，通常说明方向错了：

- residual 明显升高
- detector 频繁 fallback
- 红球对应点偏到桌面或背景
- 红球 3D 点抖动变大
- `pose ok` 变少，`depth invalid`、`plane fallback` 变多

这时不要继续往那个方向走，回到上一档。

---

## 11. 调试时的注意事项

### 11.1 红球尽量放在常用工作区

不要把红球放到非常边缘的位置调。

最好放在：

- 机械臂常抓的桌面中心区域
- 相机视野中比较常用的位置

这样调出来的外参对实际抓取最有意义。

### 11.2 调试时目标尽量静止

红球和桌面都别动，避免你调参数时把“目标移动”误判成“外参变差”。

### 11.3 一次只动一个变量

这是最重要的规则之一。

不要：

- 一边调 `x`
- 一边调 `y`
- 一边再动 `yaw`

这样最后你自己也不知道哪一步起作用。

### 11.4 先工程可用，再追求漂亮

如果已经能把 residual 从十几像素压到几像素，
而且红球 3D 点明显稳了，就已经是很大的进步。

不要为了追最后一点点数值，反而把整体稳定性搞坏。

---

## 12. 调完后怎么保存

### 12.1 在 tuner 窗口里复制参数

点击：

- `Copy launch args`

它会给你一串类似：

```bash
depth_to_rgb_x:=...
depth_to_rgb_y:=...
depth_to_rgb_z:=...
depth_to_rgb_roll:=...
depth_to_rgb_pitch:=...
depth_to_rgb_yaw:=...
```

### 12.2 写回启动参数

把这些值写回：

- [camera_tf_moveit.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/camera_tf_moveit.launch.py)

对应默认值：

- `depth_to_rgb_x`
- `depth_to_rgb_y`
- `depth_to_rgb_z`
- `depth_to_rgb_roll`
- `depth_to_rgb_pitch`
- `depth_to_rgb_yaw`

这样以后正常启动时就不用再开 tuner。

---

## 13. 调完后怎么验证

调完后建议做两轮验证。

### 13.1 视觉验证

正常启动一遍，不开 tuner：

```bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_red_detector:=true \
  use_cameras:=true \
  use_rviz:=false \
  use_joint_gui:=false \
  use_aruco_detector:=false \
  use_vision_debug_rviz:=false
```

看：

- residual 告警是否明显减少
- 红球状态是否更稳

### 13.2 抓取验证

再跑红球抓取，看：

- 抓取前半段目标点是否更稳定
- pregrasp / grasp 漂移是否变小

---

## 14. 当前这套系统最可能的有效方向

针对你当前这套相机和日志现象，优先顺序建议是：

1. 先调 `x`
2. 再调 `y`
3. 再调 `yaw_deg`
4. 最后才碰 `z / roll / pitch`

这通常比一开始就调姿态角更有效。

---

## 15. 一句话版规则

如果你后面只记一条：

**一次只动一个参数，先调 `x/y/yaw`，看 residual 和 debug 图是不是一起变好。**
