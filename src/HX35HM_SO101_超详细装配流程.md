# HX-35HM + SO101 超详细装配流程（可直接照做）

适用对象：你现在这套 **HX-35HM 总线舵机 + STM32 控制板 + SO101 单臂（follower）** 的装配与软件对齐。

目标：装完以后做到三件事：

1. 机械臂在你选定的“参考姿态”（`rest` 折叠 或 `zero` 全 0）下不别扭、不硬顶、不带预应力。
2. 之后从 ROS/MoveIt 发关节角，方向正确、零位合理、不会一上电就撞机。
3. 你能留下“可复现”的校准记录（以后拆装/换舵机不至于从头猜）。

---

## A. 装配前必须读完的 3 条原则（很重要）

### A1. “全中位(pos=500)”只能当作安全起点，不能当作最终参考姿态

HX-35HM 默认 `pos=500` 是舵机角度中位（约 120°），它不是机器人关节 `0 rad`，也不是 MoveIt 的 `rest`。

正确做法是：

1. 先全中位：让所有舵机远离极限，安全上电、方便装螺丝。
2. 然后 **逐关节** 把该关节调到“参考姿态下它应该在的位置”，再装这个关节。

### A2. 装配的核心是“零位对齐”，不是“角度居中”

你装配时要对齐的是：

- 你定义的参考姿态下的关节角（例如 `rest` 折叠的关节角）
- 以及该关节在该姿态下连杆/支架的几何位置

而不是让每个舵机都转到同一个数值。

### A3. 一次只装/验证一个关节，装完一个就做小幅正反测试

装配顺序建议：从底座到末端逐级（`shoulder_pan -> shoulder_lift -> elbow_flex -> wrist_flex -> wrist_roll -> gripper`）。

每装完一级就测试 ±10°（小幅）确认不干涉，再进行下一个关节。

---

## B. 你要先决定：按 `rest` 装配，还是按 `zero` 装配？

### B1. `rest`（折叠）姿态的定义（来自 MoveIt SRDF）

文件：`~/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/so101_arm.srdf`

`rest` 的关节角（rad）：

- `shoulder_pan = 0`
- `shoulder_lift = -1.57`（约 -90°）
- `elbow_flex = 1.57`（约 +90°）
- `wrist_flex = 0.75`（约 +43°）
- `wrist_roll = 0`

### B2. 推荐选择

- 如果你的“折叠形态”孔位更好装、空间更小、更安全：选 `rest`（更常见）。
- 如果你强烈希望“装配姿态就是软件 0 rad”：选 `zero`（最直观，但不一定好装）。

下面流程我按 **推荐：`rest`** 来写，同时每一步都会标注“如果你选 `zero` 应该怎么改”。

---

## C. 需要准备的工具与软件

### C1. 工具清单（机械）

- 内六角扳手（按你 SO101 螺丝规格）
- 螺丝胶（中强度，选用）
- 记号笔 / 美纹纸（用来标记“参考姿态线”）
- 游标卡尺（可选，用于测对称、间隙）
- 小号十字/一字螺丝刀（视结构）

### C2. 工具清单（电气）

- 稳定电源（9.6–12.6V，电流留足余量）
- 保险/限流模块（强烈建议）
- 控制板 USB 线

### C3. 工作空间脚本（你已有）

- 一键全中位：`~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py`
- 扫动测试：`~/ros2_ws/continuous_sweep.py`
- 回中位（裸）：`~/ros2_ws/return_to_home.py`
- 改舵机 ID：`~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py`

---

## D. 装配前的“单舵机准备”流程（每个舵机都做一遍）

> 目的：确认舵机正常、ID 正确、不会带病装上去。

### D1. 一次只接 1 个舵机（强制）

避免广播命令误改多个舵机 ID 或同时动作带来风险。

### D2. 给舵机设置唯一 ID（建议 1~6）

编辑并运行：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

python3 ~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py
```

你需要把脚本末尾的 `new_id` 改成目标值（例如先改成 1，再改成 2...）。

### D3. 单舵机小幅扫动（确认没有卡滞/异响）

在 `continuous_sweep.py` 里把 `SERVO_IDS` 改成只含当前舵机，例如 `[1]`，然后运行：

```bash
python3 ~/ros2_ws/continuous_sweep.py
```

确认：

- 动作顺畅、无异响
- 供电不掉压

### D4. 给“舵盘/输出轴”做参考标记

做两条线：

1. 舵机壳体上的基准线（固定不动）
2. 舵盘/输出轴上的基准线（会动）

目的：以后你看到“转了多少”一眼就能判断。

---

## E. 装配开始：先把所有舵机打到安全位置（全中位）

在你把舵机固定到机械结构之前，先做一次全中位，避免某个舵机停在危险角：

```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py
```

说明：

- 这一步只是“安全起点”
- 接下来要开始 **逐关节调整到 rest 的目标位置** 再装配

---

## F. 关键步骤：按 `rest` 逐关节装配（超详细）

### F0. 你必须先确认“关节名 -> 舵机 ID”的映射

你当前工程推荐映射（与文档一致）：

- ID1 → `shoulder_pan`
- ID2 → `shoulder_lift`
- ID3 → `elbow_flex`
- ID4 → `wrist_flex`
- ID5 → `wrist_roll`
- ID6 → `gripper`

如果你实际装配想换（比如某个舵机线长问题），务必先在纸上确认，再保证：

- 舵机 ID 设置
- `JOINT_ID_MAP`

三者一致。

> `JOINT_ID_MAP` 在：`~/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`

---

### F1. shoulder_pan（底座旋转，ID1）

**目标：** `rest` 里 `shoulder_pan = 0 rad`。

#### F1.1 装配姿态定义（你必须先定）

把底座放在桌上，定义一个“正前方”方向（例如底座某个固定边/某颗螺丝孔指向前方）。

在底座上贴一条胶带，写上“前”。

#### F1.2 让舵机停在“0 rad 对应的位置”

这里不要纠结 `pos=500` 是不是 0 rad。因为“0 rad 对应的舵机位置”应由你装配定义决定。

最稳的做法：

- 暂时先让 ID1 维持中位（pos=500），把它作为安装起点。
- 然后安装机械件时，把“输出轴的角度”对齐到你定义的“正前方”。

#### F1.3 安装步骤

1. 固定 ID1 舵机到 base 结构（只上紧到“不会晃”，先别锁死）
2. 装上旋转平台/肩部支架时，让其对齐你定义的“正前方”
3. 上紧输出轴固定螺丝
4. 再把舵机安装螺丝全部锁紧

#### F1.4 小幅验证（必须做）

用极小幅度让它左右动一下（±10°），确认不刮碰：

- 你可以临时用 ROS 启动 `hx35hm_bridge` 后从话题发命令
- 或者先用裸 SDK 脚本（如果你愿意我可以再给你一个“一键单舵机到指定 pos”的小脚本）

---

### F2. shoulder_lift（肩抬升，ID2）

**目标：** `rest` 里 `shoulder_lift = -1.57 rad`（约 -90°）。

这一步是“折叠姿态”的关键关节之一，不要用“全中位”装完。

#### F2.1 先把机械臂摆到“折叠 rest”想要的几何形态

不用通电先手摆一下（不装连杆也行）：

- 上臂应该是朝下/朝后折叠（具体取决于你的结构定义）

你需要目视确认：这个折叠形态是否是你想要的“休息位”。

#### F2.2 再让 ID2 舵机输出轴对齐到这个折叠形态

装配动作：

1. 先固定 ID2 舵机到肩部支架
2. 把上臂段摆到你想要的折叠角
3. 在这个角度下安装舵盘/连杆，让它自然贴合（不许硬掰）
4. 上紧固定螺丝

#### F2.3 小幅验证

让该关节在折叠位附近小幅摆动 ±10°：

- 正向应是你定义的 `+` 方向（之后要写进 `joint_directions`）
- 反向不应马上撞到结构

如果一动就撞：说明你装配角度选得太靠近极限，需要重新选更居中的装配位置或调整限位。

---

### F3. elbow_flex（肘，ID3）

**目标：** `rest` 里 `elbow_flex = +1.57 rad`（约 +90°）。

这也是折叠姿态关键关节。

装法与肩关节一样：

1. 固定 ID3 到肘关节座
2. 把前臂摆到折叠的 90°附近（参考你希望的折叠形态）
3. 让连杆自然对齐再锁紧
4. 小幅 ±10° 测试，确认不会干涉

---

### F4. wrist_flex（腕俯仰，ID4）

**目标：** `rest` 里 `wrist_flex = 0.75 rad`（约 +43°）。

腕部通常更容易出现“装上就干涉”的问题，原因是末端结构复杂。

步骤：

1. 固定 ID4
2. 把腕部摆到你想要的折叠角（43°只是软件定义，装配时以“末端不干涉 + 与 URDF 显示一致”为准）
3. 连杆自然贴合锁紧
4. 小幅 ±10° 测试

---

### F5. wrist_roll（腕滚转，ID5）

**目标：** `rest` 里 `wrist_roll = 0 rad`。

roll 关节的“0 rad”通常是“夹爪面朝下/面朝前”的某个定义，这个必须你自己明确。

推荐定义：

- 夹爪在 rest 姿态下“平行于桌面”或“垂直于桌面”，二选一

步骤：

1. 固定 ID5
2. 把夹爪座转到你定义的 0 角
3. 锁紧
4. 小幅 ±10° 测试

---

### F6. gripper（夹爪，ID6）

夹爪不建议把“完全闭合/完全张开”当作装配参考。

推荐：装配时让夹爪处于“半开”，这样装连杆更容易，且后续有余量调。

步骤：

1. 让夹爪舵机停在半开位置（例如 `pos=500`）
2. 装连杆，使两边对称
3. 小幅开合测试，确认不会卡死

---

## G. 装完之后必须做的 3 个软件对齐（否则后面一定难受）

### G1. 记录表（必须写下来）

写一张表（建议直接贴在文档/纸上）：

- `joint -> servo_id`
- `joint -> direction (+1/-1)`
- `joint -> 0 rad 对应的 servo pos`

没有这张表，你以后一定会忘。

### G2. 用 MoveIt 验证“rest 是否真的 rest”

启动单臂 MoveIt（HX-35HM）：

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py use_sim_time:=false
```

在 RViz 里把机械臂拉到：

- `rest`（折叠）附近
- `zero`（全 0）附近

观察：

- 模型显示与实物是否同向
- 有没有关节反向
- 有没有明显偏差（偏差大就回到装配步骤重装，不要指望软件补一切）

### G3. 在 bridge 里固化方向/零位（关键）

`hx35hm_bridge` 支持这两个关键参数（与 joint_names 顺序一致）：

- `joint_directions`：例如 `[-1, 1, 1, -1, 1, 1]`
- `joint_zero_positions`：例如 `[480, 510, 495, 505, 500, 500]`

你可以通过 launch 传参，或者在你专用 launch 里写死它们（推荐写 launch，便于复现）。

---

## H. 如果你选的是 `zero`（全 0）装配，怎么改？

把上面 F2-F5 的“折叠角”全部替换为：

- 机械臂在你认为的“零位”几何形态
- 装配时每个关节都对齐到 `0 rad`

其余流程不变：仍然是逐关节、小幅验证、最后固化方向/零位表。

---

## I. 强烈建议你让我继续做的一件事（会让装配快很多）

你现在缺一个“装配专用控制脚本”，能做到：

- `--servo-id X --pos Y` 单关节定位
- 或者直接 `--pose rest` / `--pose zero` 自动把各关节打到目标位置（目标来自 SRDF）

这样你装每个关节时不用改脚本/手算。

你一句话确认：你要按 `rest` 装配还是 `zero` 装配？
我就把这个装配脚本给你补上（并且把方向/零位做成可配置 YAML）。



脚本已补齐（按 `rest` 装配可直接用），包含你要的两种模式：

- 装配控制脚本：`so101_hx35hm_bridge/scripts/so101_assembly_pose.py`
- 配置文件模板（关节->舵机ID/方向/零位pos）：`so101_hx35hm_bridge/config/assembly_calibration.yaml`

**1) 一键把机械臂打到 SRDF 的 `rest`（折叠）姿态**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest
```
默认会打印表格并要求你输入 `YES` 才会真正下发；先预览用：
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest --dry-run --yes
```

**2) 单舵机定位（你装某个关节时用）**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --servo-id 2 --pos 500
```

**3) 列出 SRDF 里可用姿态（rest/zero/extended）**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --list-poses --pose rest
```

**4) 可选：装配时顺便处理夹爪**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest --gripper neutral
# 也可以 open/closed（按 config 里的 open_rad/closed_rad）
```

你接下来只需要改 `so101_hx35hm_bridge/config/assembly_calibration.yaml` 里的 `direction` 和 `zero_pos`（每个关节通常都不一样），脚本会自动按 SRDF 的 `rest` 角度算出每个舵机应该去的 `pos`。






脚本已补齐（按 `rest` 装配可直接用），包含你要的两种模式：

- 装配控制脚本：`so101_hx35hm_bridge/scripts/so101_assembly_pose.py`
- 配置文件模板（关节->舵机ID/方向/零位pos）：`so101_hx35hm_bridge/config/assembly_calibration.yaml`

**1) 一键把机械臂打到 SRDF 的 `rest`（折叠）姿态**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest
```
默认会打印表格并要求你输入 `YES` 才会真正下发；先预览用：
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest --dry-run --yes
```

**2) 单舵机定位（你装某个关节时用）**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --servo-id 2 --pos 500
```

**3) 列出 SRDF 里可用姿态（rest/zero/extended）**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --list-poses --pose rest
```

**4) 可选：装配时顺便处理夹爪**
```bash
python3 ~/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py --pose rest --gripper neutral
# 也可以 open/closed（按 config 里的 open_rad/closed_rad）
```

你接下来只需要改 `so101_hx35hm_bridge/config/assembly_calibration.yaml` 里的 `direction` 和 `zero_pos`（每个关节通常都不一样），脚本会自动按 SRDF 的 `rest` 角度算出每个舵机应该去的 `pos`。




/home/rog/.openclaw/workspace/clean_so101_arm.sh

## J. 当前推荐启动命令

### J1. 纯机械臂 + MoveIt（不启视觉）

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py use_octomap:=false use_rviz:=true use_cameras:=false
```

### J2. 机械臂 + MoveIt + 奥比中光 Astra 顶视 RGBD（当前默认高分辨率配置）

说明：
- RGB 走 `gscam`（UVC）
- 深度/点云走 `OpenNI2`
- 当前默认视觉配置已经指向 `so101_cameras_astra_overhead_rgbd.yaml`（RGB 640x480，更利于识别 ArUco）
- 当前 `cam_overhead` 使用的是人工可调静态外参，便于按真实桌面位置做快速校正

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_octomap:=false \
  use_rviz:=true \
  use_cameras:=true
```

### J2.1 机械臂 + MoveIt + Astra 点云避障（OctoMap）

当你已经确认 `/static_camera/points` 稳定后，可以直接开启 MoveIt OctoMap：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_octomap:=true \
  use_rviz:=true \
  use_cameras:=true
```

预期：
- MoveIt 启动时会加载 `octomap_pointcloud.yaml`
- 顶视点云 `/static_camera/points` 会进入 PlanningScene
- 机械臂规划时会考虑桌面上方的可见障碍物

如果只想单独拉起 Astra 视觉，不启动机械臂：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_bringup cameras.launch.py \
  cameras_config:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml
```

### J3. Astra 视觉启动后应检查的话题

```bash
ros2 topic list | grep static_camera
```

预期至少应有：
- `/static_camera/image_raw`
- `/static_camera/camera_info`
- `/static_camera/depth/image_raw`
- `/static_camera/depth/camera_info`
- `/static_camera/points`

### J4. 重启前清理旧进程

```bash
pkill -f move_group
pkill -f rviz2
pkill -f hx35hm_bridge
pkill -f robot_state_publisher
pkill -f openni2_camera_node
pkill -f gscam_node
```


cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view


ros2 run rqt_image_view rqt_image_view /static_camera/image_raw
ros2 run rqt_image_view rqt_image_view /static_camera/depth/image_raw




# ===== 终端1：启动主系统（机械臂+MoveIt+相机）=====
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_octomap:=false \
  use_rviz:=true \
  use_cameras:=true
# ===== 终端2：清理旧检测节点，只保留一个新版 green_block_detector =====
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
pkill -f green_block_detector || true
ros2 run so101_hx35hm_bridge green_block_detector
# ===== 终端3：检查视觉点和QoS =====
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /vision/green_block/point_base -v
# 期望：Publisher count=1，Durability=TRANSIENT_LOCAL

ros2 topic echo /vision/green_block/point_base
# 期望：持续输出 frame_id=base_link 的点
# ===== 终端4：先做“只规划不执行”验证 =====
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run so101_grasping so101_visual_pick --ros-args \
  -p execute:=false \
  -p point_timeout_s:=60.0
# ===== 终端4：再做“真实执行抓取” =====
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run so101_grasping so101_visual_pick --ros-args \
  -p execute:=true \
  -p point_timeout_s:=60.0

---

## K. ArUco 视觉标定（当前可用稳定流程）

### K1. 先确保系统里只跑一套主 launch（非常关键）

```bash
ps -ef | grep follower_hx35hm_moveit.launch.py | grep -v grep
```

如果出现两行及以上，先全部停掉再重开：

```bash
pkill -f follower_hx35hm_moveit.launch.py
pkill -f aruco_detector
```

### K2. 启动主系统（相机+MoveIt）

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_octomap:=false \
  use_rviz:=true \
  use_cameras:=true
```

### K3. 若 `/vision/aruco/*` 无输出，手动起检测节点（推荐排障用）

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run so101_hx35hm_bridge aruco_detector --ros-args \
  -p marker_id:=-1 \
  -p auto_dictionary:=true \
  -p marker_size_m:=0.02
```

### K4. 4条快速自检命令

```bash
ros2 topic hz /static_camera/image_raw
ros2 topic echo /static_camera/camera_info --once
ros2 topic echo /vision/aruco/debug_image --once
ros2 topic list | grep /vision/aruco
```

预期：
- `image_raw` 有帧率（约 20Hz）
- `camera_info` 能 `--once` 打印
- `debug_image` 能 `--once` 打印
- 至少有：`debug_image / detected_ids / pose_camera / pose_base`

### K5. 标定后直接做一次视觉抓取

当 `/vision/aruco/pose_base` 正常输出后，可以直接运行视觉抓取节点。  
默认流程是：读取 ArUco 的 `base_link` 位姿 -> 抬到预抓取位 -> 下到抓取位 -> 闭夹爪 -> 退回。

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run so101_grasping so101_visual_grasp --ros-args \
  -p pose_topic:=/vision/aruco/pose_base \
  -p execute:=false
```

先建议只规划不执行，确认 RViz 里的路径合理后，再把 `execute:=true` 打开。  
如果抓取点太高或太低，优先调这两个参数：
- `pregrasp_offset_m`
- `grasp_z_offset_m`

### K6. 直接测试红色圆形物块抓取

如果目标不是 ArUco，而是红色圆形物块，可以直接启动红色检测节点，再把检测输出喂给视觉抓取节点：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_octomap:=false \
  use_rviz:=true \
  use_cameras:=true \
  use_aruco_detector:=false \
  use_red_detector:=true
```

先确认检测结果：
```bash
ros2 topic echo /vision/red_block/pose_base --once
ros2 topic echo /vision/red_block/debug_image --once
```

然后先只规划：
```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  pose_topic:=/vision/red_block/pose_base \
  execute:=false
```

确认 RViz 路径合理后，再改成：
```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  pose_topic:=/vision/red_block/pose_base \
  execute:=true
```




完整流程如下，按顺序开两个终端就行。

**终端1：启动主系统**
```bash
pkill -f follower_hx35hm_moveit.launch.py || true
pkill -f tf_overhead_cam_moveit || true
pkill -f tf_wrist_cam_moveit || true
pkill -f static_transform_publisher || true
pkill -f move_group || true
pkill -f rviz2 || true
pkill -f hx35hm_bridge || true
pkill -f red_circle_detector || true
sleep 2

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_bringup follower_hx35hm_moveit.launch.py \
  use_octomap:=false \
  use_rviz:=true \
  use_cameras:=true \
  use_aruco_detector:=false \
  use_red_detector:=true
```

**终端2：先确认红球位姿正常**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /vision/red_block/pose_base --once
```

正常应当是这种量级：
- `x` 在机械臂前方
- `y` 在左右小范围内
- `z` 接近桌面高度附近

**终端2：先做一次只规划不执行**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  pose_topic:=/vision/red_block/pose_base \
  execute:=false
```

**终端2：确认没问题后做真实抓取**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  pose_topic:=/vision/red_block/pose_base \
  execute:=true
```

**如果方向还想微调**
现在 launch 已支持四元数参数，可这样覆盖：
```bash
ros2 launch so101_grasping so101_visual_grasp.launch.py \
  pose_topic:=/vision/red_block/pose_base \
  execute:=true \
  qx:=0.0 qy:=0.0 qz:=0.0 qw:=1.0
```

**当前默认已经修好的内容**
- 红球检测已启用
- 相机 TF 已回到可用人工外参
- 夹爪 action 已打通
- 默认末端朝向已修正
- 抓取优先按位置目标规划，不再优先走“近似不动解”

如果你愿意，我下一步可以把这整套直接补写进 [HX35HM_SO101_超详细装配流程.md](/home/rog/ros2_ws/src/HX35HM_SO101_超详细装配流程.md)。