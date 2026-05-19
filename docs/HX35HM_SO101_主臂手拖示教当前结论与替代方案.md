# HX35HM SO101 主臂手拖示教当前结论与替代方案

适用场景：

- 已经完成主臂 `/dev/so101_leader` 与从臂 `/dev/so101_follower` 串口绑定
- 已经能够启动 `teleop_hx35hm.launch.py`
- 发现主臂在某些时刻可以手动拖动，但一旦重新启动主从控制又重新上力
- 需要对“为什么之前可以、现在不行”做一个稳定结论，避免继续在同一现象上反复试错

---

## 最终结论

当前这套：

- HX35HM 总线舵机
- STM32 控制板
- `ros_robot_controller_sdk.Board`
- `so101_hx35hm_bridge`
- `teleop_hx35hm.launch.py`

**不适合被当作一个稳定、可重复、可长期依赖的“通电手拖主臂示教系统”。**

更准确地说：

- 主臂**有时**会处于“松”的状态
- 但这个状态**不稳定**
- 一旦重新建立正式控制会话，主臂经常会重新上力
- 目前没有证据表明，这个问题可以仅靠 ROS 启动顺序或简单参数稳定解决

---

## 这次已经验证过的事实

### 1. 不是从臂问题

已经验证过：

- 从臂独立控制正常
- 从臂 3 号舵机单独控制正常
- 从臂在主从链里能收到并执行大部分跟随命令

所以问题不是：

- 从臂舵机坏了
- 从臂 bridge 起不来
- 从臂串口规则错误

### 2. 不是 teleop 在反向控制主臂

已经实际检查过：

- `/leader/forward_controller/commands` 存在
- 但 `Publisher count = 0`

也就是说：

- 没有节点在给主臂持续发目标位置命令
- 主臂重新上力，不是因为 teleop 把 follower 命令反灌回 leader

### 3. 不是单纯“进程残留”就能解释全部问题

确实存在过两类干扰：

- ROS daemon 缓存导致 `ros2 node list` 看起来像还有节点
- 旧进程残留会污染判断

但在清理到干净状态之后，重新启动主控从，主臂仍然会再次上力。

所以：

- 进程残留是干扰项
- 但不是根因

### 4. 主臂“松”这个现象是真实存在的

已经多次验证过：

- 停掉相关控制进程后
- 主臂会重新变松
- 可以手动拖动

这说明：

- 主臂硬件本身不是绝对不能拖动
- “之前可以”不是错觉

### 5. 但“松”不是一个可稳定复现的控制模式

现在最关键的结论是：

- 主臂在无人继续访问总线时，可能保持松
- 但只要重新建立正式控制会话，主臂经常重新上力

这说明当前系统更像是：

- 偶尔停在一个“幸运的松状态”
- 而不是进入了一个正式、可靠的“leader drag mode”

---

## 为什么会出现“之前可以，现在不行”

最合理的解释不是：

- 之前的方法完全正确
- 现在的方法完全错误

而是：

- 之前碰到的是一个**临时成立的硬件状态**
- 现在每次重新接入控制链，都会破坏这个状态

也就是：

1. 主臂停止后，舵机暂时变松
2. 当时没有别的节点继续碰主臂总线
3. 所以看起来像“手拖示教成功了”
4. 但下一次只要重新建立主臂控制会话
5. 主臂就重新回到上力态

所以“之前可以”更像是：

- 一个停机后的暂态

而不是：

- 一条已经稳定打通的 leader 方案

---

## 目前不应再继续假设的事情

下面这些假设，当前都不应再默认成立：

### 1. 不应再假设 `disable_torque_on_startup` 足够

即使 bridge 日志明确打印：

- `Disabling torque ... during startup`

也不能证明主臂此刻一定已经真正松掉。

### 2. 不应再假设加 keepalive 就一定能解决

即使已经改成：

- 启动时卸力
- 周期性重发卸力

主臂仍然可能上力。

### 3. 不应再假设问题只在 teleop 层

已经排查过：

- leader-only
- leader + follower
- leader + follower + teleop

说明问题并不只是“多了一层 relay”这么简单。

### 4. 不应再把“当前停机后能拖动”当作正式可交付能力

这只能说明：

- 现在松了

不能说明：

- 以后每次按同样方式启动都会松

---

## 这套方案当前最靠谱的判断

当前最靠谱的判断是：

**这套 HX35HM 主臂，在当前 STM32 协议与 bridge 路线下，不具备稳定、可重复的通电手拖示教能力。**

换句话说：

- 能不能拖动，不只取决于你是否启动了某个 ROS 节点
- 更取决于当前硬件/板卡/舵机总线会话进入了什么状态

这不是一个我们应该继续靠经验撞出来的生产方案。

---

## 后续建议路线

### 路线 A：停止继续消耗在“当前主臂通电手拖”上

适用情况：

- 你希望尽快得到一个可长期复用的方案
- 不想继续在当前这套硬件语义上来回试错

建议：

- 不再把当前主臂当作稳定 leader
- 改做非手拖 leader 方案
- 或者换到更接近官方 LeRobot 的 leader 设计

### 路线 B：把当前主臂仅视为“临时实验性 leader”

适用情况：

- 只是继续做实验
- 接受每次都需要手工停机、清理、重试

建议：

- 只把它用于短时验证
- 不要把“这次松了”当作稳定能力
- 每次测试前都先执行紧急停止和环境清理

### 路线 C：重新设计 leader 方案

优先考虑：

1. 更接近官方 LeRobot 的 leader 结构/实现
2. 使用专门适合 backdrivable / teach mode 的主臂
3. 退一步改成按钮式、增量式、离线采样式主控从

---

## 路线 C：推荐重新设计方案

### 核心原则

路线 C 不再把当前 HX35HM 主臂当作稳定手拖输入设备。

新的 leader 方案应该满足：

- leader 侧只负责稳定产生 `/leader/joint_states`
- follower 侧继续复用已经跑通的 HX35HM 执行链
- 中间继续复用 `so101_teleop` 做 `/leader/joint_states` 到 `/follower/forward_controller/commands` 的转发
- leader 侧不能依赖“停机后偶然变松”这种状态

也就是说，后续应该替换的是：

- 当前 `/dev/so101_leader` + `hx35hm_bridge` 这一层

而不是推翻：

- 从臂 HX35HM bridge
- `/follower/forward_controller/commands`
- `so101_teleop`

### 推荐目标架构

```text
专用 leader 输入设备
  -> /leader/joint_states
  -> so101_teleop
  -> /follower/forward_controller/commands
  -> follower hx35hm_bridge
  -> /dev/so101_follower
  -> 从臂舵机
```

这里的“专用 leader 输入设备”可以是下面任意一种：

- 官方 SO101 leader 路线
- LeRobot/Feetech 原生 leader 驱动
- 低阻尼编码器主臂
- 自制电位器/磁编码器示教臂
- 键盘/手柄/滑条式软件 leader

### 推荐优先级

#### 方案 C1：官方/LeRobot leader 路线

这是最推荐的长期方向。

原因：

- 仓库里已经有官方风格的 `leader.launch.py`
- 仓库里也已经有标定文件：
  - `so101_bringup/config/hardware/leader_joints.yaml`
  - `so101_bringup/config/hardware/lerobot_leader_arm.json`
- 官方 leader 的设计本来就是 teleoperator，不是普通 follower 舵机硬拖

当前可参考文件：

- `/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/leader.launch.py`
- `/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/teleop.launch.py`
- `/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hardware/leader_joints.yaml`
- `/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hardware/lerobot_leader_arm.json`

目标是让 leader 侧走：

```bash
ros2 launch so101_bringup leader.launch.py \
  namespace:=leader \
  usb_port:=/dev/so101_leader \
  joint_config_file:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hardware/leader_joints.yaml \
  use_rviz:=false
```

然后继续让 follower 走 HX35HM：

```bash
ros2 launch so101_bringup teleop_hx35hm.launch.py \
  launch_leader:=false \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false
```

注意：

- 这条路线要求 leader 侧硬件和官方驱动协议匹配
- 当前 HX35HM + STM32 leader 不一定能直接使用 `leader.launch.py`
- 如果 `leader.launch.py` 无法稳定发布 `/leader/joint_states`，就说明当前 leader 硬件不适合走这条官方驱动路线

当前实测结果：

- `leader.launch.py` 当前不能直接接管现有环境
- 报错点是缺少 `feetech_ros2_driver/FeetechHardwareInterface`
- 当前 ROS 环境只声明了 `mock_components/GenericSystem`
- 所以 C1 需要先补齐官方 Feetech 驱动环境，不能直接作为今天的可执行路线

#### 方案 C2：独立编码器 leader

如果官方 leader 路线不适配当前硬件，建议单独做一个“只读传感器”的 leader。

这个 leader 不需要舵机，也不需要上力。

它只需要发布：

```text
/leader/joint_states
```

最小要求：

- 6 个关节输入
- 每个关节输出 rad
- joint name 必须和现有 teleop 一致：
  - `shoulder_pan`
  - `shoulder_lift`
  - `elbow_flex`
  - `wrist_flex`
  - `wrist_roll`
  - `gripper`

优点：

- 不会再出现主臂上力
- 输入设备和执行臂彻底解耦
- 标定清楚，后续维护成本低

缺点：

- 需要额外硬件
- 需要写一个 encoder 到 `JointState` 的 ROS2 节点

#### 方案 C3：软件 leader

如果只是先验证主控从逻辑，可以先做软件 leader。

例如：

- 滑条 GUI
- 键盘增量控制
- 手柄控制
- 预设姿态序列

它同样只需要发布：

```text
/leader/joint_states
```

然后继续复用：

```text
so101_teleop -> /follower/forward_controller/commands -> follower hx35hm_bridge
```

优点：

- 不需要新的机械硬件
- 最容易验证 teleop 与 follower 执行链

缺点：

- 不是手拖示教
- 操作体验不如真实 leader

当前已经新增软件 leader：

- `/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/scripts/software_leader.py`
- `/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/software_leader_teleop_hx35hm.launch.py`

已经验证：

- 不打开 `/dev/so101_leader`
- 不会让主臂上力
- 可以稳定发布 `/leader/joint_states`
- 发布频率约 `50 Hz`
- 默认发布 `rest` 姿态：
  - `shoulder_pan = 0.0`
  - `shoulder_lift = -1.57`
  - `elbow_flex = 1.57`
  - `wrist_flex = 0.75`
  - `wrist_roll = 0.0`
  - `gripper = 0.0`

---

## 推荐落地顺序

### 第 1 步：保留从臂执行链

从臂不要推倒重来。

保留：

- `/dev/so101_follower`
- `hx35hm_follower_bridge_params.yaml`
- `/follower/joint_states`
- `/follower/forward_controller/commands`

先保证单独从臂仍然可以正常执行。

### 第 2 步：替换 leader 输入源

不要再让当前 HX35HM 主臂作为正式手拖 leader。

改成下面三选一：

- 官方/LeRobot leader
- 独立编码器 leader
- 软件 leader

只要新的 leader 能稳定发布 `/leader/joint_states`，后面的链路就不用大改。

### 第 3 步：继续复用 teleop relay

继续使用：

```bash
ros2 launch so101_teleop teleop.launch.py \
  leader_namespace:=leader \
  follower_namespace:=follower \
  arm_controller:=forward_controller
```

这一步只要求：

- `/leader/joint_states` 存在
- `/follower/forward_controller/commands` 有订阅者

### 第 4 步：只调标定，不再调主臂扭矩

新路线里，leader 不应该再需要：

- `disable_torque_on_startup`
- `maintain_torque_disabled`
- `torque_disable_keepalive_rate_hz`

后续调试重点只剩：

- 关节方向
- 零位
- 比例
- 限位

---

## 最小验收标准

重新设计后的 leader 方案，必须同时满足：

1. 启动后主臂不会上力
2. `/leader/joint_states` 连续发布
3. 手动改变 leader 输入时，对应关节角有明显变化
4. `/follower/forward_controller/commands` 跟随变化
5. 从臂执行动作
6. 停止后不依赖 ROS daemon 缓存判断状态，必须能用 `ps` 确认进程退出

如果某个方案做不到第 1 条，它就不适合作为正式 leader。

---

## 推荐验证命令

### 1. 先清理环境

```bash
pkill -f 'teleop_hx35hm.launch.py|teleop.launch.py|follower_command_relay|hx35hm_bridge|leader_hx35hm.launch.py|leader.launch.py|follower_hx35hm_moveit.launch.py|move_group|rviz2|robot_state_publisher|ros2_control_node|spawner' || true
source /opt/ros/jazzy/setup.bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

最后一条命令理想情况没有节点输出。

### 2. 单独启动新的 leader

如果使用官方/LeRobot leader 路线：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 launch so101_bringup leader.launch.py \
  namespace:=leader \
  usb_port:=/dev/so101_leader \
  joint_config_file:=/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hardware/leader_joints.yaml \
  use_rviz:=false
```

另开终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 topic hz /leader/joint_states
ros2 topic echo /leader/joint_states --once
```

这一阶段只看 leader 是否稳定发布，不启动从臂。

如果使用软件 leader 路线：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 run so101_bringup software_leader.py --ros-args -r __ns:=/leader
```

如果当前没有桌面图形环境，可以用无 GUI 模式：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 run so101_bringup software_leader.py --no-gui --ros-args -r __ns:=/leader
```

检查：

```bash
ros2 topic echo /leader/joint_states --once
ros2 topic hz /leader/joint_states
```

### 3. 单独启动从臂 HX35HM 执行端

可以继续使用现有从臂独立链路，或者只启动 follower bridge。

关键检查：

```bash
ros2 topic hz /follower/joint_states
ros2 topic info /follower/forward_controller/commands -v
```

### 4. 启动 teleop relay

当 `/leader/joint_states` 和 `/follower/joint_states` 都稳定后，再启动 relay：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 launch so101_teleop teleop.launch.py \
  leader_namespace:=leader \
  follower_namespace:=follower \
  arm_controller:=forward_controller
```

检查：

```bash
ros2 topic hz /follower/forward_controller/commands
```

如果这个 topic 有 50Hz 左右输出，说明 leader 到 follower 的转发层已经工作。

### 5. 一条命令启动软件 leader + 从臂 HX35HM

用于验证“完全不碰主臂串口”的主从链：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 launch so101_bringup software_leader_teleop_hx35hm.launch.py
```

如果当前机器没有图形界面：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 launch so101_bringup software_leader_teleop_hx35hm.launch.py use_gui:=false headless:=true
```

---

## 迁移时的关键注意点

- 不要同时运行 `teleop_hx35hm.launch.py` 和新的 leader 路线。
- 不要让两个进程同时打开 `/dev/so101_leader`。
- 如果走官方 `leader.launch.py`，就不要再启动 `/leader/hx35hm_bridge`。
- 从臂可以继续使用 HX35HM bridge，因为从臂执行链已经证明可用。
- 新 leader 的唯一硬性接口是 `/leader/joint_states`，只要这个 topic 正确，后面链路就能复用。

---

## 当前建议

如果目标是：

- 稳定
- 可重复
- 可交付
- 后面还能继续维护

那么建议：

**停止继续把当前 HX35HM 主臂通电手拖当作正式方案推进。**

下一步更值得投入的是：

- 重新选 leader 方案
- 或靠近官方 LeRobot leader 路线
- 或定义一个非手拖的主控从方案

---

## 相关文档

- [HX35HM_SO101_主从控制紧急停止命令.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主从控制紧急停止命令.md)
- [HX35HM_SO101_STM32串口绑定_舵机编号_回中完整手册.md](/home/rog/ros2_ws/docs/HX35HM_SO101_STM32串口绑定_舵机编号_回中完整手册.md)
- [HX35HM_SO101_机械臂控制链详解.md](/home/rog/ros2_ws/docs/HX35HM_SO101_机械臂控制链详解.md)

---

## 2026-05-17 实测更新：当前 HX35HM 主臂已经可以作为实验性 leader 使用

本节覆盖上文较早阶段的判断。经过后续协议修正和实测，当前结论已经从“主臂无法稳定卸力”更新为：

```text
当前 HX35HM + STM32 主臂可以稳定进入只读卸力 leader 状态。
它已经能驱动从臂跟随。
当前主要问题已从“能否手拖”转移到“输入阶梯、跟随顿挫、三号关节异常”。
```

### 已确认的关键修正

1. 重新核对幻尔 RRC 教程后，确认当前 SDK 中 torque 子命令语义曾被理解反：

   ```text
   0x0B = 掉电/卸力
   0x0C = 上电/加载
   ```

2. 修正 `ros_robot_controller_sdk.py` 后，leader bridge 启动时可以把主臂稳定打到卸力状态。
3. leader bridge 当前配置为只读：

   ```text
   enable_command_subscription: false
   disable_torque_on_startup: true
   restore_torque_on_shutdown: false
   ```

4. 当前主控从臂启动后，主臂可以手动拖动，从臂可以跟随。

### 当前正式架构

```text
/dev/so101_leader
  -> /leader/hx35hm_bridge
  -> /leader/joint_states
  -> /follower_command_relay
  -> /follower/forward_controller/commands
  -> /follower/hx35hm_bridge
  -> /dev/so101_follower
```

关键实现：

- leader 只读，不接收位置命令。
- relay 使用 `relative` mapping。
- follower 使用异步固定频率写入。
- 第一关节已经加入自动标定 offset。

### 当前基线参数

```text
leader state_publish_rate_hz = 100.0
leader position_readback_rate_hz = 180.0

relay publish_rate_hz = 75.0
relay filter_mode = lpf
relay lpf_alpha = 0.55
relay joint_offsets[0] = 0.062832

follower stream_command_async_write = true
follower stream_write_rate_hz = 75.0
follower stream_target_smoothing = true
follower stream_max_velocity_rad_s = 2.5
follower stream_command_duration = 0.03
```

### 已做过但不建议继续使用的优化

#### 1. leader readback 240Hz

结果：

```text
出现大量串口 Input/output error
```

结论：

```text
当前 STM32 + SDK 逐舵机轮询链路不适合 240Hz。
180Hz 是当前更稳的高频参数。
```

#### 2. alpha-beta 预测滤波

结果：

```text
抖动明显，顿挫仍明显。
```

原因：

```text
leader 输入是轮询阶梯信号。
alpha-beta 在这种输入上会把台阶变化解释成速度，反而放大抖动。
```

结论：

```text
当前保留 LPF，不启用 alpha-beta。
```

### 最新诊断数据

通过：

```bash
python3 src/so101_hx35hm_bridge/scripts/diagnose_teleop_smoothness.py \
  --duration 15 \
  --output /tmp/so101_teleop_smoothness_report_moving.json
```

得到的关键结果：

```text
leader shoulder_pan  median step ~= 2.16 deg
leader shoulder_lift median step ~= 2.88 deg
leader elbow_flex    median step ~= 2.16 deg

command shoulder_pan  median step ~= 0.24 deg
command shoulder_lift median step ~= 0.29 deg
command elbow_flex    median step ~= 0.28 deg
```

这说明：

```text
relay 和 follower 写入端已经把命令平滑成较小步长。
剩余全局顿挫的主要来源是 leader 侧 round_robin 回读本身的阶梯输入。
```

另一个更严重的实测点：

```text
elbow_flex command_vs_follower median_abs_error ~= 13.1 deg
elbow_flex command_vs_follower p95_abs_error    ~= 63.3 deg
```

因此当前问题优先级已更新为：

```text
P0: 保持主臂稳定卸力和串口绑定
P1: 处理三号关节 elbow_flex 的独立异常
P2: 再处理全局顿挫
P3: 长期优化 leader 采样架构
```

### 当前结论

旧结论：

```text
当前 HX35HM 主臂不适合作为稳定 leader。
```

已经不再准确。

当前更准确的结论：

```text
当前 HX35HM 主臂已经可以作为实验性 leader 工作。
但它仍不是高质量 leader：leader 输入是轮询阶梯信号，且三号关节尚未修正。
如果目标是接近官方 leader 的顺滑手感，最终仍需要优化 leader 采样架构，优先考虑 STM32 批量状态帧或专用 leader 硬件。
```

### 当前配套文档

- [HX35HM_SO101_主从控制当前使用流程.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主从控制当前使用流程.md)
- [HX35HM_SO101_主从跟随顿挫与三号关节优化方案.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主从跟随顿挫与三号关节优化方案.md)
- [HX35HM_SO101_主从控制紧急停止命令.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主从控制紧急停止命令.md)
