# HX35HM SO101 STM32 底层协议与主臂拖动可行性研究

日期：2026-05-13

目标：判断当前 STM32 控制板 + HX-35HM 总线舵机方案，是否有希望实现“手动拖动主臂，实时控制从臂”。

## 结论先说

有希望，但不能继续按“普通位置控制 bridge”来做。

HX-35HM 舵机本体从手册看，具备“掉电/卸力后仍可手动转动并读取位置”的能力。这意味着真正的 leader teleop 并非物理上不可行。

当前卡点更像是软件/协议路径问题：

1. PC 侧 `ros_robot_controller_sdk.py` 只暴露了位置控制、停止、扭矩开关、ID/偏差/限位/电压/温度/位置读取等有限接口。
2. 本仓库没有发现 STM32 固件源码，所以 PC 端 Python/C++ 只能通过现有 STM32 固件提供的封装命令工作。
3. 当前 `hx35hm_bridge` 本质还是一个“位置控制 bridge”。如果它在 leader 上订阅命令、恢复扭矩、或者有其他节点误发命令，主臂就会重新上力。
4. 要变成真正 leader，需要让主臂保持“舵机卸力 + 只读位置回传”，并保证任何 ROS 节点都不能向主臂发位置控制命令。

## 本地证据

### 1. HX-35HM 手册证据

本地手册：

```bash
/home/rog/ros2_ws/src/1.HX-35HM总线舵机使用说明.pdf
```

手册关键信息：

1. HX-35HM 是半双工 UART 总线舵机，波特率 `115200`。
2. 支持角度回读，参数反馈包括温度、电压、位置。
3. 工作模式包括舵机模式和减速电机模式。
4. 上位机“Servo 模式”里有“马达上/掉电控制”。
5. 手册明确说明：掉电后可以手动转动舵臂，并在软件右侧面板看到手动转动后的位置信息。

这条第 5 点是最关键证据：HX-35HM 本体理论上支持“卸力但读位置”。

### 2. 当前 STM32 SDK 暴露的总线舵机命令

PC 侧 SDK：

```bash
/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py
```

STM32 外层数据包格式由 SDK 生成：

```text
0xAA 0x55 function length data... crc8
```

总线舵机使用：

```text
function = 0x05  # PACKET_FUNC_BUS_SERVO
```

当前 SDK 中已经暴露的 bus servo 子命令：

| SDK 方法 | 子命令 | 用途 |
|---|---:|---|
| `bus_servo_set_position` | `0x01` | 位置控制 |
| `bus_servo_stop` | `0x03` | 停止当前运动 |
| `bus_servo_read_position` | `0x05` | 读取位置 |
| `bus_servo_read_vin` | `0x07` | 读取电压 |
| `bus_servo_read_temp` | `0x09` | 读取温度 |
| `bus_servo_enable_torque(True)` | `0x0C` | 上扭矩/加载 |
| `bus_servo_enable_torque(False)` | `0x0B` | 关扭矩/卸载 |
| `bus_servo_read_torque_state` | `0x0D` | 读取扭矩状态 |
| `bus_servo_set_id` | `0x10` | 设置 ID |
| `bus_servo_read_id` | `0x12` | 读取 ID |
| `bus_servo_set_offset` | `0x20` | 设置偏差 |
| `bus_servo_read_offset` | `0x22` | 读取偏差 |
| `bus_servo_save_offset` | `0x24` | 保存偏差 |
| `bus_servo_set_angle_limit` | `0x30` | 设置角度限位 |
| `bus_servo_read_angle_limit` | `0x32` | 读取角度限位 |
| `bus_servo_set_vin_limit` | `0x34` | 设置电压限位 |
| `bus_servo_read_vin_limit` | `0x36` | 读取电压限位 |
| `bus_servo_set_temp_limit` | `0x38` | 设置温度限位 |
| `bus_servo_read_temp_limit` | `0x3A` | 读取温度限位 |

没有看到这些接口：

1. 柔顺模式。
2. 阻尼模式。
3. 刚度设置。
4. 电流限制。
5. 低力矩保持。
6. 原始舵机协议透传。

所以如果只用当前 SDK，我们只有两条路：

1. `0x0B` 卸力后持续读 `0x05` 位置。
2. 完全不用主臂物理舵机做 leader，改用软件 leader 或专用 leader 硬件。

### 3. 当前 ROS bridge 的风险点

当前 bridge：

```bash
/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py
```

它原本既能读位置，也能订阅 `forward_controller/commands` 并向舵机发送位置命令。

对于 follower，这是正确的。

对于 leader，这是危险的。leader 应该只读，不应该接受任何位置命令。

否则只要有节点向 `/leader/forward_controller/commands` 发消息，或 bridge 退出时恢复扭矩，主臂就会重新上力。

本次已经做了两处收紧：

1. 增加参数 `enable_command_subscription`。
2. leader 配置默认 `enable_command_subscription: false`。
3. leader 配置默认 `restore_torque_on_shutdown: false`。

目标是让 leader bridge 成为“只读卸力设备”，而不是“可控机械臂”。

## 为什么 C++/C 不一定能直接解决

C++/C 有价值，但它不能绕过固件能力边界。

如果仍然通过当前 STM32 板子的 USB 串口访问，那么 C++ 也只能发送 STM32 固件支持的外层命令。也就是说：

```text
PC C++/Python -> STM32 固件协议 -> STM32 固件 -> HX-35HM 舵机总线
```

如果 STM32 固件没有提供柔顺/阻尼/电流/原始透传接口，那么换 C++ 并不会多出这些能力。

C++ 真正有价值的场景：

1. 我们拿到了 STM32 固件源码，可以在固件里新增原始总线透传或新增舵机寄存器命令。
2. 我们绕过 STM32，用 USB-TTL 半双工适配器直接连 HX-35HM 总线，在 PC 侧用 C++ 实现舵机底层协议。
3. 当前 Python 读写时序不稳定，但协议能力已经足够，此时 C++ 可以提升实时性和稳定性。

## 推荐技术路线

### 路线 C1：先把现有 STM32 路线做成 read-only leader

这是当前最小改动路线。

leader 规则：

1. 启动后只读位置。
2. 启动后立刻发送卸力命令。
3. 周期性发送卸力 keepalive。
4. 不订阅任何 leader 控制命令。
5. 退出时不恢复扭矩。
6. teleop 只把 `/leader/joint_states` 映射到 `/follower/forward_controller/commands`。

优点：

1. 不需要新硬件。
2. 和现有 ROS2 架构兼容。
3. 可以最快验证“卸力后位置回读是否足够稳定”。

风险：

1. 如果 STM32 固件的 `0x0B` 卸力命令并不等价于官方上位机的“马达掉电”，则主臂仍可能发紧。
2. 如果位置回读在卸力后偶发丢失，teleop 会抖或卡。
3. 如果还有残留进程向 leader 发位置命令，主臂会重新上力。

### 路线 C2：改 STM32 固件，增加真正 leader 模式

如果能拿到 STM32 固件源码，这是最值得做的硬件级路线。

固件应新增一个 leader 专用模式：

1. 上电后不对主臂舵机发位置控制。
2. 对 1-6 号舵机发送卸力/掉电。
3. 固件内部固定频率轮询位置。
4. 通过 USB 串口主动上报 6 个舵机位置。
5. 可选：提供原始 HX-35HM 总线透传命令。

优点：

1. PC 侧不再频繁做请求-应答，实时性会更好。
2. 可以彻底避免“普通 bridge 误发控制命令”。
3. 如果 HX-35HM 有更多寄存器能力，固件可以直接支持。

风险：

1. 需要 STM32 固件源码和烧录环境。
2. 需要确认当前控制板总线硬件方向控制方式。

### 路线 C3：绕过 STM32，直接控制 HX-35HM 总线

如果拿不到 STM32 固件源码，但想继续深挖 HX-35HM 能力，建议使用 USB-TTL 半双工总线适配器直接接主臂舵机总线。

验证目标：

1. 直接发送舵机底层 `load/unload` 命令。
2. 卸力后直接发送 `position read`。
3. 手动转动时确认位置是否连续变化。
4. 确认 1-6 号舵机串联时轮询频率是否足够。

如果这个验证成功，再写 C++ ROS2 节点是合理的。

节点结构：

```text
hx35hm_raw_leader_node
  - 串口：115200，半双工 UART
  - 启动：unload 1..6
  - 循环：read position 1..6
  - 发布：/leader/joint_states
  - 禁止：任何 set_position / load torque
```

优点：

1. 不受当前 STM32 固件封装限制。
2. 更接近官方 leader teleoperator 的“只读传感器”思路。

风险：

1. 需要确认线序、电平、半双工方向控制。
2. 需要更仔细做电源共地和保护。
3. 需要实现底层协议和异常恢复。

### 路线 C4：如果卸力回读仍不可靠，换 leader 硬件

如果最终确认 HX-35HM 在 SO101 主臂结构上无法稳定“卸力且回读”，那就不建议继续硬拗。

更接近官方 LeRobot leader 的方案是：

1. 用无刷/无力矩/低阻尼关节编码器做 leader。
2. 或使用可配置 current/torque/compliance 的舵机。
3. leader 只输出角度，不负责施力。

这条路线机械上更干净，安全性也更高。

## 当前应执行的验证

### 1. 杀掉所有相关进程

```bash
pkill -f 'teleop_hx35hm.launch.py|software_leader_teleop_hx35hm.launch.py|leader_hx35hm.launch.py|follower_hx35hm_moveit.launch.py|follower_command_relay|hx35hm_bridge|move_group|rviz2|robot_state_publisher' || true
```

确认没有 ROS 节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 node list
```

### 2. 单独测试主臂卸力和位置回读

```bash
cd /home/rog/ros2_ws
python3 src/so101_hx35hm_bridge/scripts/leader_drag_mode.py \
  --device /dev/so101_leader \
  --mode enter \
  --ids 1 2 3 4 5 6 \
  --yes
```

然后手动转动主臂，同时看位置是否变化：

```bash
python3 src/so101_hx35hm_bridge/scripts/leader_drag_mode.py \
  --device /dev/so101_leader \
  --mode verify-readback \
  --ids 1 2 3 4 5 6 \
  --rounds 20 \
  --timeout 0.2
```

判断：

1. 如果主臂松，且 1-6 都能随手动转动持续变化，现有 STM32 路线可以继续。
2. 如果主臂松，但位置不稳定，优先降低轮询频率、拉长 timeout、排查总线供电和接线。
3. 如果主臂不松，说明当前 STM32 的卸力命令路径和官方上位机“掉电控制”不一致，需要路线 C2 或 C3。

### 3. 启动 read-only leader + follower teleop

构建后启动：

```bash
cd /home/rog/ros2_ws
colcon build --packages-select so101_hx35hm_bridge so101_bringup
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash

ros2 launch so101_bringup teleop_hx35hm.launch.py \
  leader_rviz:=false \
  follower_rviz:=false \
  use_teleop_rviz:=false
```

检查 leader 没有 command subscriber：

```bash
ros2 topic info /leader/forward_controller/commands
```

理想情况：

```text
Publisher count: 0
Subscription count: 0
```

检查数据链：

```bash
ros2 topic hz /leader/joint_states
ros2 topic hz /follower/forward_controller/commands
ros2 topic hz /follower/joint_states
```

## 当前代码处理状态

已做：

1. `hx35hm_bridge` 新增 `enable_command_subscription` 参数。
2. leader 配置设置 `enable_command_subscription: false`。
3. leader 配置设置 `restore_torque_on_shutdown: false`。
4. `leader_hx35hm.launch.py` 强制 leader bridge 禁用命令订阅。

这一步不是最终方案，但它先把主臂从“可被控制的机械臂”改成“只读传感器”的方向，符合真正 leader 的设计。

## 最终判断

当前最有希望的方案是：

```text
HX-35HM 主臂舵机卸力
  -> STM32 或直连总线只读取位置
  -> 发布 /leader/joint_states
  -> teleop C++ 映射
  -> follower bridge 控制从臂
```

如果 read-only leader 验证成功，就继续优化 ROS2 和滤波。

如果 read-only leader 仍然一启动就紧，或者卸力后无法稳定回读，就不要继续在当前 STM32 封装里绕圈，直接转路线 C2 或 C3：

1. 拿 STM32 固件源码，加真正 leader/raw passthrough 模式。
2. 或用 USB-TTL 半双工适配器直连 HX-35HM 总线，写 C++ raw leader driver。

## 2026-05-13 实测补充：当前 STM32 封装不能让主臂真正卸力

实测环境：

```text
/dev/so101_leader -> ttyACM1
/dev/so101_follower -> ttyACM0
```

已确认：

1. 主臂 STM32 通信正常，IMU 能回读。
2. 主臂 1-6 号舵机位置能稳定回读。
3. `teleop_hx35hm.launch.py` 下，leader 已经是 read-only：
   - `/leader/joint_states` 正常 50Hz 发布。
   - `/follower/forward_controller/commands` 正常 50Hz 发布。
   - `/leader/forward_controller/commands` 不存在。
4. 用户实测手感：主臂仍然非常紧。

进一步只对主臂执行无运动的底层命令：

```text
before:
1 torque=[1] pos=[516]
2 torque=[1] pos=[535]
3 torque=[1] pos=[850]
4 torque=[1] pos=[458]
5 torque=[1] pos=[461]
6 torque=[1] pos=[554]

send stop all
send torque disable x3

after:
1 torque=[1] pos=[516]
2 torque=[1] pos=[535]
3 torque=[1] pos=[850]
4 torque=[1] pos=[458]
5 torque=[1] pos=[461]
6 torque=[1] pos=[554]
```

结论：

1. 当前 `ros_robot_controller_sdk.py` 的 `bus_servo_enable_torque(id, 0)` 没有把 HX-35HM 打到手册里的“马达掉电/卸力”状态。
2. `bus_servo_stop()` 也不是卸力模式。
3. 当前问题已经不是 ROS teleop 误发命令，而是 STM32 固件封装没有提供真正的 HX-35HM `SERVO_LOAD_OR_UNLOAD_WRITE = 31` 语义，或者该封装命令与 HX-35HM 不兼容。
4. 继续在现有 `hx35hm_bridge` 上改 Python/C++ 都不能解决“主臂非常紧”这个根因，除非能改变 STM32 固件或绕过 STM32。

下一步必须切换到以下两条路线之一：

1. 获取当前 STM32 控制板固件源码，增加真正的 HX-35HM `load/unload` 或 raw passthrough。
2. 使用 BusLinker/USB-TTL 半双工适配器直连主臂 HX-35HM 总线，绕过当前 STM32 封装，直接发送舵机底层协议并验证卸力手拖。

## 2026-05-13 官方/公开资料补充：真正应实现的是 raw `LOAD_OR_UNLOAD`

用户指出：官方软件既然能做到“掉电但仍读位置”，就不应停留在当前 SDK 不支持。

重新查阅资料后，结论更新如下。

### 官方现象

Hiwonder BusLinker/总线舵机调试工具中，Torque 开关关闭后：

1. 舵机停止输出扭矩。
2. 舵机可以手动转动。
3. 调试工具仍能读取舵机位置。

这与 HX-35HM 手册中的“马达掉电后可以手动转动，并看到位置信息”一致。

### 公开协议线索

公开 Hiwonder/LewanSoul 总线舵机协议中：

```text
SERVO_LOAD_OR_UNLOAD_WRITE = 31
SERVO_LOAD_OR_UNLOAD_READ  = 32
SERVO_POS_READ             = 28
```

其中：

```text
LOAD_OR_UNLOAD_WRITE 参数 0 = unload / power down / no torque
LOAD_OR_UNLOAD_WRITE 参数 1 = load / torque on
```

raw 舵机帧格式通常为：

```text
55 55 ID LENGTH CMD PARAM... CHECKSUM
```

`CHECKSUM` 计算：

```text
~(ID + LENGTH + CMD + PARAM...) & 0xFF
```

因此，给 1 号舵机卸力的 raw 指令应类似：

```text
55 55 01 04 1F 00 DB
```

含义：

```text
ID     = 0x01
LENGTH = 0x04
CMD    = 0x1F = 31 = LOAD_OR_UNLOAD_WRITE
PARAM  = 0x00 = unload
CRC    = ~(01 + 04 + 1F + 00) & 0xFF = DB
```

### 为什么不能直接对 `/dev/so101_leader` 发这个 raw 指令

当前 `/dev/so101_leader` 连接的是 `ros_robot_controller` STM32 板。

这条链路不是 raw 舵机总线：

```text
PC
  -> /dev/so101_leader
  -> STM32 wrapper protocol: AA 55 function length data crc8
  -> STM32 firmware
  -> HX-35HM servo bus
```

所以 PC 直接向 `/dev/so101_leader` 发：

```text
55 55 ...
```

大概率不会到达舵机，而是被 STM32 固件当成无效帧。

这也解释了为什么之前尝试 Hiwonder 控制器 `CMD_MULT_SERVO_UNLOAD=20` 没有生效：那是另一类控制器协议，不是当前 STM32 wrapper 协议。

### 正确实现路线

#### 路线 1：拿到 STM32 固件源码，增加 passthrough 或新增命令

在 STM32 固件中新增一个功能：

```text
PC AA55 wrapper
  -> STM32 raw passthrough
  -> HX-35HM raw 55 55 LOAD_OR_UNLOAD_WRITE
```

建议新增：

1. `raw_bus_write(bytes)`：直接向舵机总线发送 raw bytes。
2. `raw_bus_transaction(bytes, timeout)`：发送后等待 raw reply。
3. `leader_unload(ids)`：固件内部封装 `LOAD_OR_UNLOAD_WRITE=31,param=0`。
4. `leader_read_positions(ids)`：固件内部轮询 `POS_READ=28` 并一次性返回 6 个位置。

这条路线最干净，但需要固件源码和烧录。

#### 路线 2：使用 BusLinker/USB-TTL 半双工适配器直连主臂总线

硬件链路改成：

```text
PC USB
  -> BusLinker 或 USB-TTL 半双工 UART
  -> HX-35HM 主臂舵机总线
```

然后 PC 直接发送 raw 协议：

```text
55 55 ID LENGTH CMD PARAM CHECKSUM
```

当前已新增测试脚本：

```bash
/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/hx35hm_raw_bus_leader.py
```

示例：对主臂 1-6 号发送真正 raw unload：

```bash
cd /home/rog/ros2_ws
python3 src/so101_hx35hm_bridge/scripts/hx35hm_raw_bus_leader.py \
  --device /dev/ttyUSB0 \
  --baudrate 115200 \
  --mode unload \
  --ids 1 2 3 4 5 6 \
  --yes
```

读取 load 状态和位置：

```bash
python3 src/so101_hx35hm_bridge/scripts/hx35hm_raw_bus_leader.py \
  --device /dev/ttyUSB0 \
  --baudrate 115200 \
  --mode status \
  --ids 1 2 3 4 5 6
```

拖动观察位置连续变化：

```bash
python3 src/so101_hx35hm_bridge/scripts/hx35hm_raw_bus_leader.py \
  --device /dev/ttyUSB0 \
  --baudrate 115200 \
  --mode watch \
  --ids 1 2 3 4 5 6 \
  --rounds 100
```

成功标准：

1. `load_state=0`。
2. 主臂接 12V 时也可以手动拖动。
3. 拖动时 `pos` 连续变化。
4. 1-6 号都能稳定读。

如果这 4 条成立，就可以继续写正式 ROS2 raw leader node。

### 正式 ROS2 raw leader node 设计

节点名建议：

```text
hx35hm_raw_leader_node
```

行为：

1. 打开 raw 总线串口。
2. 启动时对 1-6 号发送 `LOAD_OR_UNLOAD_WRITE=31,param=0`。
3. 以 50Hz 左右轮询 1-6 号 `POS_READ=28`。
4. 映射为 SO101 joint rad。
5. 发布 `/leader/joint_states`。
6. 不提供任何位置控制接口。
7. 退出时默认不重新 load torque，除非显式参数允许。

接入现有链路：

```text
hx35hm_raw_leader_node
  -> /leader/joint_states
  -> so101_teleop
  -> /follower/forward_controller/commands
  -> follower hx35hm_bridge
```

这样主臂就是“官方软件同语义”的 leader，而不是当前 STM32 wrapper 的半成品卸力。

### 当前临时方案与长期方案的区别

当前临时可用方案：

```text
断主臂 12V
只保留 STM32/总线弱供电
读位置控制从臂
```

优点：

1. 已经实测可跟随。
2. 不需要新硬件。

风险：

1. 欠压状态不适合长期运行。
2. 拖动有电气阻尼。
3. 可能对 STM32 供电、舵机驱动、反电动势回灌不友好。

长期推荐方案：

```text
主臂正常 12V 供电
raw LOAD_OR_UNLOAD_WRITE=31,param=0
主臂真正无扭矩但编码器在线
raw POS_READ=28
发布 /leader/joint_states
```

这才是应该追的方向。

## 2026-05-17 实测更新：STM32 封装卸力语义已修正，当前瓶颈转为采样质量

本节用于覆盖本文前半部分在 2026-05-13 阶段形成的旧判断。

### 1. 已被后续实测修正的旧结论

此前我们一度认为：

```text
当前 STM32 封装无法让主臂真正卸力。
```

这个判断现在已经不准确。

后续重新核对 RRC 指令语义并完成实测后，已经确认：

```text
0x0B = 掉电 / 卸力
0x0C = 上电 / 加载
```

此前问题的关键不是“STM32 板子做不到卸力”，而是我们一度把命令语义理解反了，导致测试过程把上力和卸力混在了一起。

### 2. 当前 STM32 路线已经能够实现的 leader 行为

当前项目中，基于现有 STM32 控制板，已经可以稳定实现一条实验性 leader 链路：

```text
/dev/so101_leader
  -> leader hx35hm_bridge
  -> 启动时对 1-6 号发送卸力
  -> 只读位置，不订阅控制命令
  -> /leader/joint_states
```

当前 leader 侧关键约束：

```text
enable_command_subscription = false
disable_torque_on_startup = true
restore_torque_on_shutdown = false
```

这说明：

```text
“主臂正常供电 + 舵机卸力 + 持续回读位置”在当前 STM32 路线上已经可行。
```

因此，本文早期提出的“只能通过 raw passthrough 或直接绕过 STM32 才可能实现手拖 leader”已经需要降级为长期优化路线，而不是当前可行性的前置条件。

### 3. 当前真正的协议瓶颈

当前剩余问题已经不再是“能不能卸力”，而是“能不能给出足够平滑、足够同步的 leader 状态”。

现阶段 PC 侧仍然通过现有 SDK 做逐舵机请求-应答：

```text
read servo 1
read servo 2
read servo 3
read servo 4
read servo 5
read servo 6
```

这带来三个直接后果：

1. 当前不是 6 关节同一时刻采样，而是 round_robin 轮询。
2. `position_readback_rate_hz = 180` 时，等价于单关节真实更新频率约 `30Hz`。
3. `state_publish_rate_hz = 100` 只能更频繁地重复最近一次读数，不能把输入变成真正连续信号。

实测中：

```text
leader readback = 240Hz
```

已经触发大量：

```text
Input/output error
```

因此当前链路不适合继续单纯向上堆轮询频率。  
`180Hz` 更接近当前这套 STM32 + SDK 封装的稳定上限。

### 4. 当前优化结果

最近一轮主从控制优化后的结论：

```text
1. follower 异步固定频率写入明显改善了实时性。
2. LPF 能降低命令抖动，但不能消除 leader 输入的阶梯特性。
3. alpha-beta 预测在当前阶梯输入上会放大抖动，不适合继续使用。
4. 三号关节 elbow_flex 还存在独立异常，不能把它和全局顿挫混为一个问题。
```

最近诊断数据：

```text
leader shoulder_pan  median step ~= 2.16 deg
leader shoulder_lift median step ~= 2.88 deg
leader elbow_flex    median step ~= 2.16 deg

command shoulder_pan  median step ~= 0.24 deg
command shoulder_lift median step ~= 0.29 deg
command elbow_flex    median step ~= 0.28 deg

elbow_flex command_vs_follower median_abs_error ~= 13.1 deg
elbow_flex command_vs_follower p95_abs_error    ~= 63.3 deg
```

这组数据说明：

```text
relay 和 follower 侧已经能把命令变平滑。
全局顿挫的主因仍在 leader 侧采样结构。
三号关节则是一个需要单独修复的异常项。
```

### 5. 更新后的技术路线判断

#### 路线 C1：继续使用当前 STM32 作为实验性 leader

当前已经成立。

适合：

- 日常验证
- 功能联调
- 继续排查三号关节

但不应把它误认为最终高质量方案。

#### 路线 C2：改 STM32 固件，增加批量状态帧

这是当前最值得优先考虑的长期优化路线。

目标不再只是“支持 unload”，而是：

```text
1. 固件内部一次性采集 1-6 号位置。
2. 通过单个 USB 帧主动上报整臂状态。
3. 避免 PC 侧逐舵机 round_robin 读取。
4. 让 /leader/joint_states 更接近同步采样。
```

如果要继续向“官方 leader 手感”靠近，这一项比继续调 ROS 侧滤波更重要。

#### 路线 C3：raw bus 直连

仍然有价值，但定位应调整为：

- 协议验证工具
- 绕过 STM32 封装做对照实验
- 固件开发前的能力确认手段

它不再是实现“基本可拖动 leader”的唯一入口。

#### 路线 C4：专用 leader 硬件

如果目标是长期稳定、低顿挫、高一致性的人机交互，专用 leader 仍然是工程上最干净的终局路线。

### 6. 当前配套文档

- [HX35HM_SO101_主从控制当前使用流程.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主从控制当前使用流程.md)
- [HX35HM_SO101_主从跟随顿挫与三号关节优化方案.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主从跟随顿挫与三号关节优化方案.md)
- [HX35HM_SO101_主臂手拖示教当前结论与替代方案.md](/home/rog/ros2_ws/docs/HX35HM_SO101_主臂手拖示教当前结论与替代方案.md)
