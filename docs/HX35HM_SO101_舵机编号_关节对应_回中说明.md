# HX35HM + SO101 舵机编号、关节对应与回中说明

这份文档用于把你当前工作空间里，HX-35HM 机械臂的 **舵机编号、关节对应关系、回中方式、装配参考位** 统一说明清楚。

它的目标不是替代硬件标定，而是作为你后续做主臂开发、主从映射、装配复位和故障排查时的统一参考。

---

## 1. 先说结论

当前仓库里，SO101 / HX35HM 的默认舵机编号是：

- `1` = `shoulder_pan`
- `2` = `shoulder_lift`
- `3` = `elbow_flex`
- `4` = `wrist_flex`
- `5` = `wrist_roll`
- `6` = `gripper`

这套编号在仓库多个地方是一致的，包括：

- `so101_hx35hm_bridge`
- `so101_bringup/config/hardware/follower_joints.yaml`
- `so101_bringup/config/hardware/leader_joints.yaml`
- `so101_hx35hm_bridge/config/assembly_calibration.yaml`

注意：

- 这是 **单臂内部的局部编号**，不是全局编号。
- 如果你有 leader 和 follower 两套硬件，它们通常都各自使用 `1~6`。
- 真正差异通常在于 **方向、零位、安装姿态、以及串口设备名**，不是 ID 本身。

如果你现在“还没有给舵机编号”，那这份表先不要当成实机现状来用，它只是后续要写进去的目标映射。

也就是说，当前阶段你要做的第一件事不是调运动，而是先把 6 个舵机逐个写成唯一 ID。

---

## 2. 关节与舵机编号对照表

### 2.1 默认对应关系

| 关节名 | 舵机 ID | 说明 |
| --- | --- | --- |
| `shoulder_pan` | `1` | 底座回转 |
| `shoulder_lift` | `2` | 大臂抬升 |
| `elbow_flex` | `3` | 肘关节弯折 |
| `wrist_flex` | `4` | 腕部俯仰 |
| `wrist_roll` | `5` | 腕部旋转 |
| `gripper` | `6` | 夹爪 |

这个对应关系在 `bridge_node.py` 里是固定的：

- [bridge_node.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py)

### 2.2 相关配置来源

你当前工作空间里相关配置文件如下：

- [follower_joints.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hardware/follower_joints.yaml)
- [leader_joints.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hardware/leader_joints.yaml)
- [assembly_calibration.yaml](/home/rog/ros2_ws/src/so101_hx35hm_bridge/config/assembly_calibration.yaml)

---

## 3. Leader 和 Follower 的区别

很多人第一次看这个仓库时会误以为：

- leader 和 follower 的舵机 ID 要不同

实际上不是。

### 3.1 它们的共同点

两只臂的默认关节顺序都是：

1. `shoulder_pan`
2. `shoulder_lift`
3. `elbow_flex`
4. `wrist_flex`
5. `wrist_roll`
6. `gripper`

也就是舵机 ID 都是 `1~6`。

### 3.2 它们的不同点

真正不同的是：

- 串口设备不同
- 安装方向不同
- 软件方向参数不同
- 零位参数不同
- 控制模式不同

例如：

- leader 常用设备是 `/dev/so101_leader`
- follower 常用设备是 `/dev/so101_follower`

对应的启动入口也不同：

- [leader_hx35hm.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/leader_hx35hm.launch.py)
- [follower_hx35hm_moveit.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py)

### 3.3 串口绑定原则

主臂和从臂的 STM32 控制板不要按 `/dev/ttyACM0`、`/dev/ttyACM1` 绑定。

这些名字由 Linux 按插拔顺序分配，插拔顺序变了就可能变化。

当前工作区采用的稳定绑定方式是：

| 控制板 | 序列号 | 稳定设备名 |
| --- | --- | --- |
| leader 主臂 | `5917010961` | `/dev/so101_leader` |
| follower 从臂 | `596F003872` | `/dev/so101_follower` |

对应规则文件：

- [99-so101.rules](/home/rog/ros2_ws/config/99-so101.rules)

同步到系统后，不管这两块板枚举成 `ttyACM0` 还是 `ttyACM1`，ROS 和调试脚本都应该只使用：

- `/dev/so101_leader`
- `/dev/so101_follower`

---

## 4. 舵机编号、方向和零位

### 4.1 默认装配校准配置

当前仓库里 `assembly_calibration.yaml` 给出了一个安全起点：

- 所有关节的 `servo_id` 都是 `1~6`
- 所有关节的 `zero_pos` 默认都是 `500`
- 部分关节的 `direction` 默认做了方向翻转

对应文件：

- [assembly_calibration.yaml](/home/rog/ros2_ws/src/so101_hx35hm_bridge/config/assembly_calibration.yaml)

### 4.2 当前默认方向

`assembly_calibration.yaml` 中的默认方向是：

| 关节名 | servo_id | direction | zero_pos |
| --- | --- | --- | --- |
| `shoulder_pan` | `1` | `-1` | `500` |
| `shoulder_lift` | `2` | `-1` | `500` |
| `elbow_flex` | `3` | `1` | `500` |
| `wrist_flex` | `4` | `-1` | `500` |
| `wrist_roll` | `5` | `1` | `500` |
| `gripper` | `6` | `1` | `500` |

这份表更像是“桥接层初始默认值”，不是最终定版。

### 4.3 如果你现在还没编号，先做什么

如果舵机 ID 还没有写入，那么建议按这个顺序来：

1. 一次只接一个舵机到总线
2. 用 `change_servo_id.py` 把它写成目标 ID
3. 重新上电或再次回读，确认 ID 生效
4. 给这个舵机贴标签，防止后面混淆
5. 继续下一个舵机

推荐的目标顺序就是仓库默认顺序：

- `shoulder_pan` -> `1`
- `shoulder_lift` -> `2`
- `elbow_flex` -> `3`
- `wrist_flex` -> `4`
- `wrist_roll` -> `5`
- `gripper` -> `6`

对应脚本：

- [change_servo_id.py](/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py)

### 4.4 给舵机编号时的安全原则

请务必遵守下面几条：

- 总线上一次只接一个舵机
- 修改 ID 前先断开其他舵机
- 不要把广播 ID `254` 当作最终常规 ID
- 编号完成后立刻验证回读
- 记录“舵机实物编号”和“关节名”的对应关系

如果你跳过这一步，后面会出现这些问题：

- 你分不清某个舵机到底是肩膀还是肘部
- 回中脚本会打错对象
- 主从臂映射容易错位
- 方向和零位标定会混乱

### 4.3 为什么不能只看 ID

同一个 ID 在不同臂上，可能出现：

- 物理转向相反
- 安装孔位不同
- 机械零点偏差不同
- 夹爪闭合方向不同

所以你最终要保存的是三件事：

1. `servo_id`
2. `direction`
3. `zero_pos`

---

## 5. 回中的两种含义

这里要特别区分两个概念：

### 5.1 回到“舵机中位”

这是最直观的“回中”。

HX-35HM 默认角度映射是：

- `0` -> `0°`
- `500` -> `120°`
- `1000` -> `240°`

也就是说，`pos=500` 通常被当作舵机物理中位。

对应脚本：

- [return_to_home.py](/home/rog/ros2_ws/tools/hardware_debug/return_to_home.py)
- [return_all_to_mid.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py)

### 5.2 回到“关节零位 / 软件 home”

这是更重要的概念。

软件里的 `0 rad` 不一定等于 `pos=500`，因为还存在：

- `direction`
- `joint_zero_positions`
- 安装偏置

也就是说：

- `pos=500` 是舵机的中位
- `0 rad` 是你在软件里定义的关节零位

这两个值可以相同，也可以不同。

---

## 6. 当前仓库里的回中脚本

### 6.1 `tools/hardware_debug/return_to_home.py`

这个脚本很简单，作用是：

- 把 `SERVO_IDS = [1, 2, 3, 4, 5, 6]`
- 统一打到 `TARGET_POS = 500`

适合场景：

- 你只是想把所有舵机回到物理中位
- 第一次上电做安全检查
- 临时验证舵机有没有响应

脚本路径：

- [return_to_home.py](/home/rog/ros2_ws/tools/hardware_debug/return_to_home.py)

### 6.2 `src/so101_hx35hm_bridge/scripts/return_all_to_mid.py`

这个脚本更完整一些，支持：

- 指定串口设备
- 指定舵机 ID 列表
- 指定目标位置
- 指定持续时间
- dry-run
- 重复发送

默认参数：

- `--device /dev/ros_robot_controller`
- `--servo-ids 1 2 3 4 5 6`
- `--pos 500`
- `--duration 1.0`

脚本路径：

- [return_all_to_mid.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py)

推荐用途：

- 你要正式做装配前的“中位归拢”
- 你想在不同设备上复用同一个脚本
- 你想先 `--dry-run` 看命令是否正确

---

## 7. 推荐的实际使用流程

### 7.1 第一次上电前

建议按这个顺序：

1. 确认每个舵机 ID 已写好
2. 确认串口设备名正确
3. 确认电源限流和急停可用
4. 先不要直接整臂大幅运动

### 7.2 先把所有舵机打到中位

适合两种方式：

```bash
python3 tools/hardware_debug/return_to_home.py
```

或者：

```bash
python3 src/so101_hx35hm_bridge/scripts/return_all_to_mid.py \
  --device /dev/ros_robot_controller \
  --servo-ids 1 2 3 4 5 6 \
  --pos 500 \
  --duration 1.0
```

如果你只想试一下，不发硬件命令，可以先：

```bash
python3 src/so101_hx35hm_bridge/scripts/return_all_to_mid.py --dry-run
```

### 7.3 再做机械装配或复位

回中后再检查：

- 关节是否在安全角度
- 连杆是否有预应力
- 是否接近机械限位
- 是否存在装反方向

### 7.4 再做软件零位确认

当机械姿态定好后，再去修正：

- `direction`
- `joint_zero_positions`

这一步才是你后续主从臂一致性的关键。

---

## 8. 建议你记录成的最终配置表

等你正式把主臂和从臂都调完后，建议你每只臂都保存一份自己的表：

| 关节名 | servo_id | direction | zero_pos | 备注 |
| --- | --- | --- | --- | --- |
| `shoulder_pan` | `1` | `-1` 或 `+1` | `xxx` | 以实机标定为准 |
| `shoulder_lift` | `2` | `-1` 或 `+1` | `xxx` | 以实机标定为准 |
| `elbow_flex` | `3` | `-1` 或 `+1` | `xxx` | 以实机标定为准 |
| `wrist_flex` | `4` | `-1` 或 `+1` | `xxx` | 以实机标定为准 |
| `wrist_roll` | `5` | `-1` 或 `+1` | `xxx` | 以实机标定为准 |
| `gripper` | `6` | `-1` 或 `+1` | `xxx` | 以实机标定为准 |

建议你把这份表分别保存成：

- leader 一份
- follower 一份

这样后面做主从映射时不会混。

### 8.1 leader / follower 分开记录的建议格式

你可以直接按下面两张表去填。

#### Leader 表

| 关节名 | servo_id | direction | zero_pos | 参考姿态 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `shoulder_pan` | `1` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `shoulder_lift` | `2` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `elbow_flex` | `3` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `wrist_flex` | `4` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `wrist_roll` | `5` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `gripper` | `6` | `+1 / -1` | `xxx` | open / mid / closed |  |

#### Follower 表

| 关节名 | servo_id | direction | zero_pos | 参考姿态 | 备注 |
| --- | --- | --- | --- | --- | --- |
| `shoulder_pan` | `1` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `shoulder_lift` | `2` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `elbow_flex` | `3` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `wrist_flex` | `4` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `wrist_roll` | `5` | `+1 / -1` | `xxx` | `zero` 或 `rest` |  |
| `gripper` | `6` | `+1 / -1` | `xxx` | open / mid / closed |  |

### 8.2 推荐你最终固化到文件里的内容

如果你后面要做主从臂长期使用，建议把每只臂最终定版的信息保存成：

- `servo_id`
- `joint_name`
- `direction`
- `zero_pos`
- `device`
- `reference_pose`

这样一来，你以后排查问题时就能快速回答：

1. 这只臂是 leader 还是 follower
2. 这只臂接的是哪个串口
3. 这只臂 0 rad 在哪里
4. 这只臂的正方向是哪边
5. 这只臂是按 `zero` 还是 `rest` 装配的

---

## 9. 和主从臂开发的关系

如果你后面要做“主臂开发”，这份编号表会直接影响这些模块：

- leader 关节读回
- teleop 映射
- MoveIt 起始状态
- 主从同步
- 回中动作
- 紧急停机后复位

换句话说：

**先把舵机编号、方向、零位和回中策略定稳，后面的主从臂才会真正好用。**

---

## 10. 最后提醒

请不要把下面三件事混为一谈：

1. `servo_id`
2. `pos=500`
3. `joint 0 rad`

它们在概念上相关，但不是天然相等。

如果你后面要继续，我建议下一步直接做两件事：

1. 把 leader 和 follower 的最终 `servo_id / direction / zero_pos` 固化成两份独立表
2. 再基于这两份表，写一个“主从臂一键回中 + 一键归零”的启动脚本
