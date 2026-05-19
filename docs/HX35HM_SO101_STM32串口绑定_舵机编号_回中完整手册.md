# HX35HM + SO101 STM32 串口绑定、舵机编号与回中完整手册

这份文档把三件事一次讲清楚：

1. STM32 控制板如何做稳定串口绑定
2. HX-35HM 舵机如何逐个编号
3. 编号后如何回到中位并做回读确认

适用场景：

- 你刚换了一块新的 STM32 控制板
- 你刚接入一批还没有编号的 HX-35HM 舵机
- 你准备做 SO101 主臂 / 从臂装配
- 你想把“串口识别、舵机编号、回中确认”整理成一套固定流程

---

## 1. 先说最重要的原则

### 1.1 STM32 串口不要直接依赖 `/dev/ttyACM0`

`/dev/ttyACM0`、`/dev/ttyACM1` 这类名字是 Linux 按插拔顺序临时分配的。

这意味着：

- 今天主臂可能是 `ttyACM0`
- 明天你先插了另一块板，它就可能变成 `ttyACM1`

所以正式使用时，不要把主臂 / 从臂永久写死到 `ttyACM0`。

应该使用稳定设备名：

- `/dev/so101_leader`
- `/dev/so101_follower`

---

### 1.2 给舵机编号时，总线上一次只接一个舵机

这是最关键的硬件安全规则。

因为“未确定 ID 的舵机”通常会用广播 ID `254` 操作。如果总线上接了多个舵机，你一次写 ID，可能把多个舵机同时写成同一个编号，后面会非常难排查。

正确做法：

- 改 ID 时只接一个舵机
- 写完立刻回读确认
- 给该舵机贴标签
- 再接下一个

---

### 1.3 回中分成两种

不要把这两个概念混在一起：

- 舵机物理中位：`pos=500`
- 软件关节零位：`joint angle = 0 rad`

在 HX-35HM 里，通常：

- `0` -> 约 `0 deg`
- `500` -> 约 `120 deg`
- `1000` -> 约 `240 deg`

所以我们这里说“回中”，默认指的是把舵机打到 `pos=500`。

---

## 2. 当前工作区里的默认约定

### 2.1 默认舵机编号

当前仓库默认关节和舵机 ID 对应关系是：

| 关节名 | 舵机 ID | 说明 |
| --- | --- | --- |
| `shoulder_pan` | `1` | 底座回转 |
| `shoulder_lift` | `2` | 大臂抬升 |
| `elbow_flex` | `3` | 肘关节弯折 |
| `wrist_flex` | `4` | 腕部俯仰 |
| `wrist_roll` | `5` | 腕部旋转 |
| `gripper` | `6` | 夹爪 |

对应配置文件：

- [assembly_calibration.yaml](/home/rog/ros2_ws/src/so101_hx35hm_bridge/config/assembly_calibration.yaml)

---

### 2.2 当前稳定串口规则

当前工作区里的 udev 规则文件是：

- [99-so101.rules](/home/rog/ros2_ws/config/99-so101.rules)

当前规则内容对应的是：

| 角色 | 序列号 | 稳定设备名 |
| --- | --- | --- |
| 主臂 STM32 | `596F003063` | `/dev/so101_leader` |
| 从臂 STM32 | `596F003872` | `/dev/so101_follower` |

注意：

- 这是你当前这套机器最近一次确认过的映射
- 如果你又换了新的 STM32 板子，序列号会变，规则也要跟着改

---

## 3. 如何查看当前接入的 STM32 序列号

先插入你要绑定的 STM32 板子，然后执行：

```bash
ls -l /dev/ttyACM*
udevadm info --query=property --name=/dev/ttyACM0 | egrep 'ID_SERIAL_SHORT|ID_VENDOR_ID|ID_MODEL_ID|DEVLINKS'
```

如果这块板当前枚举成 `ttyACM0`，你会看到类似：

```bash
ID_SERIAL_SHORT=596F003063
ID_VENDOR_ID=1a86
ID_MODEL_ID=55d4
```

其中最关键的是：

- `ID_SERIAL_SHORT`

它就是你做主臂 / 从臂稳定绑定时要用的唯一序列号。

---

## 4. 把 STM32 绑定为主臂或从臂

### 4.1 修改规则文件

编辑这个文件：

- [99-so101.rules](/home/rog/ros2_ws/config/99-so101.rules)

例如当前主臂 / 从臂规则是：

```udev
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="1a86", ENV{ID_MODEL_ID}=="55d4", ENV{ID_SERIAL_SHORT}=="596F003063", SYMLINK+="so101_leader", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="596F003872", SYMLINK+="so101_follower", GROUP="dialout", MODE="0660"
```

如果你换了一块新的主臂板子，只需要把第一行里的序列号改成新板子的 `ID_SERIAL_SHORT`。

---

### 4.2 安装到系统并重载 udev

修改好仓库里的规则后，执行：

```bash
sudo cp /home/rog/ros2_ws/config/99-so101.rules /etc/udev/rules.d/99-so101.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后重新插拔板子，或者再检查一次：

```bash
ls -l /dev/so101_leader /dev/so101_follower /dev/ttyACM*
```

如果绑定成功，你会看到类似：

```bash
/dev/so101_leader -> ttyACM0
```

或者：

```bash
/dev/so101_follower -> ttyACM1
```

---

### 4.3 为什么还会看到 `/dev/ros_robot_controller`

你的系统里可能还保留了一条旧规则，会额外生成：

- `/dev/ros_robot_controller`

这个名字是历史兼容名，不适合区分主臂 / 从臂。

建议：

- 主臂统一使用 `/dev/so101_leader`
- 从臂统一使用 `/dev/so101_follower`

只有在临时调试、规则还没安装好时，才临时用 `/dev/ttyACM0`。

---

## 5. 如何快速测试 STM32 板子本身是否正常

在不写舵机 ID、不移动舵机的情况下，可以先做板级只读测试：

```bash
python3 - <<'PY'
import sys, time
sys.path.insert(0, '/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller')
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device='/dev/so101_leader')
board.enable_reception()

for _ in range(20):
    b = board.get_battery()
    if b is not None:
        print('battery =', b)
        break
    time.sleep(0.05)

print('imu =', board.get_imu())
PY
```

如果能读到：

- `battery`
- `imu`

说明这块 STM32 板子的固件基本是活的。

如果这两个都一直是 `None`，优先怀疑：

- 板子固件不对
- 板子没正常启动
- 这不是那套 `ros_robot_controller` 固件

---

## 6. 给 HX-35HM 舵机编号

### 6.1 编号前准备

开始之前，请确认：

- 只接了一个舵机到总线
- STM32 板子已经能正常通信
- 舵机电源正常
- 你知道当前使用的是哪个串口设备

推荐使用稳定名字，例如：

- `/dev/so101_leader`

如果规则还没装好，也可以临时用：

- `/dev/ttyACM0`

---

### 6.2 默认目标编号

推荐按这套顺序逐个写入：

| 关节 | 目标 ID |
| --- | --- |
| `shoulder_pan` | `1` |
| `shoulder_lift` | `2` |
| `elbow_flex` | `3` |
| `wrist_flex` | `4` |
| `wrist_roll` | `5` |
| `gripper` | `6` |

---

### 6.3 用 Python 直接给单个舵机编号

下面例子把“当前未知 ID 的单个舵机”写成 `6`：

```bash
python3 - <<'PY'
import sys, time
sys.path.insert(0, '/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller')
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device='/dev/so101_leader')
board.enable_reception()

board.bus_servo_set_id(254, 6)
time.sleep(0.3)

print('id_254 =', board.bus_servo_read_id(254, timeout=1.0))
print('id_6 =', board.bus_servo_read_id(6, timeout=1.0))
PY
```

如果你还没把主臂稳定绑定到 `/dev/so101_leader`，把上面的设备名换成：

```bash
/dev/ttyACM0
```

---

### 6.4 给不同关节写目标 ID 的示例

把未知舵机写成 `1`：

```bash
python3 - <<'PY'
import sys, time
sys.path.insert(0, '/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller')
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device='/dev/so101_leader')
board.enable_reception()
board.bus_servo_set_id(254, 1)
time.sleep(0.3)
print('id_1 =', board.bus_servo_read_id(1, timeout=1.0))
PY
```

把未知舵机写成 `2`：

```bash
python3 - <<'PY'
import sys, time
sys.path.insert(0, '/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller')
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device='/dev/so101_leader')
board.enable_reception()
board.bus_servo_set_id(254, 2)
time.sleep(0.3)
print('id_2 =', board.bus_servo_read_id(2, timeout=1.0))
PY
```

其余 `3/4/5/6` 同理。

---

### 6.5 编号完成后的验证方式

编号后建议至少做两次检查：

1. 回读 ID
2. 回读位置

例如检查 `id=6`：

```bash
python3 - <<'PY'
import sys, time
sys.path.insert(0, '/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller')
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device='/dev/so101_leader')
board.enable_reception()
time.sleep(0.2)

print('id_6 =', board.bus_servo_read_id(6, timeout=1.0))
print('pos_6 =', board.bus_servo_read_position(6, timeout=1.0))
PY
```

如果两项都不是 `None`，说明这个舵机已经能被稳定访问。

---

## 7. 让舵机回到中位

### 7.1 单个舵机回中

当前工作区已经有现成脚本别名：

- [assembly_helper.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/assembly_helper.py)

让 `id=6` 回到中位：

```bash
cd /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts
python3 assembly_helper.py --servo-id 6 --pos 500 --device /dev/so101_leader --yes
```

如果暂时还没稳定绑定，就临时写成：

```bash
python3 assembly_helper.py --servo-id 6 --pos 500 --device /dev/ttyACM0 --yes
```

---

### 7.2 全部舵机一起回中

脚本：

- [return_all_to_mid.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py)

把 `1 2 3 4 5 6` 一起打到 `500`：

```bash
cd /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts
python3 return_all_to_mid.py --device /dev/so101_leader
```

只让部分舵机回中：

```bash
python3 return_all_to_mid.py --device /dev/so101_leader --servo-ids 1 2 3
```

先看将要发送什么，不真正下发：

```bash
python3 return_all_to_mid.py --device /dev/so101_leader --dry-run
```

---

### 7.3 用装配脚本控制单个舵机

如果你更习惯统一用同一个脚本，也可以：

```bash
cd /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts
python3 assembly_helper.py --servo-id 1 --pos 500 --device /dev/so101_leader --yes
python3 assembly_helper.py --servo-id 2 --pos 500 --device /dev/so101_leader --yes
python3 assembly_helper.py --servo-id 3 --pos 500 --device /dev/so101_leader --yes
```

---

## 8. 回中后如何回读确认

例如你刚让 `id=1` 回到 `500`，可以这样确认：

```bash
python3 - <<'PY'
import sys, time
sys.path.insert(0, '/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller')
from ros_robot_controller.ros_robot_controller_sdk import Board

board = Board(device='/dev/so101_leader')
board.enable_reception()
time.sleep(0.2)

print('id_1 =', board.bus_servo_read_id(1, timeout=1.0))
print('pos_1 =', board.bus_servo_read_position(1, timeout=1.0))
PY
```

正常时你会看到类似：

```bash
id_1 = [1]
pos_1 = [499]
```

`499`、`500`、`501` 这类都可以认为已经在中位附近。

---

## 9. 主臂 / 从臂的推荐使用方式

### 9.1 主臂

优先使用：

```bash
/dev/so101_leader
```

### 9.2 从臂

优先使用：

```bash
/dev/so101_follower
```

### 9.3 临时调试

只有在这两种情况才临时用 `ttyACM0`：

- 你刚接入一块新板，还没更新 udev 规则
- 你没有权限把规则安装到 `/etc/udev/rules.d`

---

## 10. 常见问题排查

### 10.1 `Type YES to continue:` 后输入 `yes` 被取消

这是脚本故意做的安全确认。

它要求你输入：

```text
YES
```

必须是全大写。

如果你不想交互确认，可以加：

```bash
--yes
```

---

### 10.2 板级 `battery`、`imu` 有数据，但舵机 ID 全部读不到

这通常说明：

- STM32 板子本身正常
- 但舵机总线侧没有通

优先检查：

- 舵机是否上电
- 总线线序是否正确
- TTL 线是否接反
- 当前总线上是否真的接了舵机

---

### 10.3 `battery` 也一直是 `None`

这通常不是舵机编号问题，而是 STM32 板子本身的问题。

优先怀疑：

- 这块新板固件不对
- 板子没有正常启动
- 不是 `ros_robot_controller` 那套固件

---

### 10.4 为什么我改了 `config/99-so101.rules` 还是没有 `/dev/so101_leader`

因为修改仓库文件本身还不够，你还必须执行：

```bash
sudo cp /home/rog/ros2_ws/config/99-so101.rules /etc/udev/rules.d/99-so101.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

如果没有 `sudo` 权限，系统规则不会真正生效。

---

### 10.5 为什么不推荐长期使用 `/dev/ros_robot_controller`

因为它不能自然区分主臂和从臂。

主从都存在时，更稳妥的做法一定是：

- `/dev/so101_leader`
- `/dev/so101_follower`

---

## 11. 推荐的完整操作流程

如果你现在拿到的是一块新的 STM32 板子和一批待编号舵机，建议按下面顺序做：

1. 插入 STM32，读取 `ID_SERIAL_SHORT`
2. 修改 [99-so101.rules](/home/rog/ros2_ws/config/99-so101.rules)
3. 用 `sudo cp + udevadm` 安装规则
4. 确认 `/dev/so101_leader` 或 `/dev/so101_follower` 出现
5. 用 `battery / imu` 测试确认板子固件正常
6. 一次只接一个舵机
7. 给这个舵机写目标 ID
8. 回读 ID 确认写入成功
9. 让该舵机回到 `500`
10. 回读位置确认
11. 给舵机贴上关节标签
12. 继续下一个舵机
13. 全部编号完成后，再把整条机械臂一起回中装配

---

## 12. 本工作区里最常用的相关文件

- 串口规则：
  [99-so101.rules](/home/rog/ros2_ws/config/99-so101.rules)
- 装配脚本：
  [so101_assembly_pose.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/so101_assembly_pose.py)
- 单舵机控制别名：
  [assembly_helper.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/assembly_helper.py)
- 全部回中：
  [return_all_to_mid.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/return_all_to_mid.py)
- 舵机编号示例：
  [change_servo_id.py](/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py)
- 当前装配映射配置：
  [assembly_calibration.yaml](/home/rog/ros2_ws/src/so101_hx35hm_bridge/config/assembly_calibration.yaml)

