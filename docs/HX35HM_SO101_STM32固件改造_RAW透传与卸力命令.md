# HX35HM SO101 STM32 固件改造：RAW 透传与原生卸力命令

本文档记录主臂手拖示教的下一步固件方案：在 STM32 控制板固件里增加总线舵机 RAW 透传，或直接增加 HX/Hiwonder 总线舵机 `LOAD_OR_UNLOAD_WRITE = 31` 支持。

目标不是继续依赖当前 ROS SDK 里的 `bus_servo_enable_torque(id, 0)`，而是让 STM32 真正向舵机总线发送舵机原生命令：

```text
55 55 ID LENGTH CMD PARAM... CHECKSUM
```

其中：

```text
CMD 28 = POS_READ
CMD 31 = LOAD_OR_UNLOAD_WRITE
CMD 32 = LOAD_OR_UNLOAD_READ
```

## 当前结论

1. 当前主臂在接 12V 外部供电时，普通 `0x0B` 扭矩关闭命令不能可靠卸力。
2. 断开主臂 12V，仅保留 STM32/弱供电时，主臂可以拖动，而且位置回读仍可用。
3. 这说明位置桥接链路基本可用，真正卡住的是“带正常舵机电源时如何进入舵机原生 unload/掉电状态”。
4. HX/Hiwonder 公开协议和官方工具路线都指向 `LOAD_OR_UNLOAD_WRITE = 31`。

## 本仓库已添加的内容

### 1. PC 端 SDK 新接口

文件：

```text
/home/rog/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py
```

新增接口：

```python
board.bus_servo_raw_write(raw_frame)
board.bus_servo_raw_transaction(raw_frame, timeout=0.2)
board.bus_servo_make_raw_packet(servo_id, cmd, params)
board.bus_servo_load_or_unload_raw(servo_id, enable)
board.bus_servo_unload_many([1, 2, 3, 4, 5, 6])
board.bus_servo_load_many([1, 2, 3, 4, 5, 6])
board.bus_servo_unload(servo_id)
board.bus_servo_load(servo_id)
```

注意：这些接口需要刷入新 STM32 固件后才会生效。stock 固件大概率不认识 `0xF0/0xF1/0xF2/0xF3`。

### 2. Leader 拖动脚本新增 method

文件：

```text
/home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/leader_drag_mode.py
```

新增参数：

```bash
--method sdk-torque
--method custom-unload
--method custom-raw
```

含义：

```text
sdk-torque     当前旧方式，走 STM32 stock 0x0C/0x0B。
custom-unload 走新固件专用 0xF2/0xF3，推荐方案。
custom-raw    走新固件 RAW_WRITE 0xF0，直接发送 55 55 原生舵机帧。
```

### 3. STM32 固件参考模块

文件：

```text
/home/rog/ros2_ws/src/so101_hx35hm_bridge/firmware_reference/ros_robot_controller_bus_servo_raw_extension.h
/home/rog/ros2_ws/src/so101_hx35hm_bridge/firmware_reference/ros_robot_controller_bus_servo_raw_extension.c
```

这两个文件是给官方 STM32 工程移植用的参考实现。当前本机没有官方 STM32 固件工程，所以不能直接编译烧录。

## 新 STM32 子命令设计

PC 到 STM32 仍使用现有 ros_robot_controller 外层协议：

```text
AA 55 function data_len data... crc8
```

总线舵机功能号：

```text
function = 0x05  # PACKET_FUNC_BUS_SERVO
```

在 `data[0]` 增加以下自定义子命令：

```text
0xF0 RAW_WRITE
data = [F0, raw_len, raw_bytes...]
```

用途：STM32 不解析，直接把 `raw_bytes` 写到总线舵机 UART。

```text
0xF1 RAW_TRANSACTION
data = [F1, raw_len, raw_bytes...]
response = [F1, status, resp_len, resp_bytes...]
```

用途：需要回包的原生读命令，例如位置读取、load 状态读取。

```text
0xF2 UNLOAD_MANY
data = [F2, id_count, servo_id_1, servo_id_2, ...]
```

用途：STM32 直接生成并发送 `LOAD_OR_UNLOAD_WRITE = 31, param = 0`。

```text
0xF3 LOAD_MANY
data = [F3, id_count, servo_id_1, servo_id_2, ...]
```

用途：STM32 直接生成并发送 `LOAD_OR_UNLOAD_WRITE = 31, param = 1`。

## 舵机原生帧示例

卸力 1 号舵机：

```text
55 55 01 04 1F 00 DB
```

解释：

```text
55 55  header
01     servo id
04     length = cmd + param + checksum = 3 + param_len
1F     cmd = 31 = LOAD_OR_UNLOAD_WRITE
00     param = unload
DB     checksum = ~(01 + 04 + 1F + 00) & 0xFF
```

上力 1 号舵机：

```text
55 55 01 04 1F 01 DA
```

## STM32 固件移植点

需要在官方 STM32 工程里找到总线舵机命令分发函数。通常逻辑类似：

```c
void bus_servo_cmd_handle(uint8_t *data, uint16_t len)
{
    switch (data[0]) {
    case 0x01:
        ...
        break;
    case 0x05:
        ...
        break;
    }
}
```

改成：

```c
#include "ros_robot_controller_bus_servo_raw_extension.h"

void bus_servo_cmd_handle(uint8_t *data, uint16_t len)
{
    if (so101_bus_servo_handle_custom_command(data, len)) {
        return;
    }

    /* 原来的 stock 命令处理继续保留 */
}
```

然后把参考模块里的三个 hook 接到官方工程已有函数：

```c
extern void so101_bus_uart_write(const uint8_t *data, uint16_t len);
extern uint16_t so101_bus_uart_transaction(
    const uint8_t *tx,
    uint16_t tx_len,
    uint8_t *rx_buf,
    uint16_t rx_cap,
    uint32_t timeout_ms);
extern void so101_pc_report_bus_servo(const uint8_t *data, uint16_t len);
```

含义：

```text
so101_bus_uart_write
  发送原生 55 55 帧到总线舵机 UART。

so101_bus_uart_transaction
  发送原生帧，切换到接收，等待舵机回包。

so101_pc_report_bus_servo
  用 PACKET_FUNC_BUS_SERVO 回传给 PC 端 SDK。
```

## 刷入新固件后的验证命令

先停止主从控制，避免旧节点继续写主臂：

```bash
pkill -f 'teleop_hx35hm.launch.py|follower_command_relay|hx35hm_bridge|leader_hx35hm.launch.py|follower_hx35hm_moveit.launch.py|move_group|rviz2|robot_state_publisher' || true
```

确认节点为空：

```bash
source /opt/ros/jazzy/setup.bash
source /home/rog/ros2_ws/install/setup.bash
ros2 node list
```

使用新固件专用卸力命令：

```bash
python3 /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/leader_drag_mode.py \
  --device /dev/so101_leader \
  --mode enter \
  --method custom-unload \
  --ids 1 2 3 4 5 6 \
  --yes
```

如果主臂仍然很紧，再测试 RAW 透传路径：

```bash
python3 /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/leader_drag_mode.py \
  --device /dev/so101_leader \
  --mode enter \
  --method custom-raw \
  --ids 1 2 3 4 5 6 \
  --yes
```

验证卸力后位置回读：

```bash
python3 /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/leader_drag_mode.py \
  --device /dev/so101_leader \
  --mode verify-readback \
  --ids 1 2 3 4 5 6 \
  --rounds 10
```

恢复上力：

```bash
python3 /home/rog/ros2_ws/src/so101_hx35hm_bridge/scripts/leader_drag_mode.py \
  --device /dev/so101_leader \
  --mode exit \
  --method custom-unload \
  --ids 1 2 3 4 5 6 \
  --yes
```

## 成功标准

刷入新固件后，理想状态应该满足：

1. 主臂接 12V 外部供电。
2. 执行 `--method custom-unload --mode enter` 后，主臂明显变松，可以手拖。
3. 手拖时 `verify-readback` 仍能稳定读到 1-6 号位置。
4. 启动主从控制后，leader bridge 不再给主臂发位置命令，主臂不会重新上力。
5. 从臂继续由 follower bridge 接收 `/follower/forward_controller/commands` 并跟随。

## 风险和注意事项

1. 不要在从臂上误执行 unload。这个方案主要用于 leader。
2. RAW_WRITE 权限很大，调试时优先用 `custom-unload`，确认固件没问题后再用 `custom-raw`。
3. 如果 `custom-unload` 无效，但 `custom-raw` 有效，说明 STM32 专用命令生成帧有问题。
4. 如果两者都无效，优先检查 STM32 固件是否真的刷入、是否进入新分支、总线 UART 是否接到了正确 TX/RX/方向控制。
5. 如果卸力成功但位置读不到，重点检查半双工方向切换和 transaction 回包读取。

## 下一步

需要拿到官方 STM32 固件工程后，把本仓库的参考模块移植进去并编译烧录：

```text
/home/rog/ros2_ws/src/so101_hx35hm_bridge/firmware_reference/
```

在没有固件工程本体前，本仓库已经完成 ROS2/PC 侧调用入口和 STM32 侧参考实现，但不能替代实际烧录。
