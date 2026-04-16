## 修改目的

- **问题**：原始 SDK 与脚本默认使用 `/dev/ttyACM0` 作为 STM32 控制板串口设备名，当系统中存在多个 ACM 设备或插拔顺序变化时，设备号可能变为 `/dev/ttyACM1`、`/dev/ttyACM2` 等，导致程序无法找到控制板。
- **目标**：通过 udev 规则为控制板创建一个稳定的符号链接 `/dev/ros_robot_controller`，并在所有能够修改的代码中统一使用该链接，从而提高系统稳定性和可维护性。

## 具体修改内容

### 1. 增加 udev 规则（固定控制板设备名）

- **新增文件**：`src/ros_robot_controller-ros2/src/ros_robot_controller/scripts/99-ros-robot-controller.rules`

内容：

```text
SUBSYSTEM=="tty", \
  ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", \
  SYMLINK+="ros_robot_controller", \
  MODE="0666", GROUP="dialout", \
  ENV{ID_MM_PORT_IGNORE}="1"
```

- 作用：
  - 对所有 tty 设备中，厂商 ID `1a86`、产品 ID `55d4` 的串口设备（即控制板）：
    - 创建一个稳定的符号链接：`/dev/ros_robot_controller`。
    - 设置访问权限为 `0666`，属组为 `dialout`。
    - 设置 `ID_MM_PORT_IGNORE=1`，避免被 ModemManager 干扰。
  - 原有的 `/dev/ttyACM*` 设备仍然存在，不受影响。

- 安装步骤（只需执行一次）：

```bash
cd ~/ros2_ws/src/ros_robot_controller-ros2/src/ros_robot_controller/scripts
sudo cp 99-ros-robot-controller.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔控制板后验证：

```bash
ls -l /dev/ros_robot_controller
```

应看到类似输出：

```text
/dev/ros_robot_controller -> ttyACM0
```

### 2. 修改 Board 默认串口设备名

- **修改文件**：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`

- 改动前：

```python
class Board:
    def __init__(self, device="/dev/ttyACM0", baudrate=1000000, timeout=5):
        ...
```

- 改动后：

```python
class Board:
    def __init__(self, device="/dev/ros_robot_controller", baudrate=1000000, timeout=5):
        ...
```

- 影响：
  - 所有未显式传入 `device` 参数的 `Board()` 调用，都会默认使用 `/dev/ros_robot_controller`。
  - 依赖 udev 规则提供该符号链接；如果规则未安装或未生效，原有 `/dev/ttyACM0` 仍可通过显式传参方式使用。

### 3. 修改改 ID 工具脚本使用稳定设备名

- **修改文件**：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/change_servo_id.py`

- 核心改动：
  - 函数默认参数由 `/dev/ttyACM0` 改为 `/dev/ros_robot_controller`：

```python
def change_servo_id(
    device: str = "/dev/ros_robot_controller",
    old_id: int = 254,
    new_id: int = 1,
) -> None:
```

  - `__main__` 中示例调用改为使用稳定设备名：

```python
if __name__ == "__main__":
    change_servo_id(
        device="/dev/ros_robot_controller",  # 由 udev 规则固定的设备名
        old_id=254,
        new_id=6,
    )
```

- 影响：
  - 运行改 ID 脚本时，不再关心控制板当前真实的 `ttyACM*` 号，只要 `/dev/ros_robot_controller` 存在即可。

### 4. ROS2 控制节点行为说明

- **文件**：`src/ros_robot_controller-ros2/src/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py`

- 代码中仍然是：

```python
self.board = Board()
```

由于 `Board` 的默认设备已经改为 `/dev/ros_robot_controller`，该节点现在默认会通过稳定设备名访问控制板。

- 原理：
  - udev 规则确保无论实际是 `/dev/ttyACM0` 还是 `/dev/ttyACM1`，都会链接为 `/dev/ros_robot_controller`。
  - Node 初始化中的 `Board()` 会自动连接到该链接。

## 兼容性与影响评估

- **对原有功能的影响**：
  - 原有依赖 `Board()` 默认参数的所有代码（包括 `ros_robot_controller_node.py`）现在会优先使用 `/dev/ros_robot_controller`，只要 udev 规则安装正常，行为与之前连接 `/dev/ttyACM0` 一致。
  - 未显式安装新规则的环境中：
    - 仍可通过显式传递 `device="/dev/ttyACM0"` 来使用控制板；
    - 但推荐在目标机器上安装 `/etc/udev/rules.d/99-ros-robot-controller.rules` 以获得稳定行为。
- **安全性**：
  - 规则将权限设置为 `0666`，方便开发调试；若需更严格的权限控制，可根据实际需求将 `MODE` 调整为更严格值（例如 `0660`），并确保运行用户加入 `dialout` 组。

## 使用建议

1. 在所有目标机器人/开发机上：
   - 安装并启用 `99-ros-robot-controller.rules`。
   - 确认 `/dev/ros_robot_controller` 存在且指向正确的 `ttyACM*`。
2. 在新代码中：
   - 若无特殊需求，直接调用 `Board()` 或使用 `device="/dev/ros_robot_controller"`。
   - 仅在调试或特殊场景下，才需要显式使用真实的 `ttyACM*` 设备名。

