# HX35HM SO101 主从控制紧急停止命令

适用场景：

- 主臂突然上力，无法手动拖动
- 怀疑 `teleop_hx35hm.launch.py`、`hx35hm_bridge`、`follower_command_relay` 或 MoveIt 残留进程还在运行
- 需要先把主从控制整条链完全停掉，再重新排查

## 一键停止命令

```bash
pkill -f 'teleop_hx35hm.launch.py|follower_command_relay|hx35hm_bridge|leader_hx35hm.launch.py|follower_hx35hm_moveit.launch.py|move_group|rviz2|robot_state_publisher' || true
```

## 作用

这条命令会尝试停止下面这些相关进程：

- `teleop_hx35hm.launch.py`
- `follower_command_relay`
- `hx35hm_bridge`
- `leader_hx35hm.launch.py`
- `follower_hx35hm_moveit.launch.py`
- `move_group`
- `rviz2`
- `robot_state_publisher`

## 停止后检查

执行完以后，可以用下面这条命令确认相关进程是否已经退出：

```bash
ps -ef | rg 'teleop_hx35hm|leader_hx35hm|follower_hx35hm_moveit|hx35hm_bridge|follower_command_relay|move_group|rviz2|robot_state_publisher'
```

理想情况是：只剩下 `rg` 这一条查询命令本身，没有真正的主从控制进程残留。

## 经验判断

如果执行完紧急停止命令后：

- 主臂重新变松，可以手动拖动

那么通常说明问题更像是：

- 运行态残留
- 多套 launch 混跑
- 串口或状态被旧进程污染

而不是主臂硬件本身一定坏了。
