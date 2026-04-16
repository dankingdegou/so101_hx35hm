# SO101 HX35HM Debug Log

维护约定：
- 每次出现新问题，都在本文件追加一条记录（按时间倒序或正序均可，这里用正序）。
- 每条记录至少包含：时间、现象、影响、定位/根因、修复动作、验证结果、后续事项（如有）。
- 时间统一使用 Asia/Shanghai，本机 `ros2` 日志时间戳可作为佐证。

---

## 2026-03-21

### 夹爪可规划但 `Plan and Execute` 失败，`move_group` 报未连接到 `follower/gripper_controller/gripper_cmd`
- 现象
  - 机械臂 5 轴在 MoveIt 中可正常规划执行
  - 夹爪单独规划成功，但执行阶段失败，典型日志：
    - `Action client not connected to action server: follower/gripper_controller/gripper_cmd`
    - `Failed to send trajectory part 1 of 1 to controller follower/gripper_controller`
    - `CONTROL_FAILED`
- 影响
  - RViz / MoveIt 中夹爪无法执行开合
  - 任何依赖 MoveIt gripper group 的抓取流程都会在执行阶段中断
- 定位/根因
  - [`so101_moveit_config/config/moveit_controllers.yaml`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/moveit_controllers.yaml) 中，`follower/gripper_controller` 被定义为 `ParallelGripperCommand`
  - 但 [`so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py) 之前实际提供的是 `control_msgs/action/GripperCommand` action server
  - action 路径虽然同为 `/follower/gripper_controller/gripper_cmd`，但类型不一致，MoveIt 的 `parallel_gripper_controller_handle` 无法建立连接，因此报“not connected”
  - 排查中还发现一次环境干扰：
    - 曾在 `/home/rog/ros2_ws/src` 子目录单独 `colcon build`，生成了 `/home/rog/ros2_ws/src/install`
    - 但实际运行时 `source` 的是 `/home/rog/ros2_ws/install/setup.bash`
    - 导致一度出现“源码已改、运行仍像旧版”的假象
  - 另外，旧的 launch / node 进程未完全退出时，也会让修复后的结果看起来像“未生效”
- 修复动作
  - 将 [`so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py) 中的夹爪 action server 从 `GripperCommand` 改为 `ParallelGripperCommand`
  - 同步修改 `execute_gripper_callback()`：
    - 从 `goal_handle.request.command.position[0]` 读取目标值
    - 按 `ParallelGripperCommand.Result` 填充 `state / stalled / reached_goal`
  - 在根工作区重新构建：
    - `cd ~/ros2_ws && colcon build --packages-select so101_hx35hm_bridge`
  - 重新 `source ~/ros2_ws/install/setup.bash`
  - 清理旧进程后重新启动 MoveIt bringup
- 验证结果
  - `ros2 action list | grep gripper` 可见：
    - `/follower/gripper_controller/gripper_cmd`
  - `ros2 action type /follower/gripper_controller/gripper_cmd` 返回：
    - `control_msgs/action/ParallelGripperCommand`
  - `ros2 action info /follower/gripper_controller/gripper_cmd` 显示：
    - Action client: `/moveit_simple_controller_manager`
    - Action server: `/follower/hx35hm_bridge`
  - 用户最终确认：问题已解决，夹爪可通过 MoveIt 正常执行
- 后续事项
  - 后续若再出现“名字能看到但 MoveIt 仍说 not connected”，优先检查：
    - `ros2 action type /follower/gripper_controller/gripper_cmd`
    - 当前 shell 是否 `source ~/ros2_ws/install/setup.bash`
    - 是否残留旧的 `hx35hm_bridge` / `move_group` 进程

## 2026-03-17

### 20:02 `so101_simple_pick` 执行夹爪 open 时 MoveIt 立即 aborted（缺少 gripper 组 OMPL 配置）
- 现象
  - `so101_simple_pick` 输出：
    - `Opening gripper (named target 'open')...`
    - `Plan and Execute request accepted`
    - `Plan and Execute request aborted`
    - `MoveGroupInterface::move() failed`
- 影响
  - 抓取序列卡在第一步（夹爪开合无法通过 MoveIt 执行）
- 定位/根因
  - `so101_moveit_config/config/ompl_planning.yaml` 仅配置了 `manipulator` 组的 OMPL planners
  - `gripper` 组虽然是 1-DOF，但 `MoveGroupInterface::move()` 仍会走规划管线；缺少组配置时可能导致 move_group 立即 abort
- 修复动作
  - 在 [`so101_moveit_config/config/ompl_planning.yaml`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/ompl_planning.yaml) 增加：
    - `gripper:` -> `planner_configs: [RRTConnect]`
  - `colcon build --packages-select so101_moveit_config`
  - 重启 `move_group`（必须重启才能加载新参数）
- 验证结果
  - 待用户重启 move_group 后复测 `so101_simple_pick`

### 19:45 多个 `hx35hm_bridge` 同时占用串口导致抖动 + 回读全失败
- 现象
  - `hx35hm_bridge` 持续报警：`Position readback updated 0 joints (timeouts?)`
  - 执行轨迹时舵机明显抖动、比接入视觉前更差
- 影响
  - 关节状态回读不可用，MoveIt 状态不准
  - 多源下发命令互相打架，出现抖动/抽搐
- 定位/根因
  - `/dev/ros_robot_controller -> /dev/ttyACM2` 被多个进程同时打开
  - `lsof /dev/ros_robot_controller` 显示同一设备上存在多个 `hx35hm_bridge` 进程
- 修复动作
  - 在 [`so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/bridge_node.py) 增加串口单实例锁（`/tmp/so101_hx35hm_bridge*.lock`），重复启动会直接报错退出，避免破坏总线
  - 建议操作：先停止所有旧 `ros2 launch`，必要时 `pkill -f hx35hm_bridge`
- 验证结果
  - 代码已构建：`colcon build --packages-select so101_hx35hm_bridge`
  - 需要在真实运行环境中确认：只保留 1 个 bridge 进程后抖动是否消失、readback 是否恢复
- 后续事项
  - 若只剩 1 个进程仍 readback=0：再排查供电/波特率/舵机ID/线路

### 19:37 `so101_simple_pick` 报 `robot_description` 缺失导致 URDF 解析失败
- 现象
  - 运行 `ros2 run so101_grasping so101_simple_pick ...` 报：
    - `Could not find parameter robot_description ... within 10 seconds`
    - `Error=XML_ERROR_EMPTY_DOCUMENT`
    - `Unable to construct robot model`
- 影响
  - MoveGroupInterface 无法初始化，抓取脚本直接崩溃
- 定位/根因
  - MoveIt2 的 `MoveGroupInterface` 期望从“当前节点自身参数”读取 `robot_description`/SRDF 等
  - 单独运行 `so101_simple_pick` 时该节点没有这些参数
- 修复动作
  - 在 [`so101_grasping/src/so101_simple_pick.cpp`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_grasping/src/so101_simple_pick.cpp) 增加逻辑：
    - 从运行中的 `/move_group` 通过 parameters service 拉取并注入：
      - `robot_description`
      - `robot_description_semantic`
      - `robot_description_kinematics.*`
      - `robot_description_planning.*`
    - NodeOptions：`allow_undeclared_parameters(true)` + `automatically_declare_parameters_from_overrides(true)`
    - 避免重复 declare 导致 `ParameterAlreadyDeclaredException`
- 验证结果
  - 运行时已能打印：
    - `Loaded robot model params from '/move_group' ...`
    - `Copied ... robot_description_kinematics ...`
    - `Copied ... robot_description_planning ...`
  - 后续仍出现 `Plan and Execute aborted`：属于控制器/动作服务器就绪或硬件侧问题，不再是 URDF 参数问题

### 16:43 MoveIt + Octomap 点云接入修复（话题不匹配 + updater 潜在死锁）
- 现象
  - MoveIt 启用 octomap 后，点云 updater 无法拿到正确点云（话题不一致）
  - updater 在某些构建上可能因锁/回调触发导致 `std::system_error` 崩溃风险
- 影响
  - PlanningScene 的 Octomap 不更新，避障无效或异常
  - move_group 可能崩溃
- 定位/根因
  - so101_openni2_camera 默认点云为 `/static_camera/depth_overhead/points`
  - 原配置使用了 `/static_camera/points`
  - updater 在持有写锁时调用 `tree_->triggerUpdateCallback()`，可能触发死锁/系统错误
- 修复动作
  - 修改配置：
    - [`so101_moveit_config/config/octomap_pointcloud.yaml`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/octomap_pointcloud.yaml)
  - 修改 updater：
    - [`so101_moveit_octomap_updater/src/pointcloud_octomap_updater.cpp`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_moveit_octomap_updater/src/pointcloud_octomap_updater.cpp)
    - 写锁作用域内只做 `clear/updateNode/updateInnerOccupancy`，释放锁后再 `triggerUpdateCallback()`
    - 增加互斥保护，避免多线程回调并发更新
  - 同步修正文档话题：
    - [`ORBBEC_ASTRA_PRO_PLUS_ROS2_架构与接入方案.md`](/home/rog/ros2_ws/src/ORBBEC_ASTRA_PRO_PLUS_ROS2_架构与接入方案.md)
- 验证结果
  - `ros2 launch so101_bringup follower_hx35hm_moveit.launch.py use_octomap:=true use_rviz:=false`
    - move_group 能启动并打印 updater 初始化：`topic='/static_camera/depth_overhead/points'`

### 16:40 `follower_hx35hm_moveit.launch.py` 引用未安装 launch 文件
- 现象
  - 启动报错：`No such file or directory: .../camera_tf_moveit.launch.py`
- 定位/根因
  - 运行环境混用了不同 overlay（`/home/rog/ros2_ws` vs `/home/rog/ros2_ws/src`）
  - 以及 `so101_bringup` 安装目录中曾缺少该文件（旧安装）
- 修复动作
  - 强制重建/安装 `so101_bringup`，并确认运行时 `source` 的 overlay 与 build/install 目录一致
- 验证结果
  - 之后 moveit bringup 可正常启动并拉起 `camera_tf_moveit` 静态 TF

### 16:47 `bus_servo_read_position(timeout=...)` 报关键字参数不支持
- 现象
  - `Board.bus_servo_read_position() got an unexpected keyword argument 'timeout'`
- 定位/根因
  - 运行时实际 import 的是旧版已安装的 `ros_robot_controller`（签名不含 timeout）
- 修复动作
  - 重新构建并确保使用工作空间内新版 `ros_robot_controller`：
    - `colcon build --packages-select ros_robot_controller`
    - 并在运行前 `source install/setup.bash`
- 验证结果
  - `inspect.signature(Board.bus_servo_read_position)` 显示 `timeout=None`

### 18:15 ArUco 话题无输出（视觉链路已修复）
- 现象
  - `/vision/aruco/pose_camera`、`/vision/aruco/detected_ids` 不输出。
  - 偶发只看到 `debug_image`，或采样窗口黑屏。
- 根因
  - 同时运行了两套 `follower_hx35hm_moveit.launch.py`，相机和串口资源冲突。
  - 冲突后 ArUco 输入链路不稳定，节点容易出现“有进程但无有效数据”。
  - `pose_base` 转换调用不兼容（`'PoseStamped' object has no attribute 'position'`），会导致节点异常退出。
- 修复动作
  - 运行层面：只保留一套主 launch，清理重复进程。
  - 代码层面：
    - [`so101_hx35hm_bridge/so101_hx35hm_bridge/aruco_detector_node.py`](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/aruco_detector_node.py)
      - 增加异常兜底，单帧异常不再导致节点退出。
      - 将 `pose_base` 变换改为 `tf_buffer.transform()` 标准写法，修复 PoseStamped 转换错误。
    - [`so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py`](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/follower_hx35hm_moveit.launch.py)
      - `aruco_detector` 增加 `respawn=True`、`respawn_delay=2.0`。
- 验证结果
  - `ros2 topic echo /vision/aruco/debug_image --once` 可稳定收到图像。
  - `/vision/aruco` 下 4 个关键话题已就绪：`debug_image`、`detected_ids`、`pose_camera`、`pose_base`。
