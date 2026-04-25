# HX35HM SO101 红球抓取调试日志

## 1. 文档目的

这份文档用于完整记录本项目在红球视觉抓取阶段的调试过程，重点说明：

- 初始问题是什么
- 每一轮排查发现了什么
- 哪些代码和参数被修改过
- 哪些方向最终保留
- 哪些方向已经被证明不合适并回滚
- 当前这套代码为什么会长成现在这样

这不是最终操作手册，而是一份工程调试日志。

配套文档：

- `HX35HM_SO101_红球抓取完整执行步骤.md`
- `HX35HM_SO101_手动调参指南.md`
- `HX35HM_SO101_环境清理与进程管理.md`

---

## 2. 初始状态与核心问题

项目主工作空间：

- `~/ros2_ws`

主要相关包：

- `so101_hx35hm_bridge`
- `so101-ros-physical-ai`
- `ros_robot_controller-ros2`

最初观察到的现象：

- 后抓取段比之前稳定，说明 `grasp -> close -> retreat` 这部分已经基本走通
- `pregrasp hover` 观感不够自然
- 视觉 3D 点还有残余误差
- 整体动作不够流畅，靠近目标时卡顿最明显
- 抓取后回 `rest` 的过程也不自然
- 某些时候夹爪没有正确保持闭合
- 某些时候回 `rest` 偶发失败

最初的总体判断：

- 视觉层不是唯一问题
- 任务层、规划层、执行层都有耦合问题
- 不能只靠单个 offset 或单个 duration 解决

---

## 3. 第一阶段：梳理整条执行链

### 3.1 初步架构理解

一开始先确认了红球抓取主链路：

- 视觉检测输出目标位姿
- `so101_visual_grasp` 负责生成 `hover_high / pregrasp / grasp / retreat / rest`
- 优先走 `/go_to_pose`、`/go_to_joints`
- 不可用时回退到 MoveIt
- 最终由 `hx35hm_bridge` 下发到底层控制板和舵机

这一步的核心结论：

- 任务层逻辑已经不少
- 但系统默认执行链没有完全闭合
- 有些节点设计上依赖 IK 服务，实际 bringup 时却没有默认启动

### 3.2 补齐 `cartesian_motion_node`

问题：

- `so101_visual_grasp` 设计上希望优先使用 `/go_to_pose`
- 但主 bringup 默认没有把 `cartesian_motion_node` 带起来
- 结果任务层经常退回 MoveIt fallback

处理：

- 在 `so101_bringup/launch/follower_hx35hm_moveit.launch.py` 中把 `cartesian_motion_node` 正式接入主链路

效果：

- `/go_to_pose` 和 `/go_to_joints` 可以在主 bringup 中稳定在线
- `so101_visual_grasp` 不再总是退回到 MoveIt fallback

这一步是整个调试过程的基础修复之一。

---

## 4. 第二阶段：修正 `cartesian_motion_node` 与 IK 服务链

### 4.1 服务句柄生命周期问题

现象：

- `/go_to_pose`、`/go_to_joints` 偶发消失
- 抓取节点又退回不想要的 fallback 路线

根因：

- Python 里 `create_service()` 返回对象没有保存为成员变量
- service handle 可能被回收

处理：

- 在 `so101_kinematics/cartesian_motion_node.py` 中保存服务对象引用

效果：

- `/go_to_pose` 和 `/go_to_joints` 在线状态稳定

### 4.2 到位判断过于乐观

现象：

- 轨迹发出后服务返回成功
- 但真实末端并没有可靠到位

处理：

- 在 `cartesian_motion_node.py` 中增加末端收敛检查
- 加入位置和姿态误差验证

过程中还暴露了一个实现级错误：

- 新增逻辑时漏了 `import time`

随后已修复并重新编译通过。

### 4.3 不再默认因真机误差硬失败

测试发现：

- 真机在某些 `go_to_pose` 目标上存在约 `3cm - 4cm` 的综合残余误差
- 这类误差未必代表动作无法使用

处理：

- 加入 `fail_on_goal_tolerance`
- 默认不因为这类真机误差直接把动作判死

结论：

- 系统需要区分“服务层轨迹完成”与“真机绝对精确到位”
- 否则会让流程过度敏感

### 4.4 `motion_planner` 导入路径问题

问题：

- `cartesian_motion_node.py` 实际导入的是 pip 环境中的 `robokin.motion_planner`
- 而不是工作区内本项目使用的 planner

影响：

- `/go_to_joints` 相关能力不完整
- 代码行为和预期不一致

处理：

- 改为显式使用工作区内 `so101_kinematics/motion_planner.py`
- 补上 joint-space 轨迹规划支持

---

## 5. 第三阶段：夹爪控制链排查与修复

### 5.1 夹爪没有保持闭合

现象：

- 日志显示已经执行 `Closing gripper...`
- 但实际观察到夹爪很快又被重新张开

根因：

- `/go_to_pose` 路径会向 `/follower/forward_controller/commands` 发布 6 维命令
- 其中包含旧的 gripper 打开值
- 随后的 arm 轨迹把刚关闭的夹爪又覆盖回去了

处理：

- 在 `cartesian_motion_node.py` 中将 arm-only `/go_to_pose` 输出改成只发布 5 个手臂关节
- 在 `so101_hx35hm_bridge/bridge_node.py` 中支持 5 维 arm-only command

效果：

- 夹爪的开合只由 gripper action 负责
- arm 轨迹不再无意中覆盖夹爪状态

### 5.2 夹爪闭合值确认

单独测试过：

- `/follower/gripper_controller/gripper_cmd` action 链路本身是通的
- `joint_states` 中 gripper 从打开位变到负角度

结论：

- 问题不在 gripper action 链路是否可用
- 关键在于 arm 流式控制是否错误地重新写回 gripper

### 5.3 回到 `rest` 后自动打开夹爪

需求：

- 抓住球
- 回到 `rest`
- 再打开夹爪把球放下

处理：

- 在 `so101_visual_grasp.cpp` 和 launch 中加入：
  - `open_gripper_after_return`
  - `post_return_gripper_settle_s`

效果：

- 完整抓取流程可在 `rest` 后自动释放目标

---

## 6. 第四阶段：抓取任务逻辑优化

这一阶段主要在 `so101_visual_grasp.cpp` 上推进。

### 6.1 两段式 pregrasp / hover

最初问题：

- 原始 `pregrasp` 只是 `grasp` 的简单抬高
- 视觉点如果有残余误差，hover 会直接跟着偏
- 观感也不自然

处理：

- 新增两段式 pregrasp 逻辑
- 引入参数：
  - `use_two_stage_pregrasp`
  - `hover_high_offset_m`
  - `pregrasp_x_offset_m`
  - `pregrasp_y_offset_m`
  - `pregrasp_z_extra_m`

效果：

- `hover_high -> pregrasp -> grasp` 的结构比原来合理
- 靠近目标时不再只有一个简单抬高点

### 6.2 连续 approach 模式

问题：

- `hover_high -> pregrasp -> grasp` 三段之间停顿感太强

处理：

- 加入 `continuous_approach_mode`
- 对 `hover_high` 和 `pregrasp` 的 settle 做更激进压缩
- 最终 `grasp` 只保留很短的稳定时间

效果：

- 接近段从“点一下、停一下、再点一下”改成更连续的靠近

### 6.3 抓后优先退回更高位

问题：

- 抓完只退回 `pregrasp` 高度显得局促
- 后面接回 `rest` 的动作很不自然

处理：

- 加入 `retreat_to_hover_high`
- 如果存在两段式 pregrasp，则 retreat 优先退到 `hover_high`

效果：

- 抓后先明显离开工作区
- 再进入后续回位流程

### 6.4 重试机制

问题：

- 失败时直接退出，容错差

处理：

- 靠近阶段加入：
  - `grasp_retry_count`
  - `grasp_retry_delay_s`
- 回 `rest` 阶段加入：
  - `post_grasp_return_retry_count`
  - `post_grasp_return_retry_delay_s`

设计原则：

- 夹爪闭合前失败，可以重读当前状态和目标后再次规划
- 夹爪闭合后不再重新抓球，只允许重新规划回位

效果：

- 系统在局部失败时不再立即退出
- 回 `rest` 的偶发失败也可以自动兜底

### 6.5 至少收集多个视觉样本

问题：

- 某些测试里只用了 1 帧视觉样本就开始执行
- 风险过高

处理：

- 增加 `min_pose_sample_count`
- 默认至少拿到 3 个样本再执行

效果：

- 降低偶发检测误差直接触发抓取的概率

---

## 7. 第五阶段：回 `rest` 策略的探索与取舍

这部分走了较多弯路，最终结论也比较重要。

### 7.1 直接用 `/go_to_joints` 回 `rest`

一开始尝试：

- 为了绕开 MoveIt named target 偶发失败
- 把回 `rest` 改成优先走 `/go_to_joints`

优点：

- 比较稳定

缺点：

- 这是关节空间直连
- 不理解桌面、不理解末端空间路径
- 会出现贴桌面横扫回 `rest` 的不合理行为

用户反馈：

- 明确认为这条路不合理

结论：

- 这条方案后来被否定

### 7.2 加 `lift` / `rest_hover` 中间段

后续为了缓解关节空间直回 `rest` 的问题，又尝试过：

- `post_grasp_return_lift`
- `post_grasp_rest_hover`

思路：

- 先抬高
- 再绕到一个高位 hover
- 再回 `rest`

问题：

- 仍然显得人工痕迹很重
- 逻辑越来越像补丁叠补丁

用户反馈：

- 明确表示不希望继续绕开 MoveIt 的正常路径规划

最终结论：

- 这条思路不作为最终主路线

### 7.3 回归 MoveIt named target 回位

最终保留方案：

- 抓取后回 `rest` 默认仍交给 MoveIt named target
- 不再默认走 `/go_to_joints` 作为主路径

后续补充：

- 加入 `post_grasp_return_vel_scaling`
- 加入 `post_grasp_return_acc_scaling`

目的：

- 回 `rest` 仍由 MoveIt 负责空间路径规划
- 但速度和加速度可以单独调得更柔和一些

当前保留默认值：

- `post_grasp_return_vel_scaling = 0.12`
- `post_grasp_return_acc_scaling = 0.12`

这是目前认可的最终方向。

---

## 8. 第六阶段：桌面高度、碰撞盒与视觉几何修正

### 8.1 桌面碰撞盒过大

问题：

- 早期的 `tabletop_guard` 覆盖范围太大
- MoveIt 会把基座附近和收纳区也算进桌面碰撞体

日志表现：

- `CheckStartStateCollision failed`
- `base_link - tabletop_guard`
- `gripper_link - tabletop_guard`

处理：

- 收窄并前移桌面碰撞体

当前保留参数：

- `table_center_x_m = 0.32`
- `table_center_y_m = 0.0`
- `table_size_x_m = 0.36`
- `table_size_y_m = 0.45`

效果：

- 抓取工作区仍受保护
- 但不会把基座附近全部误判成桌面

### 8.2 用深度相机估计真实桌面高度

最初问题：

- `tabletop_z_m` 完全手填，不够稳
- 视觉 fallback 平面和 MoveIt 桌面高度并不统一

处理：

- 新增 `so101_hx35hm_bridge/table_plane_estimator_node.py`
- 订阅深度图和内参
- 在桌面 ROI 中估计真实 `table top z`
- 发布：
  - `/vision/table/top_z`
  - `/vision/table/status`

在 `so101_visual_grasp.cpp` 中加入：

- `use_tabletop_z_topic`
- `tabletop_z_topic`
- `wait_tabletop_z_timeout_s`
- `fresh_tabletop_z_slack_s`

在 bringup 中加入：

- 使用 red detector 时默认启动 `table_plane_estimator`

效果：

- MoveIt 碰撞桌面和视觉平面高度开始统一
- 桌面高度不再完全依赖手工常数

---

## 9. 第七阶段：`bridge` 层调试

这一部分最关键，因为这里既有成功保留的改动，也有明确失败并回滚的实验。

### 9.1 保留的 `FollowJointTrajectory` 执行优化

处理内容：

- `trajectory_command_rate_hz: 30.0 -> 50.0`
- `trajectory_min_command_interval_s: 0.03 -> 0.015`
- `trajectory_min_segment_duration_s: 0.12 -> 0.02`
- `trajectory_min_total_duration_s: 0.80 -> 0.60`
- `trajectory_final_settle_s: 0.20 -> 0.05`

同时：

- 对 `JointTrajectoryPoint` 如果带有 `velocities`
- 优先使用三次 Hermite 插值
- 否则回退到线性插值

效果：

- MoveIt/FJT 轨迹更细、更连续
- 末尾拖尾缩短

这部分最终保留。

### 9.2 错误方向：低层 lookahead / 跳过冗余命令实验

曾经尝试过：

- `trajectory_command_lookahead_scale`
- `trajectory_min_move_duration_s`
- `trajectory_skip_redundant_commands`

目标：

- 让底层控制板执行更像流式伺服
- 尝试减少离散感

实际结果：

- 抓取完全变差
- 没抓住球
- 机械臂严重触地
- 前半段误差异常大

---

## 10. 第八阶段：`IK -> FJT` 调试中真正踩到的坑

这一阶段非常关键，因为后面发现：

- 早期把问题全部归因到 `IK -> FJT`
- 其实并不准确

真正踩到的坑不止一个。

### 10.1 错误固定四元数污染了实验

后续对比远端参考仓库时发现：

- 参考仓库抓取默认四元数是：
  - `qx=0`
  - `qy=1`
  - `qz=0`
  - `qw=0`

而本地曾经出现过另一组固定四元数：

- `qx=0.46467987`
- `qy=0.76828305`
- `qz=0.14769257`
- `qw=0.41472966`

由于抓取节点默认又是：

- `use_marker_orientation=false`
- `use_rpy=false`

这意味着末端姿态完全由这组固定四元数决定。

现场表现就是：

- `wrist_roll` 会出现以前没有的旋转
- 夹爪靠近球的姿态整体变得不对
- 很容易误以为是 FJT 导致整个抓取“突然不准了”

后来把四元数恢复成：

- `0, 1, 0, 0`

之后，行为立即恢复正常很多。

**这一步非常重要：说明之前那轮对 FJT 的负面结论，被错误姿态参数明显污染了。**

### 10.2 “只改执行后端”的干净 A/B 测试

在四元数恢复正常后，又重新做了一轮干净 A/B：

- 保持抓取姿态不变
- 保持视觉链不变
- 只切执行后端

这轮测试的意义是：

- 把“姿态异常”这个混杂因素剥离掉
- 真正比较执行风格差异

这轮测试最终用于回答一个更干净的问题：

- “如果四元数是正确的，FJT 到底行不行？”

### 10.3 `/go_to_pose` 的阶段切换停顿

即使抓取精度已经恢复，后续又发现另一个体验问题：

- 从一个 `/go_to_pose` 目标切到下一个目标时
- 每一步之间会有明显停顿

后来查明，停顿来自两层：

1. `/go_to_pose` 内部轨迹执行完后，还要再等一次末端姿态收敛
2. `so101_visual_grasp.cpp` 里阶段之间还插入了短暂 `settle`

这类停顿不是 bug，但在现场观感上会让动作显得“卡”。

### 10.4 这一轮最终做过的停顿优化

后续已经做过一轮压缩：

- `goal_wait_timeout_s` 压到 `0.35`
- `ik_pregrasp_settle_s` 压到 `0.03`
- `ik_grasp_settle_s` 压到 `0.02`
- `ik_retreat_settle_s` 压到 `0.02`

目的不是完全取消稳定时间，而是：

- 把明显的段间停顿压短
- 同时尽量不破坏抓取成功率

### 10.5 当前对这轮调试的真实结论

现在回看这轮 `IK -> FJT` 调试，比较准确的总结应该是：

1. 早期问题并不只是执行后端切换
2. 错误固定四元数是一个非常重要的混杂因素
3. 四元数修正后，FJT 才能被公平评估
4. 阶段切换的停顿主要来自：
   - 执行后的收敛等待
   - 显式 `settle`
5. 这些停顿已经被压缩优化

也就是说，这一轮调试真正收敛出来的是：

- **执行链**
- **姿态参数**
- **阶段等待**

这三者必须一起看，不能再只盯住其中一个。

---

## 11. 当前保留状态

截至当前版本，和这轮调试相关的保留状态是：

1. 抓取默认四元数保持为：
   - `qx=0.0`
   - `qy=1.0`
   - `qz=0.0`
   - `qw=0.0`
2. `IK -> FJT` 相关链路已经按修正后的版本保留
3. `/go_to_pose` 阶段间停顿已经做过一轮压缩
4. 以后再评估执行链差异时，必须保持姿态参数不变

这一步的意义非常大，因为它把这轮调试从：

- “凭感觉觉得 FJT 不好”

变成了：

- “知道当时到底有哪些变量一起被改了”
- “知道现在保留的版本为什么这样设”

这是整个调试过程中一个非常重要的反例。

结论：

- 这类低层语义重写不适合当前控制板和舵机系统
- 该实验已完整回滚

这也是后续不再继续“重塑底层协议语义”的原因。

### 9.3 真正有效的根因修复：区分流式命令与单次命令

后续重新分析后发现更关键的问题：

- `cartesian_motion_node` 以 `50Hz` 持续发布 `/follower/forward_controller/commands`
- `hx35hm_bridge` 却一直用同一个 `move_duration = 0.8` 去执行这些流式命令

这等价于：

- 每 `20ms` 发一个新命令
- 但每个命令都要求机械臂在 `0.8s` 内完成

后果：

- 机械臂一直在追赶一个慢时间常数目标
- 看起来会非常肉、拖、卡
- 精度也会下降

处理：

- 在 `bridge_node.py` 中新增 `stream_command_duration`
- `command_callback()` 对流式命令单独使用该 duration
- GUI/单次命令仍然保留原 `move_duration`

bringup 当前保留参数：

- `move_duration = 0.8`
- `stream_command_duration = 0.05`

这一步是后期最重要的保留改动之一。

### 9.4 `stream_command_duration` 的实测取舍

实际做过比较：

- `0.04`
- `0.05`
- `0.06`

观察结论：

- `0.04`
  - 更快更跟手
  - 但视觉上会更显加速度
- `0.05`
  - 精度和平滑度的平衡点最好
- `0.06`
  - 看起来更柔
  - 但开始明显损失前半段跟随质量
  - `hover/pregrasp/grasp/retreat` 误差都开始抬升

最终结论：

- `stream_command_duration = 0.05` 是当前项目最合适的默认值

---

## 10. 第八阶段：精度与平滑度的权衡

在后半程调试中，一个很明显的现象是：

- 把系统调得“更跟手”以后，抓取精度提升
- 但动作会更直接，主观上没那么“肉”

也就是说：

- 旧版那种“看起来顺”，很大一部分其实来自低层时间常数过大
- 它让动作更拖，也让误差更大

因此最终形成的共识是：

- 不能靠一味增大底层 duration 来换“平滑”
- 那会把精度一起拖坏

真正可接受的路线是：

- 保持 `stream_command_duration = 0.05`
- 在上层任务节奏上做温和调速
- 不再回到底层“慢吞吞追目标”的旧状态

当前保留的关键时长参数：

- `ik_pregrasp_duration_s = 1.7`
- `ik_grasp_duration_s = 1.1`
- `ik_retreat_duration_s = 1.1`

当前保留的回位缩放：

- `post_grasp_return_vel_scaling = 0.12`
- `post_grasp_return_acc_scaling = 0.12`

---

## 11. 第九阶段：统一手臂执行后端为 FJT

这一阶段的核心目标，是解决长期存在的结构性问题：

- 抓取前半段大多走 `/go_to_pose`
- `/go_to_pose` 原本通过 `50Hz /follower/forward_controller/commands` 流式下发
- 抓取后回 `rest` 主要走 MoveIt/FJT
- 同一条手臂存在两种完全不同的执行风格

这也是整体观感一直有段落感和机械感的重要根因之一。

### 11.1 原有 `/go_to_pose` 执行方式的问题

原有结构是：

- `cartesian_motion_node` 规划出 `(ts, qs)`
- `TrajectoryExecutor` 按时间线采样
- timer 以 `50Hz` 发送 `/follower/forward_controller/commands`

问题在于：

- 上游是轨迹
- 落地执行却变成高频离散重定目标
- 对当前控制板和总线舵机系统不够友好
- 同时和 MoveIt/FJT 路线风格完全不同

结论：

- 继续靠参数修补这条流式执行链，收益有限
- 更正确的方向是统一手臂执行后端

### 11.2 `/go_to_pose` 与 `/go_to_joints` 改走 `FollowJointTrajectory`

处理：

- 在 `so101_kinematics/cartesian_motion_node.py` 中加入 `ActionClient`
- 服务路径不再本地流式执行
- 改为：
  - 规划 `(ts, qs)`
  - 构造成完整 `JointTrajectory`
  - 通过 `/follower/arm_trajectory_controller/follow_joint_trajectory` 执行

新增关键参数：

- `fjt_action_name`
- `fjt_server_wait_timeout_s`
- `fjt_result_timeout_padding_s`

同时保留：

- `servo_target` 仍然作为交互式实时小步 IK 链路
- gripper 仍由独立 action 负责

### 11.3 验证结果

验证方式：

- 单独调用 `/go_to_pose`
- 检查 `cartesian_motion_node` 日志
- 检查 `hx35hm_bridge` 日志

验证结果：

- `cartesian_motion_node` 打印已生成完整轨迹
- `hx35hm_bridge` 收到：
  - `Received FollowJointTrajectory goal`
  - `Executing trajectory ... with N points over ...`
- 说明 `/go_to_pose` 已经不再走旧的流式关节命令后端

这一阶段的关键结论：

- 手臂执行后端统一这件事已经基本完成
- 后续调试重点不再是“是不是还在用 50Hz 流式执行”
- 而是“统一 FJT 后，IK 目标和默认姿态是否适合当前机械臂”

---

## 12. 第十阶段：`wrist_roll` / 末端姿态问题与默认抓取姿态修正

统一 FJT 后端后，新的瓶颈很快暴露出来：

- 控制夹爪旋转的舵机几乎被推到限位
- 某些姿态下 `wrist_roll` 明显接近硬边界
- 就算轨迹执行链已经统一，实际动作仍然不健康

### 12.1 根因确认

最初并不是：

- `hx35hm_bridge` 有 bug
- `FollowJointTrajectory` 执行坏了
- 控制板本身突然失效

而是：

- 默认抓取姿态本身，把 IK 推到了一个很差的关节分支
- 这个分支会让 `wrist_roll` 接近硬限位

表现包括：

- `wrist_roll` 接近 `-2.7 rad`
- bridge 层出现关节命令 clip
- 夹爪旋转舵机几乎顶死

### 12.2 先做 wrist-roll 友好的 IK seed 搜索

处理：

- 在 `so101_kinematics/motion_planner.py` 中加入 wrist-roll-aware 的 seed 搜索
- 对同一个目标姿态尝试多组 wrist_roll 初值
- 选择：
  - `abs(wrist_roll)` 更小
  - 且整体偏离起始关节更少

效果：

- 这一步改善了一部分极端分支选择问题
- 但并没有从根本上解决默认抓取姿态不合适的问题

结论：

- seed 搜索是辅助修正
- 不是根治方案

### 12.3 第一轮默认抓取姿态修正

先把默认四元数从：

- `qx=0.0`
- `qy=1.0`
- `qz=0.0`
- `qw=0.0`

改到：

- `qx=0.70710678`
- `qy=0.70710678`
- `qz=0.0`
- `qw=0.0`

效果：

- `wrist_roll` 从接近 `-2.744` 拉回到大约 `-1.34`
- 比之前安全很多

但问题仍然存在：

- `wrist_flex` 仍然不够健康
- 整体姿态还不算最佳

### 12.4 第二轮姿态搜索与最终保留默认值

后续做了离线姿态搜索：

- 固定抓取点位置
- 搜索不同末端姿态对应的 IK 分支
- 比较：
  - `wrist_roll`
  - `wrist_flex`
  - 目标误差
  - 姿态稳定性

最终选出当前保留的默认四元数：

- `qx=0.46467987`
- `qy=0.76828305`
- `qz=0.14769257`
- `qw=0.41472966`

该值已写入：

- `so101_grasping/launch/so101_visual_grasp.launch.py`

### 12.5 结果

用当前默认姿态后，前半段典型结果变成：

- `hover_high` 误差约 `0.006m`
- `pregrasp` 误差约 `0.006m`
- `grasp` 误差约 `0.006m`
- `retreat` 误差约 `0.006m`

同时典型关节状态明显健康很多：

- `wrist_roll ≈ -0.852`
- `wrist_flex` 不再长期贴着上边界

这一阶段的关键结论：

- 之前前半段“不顺、不稳、容易顶边界”，根因里有很大一块就是默认抓取姿态不适合当前机械臂
- 统一 FJT 后端只是第一步
- 选对默认抓取姿态才是真正把机械臂拉回健康工作区的关键

---

## 13. 第十一阶段：视觉链 RGB-D residual 排查

当机械臂执行链逐步稳定后，视觉侧的一个老问题重新变得突出：

- `Projective RGB-D registration residual too large`

说明当前红球 3D 点虽然能用，但几何质量仍不够理想。

### 13.1 现象

日志中经常出现：

- `Projective RGB-D registration residual too large: ...`
- detector fallback 到 approximate mapping

这意味着：

- 系统虽然配置成 `projective` RGB-D 匹配
- 但很多时候其实因为 residual 太大，已经退回近似映射
- 红球 3D 点的 XY 稳定性会受影响

### 13.2 根因分析

排查后确认：

- 红球检测本身不是主因
- 主要问题在 `cam_overhead_depth -> cam_overhead` 这组 RGB-depth 外参不够准

当前系统里：

- RGB 相机 frame：`cam_overhead`
- depth 相机 frame：`cam_overhead_depth`
- `red_circle_detector` 在 `projective` 模式下会优先使用 TF 中的 depth->rgb 外参

而当前默认 TF 只是近似值：

- `x = 0.010`
- `y = -0.014`
- `z = 0.0`
- `roll = pitch = yaw = 0`

这套值不足以把 residual 压到理想范围，所以经常卡在：

- `14 ~ 18 px`

### 13.3 结论

这一步的关键结论非常明确：

- 问题不在抓取逻辑
- 不在 MoveIt
- 不在 `/go_to_pose`
- 也不在红球 HSV 阈值

而在：

- RGB 与 depth 之间的几何外参还没有真正调准

---

## 14. 第十二阶段：引入 RGB-D tuner 与手动调试流程

### 14.1 现有工具链

项目里已经具备在线调 RGB-D 外参的工具：

- `so101_bringup/scripts/so101_depth_rgb_tuner.py`

bringup 可通过：

- `use_depth_to_rgb_tuner:=true`

启用 tuner 窗口。

### 14.2 调试原则

后续形成的调试规则是：

1. 先清理旧进程，避免多个 TF/相机/detector 同时存在
2. 用 tuner 模式启动主栈
3. 按顺序只动：
   - `x`
   - `y`
   - `yaw`
   - 再考虑 `z`
   - 最后才看 `roll/pitch`
4. 目标不是一步到最优，而是先把 residual 从 `14~18 px` 压到：
   - `<= 8 px`
   - 更理想是 `<= 5 px`

### 14.3 文档化

这一阶段新增了独立调试文档：

- `HX35HM_SO101_RGBD外参调试指南.md`

用途：

- 让后续 RGB-D 外参调试不再依赖临场口头指导
- 形成可以重复执行的操作规程

---

## 15. 当前保留的主路线总结

经过上述各轮调试后，当前保留路线可以概括为：

### 15.1 手臂执行层

- `FollowJointTrajectory` 是主执行后端
- `/go_to_pose` 与 `/go_to_joints` 已统一走 FJT
- `servo_target` 仅保留为交互式实时小步 IK 通道

### 15.2 抓取任务层

- 使用两段式 pregrasp
- `continuous_approach_mode = true`
- `retreat_to_hover_high = true`
- 默认抓取姿态采用当前保留四元数：
  - `qx=0.46467987`
  - `qy=0.76828305`
  - `qz=0.14769257`
  - `qw=0.41472966`

### 15.3 底层 bridge

- 保留 FJT 重采样与 Hermite 插值优化
- 保留：
  - `move_duration = 0.8`
  - `stream_command_duration = 0.05`
- 不再继续尝试重写底层 lookahead / 跳过冗余命令那类危险实验

### 15.4 视觉链

- 红球检测主流程已可用
- 桌面高度通过 topic 统一给抓取层使用
- RGB-D 外参仍需继续通过 tuner 收敛

---

## 16. 后续调试原则

这轮调试留下的几个经验非常重要：

1. 先确认“执行后端是不是统一”，再谈平滑度
2. 不要把“更肉、更拖”误当成真正的平滑
3. 姿态分支问题比单纯调速度参数更关键
4. 底层协议语义不要轻易重写
5. RGB-D residual 过大时，不要继续盲调抓取 offset，应先调外参

---

## 17. 文档定位

这份文件是调试日志，不是最终使用说明。

配套文档如下：

- `HX35HM_SO101_红球抓取完整执行步骤.md`
- `HX35HM_SO101_手动调参指南.md`
- `HX35HM_SO101_环境清理与进程管理.md`
- `HX35HM_SO101_机械臂控制链详解.md`
- `HX35HM_SO101_控制链接口速查表.md`
- `HX35HM_SO101_RGBD外参调试指南.md`

---

## 11. 第九阶段：`smooth` 分支的尝试与彻底删除

曾经做过一轮两档配置：

- `precision`
- `smooth`

做法：

- 在 launch 中加入 `control_profile`
- `smooth` 使用更慢的节奏和更高的 `stream_command_duration`

测试结果：

- `smooth` 并没有真正更平滑
- 但精度明显更差
- 回 `rest` 首次执行失败概率更高

用户最终判断：

- 不再需要 `smooth`

处理：

- 完整删除 `control_profile` 与 `smooth` 相关代码
- `follower_hx35hm_moveit.launch.py` 恢复单一稳定默认值
- `so101_visual_grasp.launch.py` 恢复单一稳定默认值
- 主文档中同步移除 `smooth` 说明

结论：

- 本项目不再维护 `smooth` 档
- 当前只保留单一路线

---

## 12. 调试过程中形成的关键经验

### 12.1 不能把“动作看起来慢”误认为“动作更平滑”

慢和顺不是同一个概念。

如果只是因为底层时间常数过大导致机械臂一直在追旧目标，那么：

- 看起来会更肉
- 但不是更好
- 精度和一致性都会变差

### 12.2 回 `rest` 必须交给 MoveIt 做路径规划

关节空间直接回 `rest` 虽然简单，但路径不合理。

最终确定：

- `rest` 是一个目标姿态
- 不是一条安全路径

### 12.3 不要轻易重写低层控制语义

控制板是串口 + duration 语义，不是高带宽伺服器。

对低层发包语义做过度重写，风险非常高。

### 12.4 真机系统必须允许局部重试

对于这类系统：

- 某一轮 `rest` 第一次执行失败
- 或某一轮 pregrasp 局部偏差较大

并不代表整体逻辑错误。

适度重试比“一次失败立即退出”更符合真实机械臂场景。

### 12.5 桌面几何信息必须统一

视觉平面、MoveIt 碰撞桌面、抓取安全高度，如果不是同一套几何来源，就很容易互相打架。

---

## 13. 当前保留下来的稳定主路线

截至本日志整理时，当前保留并推荐的主路线是：

1. 主 bringup 默认启动 `cartesian_motion_node`
2. `so101_visual_grasp` 优先使用 `/go_to_pose`
3. `hover_high -> pregrasp -> grasp` 使用连续靠近思路
4. 抓后优先 retreat 到更高位
5. 回 `rest` 默认仍交给 MoveIt named target
6. `rest` 后自动打开夹爪放球
7. 桌面高度优先读取 `/vision/table/top_z`
8. `stream_command_duration = 0.05`
9. 不再保留 `smooth` 档

---

## 14. 当前代码里最关键的稳定参数

### 14.1 `follower_hx35hm_moveit.launch.py`

- `move_duration = 0.8`
- `stream_command_duration = 0.05`

### 14.2 `bridge_node.py`

- `trajectory_command_rate_hz = 50.0`
- `trajectory_min_command_interval_s = 0.015`
- `trajectory_min_segment_duration_s = 0.02`
- `trajectory_min_total_duration_s = 0.60`
- `trajectory_final_settle_s = 0.05`

### 14.3 `so101_visual_grasp.launch.py`

- `continuous_approach_mode = true`
- `retreat_to_hover_high = true`
- `use_tabletop_z_topic = true`
- `tabletop_z_topic = /vision/table/top_z`
- `table_center_x_m = 0.32`
- `table_size_x_m = 0.36`
- `table_size_y_m = 0.45`
- `ik_pregrasp_duration_s = 1.7`
- `ik_grasp_duration_s = 1.1`
- `ik_retreat_duration_s = 1.1`
- `post_grasp_return_vel_scaling = 0.12`
- `post_grasp_return_acc_scaling = 0.12`
- `open_gripper_after_return = true`
- `grasp_retry_count = 1`
- `post_grasp_return_retry_count = 3`

---

## 15. 仍然存在但已经可控的问题

当前仍然可能出现：

- 回 `rest` 第一次规划或执行失败，但重试成功
- 个别轮次的 `hover_high` / `pregrasp` 偏差仍然高于最终 `grasp`
- 视觉和真机误差不会完全消失

但这些问题目前已经从“系统性失败”下降到了“局部、可控、可重试”的级别。

---

## 16. 后续调参建议

如果以后继续调，建议遵守以下顺序：

1. 先保证环境干净，避免旧节点干扰
2. 先确认桌面高度和视觉目标正常
3. 先保 `stream_command_duration = 0.05` 不动
4. 再微调：
   - `ik_pregrasp_duration_s`
   - `ik_grasp_duration_s`
   - `ik_retreat_duration_s`
5. 如果只是回位太急，再单独调：
   - `post_grasp_return_vel_scaling`
   - `post_grasp_return_acc_scaling`
6. 不建议重新尝试：
   - 低层 lookahead 风格改造
   - 通过增大底层 duration 换“假平滑”
   - 用 joint-space 直回 `rest` 当默认主路线

---

## 17. 结论

这轮调试的最终成果，不是把系统调成“完美无缺”，而是把它从一个多处逻辑互相打架、执行链不闭合、平滑与精度混在一起的状态，整理成了：

- 主链路明确
- 默认执行路径可解释
- 失败模式可识别
- 不合理方向已被证伪
- 当前参数有清晰工程依据

最关键的几条结论如下：

- `cartesian_motion_node` 必须是主链路的一部分
- gripper 不能再被 arm 流式命令覆盖
- 回 `rest` 应继续交给 MoveIt
- 桌面高度要尽量来自真实深度估计
- `stream_command_duration = 0.05` 是当前最合理的折中点
- `smooth` 路线不适合当前项目，已经删除

这份日志的价值就在于：以后如果再出现类似问题，可以直接回看这份文档，知道哪些方向已经试过、哪些路不该再走。
