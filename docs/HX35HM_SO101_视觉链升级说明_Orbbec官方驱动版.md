# HX35HM + SO101 视觉链升级说明（Orbbec 官方驱动版）

适用对象：当前这套 `HX-35HM + SO101 follower + Astra Pro Plus + 红球抓取`

文档目的：说明**这一版视觉链**和你之前那版视觉链相比，到底改了什么、为什么要改、哪些行为已经变化、现在还剩什么已知问题。

相关文档：

- `HX35HM_SO101_红球抓取完整执行步骤.md`
- `HX35HM_SO101_红球抓取调试日志.md`
- `HX35HM_SO101_RGBD外参调试指南.md`
- `HX35HM_SO101_机械臂控制链详解.md`

---

## 1. 先说结论

这次视觉链升级的核心，不是“换了一个小参数”，而是**把整套 Astra 深度链从旧的 OpenNI2 方案切到了 Orbbec 官方 ROS2 驱动方案**。

一句话概括：

**旧版是 `gscam RGB + 自定义 OpenNI2 depth`，新版是 `Orbbec 官方驱动同时输出 RGB + Depth`，然后再加一层兼容转发，继续喂给你原来的红球检测和抓取链。**

所以这一版的本质变化是：

1. 深度驱动栈变了
2. RGB 来源也变了
3. 视觉话题表面上尽量保持兼容
4. 底层相机坐标系和外参来源已经不是旧逻辑了

---

## 2. 为什么要升级

升级的直接原因是：**旧版深度链已经不稳定，甚至在当前机器上根本起不来。**

旧版的典型问题是：

- `openni2_camera_node` 报 `no devices found`
- OpenNI2 运行时不能稳定识别 Astra Pro Plus
- 即使强行让 OpenNI2 识别到设备，底层 `liborbbec.so` 也会崩
- 抓取流程经常退化成：
  - 没有真实桌面高度
  - 没有有效深度
  - 只能靠静态 `tabletop_z_m`
  - 红球位姿精度明显下降

也就是说，旧版的主要矛盾已经不是“算法还要调一调”，而是**底层深度驱动链本身不成立了**。

---

## 3. 旧版视觉链是什么样

旧版视觉链可以概括成下面这条：

1. `gscam` 负责 overhead RGB
2. `so101_openni2_camera/openni2_camera_node` 负责 depth
3. `camera_tf_moveit.launch.py` 提供：
   - `base_link -> cam_overhead`
   - `cam_overhead -> cam_overhead_depth`
4. `red_circle_detector` 同时吃 RGB 和 depth
5. `table_plane_estimator` 用 depth 估桌面高度

旧版关键特征是：

- RGB 和 depth 实际上是两条独立链
- RGB 来自 UVC 设备
- depth 来自 OpenNI2
- `red_circle_detector` 需要自己做 RGB/depth 对齐
- `/static_camera/image_raw` 是旧版 RGB 主入口
- `/static_camera/depth/image_raw` 是旧版 depth 主入口

旧版常见配置入口是：

- [so101_cameras_astra_overhead_rgbd.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml)
- 旧逻辑里它会起：
  - `gscam`
  - `so101_openni2_camera`

---

## 4. 新版视觉链是什么样

新版视觉链改成了下面这条：

1. `orbbec_camera` 官方驱动同时打开 color + depth
2. 官方驱动直接发布：
   - `/static_camera/color/image_raw`
   - `/static_camera/color/camera_info`
   - `/static_camera/depth/image_raw`
   - `/static_camera/depth/camera_info`
3. 我额外加了一个兼容节点：
   - `camera_topic_compat`
4. 它把官方 color 话题转回旧接口：
   - `/static_camera/image_raw`
   - `/static_camera/camera_info`
5. 你的 `red_circle_detector`、`aruco_detector`、`table_plane_estimator` 继续用原来的入口

新版的核心思想是：

**底层彻底换新，但上层尽量不动。**

---

## 5. 这一版具体改了哪些文件

### 5.1 相机启动逻辑

- [cameras.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/cameras.launch.py)

这次它新增支持：

- `orbbec_camera_launch`
- `so101_camera_topic_compat`

也就是说，现在相机 bringup 不再只会起 `gscam`、`usb_cam`、`openni2`，而是能直接 include Orbbec 官方 launch，并能顺手起兼容中转节点。

### 5.2 Astra RGBD 配置

- [so101_cameras_astra_overhead_rgbd.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd.yaml)
- [so101_cameras_astra_overhead_rgbd_lowbw.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd_lowbw.yaml)
- [so101_cameras_astra_overhead_rgbd_720.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/so101_cameras_astra_overhead_rgbd_720.yaml)

这三份文件原来是：

- `gscam`
- 再配一条 depth 节点

现在改成：

- 只起一个 `orbbec_camera` 官方 launch
- `enable_color:=true`
- `enable_depth:=true`
- 再补一个 `camera_topic_compat`

### 5.3 新增兼容节点

- [camera_topic_compat_node.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/so101_hx35hm_bridge/camera_topic_compat_node.py)
- [setup.py](/home/rog/ros2_ws/src/so101_hx35hm_bridge/setup.py)

这个节点的职责非常单纯：

- 订阅 `/static_camera/color/image_raw`
- 订阅 `/static_camera/color/camera_info`
- 原样转发到：
  - `/static_camera/image_raw`
  - `/static_camera/camera_info`

这样你上层节点基本不用改。

### 5.4 相机 TF 兼容

- [camera_tf_moveit.launch.py](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/launch/camera_tf_moveit.launch.py)

这次额外补了两个 frame：

- `cam_overhead -> static_camera_color_optical_frame`
- `cam_overhead_depth -> static_camera_depth_optical_frame`

这是因为官方驱动自己用的是：

- `static_camera_color_optical_frame`
- `static_camera_depth_optical_frame`

而你原来的工程主要认：

- `cam_overhead`
- `cam_overhead_depth`

### 5.5 新增 Orbbec 专用标定文件

- [static_camera_color_calib.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/static_camera_color_calib.yaml)
- [static_camera_color_calib_placeholder_320.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/static_camera_color_calib_placeholder_320.yaml)
- [static_camera_color_calib_placeholder_720.yaml](/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/static_camera_color_calib_placeholder_720.yaml)

目的不是重新标定，而是先把官方驱动的 `camera_name` 对齐，避免启动日志一直报：

- `camera_name does not match`

---

## 6. 新旧两版最核心的差异

## 6.1 驱动来源不同

旧版：

- RGB：`gscam`
- depth：`so101_openni2_camera`

新版：

- RGB：`orbbec_camera`
- depth：`orbbec_camera`

这意味着：

- 旧版是两条底层链拼起来
- 新版是同一套官方驱动同时出彩色和深度

---

## 6.2 话题来源不同，但接口尽量兼容

旧版真正的 RGB 原始来源：

- `/static_camera/image_raw`

新版真正的 RGB 原始来源：

- `/static_camera/color/image_raw`

但为了不打断旧链路，这一版又额外提供：

- `/static_camera/image_raw`

所以你现在要这样理解：

- `/static_camera/color/image_raw` 是**官方原生彩色流**
- `/static_camera/image_raw` 是**兼容转发出来的旧接口**

depth 这边则直接继续走：

- `/static_camera/depth/image_raw`
- `/static_camera/depth/camera_info`

---

## 6.3 frame_id 和 TF 体系变了

旧版更偏你自己的命名：

- `cam_overhead`
- `cam_overhead_depth`

新版官方驱动更偏设备自己的命名：

- `static_camera_color_optical_frame`
- `static_camera_depth_optical_frame`

现在是两套都存在，但你要注意：

- 上层检测节点已经开始接触到官方 optical frame
- 所以 RGB/depth 配准的误差来源，不再只是你原来那套 `cam_overhead -> cam_overhead_depth` 参数

---

## 6.4 桌面高度链恢复了

这是这次升级最重要的正向收益之一。

旧版深度起不来时，经常出现：

- `/vision/table/top_z` 没数据
- 抓取节点退回静态 `tabletop_z_m=0.0`

新版实测已经恢复：

- `/vision/table/top_z` 可以发布
- `table_plane_estimator` 真正拿到了 depth
- `so101_visual_grasp` 可以读取实时桌面高度

这会直接影响：

- grasp z 夹取高度
- 安全 clearance
- 桌面 collision box

---

## 6.5 红球检测的退化模式变了

旧版深度彻底坏掉时，常见退化是：

- 没有真实 depth
- 直接走平面回退或静态桌面高度

新版虽然 depth 已经恢复，但当前仍可能出现另一种退化：

- `Projective RGB-D registration residual too large`
- 然后 detector 回退到 approximate mapping

这两种退化不是一回事。

旧版退化是：

- **深度链没起来**

新版当前残留退化是：

- **深度链起来了，但 RGB/depth 配准还不够准**

---

## 7. 这一版已经解决了什么

当前这版已经确认解决的点：

1. `openni2_camera_node no devices found`
2. Astra Pro Plus 在当前机器上无法作为深度相机正常工作
3. `gscam` 和 depth 分链带来的启动不稳定
4. `/vision/table/top_z` 长期缺失
5. 旧版深度完全失效时抓取只能依赖静态桌面高度

换句话说，**“相机起不来”这个级别的问题，已经基本被跨过去了。**

---

## 8. 这一版还没完全解决什么

当前还没完全收口的问题主要有 3 个。

### 8.1 RGB/depth 配准还不够准

当前抓取实测已经能完整执行流程，但末端到目标仍可能有：

- 大约 `4 cm` 到 `7 cm` 的偏差

而日志里出现过：

- `Projective RGB-D registration residual too large: 20.73px`

这说明现在的主问题已经不是“没深度”，而是：

- RGB/depth 外参
- optical frame 对齐方式
- 或检测节点当前使用的投影关系

还需要继续压。

### 8.2 兼容层虽然保住了旧接口，但不是零成本

你现在还能继续用：

- `/static_camera/image_raw`
- `/static_camera/camera_info`

这是因为有兼容转发节点。

这很好，但也意味着：

- 你现在实际上处在“新底层 + 旧上层接口”的混合态
- 后面如果继续做视觉精度优化，最好逐步认清楚哪些节点吃的是原生 Orbbec 话题，哪些吃的是兼容话题

### 8.3 旧的抓取调参经验不能原样照搬

因为视觉链底层已经换了，所以旧版里一些经验要重新理解，例如：

- 某些 `x/y` 偏移补偿是否还成立
- 旧版 `depth_to_rgb_*` 手调量是否还适用
- 旧版你观察到的“球心误差模式”是不是还是同一种误差

也就是说，这一版不是“只修好了深度，其他全不变”，而是**视觉几何基础已经变了一层**。

---

## 9. 这一版启动后，你应该看到什么现象

如果当前跑的是新版视觉链，典型表现应该是：

1. 不再出现 `openni2_camera_node no devices found`
2. `follower_hx35hm_moveit.launch.py` 里会起：
   - `component_container`
   - `camera_topic_compat`
3. `/static_camera/color/image_raw` 存在
4. `/static_camera/image_raw` 也存在
5. `/vision/table/top_z` 能发布
6. `/vision/red_block/pose_base` 能发布

如果你看到这些，就说明你现在跑的是新版视觉链，而不是旧版。

---

## 10. 新版和旧版的对照表

| 维度 | 旧版 | 新版 |
|---|---|---|
| RGB 来源 | `gscam` | `orbbec_camera` |
| depth 来源 | `so101_openni2_camera` | `orbbec_camera` |
| Astra 支持 | 不稳定，甚至起不来 | 已实测可起 |
| `/static_camera/image_raw` | 原生 RGB | 兼容转发 RGB |
| `/static_camera/color/image_raw` | 通常没有 | 官方原生彩色流 |
| `/vision/table/top_z` | 经常缺失 | 已恢复 |
| 主要退化模式 | 深度设备起不来 | RGB/depth 配准残差偏大 |
| 当前主要问题 | 驱动链不成立 | 配准精度还要继续压 |

---

## 11. 对你后续调试的影响

这份升级对你后面调试的影响，可以总结成一句话：

**以后如果抓取偏了，先不要再第一反应怀疑“深度相机是不是又没起来了”，而应该先检查“RGB/depth 配准是不是还在 fallback”。**

建议你以后按这个顺序判断：

1. 看 `/vision/table/top_z` 有没有
2. 看 `/vision/red_block/pose_base` 是否连续稳定
3. 看日志里有没有：
   - `registration residual too large`
4. 再决定是调：
   - 外参
   - TF
   - detector 参数
   - 还是抓取 offset

---

## 12. 当前版本最准确的定位

如果要给这一版起一个很准确的名字，我会这样描述：

**这是一版“深度驱动链已经修通、RGB 也切到官方驱动、上层接口仍保持兼容、但 RGBD 几何精度还需要继续收口”的过渡稳定版。**

它不是旧版。

它也不是最终完全收口版。

但它已经把最大的系统性问题，也就是：

- Astra 深度驱动失效
- 桌面高度链缺失
- OpenNI2 方案不可靠

先解决掉了。

---

## 13. 你现在最应该记住的三件事

1. 这一版视觉链的底层已经换成 Orbbec 官方驱动，不再是原来的 OpenNI2 方案。
2. 旧接口 `/static_camera/image_raw` 还在，但它现在是兼容层，不是旧 `gscam` 原生输出。
3. 当前剩余主问题不是“深度没有”，而是“RGB/depth 配准还要继续压精度”。

