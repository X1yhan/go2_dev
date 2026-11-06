# 架构说明

> 本文不是"理想架构"提案，而是**从现有代码里扒出来的真实结构**，加上一条低风险的改进路线。
> 可视化版本（带"为什么"的解释）：用浏览器打开 `docs/architecture.html`。
> 相关文档：`docs/status.md`（踩过的坑）、`docs/running.md`（怎么跑）、`docs/dev_process_2026-10-30.md`（最近一次开发过程）。

---

## 0. 一句话结论

**ROS 2 已经替我们分好了层**：节点 = 进程边界，msg/srv/action = 数据契约，launch = 组合方式。
所以在这个项目里，"架构"不是"要不要拆节点"，而是三个能直接检查的问题：

| 问题 | 检查方法 | 现状 |
|---|---|---|
| 每个节点职责是否单一？ | 一个节点能不能用一句话说清它做什么 | 大体是，但 `arm_bench_visual_grasp.py` 一个类扛了 6 件事 |
| 有没有隐藏的跨职责依赖？ | `import` 关系能不能跨层 | **有**：语言层 `import` 了抓取脚本，把 cv2/moveit 全拖进来 |
| 契约是否显式？ | 话题名/帧/单位是否写在文档里 | 现在只散落在 `declare_parameter` 默认值里 |

结论：**节点切得不错，类没切。** 需要做的是把"隐含的边界"变成"可 grep、写在文档里的边界"，而不是推倒重来。

---

## 1. 五层定义（每层都对应真实文件）

```
L5 任务层      arm_bench_language.py                    只调 L4，出人可读的日志
─────────────────────────────────────────────────────────────
L4 技能层      arm_bench_visual_grasp.py  grasp_ball.py  唯一可以碰 MoveIt 的层
               chase_grasp.py  arm_bench_home.py         出 成功/失败 + 原因
─────────────────────────────────────────────────────────────
L3 世界模型    arm_bench_planning_scene.py  (set_scene)   只发 /apply_planning_scene
─────────────────────────────────────────────────────────────
L2 感知层      arm_bench_object_detector.py               只出 2D 检测 + 标注图
               go2_vision/{target_localizer,ball_detector,yolo_detector}.py
─────────────────────────────────────────────────────────────
L1 硬件/仿真   arm_bench.launch.py  arm_moveit.launch.py  只出 joint_states / 相机 / TF
               gazebo · gz_ros2_control · robot_state_publisher · move_group
```

### L1 硬件 / 仿真层

| 项 | 内容 |
|---|---|
| 真实实现 | `ros2_ws/src/go2_description/launch/arm_bench.launch.py`、`launch/arm_moveit.launch.py` |
| 组成 | Gazebo(RM75+D435i+AG95) → `gz_ros2_control/GazeboSimSystem` → 3 个控制器(`joint_state_broadcaster`/`rm_group_controller`/`gripper_controller`)；`ros_gz_bridge` 桥接相机与 odometry；`move_group` |
| 对外只出 | `/joint_states`、`/clock`、`/arm/camera/{image,depth_image,camera_info,points}`、`/model/*/odometry`、TF |
| 只接 | `/rm_group_controller/joint_trajectory`、`/gripper_controller/joint_trajectory` |
| **禁止** | 任何感知或任务逻辑；任何"给它加个参数就顺便干活"的便利写法 |

> 这一层的价值是**可替换**：真机上换成 `ros2_control` 的真硬件插件 + RealSense 驱动，上面的四层一行不改。这是 ROS 存在的全部理由，也是本层唯一的 KPI。

### L2 感知层

| 项 | 内容 |
|---|---|
| 真实实现 | `go2_description/scripts/arm_bench_object_detector.py`（184 行）、`go2_vision/scripts/{target_localizer,ball_detector,yolo_detector}.py` |
| 允许依赖 | `cv2` `numpy` `cv_bridge` `tf2_ros` `vision_msgs` `sensor_msgs` |
| 对外只出 | `/arm/vision/detections`（`vision_msgs/Detection2DArray`）、`/arm/vision/image_annotated`（`sensor_msgs/Image`） |
| **禁止** | `moveit_msgs`、任何规划/执行调用 |

> 检测器输出的 `Detection2DArray` 是**故意和真机 YOLO 管线共用同一个接口**的，这样将来换学习型检测器只需改这层的内部，上层不动。

### L3 世界模型层

| 项 | 内容 |
|---|---|
| 真实实现 | `go2_description/scripts/arm_bench_planning_scene.py`（179 行，当前为 Ubuntu 机器上的未提交改动）；旧实现散落在 `arm_bench_visual_grasp.py` 的 `set_scene/remove_object/clear_scene` |
| 允许依赖 | `moveit_msgs`（`CollisionObject` / `ApplyPlanningScene`）、`nav_msgs`（物体 odometry） |
| 对外只出 | `/apply_planning_scene`（`is_diff=True` 增量下发） |
| **为什么单独一层** | 场景是**状态**，不是动作。它需要"物体动了就更新"，这和"抓取一次"的生命周期完全不同 |

### L4 技能层

| 项 | 内容 |
|---|---|
| 真实实现 | `arm_bench_visual_grasp.py`（1162 行）、`grasp_ball.py`（277 行）、`chase_grasp.py`（466 行）、`arm_bench_home.py` |
| 允许依赖 | L2 的检测结果、L3 的场景、MoveIt 的 action/srv（`/move_action`、`/compute_ik`、`/compute_cartesian_path`）、夹爪 action |
| 对外只出 | **成功/失败 + 原因串**（当前实际只出了退出码，见 §4） |
| 铁律 | **整个仓库只有这一层可以 import `moveit_msgs`** |

> 技能的定义：**一次可被复用、有明确成功判据的动作**。`scan`（看到桌上有什么）、`pick`（拿起某个 label）、`place`（放到某个 label 旁/上）、`home`（回零）。它们不该知道自己被谁调用。

### L5 任务层

| 项 | 内容 |
|---|---|
| 真实实现 | `arm_bench_language.py`（353 行）——中文 REPL，把"抓红球放到黄盒子上"解析成动作序列 |
| 允许依赖 | 只有 L4 的接口 + 标准库 |
| **禁止** | `moveit_msgs` / `geometry_msgs` / `cv2` / 任何 TF 计算 |

> 这一层是**唯一**该知道"用户说了什么"的地方。它换一个前端（RS 的界面、语音、HTTP）时，下面的四层一行不改。

---

## 2. 话题契约表（真实值，随时可核对）

### 2.1 arm_bench 链路（仿真抓放）

| 话题 | 类型 | 发布方 | 订阅方 | QoS / 备注 |
|---|---|---|---|---|
| `/arm/camera/image` | `sensor_msgs/Image` | gz_bridge ← Gazebo RGB-D | 检测器、抓取脚本、RS | `rgb8`，30 Hz |
| `/arm/camera/depth_image` | `sensor_msgs/Image` | 同上 | 检测器、抓取脚本、RS | `32FC1`，背景 `-inf` |
| `/arm/camera/camera_info` | `sensor_msgs/CameraInfo` | 同上 | 检测器、TF 反投影 | near clip 0.05 |
| `/arm/camera/points` | `sensor_msgs/PointCloud2` | 同上 | 可视化 / 真机融合验证 | |
| `/arm/vision/detections` | `vision_msgs/Detection2DArray` | `arm_bench_object_detector` | `arm_bench_visual_grasp` | 标签 = `red_ball`/`blue_cube`/`green_cylinder`/`yellow_box` |
| `/arm/vision/image_annotated` | `sensor_msgs/Image` | `arm_bench_object_detector` | `rqt_image_view` | |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` | `move_group`、teleop | |
| `/rm_group_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | `move_group`、teleop、`arm_bench_home` | JTC | **订阅端是 BEST_EFFORT**，发端必须匹配 |
| `/gripper_controller/joint_trajectory` | 同上 | 抓取脚本、teleop | JTC | 同上 |
| `/move_action` | `moveit_msgs/MoveGroup`(action) | — | 抓取/追踪脚本 | `use_sim_time` 必须与 move_group 一致 |
| `/compute_ik`、`/compute_cartesian_path` | MoveIt srv | — | L4 | 只允许 L4 调用 |
| `/apply_planning_scene` | `moveit_msgs/ApplyPlanningScene` | L3 | `move_group` | `is_diff=True` |
| `/model/<name>/odometry` | `nav_msgs/Odometry` | gz_bridge | L3、`grasp_ball`、`chase_grasp` | `<name>` ∈ `go2`/`grasp_ball`/`target_ball`/`blue_cube`/`green_cylinder`/`yellow_box` |

### 2.2 狗载视觉链路（go2_vision）

| 话题 | 类型 | 发布方 | 订阅方 |
|---|---|---|---|
| `/camera/<name>/image`、`/camera/<name>/depth_image` | `sensor_msgs/Image` | RealSense / 仿真 | `yolo_detector` |
| `/vision/image_annotated` | `sensor_msgs/Image` | `yolo_detector` | `rqt_image_view`、**RS** |
| `/vision/detections` | `vision_msgs/Detection2DArray`（或 `TargetDetection`） | `yolo_detector` / `ball_detector` | `object_3d_localizer`、`visual_servo` |
| `/vision/target`（`TargetObservation`） | 3D 目标 | `object_3d_localizer` / `target_localizer` | `visual_servo`、`eval_monitor` |
| `/cmd_vel` | `geometry_msgs/Twist` | `visual_servo`、`go2_trot`、teleop | 控制器 |

### 2.3 坐标系（TF）

```
world ──(static, 仅 standalone URDF 里 identity)── base_link
base_link ──(move_group 广播)── Link7 ──(static)── camera_depth_optical_frame
base_link ──(move_group 广播)── gripper_finger{1,2}_joint
```

- 仿真里 Gazebo 会给传感器帧加上 `arm_bench/Link7/arm_rgbd_camera` 的前缀，`arm_bench.launch.py` 用一条 `static_transform_publisher` 把它挂到 `camera_depth_optical_frame` 上。
- **坑（已踩）**：standalone URDF 已有 `world→base_link` 的 identity，再发一次 0.6m 会造成 TF 冲突，位置会随机跳成世界系。所以桌面高度用手动 `world_offset` 换算，不走 TF。

---

## 3. 外部契约：Robot Studio（RS）

RS 用 `configs/robots/go2_arm_rm75.yaml` 描述这台臂（**这是已实测的**）：

```
flange_frame: Link7      base_frame: base_link
joints: joint1..joint7（范围与 arm_bench 一致）
cameras: /arm/camera/image  /arm/camera/depth_image  /vision/image_annotated
```

也就是说 **RS 只依赖 3 个话题 + 2 个帧名 + 7 个关节名**。这是本仓库对外的全部契约，改动这 12 个东西任何一个都要先通知 RS。

**已知不一致（值得修）**：RS 期望的 `annotated_topic` 是 `/vision/image_annotated`，这是 **go2_vision 狗载 YOLO 管线**的输出（订阅 `/camera/<name>/image`）；而 arm_bench 自己的标注图发在 `/arm/vision/image_annotated`。所以在 `arm_bench` profile 下，RS 那一格永远是"无数据"——除非另外起 dog 的 YOLO 节点（那又是另一个相机、另一条管线）。RS 的配置注释里已经写了这件事，但**契约上应该二选一**：要么 arm_bench 的检测器也往 `/vision/image_annotated` 发一份，要么 RS 的 profile 指向 `/arm/vision/image_annotated`。

---

## 4. 现在的技术债（带行号证据）

### 4.1 一个类扛了六种职责

`ros2_ws/src/go2_description/scripts/arm_bench_visual_grasp.py`：**1162 行，`class VisualGrasp(Node)` 从第 111 行到第 1150 行（1040 行），39 个方法**。

| 职责 | 方法 |
|---|---|
| 图像 / 检测 | `on_color` `on_depth` `on_info` `on_detections` `detect_label` `_detect_all` `_fresh_frame` |
| 几何 | `_tcp` `_place_center` `_free_beside_spot` `_publish_marker` `_footprint_radius` |
| 世界模型 | `_collision_object` `set_scene` `remove_object` `clear_scene` |
| 规划 / 执行 | `move_to` `move_to_joint` `_compute_ik` `_joint_move` `cartesian` `_wait_tcp` |
| 夹爪 | `grip_to` `_detachable` |
| 任务编排 | `scan_scene` `_scan_pass` `place_object` `_capture_current` `run` |

代价不是"不优雅"，是三件具体的事：

1. 改一个 HSV 阈值要 `import` 12 个 `moveit_msgs` + 3 个 service；
2. **"给 image+depth 输出物体位姿"这句话没法单测**——需要一个活着的 `move_group`、一个活着的 Gazebo；
3. 想复用"扫描桌上有什么"，只能 `subprocess` 起一个新进程。

### 4.2 任务层硬 import 技能层（最严重的一条）

`ros2_ws/src/go2_description/scripts/arm_bench_language.py:21`

```python
from arm_bench_visual_grasp import OBJECTS
```

为了拿一张物体尺寸表（`OBJECTS`），语言层把整条依赖链拖了进来：`cv2`、`numpy`、`cv_bridge`、`tf2_ros`、`control_msgs`、`moveit_msgs`。
后果：**"语言解析"这个纯逻辑，永远无法脱离 ROS + MoveIt + OpenCV 运行，也无法单独测试。** 这是层级边界被破坏最典型、最便宜就能修的一处。

### 4.3 任务层用子进程调技能层

`arm_bench_language.py:271`

```python
proc = subprocess.Popen(cmd)
```

每个动作起一个 `ros2 run`，结果是：拿不到结构化结果（只有退出码）、拿不到失败原因（只能翻日志）、每个动作都要重新初始化一次节点（几秒钟）、Ctrl+C 的语义要靠信号转发。

### 4.4 同一职责存在两份实现

`vision_sim/scripts` 与 `go2_vision/scripts` 有三个同名不同实现的脚本：

| 脚本 | vision_sim | go2_vision |
|---|---|---|
| `eval_monitor.py` | 111 行 | 138 行 |
| `target_director.py` | 141 行 | 136 行 |
| `target_localizer.py` | 231 行 | 369 行 |

外加 `TargetDetection.msg` 有两份定义（内容一致），`TargetObservation.msg` 只在 `go2_vision`。
这不是风格问题：**改一处，另一处静默不一致**，而两边跑的是"同一个实验的两个阶段"。

### 4.5 成功/失败只出退出码

L4 目前靠 `return 1` + 日志行表达失败（`'visible objects: %s'`、`'place target ... not in the vision scan'`）。上层（RS、语言层）需要一个**可编程的原因**，否则"抓失败了"永远只能靠人看终端。

---

## 5. 已落地的形态：拆了文件，没拆节点

这一段是**现状**（`refactor/layering` 分支已实现），不再是设想：

```
ros2_ws/src/go2_description/scripts/           行数    层
├── arm_bench_visual_grasp.py                    36    L4   入口薄壳：docstring + path 引导 + main
├── arm_bench_language.py                       137    L5   入口薄壳：class Shell + main
├── arm_bench_planning_scene.py                 128    L3   节点：仿真物体位姿 -> CollisionObject
├── arm_bench_object_detector.py                184    L2   节点：HSV 检测（未改动）
├── arm_bench/
│   ├── __init__.py                              18     —    模块地图 + 依赖规则
│   ├── constants.py                             18    —    关节名 / 帧名 / 工具朝向
│   ├── objects.py                               78    —    唯一一份 OBJECTS 表 + 尺寸/半径函数
│   ├── geometry.py                             100    L2   纯数学：反投影 / 抓放位姿 / 释放量
│   ├── detection.py                             82    L2   纯函数：图 + 深度 -> (u, v, z)
│   ├── scene.py                                 64    L3   场景状态：多视角观测融合（纯 stdlib）
│   ├── vision.py                               211    L2   VisionFrontend：订阅 + 反投影 + TF
│   ├── skills.py                               326    L4   ArmSkills：唯一的 MoveIt / action 出口
│   ├── world.py                                102    L3   planning scene 对象与 diff 下发
│   ├── language.py                             243    L5   中文解析 + 动作协议（纯 stdlib）
│   └── grasp.py                                539    L4   组合根：VisualGrasp(Node)
└── （原 1162 行里 1040 行的类体现在分布在上面 8 个文件里）
```

入口文件名、节点名、话题名、参数名**一个都没改**：`ros2 run go2_description arm_bench_visual_grasp.py`
仍然是那条命令，`start_arm_bench.sh` 与 RS 不需要任何改动。

### 为什么这么切（每条都有具体理由）

| 拆分 | 理由 | 立刻得到什么 |
|---|---|---|
| `detection.py` 纯函数 | 现在的检测逻辑只依赖 numpy 数组，不依赖 ROS | 可以拿几张录好的图跑回归，不需要 Gazebo |
| `geometry.py` 纯函数 | 反投影 + TF 是纯数学，现在混在节点里 | "为什么 z 差 3.6cm"这类问题可以离线复现 |
| `world.py` | 场景是状态，生命周期与动作不同 | 物体被挪动时只需重发 diff |
| `skills.py` 返回 `(ok, reason)` | 上层要的是原因，不是日志 | 语言层/RS 能直接显示"失败：目标不在视野内" |
| `grasp.py` 只剩骨架 | 参数、订阅、编排 | 一个文件能读完，改阈值不再 import 全部 |

**关键：话题名、参数名、节点名一个都不改。** 这样 RS 零感知，`start_arm_bench.sh` 零改动，随时可以回退。

---

## 6. 三条铁律（可以 grep 检查）

```bash
# 铁律 1：L5 不许直接或间接依赖 MoveIt / OpenCV / geometry_msgs
grep -nE "^\s*(from|import)\s+(moveit_msgs|cv2|geometry_msgs)" \
     ros2_ws/src/go2_description/scripts/arm_bench_language.py
#   现在的结果：直接 import 是干净的，但第 21 行 from arm_bench_visual_grasp import OBJECTS
#   间接把上面这些全拉进来了 → 违反

# 铁律 2：L2 不许 import moveit_msgs
grep -rln "moveit_msgs" --include=*.py ros2_ws/src/go2_description/scripts/arm_bench_object_detector.py \
                            ros2_ws/src/go2_vision/scripts/
#   现在的结果：空 → 符合

# 铁律 3：moveit_msgs 只允许出现在 L3 与 L4
#   （2026-11-05 修订：原写法"只有 L4 可以 import moveit_msgs"自相矛盾——
#     L3 的 planning scene 本身就是 moveit_msgs。重构时显式改成下面这条。）
grep -rln "moveit_msgs" --include=*.py ros2_ws/src/
#   结果：只有 arm_bench/world.py(L3) 与 arm_bench/skills.py(L4)，
#   加上历史遗留的 grasp_ball.py / chase_grasp.py（行为树之前的旧脚本，未重构）
scripts/check_layers.sh     # 同一件事的自动版本，违反即以非零退出
```

把这三条贴进 CI 或者 pre-commit，成本 20 行，收益是"层不会被慢慢吃掉"。

---

## 7. 什么时候才该拆节点

> **判据三问：**(a) 需要独立频率吗？(b) 需要崩溃隔离吗？(c) 需要被别的进程单独起/单独调吗？
> **三个都是"否" → 不要拆。**

| 候选 | (a) 独立频率 | (b) 崩溃隔离 | (c) 被单独调用 | 结论 |
|---|---|---|---|---|
| 检测器 | 是（30 Hz 相机） | 是（cv2 会挂） | 是（RS 要看） | **节点**（已经是） |
| 场景发布 | 是（物体动了就发） | 否 | 是（`--interactive` 单独起） | **节点**（已经是） |
| scan | 否（一次性） | 否 | 是（`scan_only:=true`） | 暂不拆，用参数 |
| pick / place | 否（一次性动作） | 否 | 否 | **不拆** |
| 语言 | 否（交互） | 否 | 否 | **不拆** |

反例警示：把 `pick` 拆成 action server 会让"等它完成"变成一套额外编排（goal/feedback/cancel），而它本来就是"调一次、等结果"——只会增加失败点。

---

## 8. 迁移顺序（每一步都能跑、都能回退）

| 步 | 动作 | 风险 | 验收 |
|---|---|---|---|
| 0 | 把 Ubuntu 机器上那份未提交改动（`arm_bench_planning_scene.py` 等）先提交备份 | 无 | `git status` 干净 |
| 1 | 抽出 `detection.py` + `geometry.py` 纯函数 | 低 | `scan_only:=true` 输出与现在逐位一致 |
| 2 | `world.py`（先只在进程内用，不发话题） | 低 | 场景物体位置与 `arm_bench.sdf` 一致 |
| 3 | `skills.py` 返回 `(ok, reason)`，`run()` 改为调用它 | 中 | 抓红球→放黄盒子全流程通过 |
| 4 | `language.py` 去掉对 `arm_bench_visual_grasp` 的 import（`OBJECTS` 挪到 `detection.py` 或独立 `objects.py`） | 低 | 铁律 1 的 grep 为空 |
| 5 | 语言层 `subprocess` → 同进程调用 skills（或 action，等 L4 稳定后） | 中 | 失败时能打印结构化原因 |
| 6 | 决定 `vision_sim` / `go2_vision` 去重还是显式声明"两套并存" | 低 | 写进本文档 §4.4 |

**为什么不先做第 5 步**：语言层换成 action 之前，L4 得先有稳定的 `(ok, reason)` 接口，否则只是把"读退出码"换成"读 action result 里的退出码"。

---

## 9. 明确不做的事

| 不做 | 理由 |
|---|---|
| 拆纯转发节点 | 每个节点都是一次序列化 + 一次调度；转发节点只增加延迟和故障点 |
| 引入行为树（`py_trees`） | 现在是"一次抓取"的线性流程，行为树会把 353 行的 REPL 变成 800 行的树。等出现"多任务并发/抢占/恢复"需求再说 |
| 现在就统一 `vision_sim` 与 `go2_vision` | 一个是仿真验证环境、一个是狗载真机管线，合并的收益要等两边话题稳定后再评估。**先写清结论，别动代码** |
| 为分层而分层（每层一个 msg） | 层间传递用 dataclass 就够；只有跨进程才需要 msg |

---

## 10. 一句话给未来的自己

> 这个项目最大的风险不是"架构不够好"，而是**明明有一个能演示的系统，却因为重构把它弄坏了**。
> 所以：每次只动一块，动完必须还能跑 `start_arm_bench.sh` 抓一次红球。
