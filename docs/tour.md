# 新人导览：这个仓库里每个文件是干什么的

> 写给第一次进来的人。读完你应该能回答三个问题：
> ① 我想改 X，该动哪个文件？② 这个文件为什么被写成这样？③ 我改完怎么知道没搞坏？
>
> 配套阅读：`docs/architecture.md`（分层与契约）、`docs/status.md`（踩过的坑）。
> 本文只讲「是什么、在哪、为什么」，不讲怎么搭环境 —— 那是 `docs/setup.md`。

---

## 0. 一句话理解这个项目

让一台 Unitree Go2 在仿真里完成 **走 → 看 → 抓**：

```
Go2 四足（走）  →  狗载相机+雷达（看）  →  RM75 机械臂（抓）
```

这三件事各自独立，但共享同一台机器人模型。所以仓库里不是"一个程序"，而是**三组能力、两种运行形态**：

| 运行形态 | 入口 | 用途 |
|---|---|---|
| **整车**（狗 + 臂 + 相机） | `launch/gazebo.launch.py` | 演示"狗走过去、低头抓"，也用于真机移植前的验证 |
| **独立臂台**（一张桌子 + 机械臂） | `launch/arm_bench.launch.py` | 日常调抓取。启动快、基座不动、相机固定，改一行看一眼不用等整车 |

**为什么要有第二种？** 因为整车的抓取会被步态干扰：狗在喘气、底盘在晃、相机跟着抖。调抓取算法时，你希望 "失败" 只可能来自算法，不来自狗站不稳。`arm_bench` 把狗摘掉，只留臂 + 一张绿台面 + 一个蓝方块，启动从 40 秒降到 20 秒，日志也干净。这就是它存在的唯一理由。

---

## 1. 目录地图（为什么长这样）

```
go2_dev/
├── ros2_ws/src/          三个 ROS 2 包（真正的代码）
│   ├── go2_description/  机器人模型 + 仿真 + 机械臂 + 抓取  ← 主战场
│   ├── go2_vision/       狗载视觉（相机 + 雷达融合、伺服跟随）
│   └── vision_sim/       独立的视觉实验台（最早期的东西）
├── scripts/              给人用的 shell 脚本（启动/清理/检查）
├── docs/                 文档
└── third_party/          gitignore，只在 Ubuntu 那台机器上有（RM75 的 URDF/mesh、Unitree SDK）
```

### 为什么是三个包，不是一个？

ROS 2 里 **包的边界 ≈ 消息类型的边界**。`go2_vision` 定义了自己的 `TargetDetection.msg`，别的东西想用这个类型，就必须 `depend` 这个包。如果全塞一个包，任何人想读一个检测结果就得把 URDF、Gazebo、MoveIt 全部依赖装上。

所以拆包的真实标准不是"功能大不大"，而是：**"别人要复用我，最少需要装什么？"**

- `vision_sim` 和 `go2_vision` 各有一份 `TargetDetection.msg`（内容一样）—— 这不是手滑，是刻意的隔离：视觉实验台不该逼你装狗的东西。代价是两份定义要手动同步，这就是一笔债（`architecture.md` §4 记着）。

---

## 2. 按"我想干什么"找文件

| 你想做的事 | 改哪里 | 为什么在那里 |
|---|---|---|
| 加一个新物体（比如绿方块） | `scripts/arm_bench/objects.py` + `worlds/arm_bench.sdf` + `launch/arm_bench.launch.py` 的 odometry 桥 | 尺寸表、仿真里的模型、位置数据的转发，三件事三个地方 |
| 改颜色阈值（识别不准） | `scripts/arm_bench_object_detector.py` 的 `HSV_RANGES` | 检测是独立进程，改完重启即可，不用碰臂 |
| 改抓取点算错了 | `scripts/arm_bench/geometry.py` | 纯数学，能离线跑（见 §5） |
| 改机械臂运动参数（速度/尝试次数） | `scripts/arm_bench/skills.py` | 唯一和 MoveIt 打交道的地方 |
| 改"抓取整体顺序" | `scripts/arm_bench/grasp.py` 的 `run()` | 只有它是流程编排 |
| 改中文说法（"搁到"/"摆到"） | `scripts/arm_bench/language.py` | 纯字符串解析，和 ROS 无关 |
| 改相机内参/外参/话题名 | `launch/arm_bench.launch.py` | 所有接线都在 launch 里 |
| 改桌子和物体的位置 | `worlds/arm_bench.sdf` | 仿真世界的真值在这里 |
| 改狗怎么走 | `scripts/go2_trot.py` | 步态生成器 |
| 加一个会发布的新节点 | 新建 `.py` + **改 `CMakeLists.txt`** | 见下面 §3.5，这一步最容易忘 |

---

## 3. 五个"为什么这么写"

### 3.1 为什么 `urdf/` 和 `xacro/` 同时存在？

- `urdf/go2_description.urdf`（760 行）：**Unitree 官方导出的模型**，mesh 引用完整、关节名是官方命名。直接用。
- `xacro/*.xacro`（约 900 行）：**自己写的参数化版本**（`leg.xacro` 272 行描述一条腿，`robot.xacro` 拼四条）。好处是可以按参数生成不同腿长/质量，也方便加 Gazebo 插件；坏处是和官方 URDF 有两份真值，容易飘。

现状是两者都用：显示和 Gazebo 各走一条。这是历史遗留，不是设计。**新东西不要往 xacro 里加，除非你要改模型本身。**

### 3.2 为什么 `go2_vision` 和 `vision_sim` 长得这么像？

因为 `vision_sim` 先有，`go2_vision` 是把它的逻辑搬到狗上、再加点云融合和 TF 得到的。搬的时候图快，直接复制。

三个同名脚本的实际差值：

| 文件 | vision_sim | go2_vision | 差别 |
|---|---|---|---|
| `target_localizer.py` | 231 行 | 369 行 | 狗载版多了雷达融合和地面平面 |
| `eval_monitor.py` | 111 | 138 | 指标口径不同 |
| `target_director.py` | 141 | 136 | 场景不同 |

**现在别急着合并**：一个是实验台、一个是产品线，合并前得先确认两边的真值源能对齐。这条结论写在 `architecture.md` §9。

### 3.3 为什么所有代码都堆在 `scripts/`，不建节点包？

因为这是"先能跑"的项目。ROS 2 的 `ament_python` 包需要 `setup.py` + `entry_points` + 每次改名字重新 build；而 `scripts/` 里放 `xxx.py`，`ros2 run 包名 xxx.py` 直接跑。

代价有两个，都要付：
1. **安装靠白名单**（见 3.5）。
2. **目录不是模块**，想 import 就得自己折腾 `sys.path` —— 这就是为什么 `arm_bench_visual_grasp.py` 顶部有一行 `sys.path.insert`。

### 3.4 为什么用 `--symlink-install`？

`colcon build --symlink-install` 让 `install/` 里放的是**指向源码的软链**，不是拷贝。于是改 `.py`、改 launch，**不用重新 build**，直接再跑就生效。

什么时候仍然要 build：改 `CMakeLists.txt`、`package.xml`、加了新文件（因为安装清单变了）。

> 这条以前 `docs/status.md` 写错过了（说改 launch 也要 build），已修。

### 3.5 为什么 `CMakeLists.txt` 要一个个列文件名？

```cmake
install(PROGRAMS
  scripts/go2_trot.py
  scripts/arm_bench_visual_grasp.py
  ...（共 11 个）
  DESTINATION lib/${PROJECT_NAME})
```

因为这是 `ament_cmake` 包，`install(PROGRAMS ...)` 是显式清单。**新加一个要 `ros2 run` 的脚本，必须往这里加一行**，否则报 `No executable found`。

反过来也有好处：`test_arm_bench_pure.py`、`check_layers.sh` 这类**不该被当节点跑**的工具，天然不会进安装树。

包目录（`scripts/arm_bench/`）另用一条：

```cmake
install(DIRECTORY scripts/arm_bench DESTINATION lib/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
```

---

## 4. 一次"把蓝方块放到台面上"的数据流

按时间顺序，每一步给话题名和文件。**看懂这条线，就看懂了整个仓库的 80%。**

```
① 仿真出图
   Gazebo 相机 sensor → ros_gz_bridge → /arm/camera/image
                                        /arm/camera/depth_image
                                        /arm/camera/camera_info
   （桥接配置在 launch/arm_bench.launch.py）

② 检测：图 → 框
   arm_bench_object_detector.py  订阅上面三路，发 /arm/vision/detections
   （vision_msgs/Detection2DArray，bbox 里不带深度）

③ 定位：框 + 深度 → 三维点
   arm_bench/vision.py  deproject() + TF(base_link ← camera_depth_optical_frame)
   输出 base=(x, y, z)  —— 这是整条链路唯一"相信"的位置

④ 世界模型：登记障碍物
   arm_bench/world.py  → /apply_planning_scene（CollisionObject，is_diff=true）
   台面（必要时再加上放置目标），让 MoveIt 知道"这些不能穿过去"

⑤ 技能：规划 + 执行
   arm_bench/skills.py
     /move_action                        （MoveGroup，规划到目标位姿）
     /compute_ik、/compute_cartesian_path（直线下探时用）
     /rm_group_controller/follow_joint_trajectory
     /gripper_controller/follow_joint_trajectory
   关节反馈走 /joint_states

⑥ 抓取物吸附
   /blue_cube/attach 、/blue_cube/detach（ign topic，不是 ROS 话题）
   DetachableJoint 插件在 launch 里注入臂的 URDF，parent 必须是 Link7

⑦ 复检
   放下后再走一次 ②③，看方块是不是真的落在目标点 2.5cm / 3.5cm 内
```

**TF 树**（为什么坐标系这么多）：

```
base_link ── Link1..Link7 ── camera_depth_optical_frame
    │                              └─ 相机在这里，点云和图像都用这个 frame
    └─ world（只差一个 z 偏移 0.6，见 architecture.md §2.3）
```

> 那个 0.6 是踩过的坑：standalone URDF 里已经有 `world→base_link`，再发一次 0.6 会造成 TF 冲突。现在用 `world_offset` 参数手动换算，不再发 TF。

---

## 5. 第一次上手，跑这三条

```bash
# ① 纯函数回归：不需要 ROS、不需要 Gazebo，1 秒出结果
python3 ros2_ws/src/go2_description/scripts/test_arm_bench_pure.py

# ② 分层检查：三条铁律，违反就退出码 1
./scripts/check_layers.sh

# ③ 起独立臂台（Ubuntu 上）
source scripts/setup_env.sh
./scripts/start_arm_bench.sh --interactive
```

**为什么强调前两条能离线跑？** 因为"能离线验证"是被刻意设计出来的：`geometry.py`、`scene.py`、`objects.py`、`language.py` 全部只用标准库或纯 numpy，不 import ROS。你在 Mac 上就能验算法，不用等 Ubuntu 那台机器。

---

## 6. 文件速查表

### `ros2_ws/src/go2_description/` —— 模型、仿真、臂

| 文件 | 行数 | 干什么 |
|---|---|---|
| `CMakeLists.txt` | 25 | 安装规则。新节点要加进 `PROGRAMS` |
| `package.xml` | — | 依赖声明（`moveit_msgs`、`ros_gz_*`、`controller_manager` 等） |
| `urdf/go2_description.urdf` | 760 | 官方 Go2 模型（mesh + 关节） |
| `xacro/robot.xacro` | 135 | 参数化模型入口，拼四条腿 |
| `xacro/leg.xacro` | 272 | 一条腿（hip/thigh/calf 三段） |
| `xacro/gazebo.xacro` | 257 | Gazebo 插件与传感器标签 |
| `xacro/const.xacro` | 159 | 尺寸、质量、限位常量 |
| `xacro/transmission.xacro` | 42 | 关节传动（ros2_control 需要） |
| `dae/*.dae` | — | 腿部外观 mesh（7 个） |
| `worlds/go2_sim.sdf` | 167 | 整车仿真世界（地面 + 一个球 `grasp_ball`） |
| `worlds/arm_bench.sdf` | 275 | 独立臂台世界（6 个 model：地面、0.9x0.8m 桌、4 个可抓物体）。**物体坐标的真值在这** |
| `launch/gazebo.launch.py` | 456 | 整车：模型 + `ros2_control` + 相机 + 桥接 + 可选臂 |
| `launch/arm_bench.launch.py` | 430 | 独立臂台：臂 + 桌 + 相机 + 抓手插件 + 4 路 odometry 桥 |
| `launch/arm_moveit.launch.py` | 121 | `move_group`（`use_sim_time:=true`，忘了会规划失败） |
| `launch/display.launch.py` | 52 | 只在 RViz 里看狗，不起物理 |
| `launch/display_arm.launch.py` | 141 | 只在 RViz 里看臂（需要 `third_party` 的 RM75 真实 URDF） |
| `config/go2_controllers.yaml` | — | 只有狗时的控制器 |
| `config/go2_arm_controllers.yaml` | 66 | 狗 + 臂 |
| `config/arm_controllers.yaml` | 47 | 独立臂台 |
| `config/robot_control.yaml` | 70 | ROS 1 风格旧配置，历史遗留，现在不用 |
| `config/joint_names_go2_description.yaml` | — | 关节名清单（供 MoveIt/控制器对齐） |
| `rviz/*.rviz` | — | 四套 RViz 布局（狗 / 传感器 / 臂 / 检查关节） |

### `scripts/`（包内）—— 可执行节点

| 文件 | 行数 | 干什么 |
|---|---|---|
| `go2_trot.py` | 181 | trot 步态：订阅 `/cmd_vel`，发关节目标 |
| `go2_teleop.py` | 80 | 键盘控制狗移动 |
| `build_arm_model.py` | 283 | 把 RM75 URDF + 简化夹爪 + 相机拼成一台臂（被抓取物要用） |
| `arm_bench_visual_grasp.py` | 36 | **薄壳**：解析参数 → 交给 `arm_bench.grasp` |
| `arm_bench_language.py` | 137 | **薄壳**：中文 REPL → 调 `arm_bench.language` + 起子进程 |
| `arm_bench_planning_scene.py` | 128 | 把桌子和物体告诉 MoveIt（订阅 odometry 拿真位置） |
| `arm_bench_object_detector.py` | 184 | HSV 检测 → `Detection2DArray` + 标注图 |
| `arm_bench_home.py` | 67 | 开机把臂摆到 forward 姿态、张开夹爪 |
| `arm_bench_teleop.py` | 206 | 键盘摇臂（选关节 / 调步长 / 开关爪） |
| `grasp_ball.py` | 277 | 早期抓球 demo（固定球位），现在是回归用例 |
| `chase_grasp.py` | 466 | 追运动中的球并拦截抓取（含翻倒看门狗） |
| `test_arm_bench_pure.py` | 133 | **纯函数回归**，不需要 ROS |

### `scripts/arm_bench/` —— 分层后的包（11 个文件；整个臂台脚本约 3.3k 行）

| 文件 | 行数 | 层 | 干什么 |
|---|---|---|---|
| `constants.py` | 18 | — | 关节名、帧名、朝向四元数 |
| `objects.py` | 78 | — | **唯一一份物体尺寸表**（加物体改这里） |
| `geometry.py` | 100 | — | 纯数学：反投影、抓取位姿、放置点、旁侧空位 |
| `detection.py` | 82 | L2 | 纯函数：彩色图 + 深度 + 框 → 三维点 |
| `scene.py` | 64 | L3 | 纯 stdlib：多次观测融合成一个位置 |
| `world.py` | 102 | L3 | planning scene 的唯一出口（`moveit_msgs` 只在这里） |
| `vision.py` | 211 | L2 | 相机订阅 + TF + 检测结果 → base_link 坐标 |
| `skills.py` | 326 | L4 | MoveIt 动作封装：`move_to` / `compute_ik` / `cartesian` / `grip_to` |
| `language.py` | 243 | L5 | 中文解析（纯 stdlib，能脱机跑） |
| `grasp.py` | 539 | 组合根 | `run()`：扫描 → 抓 → 抬 → 搬 → 放 → 复检 |

### `ros2_ws/src/go2_vision/` —— 狗载视觉

| 文件 | 行数 | 干什么 |
|---|---|---|
| `msg/TargetDetection.msg` | — | 检测结果（含 3D 点与置信度） |
| `msg/TargetObservation.msg` | — | 融合后的观测（含世界坐标） |
| `scripts/target_localizer.py` | 369 | **核心**：检测框 + 点云 + 地面平面 → 目标三维位置（rms 2.6cm） |
| `scripts/object_3d_localizer.py` | 149 | 轻量版：只用深度反投影（不碰点云） |
| `scripts/yolo_detector.py` | 159 | YOLO 检测器，替代 HSV（输出同一种 msg） |
| `scripts/ball_detector.py` | 81 | HSV 红球检测（最小可用版） |
| `scripts/calib_check.py` | 275 | 标定自检：点云投到图像、拟合地面、反算球心 |
| `scripts/eval_monitor.py` | 138 | 评估：融合定位 vs 真值对账 |
| `scripts/target_director.py` | 136 | 让目标按脚本轨迹动（static/line/sine/circle），同时发真值 |
| `scripts/visual_servo.py` | 180 | 视觉伺服：`/cmd_vel` 闭环跟目标 |
| `scripts/stand_keeper.py` | 37 | 持续发零速度让狗站稳（视觉测试用） |
| `launch/vision_pipeline.launch.py` | — | 视觉主链路编排 |
| `launch/yolo_d435i.launch.py` | — | YOLO + D435i |
| `launch/visual_servo.launch.py` | — | 伺服跟随 |

### `ros2_ws/src/vision_sim/` —— 独立视觉实验台

最早的东西。一套完整的"相机 + 目标 + 检测 + 评估"闭环，不依赖狗。现在主要用来对照算法，和 `go2_vision` 有重复（见 3.2）。

`target_detector.py`（145）造检测、`target_localizer.py`（231）定位、`target_director.py`（141）驱动目标、`eval_monitor.py`（111）评估、`worlds/vision_bench.sdf`（107）是它的场景。

### 根目录 `scripts/` —— 给人用的

| 文件 | 干什么 |
|---|---|
| `setup_env.sh` | `source` 它来设置环境（ROS + 本工作空间） |
| `start_arm_bench.sh` | 一键起独立臂台；`--interactive` 只起 Gazebo+MoveIt+规划场景 |
| `kill_sim.sh` | 清理仿真进程。**它是全局 pkill，会连别人的仿真一起杀** |
| `check_layers.sh` | 三条铁律检查（见 `architecture.md` §6） |

### `docs/`

| 文件 | 行数 | 什么时候看 |
|---|---|---|
| `setup.md` | 246 | 第一次搭环境 |
| `running.md` | 480 | 查"这条命令怎么跑" |
| `status.md` | 363 | **改东西之前**先扫一遍坑列表 |
| `architecture.md` | 332 | 想知道"为什么这么分层、话题契约是什么" |
| `versions.md` | 50 | 版本号对不上时 |
| `dev_process_2026-10-30.md` | 550 | 某次功能开发的完整过程记录 |
| `tour.md` | 本文 | 认路 |
| `architecture.html` | — | 上面那份的图解版，浏览器打开 |

---

## 7. 三句话记住这个仓库

1. **三种能力（走/看/抓）× 两种形态（整车/独立臂）**，所有文件都能归到这张表里。
2. **接线在 launch，真值在 world.sdf，算法在 scripts**。找 bug 时先问"这是接线问题、场景问题，还是算法问题"。
3. **能离线验证的才是能长期活下来的**。`test_arm_bench_pure.py` 和 `check_layers.sh` 是这仓库里最便宜的两个保险。
