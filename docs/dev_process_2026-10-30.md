# 从多物体抓取到“纯视觉 + 自然语言”：arm_bench 开发过程记录

> 日期：2026-10-30  
> 起点提交：`335ea74 feat: 独立机械臂桌面仿真与简单视觉抓取`  
> 本次主要提交：`8fe32ac feat: 装配模型 MoveIt + 纯视觉扫描抓放 + 自然语言/一键启动`  
> 目标读者：第一次接手 `go2_dev` 的新同学，或想复现这条链路的人。

这篇文档记录的不是“最终答案”，而是这次开发**怎么一步步走过来的**：
需求怎么拆、每个阶段先改什么、遇到问题怎么定位、哪些方案被推翻、最后怎么验证。
代码细节以仓库当前状态为准；本文重点放在过程和方法。

---

## 0. 先看结果

一条命令启动（详见第 6 节）：

```bash
cd ~/go2_dev
scripts/start_arm_bench.sh
```

它会启动：

- Gazebo 机械臂桌面仿真（RM75 + D435i + AG95 夹爪）
- 机械臂 RViz + 相机窗口
- MoveIt2（装配模型，相机/夹爪参与碰撞检查）
- 多物体检测节点
- 中文自然语言终端

终端里输入：

```text
指令> 把红球放到黄盒子旁边
解析结果:
1. 抓取 红球(red_ball) → 放到 黄盒子(yellow_box) 旁边(3cm 空位)
确认执行? [y/N] y
...
```

已验证跑通的能力：

- 桌面 4 个物体：红球、蓝方块、绿圆柱、黄盒子
- 纯视觉扫描定位（位置不读物体 odometry）
- 抓取 + 放到上方（on_top）/ 旁边（beside）
- 多步指令、查询、确认、`结束了` 退出
- MoveIt 使用装配模型，规划时能看到相机/夹爪
- 一键启动 / 关闭

当前仍已知的问题（见第 8 节）：

- 扫描目前无条件走完 6 个观察位，开始抓取前会有较多“环顾”动作
- 放置阶段仍偶发相机与机械臂本体的物理碰撞
- 红球是球体，抓取时偶尔夹持不对称，叠放到平面上会滚
- 支撑物被上层物体遮挡时，其 x/y 仍会有 1–2cm 偏差

---

## 1. 起点：仓库里已经有什么

接手时先读了：

- `README.md`：从零复现、依赖、启动
- `docs/status.md`：跨会话交接、踩坑、待办（**新同学第一份要读的**）
- `docs/running.md`：所有启动命令
- `ros2_ws/src/go2_description/`：四足、机械臂、抓取脚本
- `ros2_ws/src/go2_vision/`：真机 D435i + YOLO 管线
- `ros2_ws/src/vision_sim/`：独立视觉验证环境

当时 `arm_bench` 的状态：

- RM75 固定在 0.6m 桌面，末端有 D435i 和简化 AG95
- `arm_bench_visual_grasp.py` 只抓一个红球：
  - HSV 检测 + 深度反投影
  - MoveIt 预抓取 + Cartesian 下探
  - **位置校验用物体 odometry**
- MoveIt 用的是 `~/rm_ws` 里标准 RM75 的配置，**不含相机/夹爪**
- RViz 里只有相机点云/图像，没有 Gazebo 的桌面世界模型

先跑通旧链路很重要：不要一上来就大改，先确认环境、启动顺序、基础抓取是好的。

```bash
cd ~/go2_dev
source scripts/setup_env.sh
ros2 launch go2_description arm_bench.launch.py rviz:=true view:=true
# 另一个终端
source ~/rm_ws/install/setup.bash
ros2 launch go2_description arm_moveit.launch.py
```

---

## 2. 需求拆解：把“感觉”变成可执行条目

最初的需求是：

> 在终端里说“去抓红球，放到蓝方块上”，机械臂去完成；自然语言要翻译成指令。

如果直接写代码，很容易做成“只能执行一句话”的脚本。先把问题拆成可确认的条目：

| 问题 | 选择 |
| --- | --- |
| 解析方式 | 规则/语法匹配（离线、可控），暂不接 LLM |
| 交互方式 | 持续 REPL，`结束了` 退出 |
| 执行前 | 显示解析结果并确认，默认不执行 |
| 范围 | 抓取、放到上方、放到旁边、查询物体、多步指令 |
| 旁边距离 | 边到边约 3cm，自动找空位 |
| 定位方式 | 纯视觉，不使用物体 odometry；类别尺寸/桌面高度可作先验 |
| 扫描策略 | 每次指令先扫描；找不到目标就环顾，仍找不到则停止 |
| MoveIt | 方案 A：装配 URDF 接入 MoveIt，相机/夹爪参与碰撞 |

这一步的产出是**格式约定**：

```text
指令> 把红球放到黄盒子旁边
解析结果:
1. 抓取 红球(red_ball) → 放到 黄盒子(yellow_box) 旁边(3cm 空位)
确认执行? [y/N]
```

以及内部 intent：

```json
{"action": "pick_place", "pick": "red_ball", "place": "yellow_box", "mode": "beside"}
```

多步就是 intent 列表；查询/帮助/退出是特殊命令。

> 经验：需求里的“自然语言”不是先写 NLP，而是先把**领域词表 + 句式 + 失败策略**定清楚。
> 这个项目的词表只有 4 个物体、抓/放两种动作，规则匹配比上大模型更稳、更容易调试。

---

## 3. 分阶段开发

### 3.1 先修 RViz 空白和坐标系

问题：启动 RViz 后画面空白，或者机械臂看起来“站在地上”。

原因：

- `arm_bench.rviz` 的 Fixed Frame 是 `world`，但当时 ROS 里没有稳定的 `world` TF
- Gazebo 的桌面顶面在世界 `z=0.6`，RViz 的 Grid 画在 Fixed Frame 的 `z=0`
- 把 Fixed Frame 改成 `base_link` 后，机械臂的底座平面就是 Grid 平面，
  所以看起来“在地上”；Gazebo 里它在桌面上

结论：

- RViz 的 Fixed Frame 用 `base_link`，先保证能看到机器人和相机
- 桌面/世界高度不要靠 RViz 的 Grid 判断；以 Gazebo 或点云为准
- 后来加入装配 URDF 时，standalone URDF 自带 `world -> base_link` 的
  identity，**不要再发一个 `world -> base_link` 的静态 TF**，否则会出现
  两个同名变换，查询/规划用的 world 坐标会随机变成错的

### 3.2 多物体场景 + 检测 + 抓取（第一版用 odom）

先做“能抓多个物体”，再上自然语言：

- `worlds/arm_bench.sdf` 加 `blue_cube`、`green_cylinder`、`yellow_box`
- 每个物体带 odometry（当时用于建场景和校验）
- 新增 `arm_bench_object_detector.py`：HSV 颜色 → 标签 + bbox + 深度
- `arm_bench_visual_grasp.py` 支持 `target_class`

这一版暴露了很多机械臂细节问题，都很值得新同学知道：

1. **简化夹爪闭口间隙太大**
   - 初始 q=0 时两指内距还有 6.8cm，7cm 的球只能勉强夹住
   - 修改 `build_arm_model.py`：手指基距改为 ±1.5cm，闭口内距约 1.8cm
2. **工具朝下的可达高度有限**
   - Link7 `z<0.19` 时 IK/跟踪明显变差，`z=0.161` 会卡在 4.7cm 外
   - 抓取点定在 `z≈0.20`；预抓取 `z≈0.30`
   - 不要盲目追求“更低”，先看 IK/状态有效性
3. **预抓取太矮会扫到球**
   - 球很轻，夹爪指尖擦一下就把球碰飞
   - 预抓取抬高，下探走垂直 Cartesian
4. **夹爪开太宽会碰旁边物体**
   - 从 0.09 调到 0.04；后来按类别给 open，球 0.05，其它 0.04
5. **闭合太狠会把物体挤飞**
   - 圆柱 close=0.028、黄盒 0.025，其它 0
6. **物体要分开放**
   - 球和邻物的 y 间距要大于张开手指的宽度，否则规划/下探会扫到

这一版的验收：

```text
red_ball        z 0.635 -> 0.807
blue_cube       z 0.635 -> 0.809
green_cylinder  z 0.650 -> 0.828
yellow_box      z 0.635 -> 0.816
```

但注意：这版**放置和校验用的是 odometry**。这是后面必须推翻的地方。

### 3.3 自然语言层

新增 `arm_bench_language.py`，只做三件事：

1. 规则解析
2. 确认/交互
3. 调用抓取子进程

不做的事：

- 不把 MoveIt/夹爪逻辑塞进解析器
- 不让解析器直接改关节

解析规则：

- 物体别名：红球/红色球/小球、蓝方块/蓝色方块、绿圆柱、黄盒子
- 动作词：抓/拿/取/夹
- 放置词：放到/放在/摆到/搁到
- 多步分隔：`然后 / 再 / 接着 / ；`
- 查询：`桌上有什么` → 触发扫描
- 退出：`结束了 / 退出 / q`
- 确认：`y/yes/是/确认/执行/可以`，其它一律取消

实现上用一个简单状态：

```text
parse -> validate -> describe -> confirm -> run_action(s)
```

每一步都有清晰的输出，失败就停止，不继续下一步。

> 经验：交互层的价值不只是“能说人话”，而是把**确认**和**失败停止**做进去。
> 机械臂误动作的代价比多按一次回车大得多。

### 3.4 纯视觉：把 odom 全部拿掉

用户追问：“你确定位置是摄像头来的吗？视野里没有物体怎么办？”

审计后的真实情况：

| 环节 | 原实现 |
| --- | --- |
| 抓取目标 | 相机 + 深度 + TF |
| 放置目标/旁边空位 | **odometry** |
| planning scene | **odometry** |
| 成功校验 | **odometry** |

这不是纯视觉。改成：

1. **扫描**：6 个观察位，默认位放最后；每个位置用检测器 + 深度 + TF
   计算 `base_link` 坐标；两轮仍找不到就报“不在视野”并停止
2. **数据源**：位置全部来自扫描；球用半径/桌面先验，方块/圆柱用顶面深度
3. **颜色 mask**：检测时在 bbox 内再按标签颜色过滤深度，避免支撑物 bbox
   里有上层物体时取错顶面
4. **放置**：目标中心来自扫描；旁放空位用扫描到的其它物体位置搜索
5. **校验**：放完再检测一次，检查物体是否在目标位置；不再读 odom
6. **查询**：`桌上有什么` 直接调用 `scan_only` 子进程，输出相机坐标

纯视觉后暴露的新问题：

- 球的位置误差 1–2cm，且会滚；扫描到抓取之间目标会动
- 相机在预抓取视角下对球的定位不如默认观察位，做“预抓取视觉伺服”
  反而更容易把目标修偏；先把扫描默认位放最后、抓取前不随意改目标
- 夹爪闭合不对称（例如 `[0.003, 0.049]`）时能夹住但余量小
- 球放平面会滚：给球的 velocity_decay 加到 2.0，仍建议方块/圆柱叠放

这一阶段的验收是多步：

```text
把红球放到黄盒子旁边,然后抓蓝方块放到黄盒子上
```

日志：

```text
PLACE SUCCESS red_ball -> yellow_box:
  visual base (0.239, 0.043, 0.020), err_xy=0.012

GRASP VERIFIED blue_cube:
  visual base=(0.225, -0.157, 0.150), tcp_dist=0.237
  fingers=[0.0255, 0.0265]

PLACE SUCCESS blue_cube -> yellow_box:
  visual base (0.223, 0.159, 0.105), expected z=0.105, err_xy=0.027
```

### 3.5 MoveIt 装配模型（A 方案）

用户观察到：运动过程中相机会和机械臂本体碰撞。

原因不是 Gazebo 错了，而是：

- MoveIt 用的是标准 RM75 配置，模型里**没有相机/夹爪**
- 抓取脚本的 Cartesian 还显式 `avoid_collisions=False`
- 相机装在 Link7 侧面，约 12cm，规划器看不到它

方案 A：

1. `arm_bench.launch.py` 生成装配 URDF（RM75 + tool0 + D435i + AG95），
   去掉 Gazebo 的 `world` link，写到 `/tmp/arm_bench_moveit.urdf`
2. `arm_moveit.launch.py` 读这份 URDF：
   - 以 `rm_75_config` 的 kinematics/limits/controllers 为基底
   - SRDF = 原 RM75 SRDF + 相机/夹爪固定连接的 `disable_collisions`
   - 覆盖 `robot_description` / `robot_description_semantic`
3. MoveIt 模型从 7 links 变成 28 links，相机/夹爪参与碰撞检查
4. 抓取脚本：
   - `compute_cartesian_path(avoid_collisions=True)`
   - 下探前只把**被抓目标**从 planning scene 移除
   - 其余相机/夹爪 vs 机械臂/桌子/其它物体仍然检查

排查时用了 MoveIt 的服务：

```bash
# IK 是否可达
ros2 service call /compute_ik ...
# 抓取位/预抓取位是否有自碰撞，返回 contacts
ros2 service call /check_state_validity ...
# Cartesian 能走多少
ros2 service call /compute_cartesian_path ...
```

诊断结果：

- 抓取位、预抓取位 valid，0 contacts
- Cartesian fraction 1.0
- 之前“JTC 成功但差 7.3cm”主要不是相机碰撞，而是：
  - 预抓取视觉修正后走了一条斜的下探路径，IK 分支/跟踪不稳
  - 释放后张爪太宽，提升时夹爪和旁边黄盒干涉
- 修正：垂直下探；释放按物体宽度只张开到 `q≈0.029`；提升失败回退 MoveIt

> 经验：**“规划路径无碰撞”不等于“实际运动无碰撞”**。
> 还要看 JTC 跟踪、被持物体、未建模的支架、以及 planning scene 是否过期。

### 3.6 一键启动

四个终端太麻烦，新增 `scripts/start_arm_bench.sh`：

- 默认：Gazebo GUI + 机械臂 RViz + 相机窗口 + MoveIt + 检测 + 语言前台
- `--headless`：无界面
- `--moveit-rviz`：用 MoveIt RViz 看 planning scene
- `--no-build`：跳过 colcon
- 日志：`/tmp/arm_bench_{sim,moveit,detector}.log`
- 输入 `结束了` 或 Ctrl+C 时清理后台进程

启动顺序在脚本里固定为：arm_bench → 等 URDF → MoveIt → 等 `/move_action`
和 `rm_group_controller` → 检测 → 语言。新同学不要再手动颠倒顺序。

---

## 4. 这次用到的调试方法

### 4.1 先看数据源，再看现象

每次怀疑“位置不对”时，先问：

- 这个位置来自相机、odom、还是硬编码？
- 这个 frame 是谁发布的？
- 有没有两个同名 TF？

```bash
ros2 topic echo /arm/vision/detections --once
ros2 run tf2_ros tf2_echo base_link Link7
ros2 run tf2_ros tf2_echo world base_link
ros2 topic info /tf_static -v
```

### 4.2 headless + 日志

```bash
ros2 launch go2_description arm_bench.launch.py use_gui:=false
ros2 launch go2_description arm_moveit.launch.py rviz:=false
ros2 run go2_description arm_bench_object_detector.py
ros2 run go2_description arm_bench_visual_grasp.py --ros-args -p target_class:=red_ball
```

日志重定向到 `/tmp`，用 `tail` 看关键行：`TARGET`、`GRASP`、`PLACE`、
`cartesian fraction`、`valid`、`contacts`。

### 4.3 用 MoveIt 服务做诊断

- `/compute_ik`：这个位姿到底有没有解
- `/check_state_validity`：这个状态和场景有没有碰撞，`contacts` 是哪对 link
- `/compute_cartesian_path`：直线路径能走多少，fraction 是多少
- `/apply_planning_scene`：确认场景里有什么、目标物是否已移除

### 4.4 验证语言 REPL

用 `script` 给交互程序一个 pty，再用管道输入：

```bash
(printf '把红球放到黄盒子旁边\n'; sleep 1; printf 'y\n'; sleep 90; printf '结束了\n') \
  | script -qec "ros2 run go2_description arm_bench_language.py" /dev/null
```

### 4.5 清理

```bash
scripts/kill_sim.sh --force   # Gazebo/RViz/MoveIt
```

坑：不要在**同一条命令**里同时写 `kill_sim.sh` 和 `pkill -f "ign gazebo"`
（或检测节点名）。`kill_sim.sh` 会按模式杀进程，你的命令行里如果包含该
模式，会把当前 shell 也杀掉。单独调用脚本，或用 `[i]gn` 这种拆词写法。

### 4.6 构建

```bash
cd ros2_ws
colcon build --packages-select go2_description --symlink-install
```

用了 `--symlink-install` 后，脚本/launch/rviz/world 的修改会直接生效；
但 package.xml/CMake 变更仍建议重新 build。

---

## 5. 踩坑清单（按症状）

| 症状 | 原因 | 处理 |
| --- | --- | --- |
| RViz 空白 | Fixed Frame `world` 没有 TF | 改 `base_link` |
| 机械臂在 RViz 里“站地上” | Grid 画在 base_link 的 z=0，桌面顶面就是这个平面 | 以 Gazebo/点云为准；必要时加世界 marker |
| RViz 物体布局和 Gazebo 不同 | RViz 不加载 SDF；看到的是相机点云/扫描 collision objects | 不要拿 RViz 当 Gazebo 世界 |
| planning scene 里蓝方块在黄盒上，Gazebo 在角落 | MoveIt scene 是上一次动作留下的，和 Gazebo 不同步 | 重启 A+B，或重新扫描 |
| MoveIt 报 `assembled URDF not found` | 先启动了 arm_moveit | 先 arm_bench，再 arm_moveit |
| 球抓不住/被碰飞 | 预抓取太低、open 太窄/太宽、夹爪闭口间隙大 | 抬高预抓取、垂直下探、按类别 open、改夹爪基距 |
| 夹爪闭合不对称 | 球的位置误差 1–2cm、球会滚 | 默认观察位优先、球加阻尼、后续做球心拟合 |
| Cartesian 成功但实际差很远 | 斜向下探/IK 分支/JTC 跟踪 | 垂直下探；看 fraction；检查 JTC 状态 |
| 释放后提升失败 | 张爪太宽，和旁边物体碰撞 | 按物体宽度只张开到刚好释放 |
| 支撑物 z 被误判 | bbox 里有上层物体，取到上层顶面深度 | 深度按标签颜色 mask 过滤 |
| 相机和本体碰撞 | MoveIt 模型/未建模支架/JTC 偏差/场景过期 | 装配模型 A；查 contacts；补支架/被持物碰撞；检查跟踪 |
| 扫描很慢、动作多 | 无条件走完 6 个观察位 | 待做 lazy 扫描：默认位优先，缺目标再环顾 |

---

## 6. 验证记录

### 6.1 多步纯视觉

命令：

```text
把红球放到黄盒子旁边,然后抓蓝方块放到黄盒子上
```

结果：

- 红球 → 黄盒旁边：`PLACE SUCCESS`，视觉校验 `(0.239, 0.043, 0.020)`
- 蓝方块 → 黄盒上方：`GRASP VERIFIED`，`PLACE SUCCESS`，
  视觉校验 `(0.223, 0.159, 0.105)`

### 6.2 扫描

扫描输出示例（某次）：

```text
VISIBLE blue_cube base=(0.259, -0.159, 0.035)
VISIBLE green_cylinder base=(0.150, 0.107, 0.050)
VISIBLE red_ball base=(0.221, 0.001, 0.035)
VISIBLE yellow_box base=(0.250, 0.156, 0.035)
```

和 Gazebo 真值相比，方块/圆柱/黄盒约 1cm，红球约 1–2cm。

### 6.3 MoveIt 诊断

- 抓取位、预抓取位：`valid=True`，`contacts=0`
- 垂直下探 Cartesian：`fraction=1.00`
- 放置释放后提升（按宽度张开）：`fraction=1.00`

### 6.4 一键启动

已用 `--headless` 测试：

```bash
printf '结束了\n' | scripts/start_arm_bench.sh --headless --no-build
```

启动、语言前台、退出清理都正常。

---

## 7. 当前分工/文件地图

| 文件 | 作用 |
| --- | --- |
| `scripts/start_arm_bench.sh` | 一键启动/清理 |
| `arm_bench_language.py` | 中文规则解析 + REPL |
| `arm_bench_object_detector.py` | 多物体 HSV 检测 + 标注图 |
| `arm_bench_visual_grasp.py` | 扫描定位、抓取、放置、视觉复检 |
| `arm_bench_teleop.py` | 键盘关节示教 |
| `build_arm_model.py` | 装配 URDF、夹爪几何 |
| `arm_moveit.launch.py` | 装配模型 MoveIt |
| `worlds/arm_bench.sdf` | 桌面 + 4 物体 |
| `docs/status.md` | 跨会话交接/踩坑 |
| `docs/dev_process_2026-10-30.md` | 本文 |

---

## 8. 已知限制和下一步

1. **lazy 扫描**（优先级最高）
   - 现在 6 个观察位全部执行，默认位通常已经看到全部物体
   - 改成：先到默认位；抓取/放置目标和场景物体都可见就停止；
     缺目标再环顾；两轮仍缺才报错
2. **相机物理碰撞**
   - 装配模型已接入，但物理上仍偶发；下一步用 `--moveit-rviz` 查
     `contacts`，补相机支架/被持物体的碰撞包络，检查 JTC 跟踪
3. **球体稳定性**
   - 夹持不对称、放置后会滚；后续做球心拟合/预抓取微调，
     或把球放到旁边/用方块叠放
4. **支撑物遮挡**
   - 黄盒被蓝方块压住时，x/y 会偏 1–2cm；后续按可见 mask 估计中心，
     或者规划放置空位而不是直接取支撑物中心
5. **测试**
   - 目前以人工多步 + 日志为主；后续可把语言解析、扫描、放置校验
     做成自动回归

---

## 9. 给新同学的建议

1. 先读 `docs/status.md`，再读本文；不要直接改 `arm_bench_visual_grasp.py`
2. 先跑一键启动，确认环境；再用 `--headless` + 日志做小步实验
3. 每次只改一件事，改完立刻验证：扫描坐标、抓取位 validity、Cartesian
   fraction、最终视觉校验
4. 遇到“位置不对”，先查数据源和 frame，不要先调参数
5. RViz 不是 Gazebo：点云/planning scene 和 Gazebo 世界不一致是正常的
6. 机械臂安全：确认前不要执行；失败要停止；不要用 odom 偷偷兜底
7. 提交前跑一遍 `git diff --check`，把过程写进 `docs/`

---

## 附录：常用命令

```bash
# 一键启动
cd ~/go2_dev && scripts/start_arm_bench.sh
scripts/start_arm_bench.sh --headless
scripts/start_arm_bench.sh --moveit-rviz

# 手动启动
cd ~/go2_dev && source scripts/setup_env.sh
ros2 launch go2_description arm_bench.launch.py rviz:=true view:=true
source ~/rm_ws/install/setup.bash
ros2 launch go2_description arm_moveit.launch.py rviz:=false
ros2 run go2_description arm_bench_object_detector.py
ros2 run go2_description arm_bench_language.py

# 扫描/抓取
ros2 run go2_description arm_bench_visual_grasp.py --ros-args -p scan_only:=true
ros2 run go2_description arm_bench_visual_grasp.py \
  --ros-args -p target_class:=red_ball -p place_class:=yellow_box -p place_mode:=beside

# 清理
scripts/kill_sim.sh --force
```
