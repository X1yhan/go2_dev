# 启动指南

本文档汇总 go2_dev 的启动方式:四足仿真(go2_description)、视觉仿真(vision_sim),
以及命令行 / VSCode 图形化两种入口。所有命令都在仓库根目录 `~/go2_dev` 下执行。

## 0. 一次性准备:编译

```bash
cd ~/go2_dev/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

或在 VSCode 里按 `Ctrl+Shift+B`(默认构建任务)。

## 1. 每个终端先加载环境

```bash
cd ~/go2_dev
source scripts/setup_env.sh
```

脚本会 source ROS2 Humble 和 `ros2_ws/install/setup.bash`。
未编译时脚本会提示先执行 colcon build。

## 2. 四足仿真(go2_description)

### 先搞清楚:Gazebo 和 RViz 的关系

两者是**互相独立**的程序,通过 ROS 话题连接:

```
Gazebo(物理引擎 + 自带 3D 界面)          RViz(纯可视化,不做仿真)
  ├─ 算物理、渲染相机/雷达                  ├─ 订阅 /robot_description 画机器人模型
  ├─ 显示世界、机器人、相机小窗、雷达射线    ├─ 订阅 TF 画坐标系
  │                                        ├─ 订阅 /go2/camera/image 显示画面
  └──── ros_gz 桥接 ──→ ROS 话题 ─────────→└─ 订阅 /go2/lidar/points 显示点云
```

- **Gazebo** 是"世界本身":狗怎么走、相机拍什么、雷达扫什么,都是它算的。
- **RViz** 只是"看数据":它不仿真,关掉 Gazebo 后 RViz 就没有数据了。
- launch 参数里 `use_gui` 控制 Gazebo 窗口,`rviz` 控制 RViz 窗口,可任意组合。

### 推荐启动流程

**场景 1:仿真 + 看画面 + 遥控(最常用)**

```bash
# 终端 A:拉起 Gazebo(服务器+窗口)、机器人、传感器、桥接、RViz、步态节点
cd ~/go2_dev
source scripts/setup_env.sh
ros2 launch go2_description gazebo.launch.py sensors:=true walk:=true rviz:=true

# 终端 B:键盘遥控(必须单独一个终端,要接收键盘输入)
cd ~/go2_dev
source scripts/setup_env.sh
ros2 run go2_description go2_teleop.py
```

启动后应看到:Gazebo 窗口(狗 + 红箱/蓝柱 + 相机浮动小窗 + 雷达射线)、
RViz 窗口(机器人模型 / FrontCamera 图像 / LiDAR 点云)。
注意:`walk:=true` 后、终端 B 还没启动前,狗会按旧的开环模式自动往前走;
终端 B 一启动就会发零速度,狗停下等按键。

**场景 2:不要 Gazebo 窗口(省显卡),只留 RViz**

```bash
ros2 launch go2_description gazebo.launch.py sensors:=true walk:=true use_gui:=false rviz:=true
```

Gazebo 在后台算物理,画面全由 RViz 提供;遥控同上(另开终端)。

**场景 3:只看机器人和关节(不开 Gazebo)**

```bash
ros2 launch go2_description display.launch.py     # RViz + 关节滑块
```

### 命令与参数速查

| 命令 | 说明 |
| --- | --- |
| `ros2 launch go2_description gazebo.launch.py` | Gazebo 站姿(默认打开 GUI) |
| `ros2 launch go2_description gazebo.launch.py walk:=true` | Gazebo 对角小跑(开环步态) |
| `ros2 launch go2_description gazebo.launch.py sensors:=true` | 加装狗载前视相机 + 激光雷达 |
| `ros2 launch go2_description gazebo.launch.py sensors:=true rviz:=true` | 同上,并开 RViz2 看相机画面/点云 |
| `ros2 launch go2_description gazebo.launch.py use_gui:=false` | 无界面(仅 Gazebo 服务器) |
| `ros2 launch go2_description display.launch.py` | RViz2 + 关节滑块 |

launch 参数:

- `use_gui`(默认 `true`):是否打开 Gazebo GUI 客户端
- `world`(默认 `worlds/go2_sim.sdf`):要加载的 world(默认世界带地面 + 红箱/蓝柱参照物
  和 Sensors 系统;换回空世界用 `world:=empty.sdf` 则传感器不工作)
- `gz_world_name`(默认 `go2_sim`):world SDF 里的世界名,多 Gazebo 实例并存时必须对上
- `spawn_z`(默认 `0.45`):Go2 出生高度
- `walk`(默认 `false`):是否启动 `go2_trot.py` 开环步态节点
- `sensors`(默认 `false`):是否给狗加前视相机 + 激光雷达
- `rviz`(默认 `false`):是否启动 RViz2(用 `rviz/sensors.rviz`,依赖 `sensors:=true`)
- `lidar_x/lidar_y/lidar_z`(默认 `0.28945 / 0 / 0.10`):雷达安装位置(base 系)。
  官方 URDF 的 `radar` 点在头部网格内部(射线出不来),仿真默认放头顶

### 狗载传感器话题(`sensors:=true` 时)

| 话题 | 说明 |
| --- | --- |
| `/go2/camera/image` | 前视相机彩色图(1280x720@30,内参按社区 720p 标定,H-FOV 73°) |
| `/go2/camera/camera_info` | 相机内参(frame_id 为 `go2/base/front_camera`) |
| `/go2/lidar/points` | 雷达点云(PointCloud2,16 线 x 720,10 Hz,frame `go2/base/utlidar_lidar`) |
| `/go2/lidar`(gz 侧) | 雷达 LaserScan(未桥接到 ROS) |

Gazebo GUI 里相机会自动弹出画面窗口,雷达射线也会绘制;RViz 里看 `Image` + `PointCloud2`。
雷达量程/FOV/帧率目前是 L1 占位参数(360°x90°、30 m、10 Hz),官方规格待补。

### 操控狗移动(键盘遥控)

```bash
# 终端 A:仿真 + 步态节点
ros2 launch go2_description gazebo.launch.py walk:=true rviz:=true

# 终端 B:键盘遥控
source scripts/setup_env.sh
ros2 run go2_description go2_teleop.py
```

按键:`w/s` 前进/后退,`a/d` 或 `q/e` 左转/右转,`k` 或空格停止,`+/-` 加减速,`x` 退出。

不装遥控也可以直接发速度指令:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

说明:步态节点 `go2_trot.py`(由 `walk:=true` 启动)订阅 `/cmd_vel`,收到第一条速度
指令之前保持自动前进,收到后按指令走(零指令 = 站立)。当前开环小跑实测约
0.13 m/s(前进)、0.5 rad/s(转向),横向平移不支持;位姿反馈来自
`/model/go2/odometry`(Gazebo OdometryPublisher)。
不想让它先自动走的话加 `auto_forward:=false`,步态节点会原地站住。

### 狗载视觉链路(相机 + 雷达融合定位)

```bash
# 终端 A:仿真(相机+雷达),步态节点原地站立
ros2 launch go2_description gazebo.launch.py sensors:=true walk:=true auto_forward:=false

# 终端 B:运动目标 + 检测 + 融合定位 + 评估
ros2 launch go2_vision vision_pipeline.launch.py trajectory:=sine speed:=0.5 amplitude:=0.5
```

- `target_director`:红球沿脚本轨迹运动(static|line|sine|circle),发真值 `/target/ground_truth`
- `ball_detector`:HSV 红球检测 → `/target/detections`、调试图 `/target/debug_image`
- `target_localizer`:融合定位(bbox 内雷达点 / 射线∩地面 + 半径 / 单目尺寸)→
  `/target/position`(base 系)、`/target/position_world`(world 系),带 method/quality
- `eval_monitor`:每 5s 打印与真值误差(含球的实际位姿 vs 指令轨迹两种口径)
- `stand_keeper`:持续发零 `/cmd_vel`,让步态节点稳住站姿(视觉测试用)

看调试画面:`ros2 run rqt_image_view rqt_image_view /target/debug_image`
标定对齐检查:`ros2 run go2_vision calib_check.py`(打印残差并存 overlay 图)

红球(r=0.15m)首测结果:静态 xy 0.2cm;正弦 0.5 m/s 与 1.0 m/s 均
**rms 2.6cm / max ~5cm**,方法为 lidar_ground,30Hz 无丢失。

## 3. 视觉仿真(vision_sim)

| 命令 | 说明 |
| --- | --- |
| `ros2 launch vision_sim vision_sim.launch.py` | 全链路 + Gazebo GUI + RViz2 |
| `ros2 launch vision_sim vision_sim.launch.py use_gui:=false rviz:=false` | 无界面完整链路 |
| `ros2 launch vision_sim vision_sim.launch.py pipeline:=false visualize:=true` | 只看相机链路(不起检测/定位/评估) |
| `ros2 launch vision_sim vision_sim.launch.py trajectory:=circle speed:=0.7` | 换目标轨迹/速度 |

launch 参数:

- `use_gui`(默认 `true`):Gazebo GUI 开关
- `rviz`(默认 `true`):RViz2 开关
- `pipeline`(默认 `true`):检测 / 定位 / 评估节点总开关
- `visualize`(默认 `false`):在 Gazebo 窗口叠加相机画面
- `world`(默认 `worlds/vision_bench.sdf`)
- 相机位姿:`camera_x/camera_y/camera_z`(默认 `0,0,0.8`),
  `camera_roll/camera_pitch/camera_yaw`(默认 `0,0.26,0`)
- 相机内参:`width`(640)/`height`(360)/`fps`(30)/`hfov`(1.518 rad ≈ 87°)
- 轨迹:`trajectory`(默认 `sine`,可选 `static|line|sine|circle`)、`speed`(默认 `0.5` m/s)

轨迹的振幅/半径/中心点等参数不在 launch 里,改
`ros2_ws/src/vision_sim/config/vision_params.yaml` 的 `target_director` 段
(`amplitude`、`radius`、`center_x`、`center_y`)。

看画面:

```bash
ros2 run rqt_image_view rqt_image_view /viz/camera/image      # 彩色
ros2 run rqt_image_view rqt_image_view /target/debug_image    # 检测框
```

## 4. 图形化启动(VSCode 任务)

用 VSCode 打开 `~/go2_dev` 文件夹,然后:

- `Ctrl+Shift+B`:运行默认构建任务 **colcon build**
- `Ctrl+Shift+P` → `Tasks: Run Task`,可选任务:
  - `Gazebo: 站姿`
  - `Gazebo: 走路`
  - `键盘遥控 (需先开 Gazebo: 走路)`
  - `Gazebo: 相机+雷达`
  - `RViz2: 显示`

Gazebo 和 RViz2 本身就是图形窗口,由 launch 自动打开(`use_gui:=true`)。

注意:`.vscode/tasks.json` 目前只有 go2_description 的任务,
vision_sim 还没有对应任务,需要命令行启动。

## 5. 停止与清理

- 在运行 launch 的终端按 `Ctrl+C` 即可停止。
- 有残留进程就用 `scripts/kill_sim.sh` 一键清理(Gazebo / ros_gz / RViz 等,
  注意会杀掉本机所有 Gazebo 实例);顽固进程用 `scripts/kill_sim.sh --force`。
- 不要在命令行里直接 `pkill -f "vision_sim.launch"`(会自匹配杀掉当前 shell);
  需要手写 pgrep 时把关键字拆成 `[x]` 形式,例如 `pgrep -f "ign[ ]gazebo"`。

## 常见问题

- **相机没有画面**:确认启动带了 `sensors:=true`;别用 `world:=empty.sdf`
  (那个世界不带 Sensors 系统,传感器不会出图)。
- **RViz 里点云不显示**:Fixed Frame 保持 `base`(`rviz/sensors.rviz` 已配好);
  图像显示不依赖 TF。
- **无头启动出现 `libEGL warning: egl: failed to create dri2 screen`**:
  不影响相机出图/出深度,可忽略。
- **Gazebo 窗口视角乱跑**:多为右键误开 Follow,在右键菜单里取消勾选即可。
- **Gazebo 里 `<visualize>true</visualize>` 显示的是深度灰度图**,
  看彩色要用 rqt_image_view 或 RViz。
