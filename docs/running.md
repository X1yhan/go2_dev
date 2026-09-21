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

| 命令 | 说明 |
| --- | --- |
| `ros2 launch go2_description gazebo.launch.py` | Gazebo 站姿(默认打开 GUI) |
| `ros2 launch go2_description gazebo.launch.py walk:=true` | Gazebo 对角小跑(开环步态) |
| `ros2 launch go2_description gazebo.launch.py use_gui:=false` | 无界面(仅 Gazebo 服务器) |
| `ros2 launch go2_description display.launch.py` | RViz2 + 关节滑块 |

launch 参数:

- `use_gui`(默认 `true`):是否打开 Gazebo GUI 客户端
- `world`(默认 `empty.sdf`):要加载的 world
- `spawn_z`(默认 `0.45`):Go2 出生高度
- `walk`(默认 `false`):是否启动 `go2_trot.py` 开环步态节点

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
  - `RViz2: 显示`

Gazebo 和 RViz2 本身就是图形窗口,由 launch 自动打开(`use_gui:=true`)。

注意:`.vscode/tasks.json` 目前只有 go2_description 的任务,
vision_sim 还没有对应任务,需要命令行启动。

## 5. 停止与清理

- 在运行 launch 的终端按 `Ctrl+C` 即可停止。
- 若有残留 Gazebo 进程,先 `pgrep -af ign` 查看 PID,再 `kill <PID>`;
  注意会连带杀掉其它 Gazebo 实例。
- 不要在命令行里直接 `pkill -f "vision_sim.launch"`,会自匹配杀掉当前 shell。

## 常见问题

- **无头启动出现 `libEGL warning: egl: failed to create dri2 screen`**:
  不影响相机出图/出深度,可忽略。
- **Gazebo 窗口视角乱跑**:多为右键误开 Follow,在右键菜单里取消勾选即可。
- **Gazebo 里 `<visualize>true</visualize>` 显示的是深度灰度图**,
  看彩色要用 rqt_image_view 或 RViz。
