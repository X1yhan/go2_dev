# Go2 开发工程

Unitree Go2 EDU 的 ROS2 仿真、视觉与机械臂开发工程。

本文档给出**从零 clone 到复现“Go2 + RM75 机械臂抓取小球”仿真**的完整步骤，
包括需要下载哪些第三方包、下载到哪里、编译到哪里。按第 1～6 节操作即可。

## 已实现功能

- Gazebo Fortress 四足仿真：站姿、对角小跑、键盘遥控
- 狗载传感器仿真：前视相机 + 16 线激光雷达
- 独立视觉仿真 `vision_sim`：D435i + 移动目标 + 3D 定位精度评估
- 狗载视觉 `go2_vision`：红球检测 + 相机/雷达融合定位
- 狗背 RM75 机械臂：URDF 拼接、ros2_control、MoveIt2 规划
- 独立机械臂仿真 `arm_bench`：RM75 + D435i RGB-D + 夹爪，固定在正方体桌面上
- 简单视觉抓取：相机检测桌面红球 → 深度反投影 → MoveIt 抓取
- 多物体识别抓取：红球/蓝方块/绿圆柱/黄盒子按 `target_class` 选择并抓取
- 自然语言交互：终端输入“把红球放到黄盒子旁边”等中文指令，确认后执行
- **机械臂抓球（本文重点）**：
  - `grasp_ball.py`：固定位置绿球抓取，已验证球从 z=0.30m 抬到 z=0.45m
  - `chase_grasp.py`：狗追运动球 + 侧向拦截抓取

## 开发过程记录

本次“多物体抓取 → 纯视觉定位 → 自然语言交互 → 装配模型 MoveIt → 一键启动”
的完整开发过程（需求拆解、分阶段实现、踩坑、调试命令、验证记录、未完成项）
单独整理在：

- [`docs/dev_process_2026-10-30.md`](docs/dev_process_2026-10-30.md)

新同学建议先读 `docs/status.md` 了解当前状态，再按这份记录复现和继续开发。

## 目录结构

```text
go2_dev/
├── ros2_ws/src/
│   ├── go2_description/   # Go2 模型、Gazebo、步态、RM75 机械臂、抓取脚本
│   ├── go2_vision/        # 狗载视觉（真机/仿真）
│   └── vision_sim/        # 独立视觉仿真验证环境
├── third_party/           # 第三方 SDK（不在 git 中，需要按第 4 节自行下载）
├── models/                # YOLO 权重（不在 git 中，仅视觉功能需要）
├── docs/                  # 环境、启动、状态、版本说明
├── scripts/               # setup_env.sh / kill_sim.sh
└── README.md
```

注意：`third_party/`、`models/`、`build/`、`install/`、`log/` 都在 `.gitignore` 中，
所以别人 clone 下来默认是**没有**这些目录的。其中抓球仿真只需要补
`third_party/ros2_rm_robot`，其余 SDK 不需要。只跑基础四足仿真（站姿、走路、
狗载传感器）时，`third_party/` 一个都不需要。

---

## 1. 环境要求

- Ubuntu 22.04 LTS，x86_64
- ROS2 Humble
- Gazebo Fortress（`ign gazebo`，不要用 Gazebo Classic）
- 建议 16GB 内存、20GB 以上空闲磁盘
- 有独显更好，但不是抓球仿真的硬性要求

以下命令默认仓库位于 `~/go2_dev`，RealMan SDK 工作空间位于 `~/rm_ws`。
如果仓库放在其他路径，请同步修改命令中的路径；机械臂路径也可以通过
`RM_ROBOT_ROOT` 或 launch 参数 `arm_root:=...` 指定。

## 2. 安装系统依赖

### 2.1 ROS2 Humble（已安装可跳过）

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions
sudo rosdep init 2>/dev/null || true
rosdep update
```

### 2.2 仿真、MoveIt 与视觉所需 apt 包

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-xacro ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-moveit ros-humble-realsense2-description \
  ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-rqt-image-view \
  python3-opencv python3-numpy \
  ros-humble-nav-msgs ros-humble-control-msgs \
  ros-humble-trajectory-msgs ros-humble-shape-msgs ros-humble-sensor-msgs \
  ros-humble-actionlib-msgs ros-humble-rclcpp-action \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-ament-lint-auto ros-humble-ament-lint-common \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest
```

说明：

- `ros-humble-ros-gz` 会带入 Gazebo Fortress
- `gz_ros2_control` 提供 Gazebo 的 ros2_control 硬件接口
- `ros-humble-moveit` 提供 MoveIt2，抓球规划必需
- `ros-humble-realsense2-description` 只提供 D435i 的 URDF 模型，**不是**相机驱动
- `ros-humble-cv-bridge` / `vision_msgs` 供 `vision_sim` 和 `go2_vision` 使用
- 纯抓球仿真不需要 `realsense2_camera`、librealsense、YOLO、torch

### 2.3 Python 依赖（仅视觉功能需要）

抓球仿真不需要额外 pip 包。运行 `vision_sim` / `go2_vision` 时再执行：

```bash
pip3 install --user numpy==1.26.4 opencv-python \
  torch torchvision ultralytics \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

`numpy` 必须小于 2，否则会与 ROS2 / cv_bridge 冲突。

## 3. 克隆本仓库

```bash
git clone git@gitee.com:X1yhan/go2_dev.git ~/go2_dev

# 没有配置 SSH key 时用 HTTPS：
# git clone https://gitee.com/X1yhan/go2_dev.git ~/go2_dev
```

## 4. 准备被 gitignore 的第三方包

`third_party/` 不在仓库里。**抓球仿真在 `third_party/` 里只需要 RealMan SDK**
（D435i 模型由 apt 的 `realsense2_description` 提供）。下载后必须放在
`~/go2_dev/third_party/ros2_rm_robot`；launch 默认就是从这个路径找机械臂模型和
MoveIt2 配置。

```bash
mkdir -p ~/go2_dev/third_party
cd ~/go2_dev/third_party

# 必需：RM75 机械臂 URDF、mesh、MoveIt2 配置、rm_ros_interfaces
git clone -b humble https://github.com/RealManRobot/ros2_rm_robot.git
cd ros2_rm_robot
git checkout c941b56   # 本项目验证过的版本（humble1.7.0 之后一个提交）
cd ..

# 可选：大寰 AG95 夹爪完整 URDF
# 只有 display_arm.launch.py 显示真实夹爪外观时需要；
# 抓球仿真用的是简化平行爪，不装也能跑。
git clone https://github.com/DH-Robotics/dh_gripper_ros.git
```

下载完成后的目录应为：

```text
~/go2_dev/third_party/
├── ros2_rm_robot/          # 必需
│   ├── rm_description/
│   ├── rm_moveit2_config/rm_75_config/
│   └── ...
└── dh_gripper_ros/         # 可选
    └── dh_robotics_ag95_gripper/
```

### 4.1 各第三方包是否需要下载

| 包 / 文件 | 抓球仿真是否需要 | 用途 | 放置位置 |
| --- | --- | --- | --- |
| `ros2_rm_robot` | **必需** | RM75 模型、MoveIt2 配置 | `~/go2_dev/third_party/ros2_rm_robot` |
| `realsense2_description` | **必需** | D435i 模型（apt 安装） | `/opt/ros/humble/share/realsense2_description` |
| `dh_gripper_ros` | 可选 | `display_arm` 显示真实 AG95 外观 | `~/go2_dev/third_party/dh_gripper_ros` |
| `models/yolo11n.pt` | 不需要 | YOLO 识别 | 视觉功能自动下载或手动放到 `~/go2_dev/models/` |
| `unitree_ros2` | 不需要 | Go2 真机 ROS2 接口 | 真机才用 |
| `unitree_sdk2` | 不需要 | Go2 C++ SDK | 真机/机载才用 |
| `unitree_sdk2_python` | 不需要 | Go2 Python SDK | 真机才用 |
| `unitree_ros` | 不需要 | ROS1 描述包 | 仅参考资料 |
| `go2_ros2_sdk` | 不需要 | 真机 WebRTC 桥 | 真机才用 |
| `realsense-ros` | 不需要 | D435i 驱动源码 | 真机才用，仿真用 apt 描述包即可 |

如果 `ros2_rm_robot` 放在了其他位置，可以设置环境变量：

```bash
export RM_ROBOT_ROOT=/你的路径/ros2_rm_robot
```

或者在启动时传参：

```bash
ros2 launch go2_description gazebo.launch.py arm:=true arm_root:=/你的路径/ros2_rm_robot
```

注意：`arm_root` 只影响 Gazebo / `display_arm` 读取 URDF 的路径；MoveIt2 使用的是
`~/rm_ws` 中编译出的 `rm_75_config`，所以第 5.1 节的复制和编译不能省。

### 4.2 检查第三方包是否放对

```bash
test -f ~/go2_dev/third_party/ros2_rm_robot/rm_description/urdf/rm_75.urdf \
  && echo "RM75 URDF OK"
test -f ~/go2_dev/third_party/ros2_rm_robot/rm_moveit2_config/rm_75_config/config/rm_75_description.srdf \
  && echo "MoveIt config OK"
```

## 5. 编译

### 5.1 编译 RealMan SDK 到独立工作空间 `~/rm_ws`

不要把 `third_party/ros2_rm_robot` 直接在当前工作空间编译，按官方建议复制到
`~/rm_ws/src`，避免污染仓库。

```bash
mkdir -p ~/rm_ws/src
rm -rf ~/rm_ws/src/ros2_rm_robot
cp -r ~/go2_dev/third_party/ros2_rm_robot ~/rm_ws/src/

cd ~/rm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

验证：

```bash
source ~/rm_ws/install/setup.bash
ros2 pkg prefix rm_75_config
ros2 pkg prefix rm_description
```

应分别输出 `~/rm_ws/install/rm_75_config` 和 `~/rm_ws/install/rm_description`。

### 5.2 编译本项目

```bash
cd ~/go2_dev/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

验证：

```bash
cd ~/go2_dev
source scripts/setup_env.sh
source ~/rm_ws/install/setup.bash
ros2 pkg prefix go2_description
ros2 pkg prefix rm_75_config
ros2 pkg prefix realsense2_description
```

应分别指向 `~/go2_dev/ros2_ws/install/go2_description`、
`~/rm_ws/install/rm_75_config` 和 `/opt/ros/humble`。

## 6. 运行机械臂抓球仿真

抓球 demo 需要三个终端。每次开跑前建议先清理残留进程：

```bash
~/go2_dev/scripts/kill_sim.sh --force
```

### 终端 A：启动 Gazebo 与 Go2 + RM75

```bash
cd ~/go2_dev
source scripts/setup_env.sh
ros2 launch go2_description gazebo.launch.py \
  arm:=true walk:=true auto_forward:=false
```

这条命令会启动：

- Gazebo 世界 `go2_sim.sdf`（地面、浮空绿色目标球、传感器系统）
- Go2 四足 + RM75 机械臂 + D435i + 简化 AG95 平行爪
- 腿、臂、夹爪的 `controller_manager` 和控制器
- 狗的里程计 `/model/go2/odometry`、球的里程计 `/model/grasp_ball/odometry`

等待 Gazebo 窗口正常显示，并在另一个已加载环境的终端里确认控制器已经加载：

```bash
cd ~/go2_dev
source scripts/setup_env.sh
ros2 control list_controllers
```

应看到 4 个 `active` 控制器：`joint_state_broadcaster`、
`joint_group_position_controller`、`rm_group_controller`、`gripper_controller`。

### 终端 B：启动 MoveIt2

```bash
cd ~/go2_dev
source scripts/setup_env.sh
source ~/rm_ws/install/setup.bash

ros2 launch go2_description arm_moveit.launch.py
```

等待 move_group 初始化完成。RViz 会打开，MoveIt 使用仿真时间。
无显示器时把最后一条命令改成 `ros2 launch go2_description arm_moveit.launch.py rviz:=false`。

### 终端 C：复位小球并执行抓球

当前 `go2_sim.sdf` 里的绿球默认在 `x=1.5`，是给追球脚本用的；简单抓球需要先把球
放到机械臂前方 `(0.35, 0.25, 0.30)`。在终端 C 先调用 Gazebo 的 `set_pose` 服务：

```bash
cd ~/go2_dev
source scripts/setup_env.sh
source ~/rm_ws/install/setup.bash

ros2 service call /world/go2_sim/set_pose \
  ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: grasp_ball, type: 2}, pose: {position: {x: 0.35, y: 0.25, z: 0.30}, orientation: {w: 1.0}}}"
```

应返回 `success=True`。然后运行抓球脚本：

```bash
ros2 run go2_description grasp_ball.py
```

脚本会依次执行：

1. 机械臂回零
2. 打开夹爪
3. 用 MoveIt 移动到球上方的预抓取位
4. 笛卡尔直线下探
5. 夹爪闭合
6. 直线抬起
7. 检查球是否被抬起

成功时终端 C 最后一行会包含：

```text
GRASP SUCCESS (ball z 0.250 -> 0.4xx)
```

其中 `0.250` 是脚本 `BALL_GOAL` 里的判定基准高度；实际目标球初始高度是
z=0.30m，抓取成功后会被抬到约 z=0.45m。目标球是**浮空的绿色小球
`grasp_ball`**（半径 0.05m），不是视觉链路里的红球。抓取时脚本还会在
MoveIt 规划场景里添加一个 0.25m 高的台面作为碰撞对象。抓过一次后球不会自动复位，
重新执行上面的 `set_pose` 服务即可再抓一次（也可以重启终端 A）。

## 7. 抓取流程与代码位置

抓球仿真用到的文件：

| 文件 | 作用 |
| --- | --- |
| `ros2_ws/src/go2_description/launch/gazebo.launch.py` | 组装 Go2 + RM75 + D435i + 夹爪，启动控制器 |
| `ros2_ws/src/go2_description/launch/arm_moveit.launch.py` | 启动 MoveIt2 `move_group` |
| `ros2_ws/src/go2_description/scripts/build_arm_model.py` | 合并 Go2 URDF 与 RM75 URDF |
| `ros2_ws/src/go2_description/scripts/grasp_ball.py` | 固定位置抓球流程 |
| `ros2_ws/src/go2_description/scripts/chase_grasp.py` | 追运动球并侧向拦截抓取 |
| `ros2_ws/src/go2_description/worlds/go2_sim.sdf` | 地面、绿球、传感器系统 |
| `ros2_ws/src/go2_description/config/go2_arm_controllers.yaml` | 腿/臂/夹爪控制器参数 |

抓球脚本通过 `/model/grasp_ball/odometry` 读取球的位置，通过
`/model/go2/odometry` 读取狗的位置和朝向，把球坐标转换到机械臂 `base_link`
坐标系，然后用 MoveIt 规划到预抓取点，最后用笛卡尔直线完成下探和抬起。

控制链：

- 腿：`joint_group_position_controller` 保持站姿
- 臂：`rm_group_controller`（JointTrajectoryController，7 个关节）
- 夹爪：`gripper_controller`（简化平行爪，两个 prismatic 指）
- 规划：`arm_moveit.launch.py` 启动 `move_group`，`grasp_ball.py` 通过
  `/move_action` 和 `/compute_cartesian_path` 调用
- 抓取判定：球被抬起后 z > 0.31m 即判定成功

### 追球抓取（可选）

前提与上面完全相同，只需把终端 C 的命令换成：

```bash
ros2 run go2_description chase_grasp.py
```

该脚本会让狗追移动球、折叠机械臂到监控姿态、预测拦截点并完成抓取。
`chase_grasp.py` 会自己把球设置到运动起点，**不需要**执行上面的 `set_pose`。

## 8. 其他已实现功能

### 8.1 基础四足仿真

```bash
cd ~/go2_dev
source scripts/setup_env.sh

ros2 launch go2_description gazebo.launch.py
ros2 launch go2_description gazebo.launch.py walk:=true
ros2 launch go2_description gazebo.launch.py sensors:=true rviz:=true
```

键盘遥控另开终端：

```bash
cd ~/go2_dev && source scripts/setup_env.sh
ros2 run go2_description go2_teleop.py
```

### 8.2 独立视觉仿真

只需要第 2.2 节的 apt 包（含 `realsense2_description`、`cv_bridge`）和
Python 的 `numpy` / `opencv-python`（第 2.3 节中的 torch / ultralytics
只有 `go2_vision` 的 YOLO 真机功能才需要），**不需要** `third_party/`，
也**不需要** YOLO 权重。

```bash
cd ~/go2_dev
source scripts/setup_env.sh
ros2 launch vision_sim vision_sim.launch.py
```

### 8.3 机械臂 URDF 显示

```bash
cd ~/go2_dev
source scripts/setup_env.sh
ros2 launch go2_description display_arm.launch.py
```

`display_arm.launch.py` 默认会加载 AG95 夹爪，需要第 4 节可选的
`dh_gripper_ros`。没装时用 `gripper:=none` 跳过夹爪：

```bash
ros2 launch go2_description display_arm.launch.py gripper:=none
```

### 8.4 独立机械臂仿真 `arm_bench`

不启动 Go2，只仿真 RM75 机械臂。机械臂基座固定在一个 0.6m 正方体桌面上，
Link7 末端装 D435i RGB-D 相机和简化 AG95 二指夹爪。

```bash
cd ~/go2_dev
source scripts/setup_env.sh
ros2 launch go2_description arm_bench.launch.py
```

常用参数：

```bash
# 无 Gazebo 界面
ros2 launch go2_description arm_bench.launch.py use_gui:=false

# 同时打开 RViz（机器人 + 彩色图 + 深度图 + 点云）
ros2 launch go2_description arm_bench.launch.py rviz:=true

# 另开 rqt_image_view 看末端相机实时画面（view_topic 可换话题）
ros2 launch go2_description arm_bench.launch.py view:=true

# 不要相机 / 不要夹爪
ros2 launch go2_description arm_bench.launch.py camera:=none
ros2 launch go2_description arm_bench.launch.py gripper:=none
```

启动内容：

- 世界 `worlds/arm_bench.sdf`：地面、0.6m 立方桌、桌面上 4 个彩色物体
  （红球 / 蓝方块 / 绿圆柱 / 黄盒子，各带 odometry）
- 机械臂基座在 `(0, 0, 0.6)`，通过固定关节锁在世界系
- 控制器：`joint_state_broadcaster`、`rm_group_controller`、
  `gripper_controller`
- 默认保持零位自然竖立、不动；需要 `forward` 准备姿态时加
  `start_forward:=true`（同时张开夹爪）
- 相机话题：
  - `/arm/camera/image`（彩色）
  - `/arm/camera/depth_image`（深度，背景为 `-inf`，使用前过滤 finite）
  - `/arm/camera/camera_info`
  - `/arm/camera/points`（RGB 点云，PointCloud2）
- 相机数据帧：`arm_bench/Link7/arm_rgbd_camera`，已通过静态 TF 接到
  `camera_depth_optical_frame`

需要 MoveIt2 规划时，另开终端：

```bash
cd ~/go2_dev
source scripts/setup_env.sh
source ~/rm_ws/install/setup.bash
ros2 launch go2_description arm_moveit.launch.py
```

MoveIt 使用 `rm_75_config` 里的标准 RM75 模型（不含相机/夹爪外观），控制的仍是
同一套 `rm_group_controller`。可在 RViz 里拖动 MotionPlanning 目标继续调姿态
（适合笛卡尔点动，Plan & Execute，不用写程序）。

想在仿真里像示教器一样实时点动关节，用键盘示教脚本（无需额外安装）：

```bash
# 终端 B：arm_bench.launch.py 运行中
ros2 run go2_description arm_bench_teleop.py
#   1..7 选关节，w/s 正/负方向，+/- 调步长，g 夹爪开合，
#   0 回零位，f forward 姿态，p 打印关节角，q 退出
```

也可以装 `sudo apt install ros-humble-rqt-joint-trajectory-controller`，然后
`ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller`，
选 `/controller_manager` + `rm_group_controller`，用滑条像示教器一样拖动。

在已完成第 2.2 节 apt 依赖的前提下，不要 Go2 时这个仿真只需要：

- apt 的 `ros-humble-realsense2-description`（D435i 模型）
- `third_party/ros2_rm_robot`（RM75 URDF）
- 可选 `third_party/dh_gripper_ros`（只用于真实 AG95 外观；默认简化夹爪不需要）

### 8.5 多物体识别与纯视觉抓放（`arm_bench`）

桌面上放了 4 个彩色物体，检测节点识别标签，抓取脚本用**相机 + 深度 + TF**
扫描定位，再做抓取/放置。**位置不使用 Gazebo odometry**；类别尺寸和桌面
高度作为先验。

一键启动见 8.6。手动分终端：

```bash
# 终端 A：机械臂仿真（先启动，会生成 MoveIt 用的装配 URDF）
cd ~/go2_dev && source scripts/setup_env.sh
ros2 launch go2_description arm_bench.launch.py rviz:=true view:=true

# 终端 B：MoveIt（必须在 A 之后；使用装配模型，含相机/夹爪碰撞）
source ~/rm_ws/install/setup.bash
ros2 launch go2_description arm_moveit.launch.py

# 终端 C：多物体检测（必须运行）
ros2 run go2_description arm_bench_object_detector.py

# 终端 D：扫描 + 抓取（默认 red_ball）
source ~/rm_ws/install/setup.bash
ros2 run go2_description arm_bench_visual_grasp.py \
  --ros-args -p target_class:=blue_cube
```

桌面物体（世界坐标，base_link 在桌面中心 `z=0.6`）：

| 标签 | 模型 | 位置 | 尺寸 |
| --- | --- | --- | --- |
| `red_ball` | `target_ball` | `(0.22, 0, 0.635)` | 球 r=0.035 |
| `blue_cube` | `blue_cube` | `(0.26, -0.16, 0.635)` | 0.07 立方 |
| `green_cylinder` | `green_cylinder` | `(0.15, 0.11, 0.65)` | r=0.04, h=0.10 |
| `yellow_box` | `yellow_box` | `(0.25, 0.16, 0.635)` | 0.10 x 0.075 x 0.07 |

检测/扫描：

- 检测节点按 HSV 颜色输出 `/arm/vision/detections`（`Detection2DArray`，
  `class_id`=标签）和 `/arm/vision/image_annotated`
- 抓取脚本先做 6 个观察位扫描（默认观察位最后）并在**没找到目标时补扫**；
  两轮都没有就报“不在视野”并停止
- 3D 定位：球用 bbox+深度+半径/桌面先验；方块/圆柱用颜色 mask 内的
  最近深度顶面；都用 `camera_depth_optical_frame -> base_link` 的 TF
- 全部位置来自扫描；不再订阅 `/model/*/odometry`

抓取/放置流程：

1. 扫描结果加入 MoveIt planning scene；确认抓取目标和放置目标都在扫描里
2. MoveIt 预抓取（装配模型含相机/夹爪，避免撞本体/桌子）→ 把被抓物从
   collision scene 移除 → 垂直 Cartesian 下探 → 按类别 open 接近 →
   按物体宽度闭合 → 垂直抬起
3. 搬运用高路径；放置按物体宽度只张开到刚好释放；放完再用相机复检
4. `place_mode=on_top` 只支持方块/黄盒；旁边放置自动找 3cm 边到边空位

实测：

- 语言多步命令（红球放黄盒旁边 → 蓝方块叠到黄盒上）跑通；
  放置后视觉校验分别约 `(0.239,0.043,0.020)` 和 `(0.223,0.159,0.105)`
- 红球是球体，抓取时手指偶尔不对称；球叠到平面上会滚落，建议用方块/
  圆柱叠放，或把球放“旁边”
- 扫描约 60–90s/次（6 个观察位），默认位通常已经能看到全部物体，
  后续会把扫描改成“默认位优先、缺目标再环顾”的 lazy 模式

可调参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_class` | `red_ball` | 抓取目标标签 |
| `grip_offset` | `-1` | -1 用类别默认（球/方块 0.17，圆柱 0.15） |
| `pre_offset` | `0.10` | 预抓取高度 |
| `lift_offset` | `0.18` | 抬起高度 |
| `open` | `-1` | -1 用类别默认（球 0.05，其它 0.04） |
| `close` | `-1` | -1 用类别默认（圆柱 0.028，黄盒 0.025，其余 0） |
| `place_class` | 空 | 放置目标；空=只抓取 |
| `place_mode` | `on_top` | `on_top`（仅方块/黄盒）或 `beside` |
| `place_offset` | `0.03` | 旁放边到边间隙（米） |
| `place_clearance` | `0.0` | 释放余量 |
| `scan_only` | `false` | 只扫描并打印可见物体/坐标 |
| `detect_only` | `false` | 扫描后只打印，不抓 |

说明：

- MoveIt 使用的是和 Gazebo 同一套装配 URDF（RM75 + D435i + AG95），
  相机/夹爪参与碰撞检查；`arm_moveit.launch.py` 必须在
  `arm_bench.launch.py` 之后启动（前者读取后者写出的装配 URDF）
- RViz 不加载 Gazebo 的 SDF 世界：A 的 RViz 里看到的是相机点云/图像，
  MoveIt RViz 里看到的是扫描出来的 collision objects，不是 Gazebo 模型；
  两边布局不完全一致是数据源不同，不是 Gazebo 布局变了

### 8.6 自然语言交互（规则语法）

终端输入中文，程序解析成动作并**先显示计划、确认后才执行**；持续接收指令，
说“结束了”退出。

**一键启动（推荐，单个终端）**

```bash
cd ~/go2_dev
scripts/start_arm_bench.sh
```

脚本会自动编译、启动 Gazebo + 机械臂 RViz + 相机窗口 + MoveIt + 物体检测，
最后把自然语言终端放在前台。输入“结束了”时所有后台进程一起停止。

```bash
scripts/start_arm_bench.sh --headless      # 无 GUI/RViz/相机窗口
scripts/start_arm_bench.sh --moveit-rviz   # 用 MoveIt RViz 替代机械臂 RViz
scripts/start_arm_bench.sh --no-build      # 跳过 colcon build
```

日志在 `/tmp/arm_bench_{sim,moveit,detector}.log`。

手动分终端启动（可选）：

```bash
cd ~/go2_dev && source scripts/setup_env.sh
source ~/rm_ws/install/setup.bash
ros2 run go2_description arm_bench_language.py
```

交互示例：

```text
指令> 把红球放到黄盒子旁边
解析结果:
1. 抓取 红球(red_ball) → 放到 黄盒子(yellow_box) 旁边(3cm 空位)
确认执行? [y/N] y
[1/1] 抓取红球 → 放到黄盒旁边
  ...（MoveIt/夹爪/odometry 日志）...
  PLACE SUCCESS red_ball -> yellow_box ...
完成。
指令> 桌上有什么
当前物体(标签/是否可见/base_link 坐标):
  red_ball 可见 (0.205, 0.027, 0.035)
  ...
指令> 结束了
再见。
```

支持的句式：

- `抓红球` / `把红球拿起来`：只抓取（会把物体留在夹爪里）
- `去抓红球,放到黄盒子上`：抓取 + 叠放（目标必须是方块/黄盒子）
- `把蓝方块放到绿圆柱旁边`：抓取 + 旁边放置（自动找 3cm 空位）
- `抓红球放到黄盒子上,然后抓蓝方块放到绿圆柱旁边`：多步顺序执行
- `桌上有什么` / `查询物体`；`帮助`；`停止`；`结束了`/`退出`/`q`

规则：

- 多步用 `然后` / `再` / `接着` / `；` 连接；句内逗号不影响“抓X,放到Y”
- 物体别名：红球、蓝方块、绿圆柱、黄盒子；颜色只认红/蓝/绿/黄
- 确认必须输入 `y/yes/是/确认/执行/可以`，直接回车或其它内容=取消
- 执行前显示解析结果；多步只确认一次；某一步失败立即停止
- “放到旁边”在目标周围 8 个方向找空位，边到边约 3cm，并检查桌面边界和
  其它物体间距；“放到上面”会检查目标顶面，球/圆柱顶面太小会拒绝
- 球体（红球）叠放到平面上机构上能放，但可能几秒后滚落；需要稳定叠放时
  用方块/圆柱，或说“放到旁边”；REPL 会给出提示
- 语言脚本用子进程逐个调用 `arm_bench_visual_grasp.py`；中间失败会停止并
  报告是哪一步

### 8.7 真机功能

真机部分才需要 `unitree_ros2`、`unitree_sdk2`、`unitree_sdk2_python`、
`go2_ros2_sdk`、`realsense-ros` 等，按 `docs/setup.md` 第 11 节操作。
纯仿真抓球不需要这些包。

## 9. 常见问题

### 9.1 `arm urdf not found` 或 `Package 'rm_75_config' not found`

检查：

```bash
ls ~/go2_dev/third_party/ros2_rm_robot/rm_description/urdf/rm_75.urdf
source ~/rm_ws/install/setup.bash
ros2 pkg prefix rm_75_config
```

没有 `rm_75_config` 时重新执行 5.1 节；终端 B/C 必须 source `~/rm_ws/install/setup.bash`。

### 9.2 `camera=d435i needs --realsense-share`

缺少 D435i 模型包：

```bash
sudo apt install -y ros-humble-realsense2-description
```

如果暂时不想装相机，也可以启动时加 `camera:=none`；抓球本身不依赖相机。

### 9.3 `package 'go2_description' not found`

先编译，并在新终端 source：

```bash
cd ~/go2_dev/ros2_ws && source /opt/ros/humble/setup.bash && colcon build
cd ~/go2_dev && source scripts/setup_env.sh
```

### 9.4 MoveIt 规划失败或控制器报 `CONTROL_FAILED`

- 先启动终端 A，确认 `ros2 control list_controllers` 里 4 个控制器都是 `active`
- 再启动终端 B，等 move_group 完全就绪
- 最后才运行终端 C
- `arm_moveit.launch.py` 已经设置 `use_sim_time=True`，不要删掉

### 9.5 Gazebo 启动异常或模型互相抢占

```bash
~/go2_dev/scripts/kill_sim.sh --force
```

不要手写 `pkill -f "xxx.launch"`，容易把当前 shell 一起杀掉。

### 9.6 无显示器 / 无 GPU

Gazebo 可以无界面运行：

```bash
ros2 launch go2_description gazebo.launch.py arm:=true walk:=true auto_forward:=false use_gui:=false
```

出现的 `libEGL warning: egl: failed to create dri2 screen` 通常不影响仿真数据。

### 9.7 视觉功能里 numpy 报错

`numpy` 必须是 1.26.x，不能是 2.x：

```bash
pip3 install --user "numpy<2"
```

### 9.8 视觉抓取检测不到球

- 先 `arm_bench.launch.py` + `arm_moveit.launch.py` 都启动，再跑抓取脚本
- 在 RViz 看 `GraspDebug` 图像；球不在画面里就调 `observe_xyz`
- 确认球是红色、没有被机械臂挡住；`camera_clip_near` 默认 0.05，桌面距离约 0.25m
- 先用 `detect_only:=true` 只验证坐标，再抓取

## 10. 更多文档

- `docs/setup.md`：完整环境搭建（含真机、Jetson、RealSense）
- `docs/running.md`：所有 launch 参数与运行场景
- `docs/status.md`：当前进度、实测数据和踩坑记录
- `docs/versions.md`：第三方包版本与上游地址
- `ros2_ws/src/vision_sim/README.md`：视觉仿真细节
