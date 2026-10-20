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
- **机械臂抓球（本文重点）**：
  - `grasp_ball.py`：固定位置绿球抓取，已验证球从 z=0.30m 抬到 z=0.45m
  - `chase_grasp.py`：狗追运动球 + 侧向拦截抓取

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
  ros-humble-rviz2 ros-humble-tf2-ros \
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

### 8.4 真机功能

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

## 10. 更多文档

- `docs/setup.md`：完整环境搭建（含真机、Jetson、RealSense）
- `docs/running.md`：所有 launch 参数与运行场景
- `docs/status.md`：当前进度、实测数据和踩坑记录
- `docs/versions.md`：第三方包版本与上游地址
- `ros2_ws/src/vision_sim/README.md`：视觉仿真细节
