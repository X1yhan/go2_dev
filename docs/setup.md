# 环境搭建指南

从干净的 Ubuntu 22.04 开始，逐步搭出 go2_dev 的完整开发环境（仿真 → 视觉 → 机械臂 → 真机）。
版本清单见 `docs/versions.md`，启动方式见 `docs/running.md`。

## 0. 环境总览（本机当前配置）

| 组件 | 版本 |
| --- | --- |
| Ubuntu | 22.04.5 LTS (jammy)，内核 6.8 |
| ROS2 | Humble（ros-base + 341 个 ros-humble-* 包） |
| Gazebo | Fortress 6.18（`ign gazebo`） |
| ros_gz | 0.244.25（gz ↔ ROS2 桥） |
| ros2_control / ros2_controllers | 2.54.0 / 2.53.3 |
| gz_ros2_control | 0.7.20 |
| MoveIt2 | apt ros-humble-moveit（狗背 RM75 规划用） |
| xacro / cv_bridge / vision_msgs | 2.1.1 / 3.2.1 / 4.1.1 |
| realsense2_description | 4.58.4（apt，仿真用） |
| librealsense2 / realsense2_camera | 2.58.4 / 4.58.4（真机 D435i） |
| Python | 3.10.12 |
| Python 库（pip --user） | numpy 1.26.4（锁 <2）、opencv-python 4.11、ultralytics 8.4.165、torch 2.14.0（4060 可用） |
| RealMan SDK | humble v1.7.0 → 独立工作空间 `~/rm_ws` |
| NVIDIA 驱动 | 595.91.07（RTX 4060 Laptop，可选） |

约定：仿真与控制全部用 **Gazebo Fortress（ign gazebo）**，不是 Gazebo Classic；
所有命令在仓库根目录 `~/go2_dev` 执行。

## 1. 系统前提

- Ubuntu 22.04 LTS，x86_64；建议 ≥16GB 内存、≥30GB 空闲磁盘、独显（相机渲染）
- 真机开发需要一根有线网口（接 Go2 内网），开发机有线网卡名可用 `ip link` 查

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl git build-essential python3-pip
```

## 2. 安装 ROS2 Humble

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions
sudo rosdep init && rosdep update
```

（国内网络慢可换清华/鱼香镜像源；本机 `.bashrc` 里已有 fishros 的初始化块。）

## 3. 仿真依赖（apt）

```bash
sudo apt install -y \
  ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ros-humble-ros-gz-interfaces \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control \
  ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher ros-humble-rviz2 \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-rqt-image-view \
  ros-humble-realsense2-description \
  python3-opencv python3-numpy
```

说明：

- `ros-humble-ros-gz` 会自动带入 **Gazebo Fortress 6.18**（`ign gazebo` 命令）
- `gz_ros2_control` 给 Gazebo 提供 ros2_control 硬件接口（腿 12 关节 + 臂 7 关节靠它）

## 4. 机械臂依赖（MoveIt2 + RealMan SDK）

```bash
sudo apt install -y ros-humble-moveit
```

RealMan SDK 必须放在**独立工作空间**编译（`~/rm_ws`），命令与官方一致：

```bash
mkdir -p ~/rm_ws/src
cp -r ~/go2_dev/third_party/ros2_rm_robot ~/rm_ws/src/
cd ~/rm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build
```

真机用驱动时再按 `third_party/ros2_rm_robot/README_CN.md` 执行 `rm_driver` 的 `lib_install.sh`
（装串口/CAN 动态库），纯仿真不需要。

## 5. Python 识别栈（YOLO，pip --user）

```bash
pip3 install --user numpy==1.26.4 opencv-python \
  torch torchvision ultralytics \
  -i https://pypi.tuna.tsinghua.edu.cn/simple
```

注意：

- **numpy 必须锁 1.26.4（<2）**，numpy 2.x 与 ROS/cv_bridge 冲突
- torch 想用 CUDA 就按 pytorch 官网 `--index-url` 装对应 CUDA 版本；本机实测 4060 推理 yolo11n 仅 4ms
- pip `--user` 安装对 rclpy 节点同样可见，无需改系统 Python

## 6. 获取工程与第三方 SDK

```bash
git clone git@gitee.com:X1yhan/go2_dev.git ~/go2_dev   # 或 https://gitee.com/X1yhan/go2_dev.git
```

`third_party/` 未纳入 git（`.gitignore`），需从上游重新获取。当前归档版本见 `docs/versions.md`，
对应上游地址：

| 目录 | 上游 | 说明 |
| --- | --- | --- |
| unitree_ros2 | github.com/unitreerobotics/unitree_ros2 | 官方 ROS2 接口（CycloneDDS + 消息 + 示例） |
| unitree_sdk2 | github.com/unitreerobotics/unitree_sdk2 | C++ SDK（x86_64 + aarch64） |
| unitree_sdk2_python | github.com/unitreerobotics/unitree_sdk2_python | Python SDK |
| unitree_ros | github.com/unitreerobotics/unitree_ros | ROS1 描述包（只取 robots/go2_description） |
| go2_ros2_sdk | github.com/abizovnuralem/go2_ros2_sdk | 社区 WebRTC ROS2 桥（相机/点云/scan） |
| ros2_rm_robot | github.com/RealManRobot/ros2_rm_robot (humble 分支) | RealMan 机械臂 SDK |
| realsense-ros | github.com/IntelRealSense/realsense-ros | D435i 驱动 + 描述包 |
| dh_gripper_ros | 大寰官方 | AG95 夹爪（ROS1 驱动，仅参考） |

特别说明：

- go2_ros2_sdk 的 `go2_robot_sdk/external_lib/aioice` 子模块 GitHub zip 里缺失，
  已用社区补丁版补全（github.com/legion1581/aioice，go2 分支）
- go2_ros2_sdk 的 Python 依赖（真机 WebRTC 用）：
  `pip3 install --user -r third_party/go2_ros2_sdk/requirements.txt`（含 numpy==1.26.4）

## 7. 编译主工作空间

```bash
cd ~/go2_dev/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

共 3 个包：`go2_description`（模型/仿真/步态）、`vision_sim`（独立视觉验证）、
`go2_vision`（狗载视觉）。可在 VSCode 里 `Ctrl+Shift+B` 一键构建。

## 8. 模型权重（YOLO）

`models/` 已 gitignore；首次运行 YOLO 时 ultralytics 会自动下载，也可手动放置：

```bash
mkdir -p ~/go2_dev/models
# 将 yolo11n.pt 放到 ~/go2_dev/models/yolo11n.pt（脚本默认路径）
```

## 9. 每个终端加载环境（或写进 .bashrc）

```bash
cd ~/go2_dev && source scripts/setup_env.sh
```

脚本会 source ROS2 Humble 和 `ros2_ws/install/setup.bash`。
可选：把上面两行（或 `source ~/go2_dev/ros2_ws/install/setup.bash`）加进 `~/.bashrc`。

## 10. 验证（冒烟测试）

```bash
cd ~/go2_dev && source scripts/setup_env.sh

# ① 模型显示（RViz2 + 关节滑块）
ros2 launch go2_description display.launch.py

# ② Gazebo 站姿 / 走路
ros2 launch go2_description gazebo.launch.py
ros2 launch go2_description gazebo.launch.py walk:=true

# ③ 狗载相机 + 雷达（RViz 看图像/点云）
ros2 launch go2_description gazebo.launch.py sensors:=true rviz:=true

# ④ 独立视觉仿真全链路
ros2 launch vision_sim vision_sim.launch.py use_gui:=false rviz:=false

# ⑤ 狗背 RM75 + MoveIt（需先完成第 4 步）
ros2 launch go2_description gazebo.launch.py arm:=true walk:=true auto_forward:=false
source ~/rm_ws/install/setup.bash && ros2 launch go2_description arm_moveit.launch.py

# ⑥ 独立 RM75 桌面仿真（带 D435i RGB-D + 夹爪）
ros2 launch go2_description arm_bench.launch.py use_gui:=false
```

清理残留进程：`scripts/kill_sim.sh`（顽固情况 `--force`）。

## 11. 真机部分（可选，按需）

### 11.1 D435i 真机驱动

```bash
sudo apt install -y ros-humble-librealsense2 ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs
# udev 规则（apt 包不自带，取 Intel v2.58.4 config/99-realsense-libusb.rules）
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

相机必须插 USB3（蓝色/SS 口，USB2 只有 15fps）；验证：

```bash
ros2 launch realsense2_camera rs_launch.py camera_name:=d435i
ros2 launch go2_vision yolo_d435i.launch.py        # 相机 + YOLO + rqt 标注图
```

### 11.2 Go2 真机网络（unitree_ros2 / CycloneDDS）

```bash
# 1) 有线网卡配静态 IP（Go2 内网 192.168.123.x，PC 用 .18）
sudo nmcli con add type ethernet ifname <网卡名> ip4 192.168.123.18/24

# 2) 依赖 + 编译官方接口
sudo apt install -y ros-humble-rmw-cyclonedds-cpp libyaml-cpp-dev
cd ~/go2_dev/third_party/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash && colcon build

# 3) 环境（注意 setup.sh 里网卡名和 ROS 版本要改成本机的）
#    unitree_ros2/setup.sh 默认 enp3s0 + foxy → 本机 enp4s0 + humble
source ~/go2_dev/third_party/unitree_ros2/setup.sh
ping 192.168.123.161        # Go2 主控
ros2 topic list             # 应能看到 unitree 相关话题
```

注意：

- Humble 环境**不需要**重新编译 cyclonedds（官方说明，apt 的 rmw-cyclonedds-cpp 即可）
- Go2 固件版本影响低层控制功能，在手机 App「设备信息」查看
- 大寰 AG95 夹爪走串口控制时，把用户加入 dialout 组：`sudo usermod -aG dialout $USER`

### 11.3 Go2 机载 Jetson（aarch64，后续）

`unitree_sdk2` 内含 aarch64 库，部署时用交叉/机载编译，环境按同样思路（Jetson 上装 Humble）。

## 12. 常见坑速查

- 相机/雷达没数据：launch 必须带 `sensors:=true`，且别用 `world:=empty.sdf`（不带 Sensors 系统）
- 无头启动的 `libEGL warning: egl: failed to create dri2 screen` 可忽略，不影响出图
- 多 Gazebo 实例会互相抢模型，测试前先 `scripts/kill_sim.sh`
- 清理进程不要手写 `pkill -f "xxx.launch"`（会自匹配杀掉当前 shell），统一用 kill_sim.sh
- 算法节点（go2_vision）必须带 `use_sim_time`；真机相机话题是 BEST_EFFORT QoS
- numpy 升级到 2.x 会崩 ROS 工具链，务必锁 1.26.4
