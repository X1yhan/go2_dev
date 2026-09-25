# 版本记录

| 组件 | 版本/日期 | 来源 | 说明 |
| --- | --- | --- | --- |
| ROS2 | Humble | apt (/opt/ros/humble) | 系统已装 |
| Gazebo | Fortress 6.18 (ign gazebo) | apt | 系统已装 |
| ros_gz | 0.244.25 | apt | gz ↔ ROS2 桥 |
| ros2_control | 2.54.0 | apt | -- |
| gz_ros2_control | 0.7.20 | apt | Gazebo 硬件接口 |
| unitree_ros2 | 0.3.0 | Desktop/unitree_ros2-master.zip | third_party/，官方 ROS2 接口 |
| unitree_sdk2 | 快照 2026-08-20 | Desktop/unitree_sdk2-main.zip | third_party/，C++ SDK |
| unitree_sdk2_python | 快照 | Desktop/unitree_sdk2_python-master.zip | third_party/，Python SDK |
| unitree_ros (ROS1) | 快照 2026-08-28 | Desktop/unitree_ros-master.zip | third_party/unitree_ros/，仅提取 robots/go2_description |
| go2_ros2_sdk | master 快照 2026-07-14 | Desktop/go2_ros2_sdk-master.zip | third_party/，社区 WebRTC ROS2 桥（相机/点云/scan/nav2） |
| ros2_rm_robot | humble v1.7.0 (控制器 1.7.3) | Desktop/ros2_rm_robot (GitHub RealManRobot) | third_party/，RealMan 机械臂 ROS2 SDK（URDF/Gazebo/MoveIt2/driver，含 RM75） |
| realsense-ros | 4.58.4 (ros2-master) | Desktop/realsense-ros (GitHub IntelRealSense) | third_party/，D435i 驱动 + 描述包（模型用 apt 的 realsense2_description，驱动留给真机） |
| dh_gripper_ros | ROS1 catkin 快照 | Desktop/dh_gripper_ros | third_party/，大寰夹爪（AG95/AG145/PGC140/DH3 URDF + 串口驱动/消息，实物 AG95） |
| aioice (patched) | go2 分支快照 2026-09-21 | github.com/legion1581/aioice | third_party/go2_ros2_sdk/go2_robot_sdk/external_lib/，GitHub zip 不含子模块，已单独补全 |
| go2_description | 1.0.0 | 本仓库 | 仿真/模型/步态包 |
| realsense2_description | apt (ros-humble) | /opt/ros/humble/share | D435i 模型/TF，vision_sim 使用 |
| vision_sim | 0.1.0 | 本仓库 | 独立视觉仿真验证环境 |

## 仓库内传感器参考数据（docs/reference/）

| 文件 | 说明 |
| --- | --- |
| front_camera_720.yaml | Go2 前视相机社区标定，1280x720，K/D/P 完整可用（FOV≈73°×45°） |
| front_camera_1080.yaml | 1920x1080 标定，**K 矩阵数据有误**（k[3]、k[6] 应为 0），仅作参考 |
| go2_with_realsense.urdf | 社区 D435i 挂载方案（Head_upper 上的 camera_mount_link） |
| Intel_D435i_specifications.csv | Intel 官方 D435i 规格（深度 87°×58°，0.3–3 m） |

## 上游仓库地址（供以后更新）

- https://github.com/unitreerobotics/unitree_ros2
- https://github.com/unitreerobotics/unitree_sdk2
- https://github.com/unitreerobotics/unitree_sdk2_python
- https://github.com/unitreerobotics/unitree_ros

## 已知注意事项

- `unitree_ros2/setup.sh` 里网卡名是 `enp3s0`，本机有线网卡是 `enp4s0`，连真机时要改。
- Go2 内部网段 192.168.123.x，PC 一般配 192.168.123.18。
- 低层控制功能与 Go2 固件版本相关，EDU 在 App「设备信息」里查看。
- vision_sim 无头启动时会出现 `libEGL warning: egl: failed to create dri2 screen`，
  不影响相机出图/出深度（实测渲染正常）。
- RealSense URDF 的光学系 frame 只在 `use_nominal_extrinsics:=true` 时生成，
  vision_sim 的 xacro 已显式传该参数。
- go2_ros2_sdk 的相机标定/挂载是社区数据，真机使用前建议自行复标；
  1080p 标定文件已知有误（见上表）。
- L1 激光雷达的量程/FOV/帧率等官方规格仍未收集，仿真先用 gpu_lidar 占位。
