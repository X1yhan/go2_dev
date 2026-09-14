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
| unitree_ros (ROS1) | 快照 2026-08-28 | Desktop/unitree_ros-master.zip | 仅移植了 go2_description |
| go2_description | 1.0.0 | 本仓库 | 仿真/模型/步态包 |

## 上游仓库地址（供以后更新）

- https://github.com/unitreerobotics/unitree_ros2
- https://github.com/unitreerobotics/unitree_sdk2
- https://github.com/unitreerobotics/unitree_sdk2_python
- https://github.com/unitreerobotics/unitree_ros

## 已知注意事项

- `unitree_ros2/setup.sh` 里网卡名是 `enp3s0`，本机有线网卡是 `enp4s0`，连真机时要改。
- Go2 内部网段 192.168.123.x，PC 一般配 192.168.123.18。
- 低层控制功能与 Go2 固件版本相关，EDU 在 App「设备信息」里查看。
