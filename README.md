# Go2 开发工程

Unitree Go2 EDU 的仿真与真机开发工程。

## 环境

- Ubuntu 22.04 + ROS2 Humble
- Gazebo Fortress (ign gazebo 6.18) + ros_gz
- ros2_control + gz_ros2_control

## 目录结构

```
go2_dev/
├── ros2_ws/                  # colcon 工作空间
│   └── src/
│       └── go2_description/  # Go2 模型/仿真/步态包
├── third_party/              # 上游 SDK（未纳入 git，见 docs/versions.md）
│   ├── unitree_ros2/         # 官方 ROS2 接口（CycloneDDS + 消息 + 示例）
│   ├── unitree_sdk2/         # C++ SDK（x86_64 + aarch64）
│   └── unitree_sdk2_python/  # Python SDK
├── docs/                     # 文档与笔记
├── scripts/                  # 环境脚本
└── .vscode/                  # VSCode 任务
```

## 快速开始

```bash
# 1. 编译
cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build

# 2. 环境
source scripts/setup_env.sh        # 在仓库根目录

# 3. 运行
ros2 launch go2_description gazebo.launch.py             # Gazebo 站姿
ros2 launch go2_description gazebo.launch.py walk:=true  # Gazebo 走路
ros2 launch go2_description display.launch.py            # RViz2 + 关节滑块
```

VSCode 里也可以直接用 `.vscode/tasks.json` 里的任务（Ctrl+Shift+B / 任务面板）。

## 仿真实现说明

- `go2_description.urdf` 为原始模型；Gazebo 启动时动态注入：
  - `file://` 网格绝对路径（避免 package:// 解析问题）
  - `<ros2_control>` 12 关节 position 命令接口 + `gz_ros2_control` 插件
  - 足端摩擦参数（mu=1.5）
  - PosePublisher（给步态节点提供航向反馈）
- 步态：`scripts/go2_trot.py` 对角小跑，2 连杆 IK + 椭圆足端轨迹 + 偏航差速修正
- 控制器参数：`config/go2_controllers.yaml`（位置环 P 增益 10）

## Roadmap

- [x] URDF 导入 ROS2，RViz2 可视化
- [x] Gazebo 站姿（ros2_control）
- [x] 开环 trot 行走 + 航向稳定
- [ ] 步态提速 / 稳定性优化
- [ ] 真机：CycloneDDS + unitree_ros2 / unitree_sdk2_python
- [ ] Go2 机载 Jetson（aarch64）部署
