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
│       ├── go2_description/  # Go2 模型/仿真/步态包
│   ├── vision_sim/       # 独立视觉仿真验证环境（D435i + 移动目标）
│   └── go2_vision/       # 狗载视觉：检测 + 相机/雷达融合定位 + 评估
├── third_party/              # 上游 SDK / 社区包（未纳入 git，见 docs/versions.md）
│   ├── unitree_ros2/         # 官方 ROS2 接口（CycloneDDS + 消息 + 示例）
│   ├── unitree_sdk2/         # C++ SDK（x86_64 + aarch64）
│   ├── unitree_sdk2_python/  # Python SDK
│   ├── unitree_ros/          # ROS1 描述包（仅 go2_description）
│   └── go2_ros2_sdk/         # 社区 WebRTC ROS2 桥（相机/点云/scan）
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
ros2 launch go2_description gazebo.launch.py sensors:=true rviz:=true  # 狗载相机+雷达
ros2 launch go2_description display.launch.py            # RViz2 + 关节滑块

# 4. 纯视觉仿真（独立于四足本体）
ros2 launch vision_sim vision_sim.launch.py              # D435i + 移动目标 + 3D 定位
```

VSCode 里也可以直接用 `.vscode/tasks.json` 里的任务（Ctrl+Shift+B / 任务面板）。
完整启动方式（命令行 / VSCode 任务 / 全部 launch 参数）见 `docs/running.md`。

## 仿真实现说明

- `go2_description.urdf` 为原始模型；Gazebo 启动时动态注入：
  - `file://` 网格绝对路径（避免 package:// 解析问题）
  - `<ros2_control>` 12 关节 position 命令接口 + `gz_ros2_control` 插件
  - 足端摩擦参数（mu=1.5）
  - OdometryPublisher（`/model/go2/odometry`，给步态节点提供位姿/航向反馈）
- 步态：`scripts/go2_trot.py` 对角小跑，2 连杆 IK + 椭圆足端轨迹 + 偏航修正；
  订阅 `/cmd_vel`，可用 `scripts/go2_teleop.py` 键盘遥控
- 控制器参数：`config/go2_controllers.yaml`（位置环 P 增益 10）
- 狗载传感器（`sensors:=true`）：前视相机（社区 720p 标定参数）+ `gpu_lidar`
  雷达占位；世界 `worlds/go2_sim.sdf` 自带 Sensors 系统

## 视觉仿真验证环境（vision_sim）

剥离四足本体、单独验证视觉链路：固定 D435i 相机 + 移动目标，跑通
图像采集 → 运动目标检测 → 三维坐标解算，并与真值对比。
详见 `ros2_ws/src/vision_sim/README.md`（首版实测：检测 0 漏检，3D 定位误差 mean≈1.2cm）；
进度与交接备忘见 `docs/status.md`。

## Roadmap

- [x] URDF 导入 ROS2，RViz2 可视化
- [x] Gazebo 站姿（ros2_control）
- [x] 开环 trot 行走 + 航向稳定
- [x] 独立视觉仿真环境（检测 + 3D 定位 + 真值对比）
- [x] 狗载传感器仿真（前视相机 + L1 雷达占位）
- [x] 狗载视觉：红球检测 + 相机/雷达融合定位（运动目标 rms 2.6cm）
- [ ] 步态提速 / 稳定性优化
- [ ] 视觉伺服：远距导航 + 近距对准（仿真）
- [ ] 真机：CycloneDDS + unitree_ros2 / unitree_sdk2_python
- [ ] Go2 机载 Jetson（aarch64）部署
