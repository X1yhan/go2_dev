# vision_sim — 独立视觉仿真验证环境

剥离四足本体的纯视觉链路仿真:一台固定的 Intel RealSense D435i + 移动目标,
跑通 **图像采集 → 运动目标检测 → 三维坐标解算 → 与真值对比** 全链路。

```
Gazebo rgbd_camera ──ros_gz bridge──▶ 检测(颜色/HSV) ──▶ 深度融合/反投影 ──▶ 3D 坐标
        │                                                                      │
        └── target_director(轨迹 + ground truth) ──────▶ eval_monitor(误差对比) ◀┘
```

## 依赖

- ROS2 Humble + Gazebo Fortress + ros_gz(bridge/sim/image/interfaces)
- `realsense2_description`(apt 安装,提供 D435i 模型/TF 系)
- cv_bridge / python3-opencv / numpy / xacro / tf2_ros

## 编译与运行

```bash
cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select vision_sim
source install/setup.bash

ros2 launch vision_sim vision_sim.launch.py                    # Gazebo GUI + RViz2
ros2 launch vision_sim vision_sim.launch.py use_gui:=false rviz:=false  # 无界面
ros2 launch vision_sim vision_sim.launch.py pipeline:=false visualize:=true  # 只看相机

# 实验参数
ros2 launch vision_sim vision_sim.launch.py trajectory:=circle speed:=0.7
ros2 launch vision_sim vision_sim.launch.py camera_z:=0.6 camera_pitch:=0.35
```

常用开关:`pipeline:=false` 只启动相机链路(不起检测/定位/评估节点),
`visualize:=true` 在 Gazebo 窗口里叠加显示相机画面,`rviz:=false` 不启动 RViz。

## 话题 / 节点

| 话题 | 说明 |
| --- | --- |
| `/viz/camera/image` `/depth_image` `/camera_info` | Gazebo RGBD 传感器(经 bridge) |
| `/target/ground_truth` | 目标真值位姿(世界系,`target_director`) |
| `/target/detections` | 2D 检测框(`TargetDetection` 自定义消息) |
| `/target/position_camera` | 目标 3D 坐标(相机光学系) |
| `/target/position_world` | 目标 3D 坐标(世界系,经 TF) |
| `/target/error` `/target/error_stats` | 与真值的误差(即时/统计) |
| `/target/debug_image` | 画了检测框的调试图像 |

节点:`target_director`(运动导演)、`target_detector`(检测)、
`target_localizer`(3D 定位)、`eval_monitor`(精度评估)。

## 关键参数

- 相机:`urdf/d435i_vision.urdf.xacro` 里 FOV/分辨率/帧率/内外参,
  launch 参数 `width/height/fps/hfov` 可覆盖(默认 640x360@30、HFOV 87°,对齐 D435i 深度规格)。
- 检测:`config/vision_params.yaml` 里 HSV 阈值、最小面积。
- 定位:`patch_fraction`(取 bbox 中心区域深度)、`target_radius`(球体目标补半径到球心)。
- 轨迹:`static|line|sine|circle` + `speed/amplitude/radius/center_x/center_y`。
  目标通过 `set_pose` 服务 teleport,轨迹精确可复现。

## 首版验证结果(headless,Gazebo 服务器)

- 相机 30 fps 稳定,图像/深度/camera_info 全部桥接成功。
- 检测:2400/2400 帧命中(红色球,无漏检、无误检)。
- 定位:2500 样本 0 失败,`sine` 轨迹 0.5 m/s 下
  **误差 mean ≈ 1.2 cm / rms ≈ 1.3 cm / max ≈ 2.4 cm**。
  残余误差主要来自图像到估计的时间滞后(0.5 m/s × ~25 ms ≈ 1.2 cm),与实测吻合。

## 已知简化(后续可加严)

- Sim 的 `rgbd_camera` 彩色与深度共用一套内参(真机 D435i 彩色 69.4°、深度 87°);
  对齐深度->彩色的真实流程后续可用双 sensor 分解。
- 当前深度无噪声、无丢失;真机链路延迟要单独注入模拟(见 M1/M2 计划)。
- 运动目标用 teleport,不模拟刚体动力学/滚动。
- `libEGL warning: egl: failed to create dri2 screen` 在无头环境出现,但不影响渲染。

## 目录

```
vision_sim/
├── urdf/d435i_vision.urdf.xacro   # D435i 模型 + rgbd_camera 传感器注入
├── worlds/vision_bench.sdf        # 地面/光照/移动目标
├── scripts/                       # 四个节点
├── launch/vision_sim.launch.py    # xacro→SDF(static)→spawn + 桥接 + TF
├── config/vision_params.yaml
├── rviz/vision_sim.rviz
└── msg/TargetDetection.msg
```
