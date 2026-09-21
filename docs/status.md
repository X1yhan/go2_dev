# 项目状态备忘(2026-09-21)

> 用途:跨会话交接。下次开工先读这份,再继续。

## 已完成

### 狗载传感器仿真(2026-09-21)
- `ros2 launch go2_description gazebo.launch.py sensors:=true [rviz:=true]`
- 相机:1280x720@30,Gazebo GUI 弹窗/RViz 可见;雷达:16x720@10Hz 点云+地面回波,
  与 vision_sim 的 RGBD 链路不同,这里直接用 Gazebo 原生 sensor
- 关键改动:`gazebo.launch.py`(sensors/rviz/lidar_x,y,z/gz_world_name 参数、
  OpaqueFunction 组装带传感器的 URDF、gz→ROS 传感器桥、gz 前缀 frame 的静态 TF)、
  `worlds/go2_sim.sdf`(empty.sdf + Sensors 系统 + 红箱/蓝柱)、`rviz/sensors.rviz`
- 已知:官方 URDF `radar` 点在头部网格内部,雷达默认装头顶(xyz 0.28945/0/0.10,可调);
  L1 量程/FOV/帧率仍是占位(360°x90°、30m、10Hz)
- 社区 go2_ros2_sdk 已入 `third_party/go2_ros2_sdk`(含相机标定、D435i 挂载、真机
  WebRTC 桥),小文件备份在 `docs/reference/`

### NVIDIA 驱动(2026-09-21)
- 重启后已恢复正常:`nvidia-smi` 595.91.07,内核模块与用户态版本一致

### 仿真狗遥控(2026-09-21)
- 步态节点 `go2_trot.py` 订阅 `/cmd_vel`(收到指令前保持自动前进,零指令=站立);
  `go2_teleop.py` 键盘遥控(w/s 前后,a/d/q/e 转向,k 停,±调速)
- 位姿反馈从 PosePublisher 换成 **OdometryPublisher**(`/model/go2/odometry`,
  nav_msgs/Odometry):gz 6.18 的 PosePublisher 只广播话题不发消息
- 实测:前进 ~0.13 m/s、转向 ~0.5 rad/s(开环,指令跟踪约 50%),航向稳定不摔;
  横向平移无效(对角小跑特性,转向前馈+反馈已够用)

### 独立视觉仿真验证环境 `vision_sim`
- 位置:`ros2_ws/src/vision_sim/`(详见包内 README)
- 提交:`5cd6e0a feat: 独立视觉仿真验证环境 vision_sim(...)`(**未 push 到 Gitee**)
- 链路:Gazebo D435i `rgbd_camera` → ros_gz 桥接 → HSV 检测 → 深度中值/反投影 → 相机系+世界系 3D 坐标 → 与真值对比
- headless 实测:相机 30fps 稳定;检测 2400/2400;定位 0 失败;
  sine 0.5 m/s 误差 mean≈1.2cm / rms≈1.3cm / max≈2.4cm(残余主要是 ~25ms 时延)
- 关键文件:`urdf/d435i_vision.urdf.xacro`、`worlds/vision_bench.sdf`、
  `launch/vision_sim.launch.py`、`scripts/{target_director,target_detector,target_localizer,eval_monitor}.py`

### Gitee issue(仓库 X1yhan/go2_dev)
- 规划 issue:#IKG0M1 ~ #IKG0MA(标签/里程碑 M1~M4)
- 本次新增:#IKHCYH(独立视觉仿真环境,含验证数据)
- 已在 IKG0M1、IKG0M4 下留言说明复用关系
- 令牌在 `~/.config/gitee/token`(600),创建 issue 接口用 `POST /v5/repos/{owner}/issues`(repo 走 form 参数)

## 下次待办(按优先级)

1. **给 vision_sim 加 `use_nvidia:=true` 开关**(on-demand 模式):
   GUI 用 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`,
   无头用 `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`;
   再做核显 vs 独显相机帧率/RTF 对比,数据决定后续。
2. **加严仿真**:深度噪声/丢失、图像链路延迟注入(让误差接近真机)。
3. **检测器升级**:YOLO/TensorRT(独显的主战场),评估真实检测误差。
4. **真机传感器数据补齐**:L1 官方规格(量程/FOV/帧率/点频)、前视相机复标、
   真机 `utlidar_lidar` 外参;然后跑 `third_party/go2_ros2_sdk`(WebRTC 相机+点云)。
5. **视觉伺服路线(已定)**:SCOUT(远距目标记忆/导航)→ APPROACH(近距视觉伺服对准)
   → DOCK(固定距离+朝向停);机械臂到货后扩展 6D 位姿 + 抓取。
   感知分三阶段调试:①完美感知(真值投影)②真实检测器 ③注入延迟/噪声。
6. (可选)Gazebo GUI 视角自动回位问题:多为右键误开 Follow,取消勾选即可;
   老出问题就给 world 加 `<gui>` 固定初始视角并去掉跟踪插件。

## 常用命令

```bash
cd ~/go2_dev/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch vision_sim vision_sim.launch.py                    # 全链路 + GUI + RViz
ros2 launch vision_sim vision_sim.launch.py pipeline:=false visualize:=true   # 只看相机
ros2 launch vision_sim vision_sim.launch.py trajectory:=circle speed:=0.7
ros2 run rqt_image_view rqt_image_view /viz/camera/image       # 看彩色画面

ros2 launch go2_description gazebo.launch.py sensors:=true rviz:=true  # 狗载相机+雷达
ros2 run rqt_image_view rqt_image_view /go2/camera/image       # 狗载相机画面
ros2 launch go2_description gazebo.launch.py walk:=true        # 步态节点(接收 /cmd_vel)
ros2 run go2_description go2_teleop.py                         # 键盘遥控(需 walk 已开)

scripts/kill_sim.sh                                            # 清理全部仿真进程(含 Gazebo/RViz)
```

所有命令在仓库根目录执行(已 source `scripts/setup_env.sh`)。

## 坑与约定(踩过的)

- RealSense URDF 的光学系 frame 只在 `use_nominal_extrinsics:=true` 时生成。
- 相机 spawn 为静态模型:launch 里 URDF→SDF(`ign sdf -p`)后注入 `<static>true</static>`。
- 相机默认位姿 `(0,0,0.8)`,pitch `+0.26`(低头),640x360@30;球在 `(1.5, y(t), 0.15)`。
- Gazebo 里 `<visualize>true</visualize>` 的悬浮窗对 rgbd 相机显示的是深度(灰度),
  看彩色要用 rqt_image_view / RViz。
- 清理进程别用 `pkill -f "vision_sim.launch"` 直接写在命令里(会自匹配杀掉自己的 shell);
  用 `scripts/kill_sim.sh`(模式放脚本里)或 pgrep 时用 `[x]` 拆词。
- Gazebo 的 `empty.sdf` 不带 **Sensors 系统**,相机/雷达不会出图;
  go2 仿真用 `worlds/go2_sim.sdf`(empty + Sensors + 参照物)。
- 多个 Gazebo 实例并存时,`ros_gz_sim create` 可能把模型发到别的世界(超时);
  launch 里已加 `-world`,但残留进程仍会干扰,测试前先 `scripts/kill_sim.sh`。
- 官方 URDF 的 `radar` link 在头部网格内部,雷达射线打不出去;仿真雷达默认装
  头顶 `(0.28945, 0, 0.10)`,可用 `lidar_x/lidar_y/lidar_z` 调。
- gz 传感器消息的 frame 带模型前缀(`go2/base/front_camera`、`go2/base/utlidar_lidar`),
  不是 URDF 里的 `front_camera/utlidar_lidar`;launch 里补了同名静态 TF 供 RViz 转换。
- gz 6.18 的 PosePublisher(`publish_model_pose=true`)只广播话题不发消息(踩坑验证),
  已改用 OdometryPublisher → `/model/go2/odometry`(nav_msgs/Odometry,30 Hz)。
- `ign gazebo ... &` 抓到的 `$!` 常常是子 shell/ruby wrapper,杀不干净;清理统一用
  `scripts/kill_sim.sh`,别在命令里直接写要 pkill 的模式(会自匹配杀掉当前 shell)。
- Gitee 不支持 SSH 建 issue,必须走 OpenAPI + 私人令牌。
