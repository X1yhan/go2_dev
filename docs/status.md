# 项目状态备忘(2026-09-20)

> 用途:跨会话交接。下次开工先读这份,再继续。

## 已完成

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

1. **[关机重启后] 修 NVIDIA 驱动**:关机前 `nvidia-smi` 报 "Driver/library version mismatch"
   (内核模块 595.84 vs 用户态 595.91),重启后先确认 `nvidia-smi` 正常。
2. **给 vision_sim 加 `use_nvidia:=true` 开关**(on-demand 模式):
   GUI 用 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`,
   无头用 `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`;
   再做核显 vs 独显相机帧率/RTF 对比,数据决定后续。
3. **加严仿真**:深度噪声/丢失、图像链路延迟注入(让误差接近真机)。
4. **检测器升级**:YOLO/TensorRT(独显的主战场),评估真实检测误差。
5. **迁移到 Go2**:相机/雷达挂到狗身(front_camera 复用 D435i 方案,再补
   `gpu_lidar` 对应 `utlidar_lidar`),然后做视觉伺服。
6. **视觉伺服路线(已定)**:SCOUT(远距目标记忆/导航)→ APPROACH(近距视觉伺服对准)
   → DOCK(固定距离+朝向停);机械臂到货后扩展 6D 位姿 + 抓取。
   感知分三阶段调试:①完美感知(真值投影)②真实检测器 ③注入延迟/噪声。
7. (可选)Gazebo GUI 视角自动回位问题:多为右键误开 Follow,取消勾选即可;
   老出问题就给 world 加 `<gui>` 固定初始视角并去掉跟踪插件。

## 常用命令

```bash
cd ~/go2_dev/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch vision_sim vision_sim.launch.py                    # 全链路 + GUI + RViz
ros2 launch vision_sim vision_sim.launch.py pipeline:=false visualize:=true   # 只看相机
ros2 launch vision_sim vision_sim.launch.py trajectory:=circle speed:=0.7
ros2 run rqt_image_view rqt_image_view /viz/camera/image       # 看彩色画面
/tmp/opencode/kill_vision.sh                                    # 清理测试进程(注意会杀掉其它 Gazebo 实例)
```

## 坑与约定(踩过的)

- RealSense URDF 的光学系 frame 只在 `use_nominal_extrinsics:=true` 时生成。
- 相机 spawn 为静态模型:launch 里 URDF→SDF(`ign sdf -p`)后注入 `<static>true</static>`。
- 相机默认位姿 `(0,0,0.8)`,pitch `+0.26`(低头),640x360@30;球在 `(1.5, y(t), 0.15)`。
- Gazebo 里 `<visualize>true</visualize>` 的悬浮窗对 rgbd 相机显示的是深度(灰度),
  看彩色要用 rqt_image_view / RViz。
- 清理进程别用 `pkill -f "vision_sim.launch"` 直接写在命令里(会自匹配杀掉自己的 shell),
  用 `/tmp/opencode/kill_vision.sh`(模式放脚本里)或 pgrep+变量。
- Gitee 不支持 SSH 建 issue,必须走 OpenAPI + 私人令牌。
