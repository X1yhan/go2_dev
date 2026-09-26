#!/usr/bin/env bash
# 清理 go2_dev 仿真相关进程。
#
# 用法:  scripts/kill_sim.sh          (TERM,等 2 秒)
#        scripts/kill_sim.sh --force  (直接 KILL)
#
# 注意: 会杀掉本机所有 Gazebo / ros_gz / RViz 实例(vision_sim 也会被清掉);
#        模式写在脚本里,避免在命令行里 pkill -f 自匹配杀掉当前 shell。

set -u

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

PATTERNS=(
    "ign gazebo"
    "robot_state_publisher"
    "parameter_bridge"
    "ros_gz_sim"
    "static_transform_publisher"
    "go2_trot.py"
    "go2_teleop.py"
    "rviz2"
    "controller_manager"
    "go2_vision"
    "move_group"
    "arm_moveit.launch.py"
)

kill_matching() {
    local sig="$1"
    for pat in "${PATTERNS[@]}"; do
        pkill "-$sig" -f "$pat" 2>/dev/null
    done
}

if [ "$FORCE" -eq 1 ]; then
    kill_matching 9
else
    kill_matching TERM
    sleep 2
fi

left=$(pgrep -f "ign gazebo|robot_state_publisher|parameter_bridge|ros_gz_sim" | wc -l)
echo "[kill_sim] remaining processes: $left"
