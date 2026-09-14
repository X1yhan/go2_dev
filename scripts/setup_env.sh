#!/usr/bin/env bash
# 用法：source scripts/setup_env.sh

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="$(dirname "$script_dir")"

source /opt/ros/humble/setup.bash

if [ -f "$root_dir/ros2_ws/install/setup.bash" ]; then
    source "$root_dir/ros2_ws/install/setup.bash"
else
    echo "[setup_env] 工作空间尚未构建，请先执行：cd ros2_ws && colcon build" >&2
fi
