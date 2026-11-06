#!/usr/bin/env bash
# 分层的三条铁律，用 grep 检查。违反则退出码 1。
#
# 用法:  scripts/check_layers.sh          (在仓库根目录或任意目录下都能跑)
#
# 这三条不是审美问题，每一条都对应一个具体的后果：
#   铁律 1 破了 -> 中文解析要拉着 ROS + MoveIt + OpenCV 才能跑，改一句话得先起仿真
#   铁律 2 破了 -> 检测逻辑开始依赖机器人模型，换相机就得动规划代码
#   铁律 3 破了 -> 规划依赖散到各处，"唯一出口"消失，回退 MoveIt 版本变成大海捞针
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2
SCRIPTS="ros2_ws/src/go2_description/scripts"
PKG="$SCRIPTS/arm_bench"

# 只匹配真正的 import 行：注释和 docstring 里出现 moveit_msgs 不算违反。
IMPORT_RE='^[[:space:]]*(from|import)[[:space:]]+[A-Za-z_.]*(moveit_msgs|rclpy|cv2|tf2_ros|control_msgs|numpy)'
MV_RE='^[[:space:]]*(from|import)[[:space:]]+moveit_msgs'

fail=0
report() { # $1=ok|bad  $2=说明
    if [ "$1" = ok ]; then printf '  OK    %s\n' "$2"
    else printf '  BAD   %s\n' "$2"; fail=1; fi
}

echo "铁律 1  任务层(L5) 不直接依赖 MoveIt / OpenCV / 机器人模型"
hits=$(grep -nE "$IMPORT_RE" "$SCRIPTS/arm_bench_language.py" "$PKG/language.py" 2>/dev/null)
if [ -z "$hits" ]; then report ok "L5 没有 import 重依赖"
else report bad "L5 直接 import 了重依赖:"; echo "$hits"; fi

hits=$(grep -nE '^[[:space:]]*(from|import)[[:space:]]+arm_bench\.(grasp|skills|vision|world|detection|scene)' \
        "$SCRIPTS/arm_bench_language.py" "$PKG/language.py" 2>/dev/null)
if [ -z "$hits" ]; then report ok "L5 只 import arm_bench.language / arm_bench.objects"
else report bad "L5 越层 import:"; echo "$hits"; fi

echo "铁律 2  感知层(L2) 不依赖 moveit_msgs"
hits=$(grep -rnE "$MV_RE" \
        "$PKG/detection.py" "$PKG/vision.py" \
        "$SCRIPTS/arm_bench_object_detector.py" \
        ros2_ws/src/go2_vision/scripts/ 2>/dev/null)
if [ -z "$hits" ]; then report ok "L2 干净（detection.py / vision.py / detector / go2_vision）"
else report bad "L2 里出现 moveit_msgs:"; echo "$hits"; fi

echo "铁律 3  moveit_msgs 只出现在 L3 与 L4 的固定文件里"
ALLOWED="$PKG/world.py $PKG/skills.py $SCRIPTS/arm_bench_planning_scene.py $SCRIPTS/grasp_ball.py $SCRIPTS/chase_grasp.py"
hits=$(grep -rlE "$MV_RE" --include=*.py ros2_ws/src/ 2>/dev/null)
unexpected=""
for f in $hits; do
    ok=0
    for a in $ALLOWED; do [ "$f" = "$a" ] && ok=1; done
    [ "$ok" = 0 ] && unexpected="$unexpected $f"
done
if [ -z "$unexpected" ]; then
    report ok "只有 5 个文件碰 moveit_msgs（L3 world/planning_scene，L4 skills + 两个旧脚本）"
else
    report bad "这些地方也 import 了 moveit_msgs:$unexpected"
fi

echo "附加    arm_bench 的纯函数层不碰 ROS / OpenCV"
PURE="$PKG/geometry.py $PKG/scene.py $PKG/objects.py $PKG/language.py $PKG/constants.py"
hits=$(grep -nE "$IMPORT_RE" $PURE 2>/dev/null)
if [ -z "$hits" ]; then report ok "纯函数层（geometry / scene / objects / language / constants）可离线跑"
else report bad "纯函数层碰了重依赖:"; echo "$hits"; fi

if [ "$fail" = 0 ]; then
    echo; echo "三条铁律都在。"
else
    echo; echo "有违反项，见上面标 BAD 的行。"
fi
exit "$fail"
