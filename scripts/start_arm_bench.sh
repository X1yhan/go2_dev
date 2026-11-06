#!/usr/bin/env bash
# One-shot launcher: arm bench sim + MoveIt + object detector + language shell.
#
# Usage:
#   scripts/start_arm_bench.sh                 # Gazebo GUI + RViz + camera window
#   scripts/start_arm_bench.sh --headless      # no GUI/RViz/camera window
#   scripts/start_arm_bench.sh --moveit-rviz   # MoveIt RViz instead of arm_bench RViz
#   scripts/start_arm_bench.sh --interactive   # MoveIt RViz + collision scene only
#   scripts/start_arm_bench.sh --no-build      # skip colcon build
#
# The language shell runs in the foreground; type "结束了" to exit and stop
# all background processes.  --interactive skips the detector/language shell
# and waits until Ctrl+C, so the MotionPlanning panel can be used directly.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$ROOT_DIR/ros2_ws"
SIM_LOG="/tmp/arm_bench_sim.log"
MOVEIT_LOG="/tmp/arm_bench_moveit.log"
DETECTOR_LOG="/tmp/arm_bench_detector.log"
SCENE_LOG="/tmp/arm_bench_scene.log"

HEADLESS=0
DO_BUILD=1
MOVEIT_RVIZ=0
INTERACTIVE=0
for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=1 ;;
        --no-build) DO_BUILD=0 ;;
        --moveit-rviz) MOVEIT_RVIZ=1 ;;
        --interactive) INTERACTIVE=1 ;;
        -h|--help)
            sed -n '2,14p' "$0"
            exit 0
            ;;
        *)
            echo "unknown option: $arg (try --help)" >&2
            exit 2
            ;;
    esac
done

if [ "$INTERACTIVE" = 1 ] && [ "$HEADLESS" = 1 ]; then
    echo "--interactive needs a display; do not combine it with --headless" >&2
    exit 2
fi

SIM_PID=""
MOVEIT_PID=""
DET_PID=""
SCENE_PID=""

cleanup() {
    trap - EXIT INT TERM
    if [ -n "$SCENE_PID" ]; then
        kill "$SCENE_PID" 2>/dev/null || true
    fi
    if [ -n "$DET_PID" ]; then
        kill "$DET_PID" 2>/dev/null || true
    fi
    if [ -n "$MOVEIT_PID" ]; then
        kill "$MOVEIT_PID" 2>/dev/null || true
    fi
    if [ -n "$SIM_PID" ]; then
        kill "$SIM_PID" 2>/dev/null || true
    fi
    sleep 1
    "$ROOT_DIR/scripts/kill_sim.sh" --force >/dev/null 2>&1 || true
    pkill -f arm_bench_object_detector 2>/dev/null || true
    pkill -f arm_bench_visual_grasp 2>/dev/null || true
    pkill -f arm_bench_planning_scene 2>/dev/null || true
    echo "[start_arm_bench] stopped"
}
trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
if [ "$DO_BUILD" = 1 ]; then
    echo "[start_arm_bench] building go2_description ..."
    (cd "$WS_DIR" && colcon build --packages-select go2_description \
        --symlink-install) || {
        echo "[start_arm_bench] build failed" >&2
        exit 1
    }
fi
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "[start_arm_bench] workspace not built: $WS_DIR/install/setup.bash" >&2
    exit 1
fi
source "$WS_DIR/install/setup.bash"
if [ ! -f "$HOME/rm_ws/install/setup.bash" ]; then
    echo "[start_arm_bench] rm_ws not built: ~/rm_ws/install/setup.bash" >&2
    exit 1
fi
source "$HOME/rm_ws/install/setup.bash"

rm -f /tmp/arm_bench_moveit.urdf /tmp/arm_bench_moveit.srdf

if [ "$INTERACTIVE" = 1 ]; then
    MOVEIT_RVIZ=1
fi

SIM_ARGS=(use_gui:=true rviz:=true view:=true)
MOVEIT_ARGS=(rviz:=false)
if [ "$HEADLESS" = 1 ]; then
    SIM_ARGS=(use_gui:=false rviz:=false view:=false)
elif [ "$MOVEIT_RVIZ" = 1 ]; then
    SIM_ARGS=(use_gui:=true rviz:=false view:=true)
    MOVEIT_ARGS=(rviz:=true)
fi

echo "[start_arm_bench] starting Gazebo/arm bench ..."
ros2 launch go2_description arm_bench.launch.py "${SIM_ARGS[@]}" \
    >"$SIM_LOG" 2>&1 &
SIM_PID=$!

for _ in $(seq 1 40); do
    [ -f /tmp/arm_bench_moveit.urdf ] && break
    sleep 1
done
if [ ! -f /tmp/arm_bench_moveit.urdf ]; then
    echo "[start_arm_bench] assembled URDF was not written" >&2
    tail -20 "$SIM_LOG" >&2
    exit 1
fi

echo "[start_arm_bench] starting MoveIt ..."
ros2 launch go2_description arm_moveit.launch.py "${MOVEIT_ARGS[@]}" \
    >"$MOVEIT_LOG" 2>&1 &
MOVEIT_PID=$!

echo "[start_arm_bench] waiting for MoveIt /move_action ..."
for _ in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -q '/move_action'; then
        break
    fi
    sleep 1
done

echo "[start_arm_bench] waiting for rm_group_controller ..."
for _ in $(seq 1 60); do
    if ros2 action list 2>/dev/null | \
            grep -q '/rm_group_controller/follow_joint_trajectory'; then
        break
    fi
    sleep 1
done

if [ "$INTERACTIVE" = 1 ]; then
    echo "[start_arm_bench] populating MoveIt collision scene ..."
    ros2 run go2_description arm_bench_planning_scene.py \
        >"$SCENE_LOG" 2>&1 &
    SCENE_PID=$!
    sleep 2
    echo "[start_arm_bench] logs: $SIM_LOG $MOVEIT_LOG $SCENE_LOG"
    echo "[start_arm_bench] interactive MoveIt RViz ready:"
    echo "  MotionPlanning -> Planning Group rm_group"
    echo "  drag the Goal State marker, then Plan / Execute"
    echo "  Ctrl+C stops Gazebo, MoveIt and RViz"
    wait "$SIM_PID"
    exit 0
fi

echo "[start_arm_bench] starting object detector ..."
ros2 run go2_description arm_bench_object_detector.py \
    >"$DETECTOR_LOG" 2>&1 &
DET_PID=$!
sleep 2

echo "[start_arm_bench] logs: $SIM_LOG $MOVEIT_LOG $DETECTOR_LOG"
echo "[start_arm_bench] language shell (输入“结束了”退出)"
ros2 run go2_description arm_bench_language.py
