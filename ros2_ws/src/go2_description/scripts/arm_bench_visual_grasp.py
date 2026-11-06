#!/usr/bin/env python3
"""Object-selective visual grasp on the standalone arm bench.

Flow:
    move to observation pose -> run the color/shape object detector ->
    pick the detection whose label matches `target_class` -> compute its 3D
    center in base_link -> MoveIt to pre-grasp -> Cartesian descend ->
    close gripper -> Cartesian lift -> verify the target odometry z.

Requires:
    arm_bench.launch.py + arm_moveit.launch.py + arm_bench_object_detector.py

Usage:
    ros2 run go2_description arm_bench_visual_grasp.py \
        --ros-args -p target_class:=blue_cube
    ros2 run go2_description arm_bench_visual_grasp.py \
        --ros-args -p target_class:=green_cylinder -p detect_only:=true

This file is now only the entry point.  The implementation lives in the
`arm_bench` package next to it:

    arm_bench/vision.py    camera frames + detections -> base_link points
    arm_bench/scene.py     fused observations of what is on the table
    arm_bench/world.py     collision scene published to MoveIt
    arm_bench/skills.py    move_to / cartesian / grip, the only MoveIt user
    arm_bench/grasp.py     the node, its parameters, and the task script
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm_bench.grasp import VisualGrasp, main  # noqa: E402,F401

if __name__ == '__main__':
    sys.exit(main())
