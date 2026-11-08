#!/usr/bin/env python3
"""Move the standalone arm bench to its ready pose on startup.

Arm: RM75 `forward` state from rm_75_config.
Gripper: open (if gripper_controller is available).
"""
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
FORWARD = [0.0, 0.8395, 0.0, 1.4442, 0.0, 0.8513, 0.0]
OPEN = 0.09


class Home(Node):
    def __init__(self):
        super().__init__('arm_bench_home')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/rm_group_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory')

    def send(self, client, joints, positions, seconds):
        if not client.wait_for_server(timeout_sec=20.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = seconds
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        deadline = time.time() + seconds + 15.0
        while time.time() < deadline and not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return result.done()


def main():
    rclpy.init()
    node = Home()
    ok = node.send(node.arm, ARM_JOINTS, FORWARD, 4)
    node.get_logger().info('arm forward pose: %s' % ('ok' if ok else 'failed'))
    if node.grip.wait_for_server(timeout_sec=3.0):
        node.send(node.grip, ['gripper_finger1_joint',
                              'gripper_finger2_joint'], [OPEN, OPEN], 2)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
