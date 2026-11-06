#!/usr/bin/env python3
"""Populate the MoveIt planning scene for interactive arm bench planning.

Adds the table and the four tabletop objects in base_link coordinates so the
MotionPlanning panel checks collisions against the simulated world.  Object
poses come from Gazebo odometry when available; otherwise the known world
layout is used.

The object table and the CollisionObject builders are shared with the grasp
node (`arm_bench/objects.py`, `arm_bench/world.py`), so the two can no
longer drift apart.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from moveit_msgs.srv import ApplyPlanningScene  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from arm_bench import world  # noqa: E402
from arm_bench.constants import BASE_FRAME  # noqa: E402
from arm_bench.objects import OBJECTS  # noqa: E402


class ArmBenchPlanningScene(Node):

    def __init__(self):
        super().__init__(
            'arm_bench_planning_scene',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('use_odom', True)
        self.declare_parameter('base_z_offset', 0.6)
        self.declare_parameter('odom_grace_period', 5.0)
        self.use_odom = bool(self.get_parameter('use_odom').value)
        self.base_z_offset = float(
            self.get_parameter('base_z_offset').value)
        self.grace_period = float(
            self.get_parameter('odom_grace_period').value)

        self.started = time.monotonic()
        self.odom_poses = {}
        self.pending = False
        self.done = False

        self.apply_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene')
        if self.use_odom:
            for label, cfg in OBJECTS.items():
                self.create_subscription(
                    Odometry, '/model/%s/odometry' % cfg['model'],
                    lambda msg, key=label: self._on_odom(key, msg), 10)
        self.timer = self.create_timer(1.0, self._publish)

    def _on_odom(self, label, msg):
        position = msg.pose.pose.position
        self.odom_poses[label] = world.pose(
            (position.x, position.y, position.z - self.base_z_offset),
            msg.pose.pose.orientation)

    def _waiting_for_odom(self):
        if not self.use_odom:
            return False
        if len(self.odom_poses) == len(OBJECTS):
            return False
        return time.monotonic() - self.started < self.grace_period

    def _build_scene(self):
        objects = [world.table_object(BASE_FRAME)]
        for label, cfg in OBJECTS.items():
            pose = self.odom_poses.get(label)
            if pose is None:
                pose = world.pose(cfg['default'])
            objects.append(world.collision_object(
                cfg['model'], cfg['shape'], cfg['dims'], pose, BASE_FRAME))
        return world.diff_scene(objects)

    def _publish(self):
        if self.done or self.pending:
            return
        if self._waiting_for_odom():
            return
        if not self.apply_client.service_is_ready():
            return
        self.pending = True
        request = ApplyPlanningScene.Request()
        request.scene = self._build_scene()
        future = self.apply_client.call_async(request)
        future.add_done_callback(self._on_applied)

    def _on_applied(self, future):
        self.pending = False
        try:
            response = future.result()
            ok = bool(response is not None and response.success)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('apply_planning_scene failed: %s' % exc)
            return
        if not ok:
            self.get_logger().warn(
                'apply_planning_scene rejected, will retry')
            return
        self.done = True
        self.timer.cancel()
        names = ', '.join(cfg['model'] for cfg in OBJECTS.values())
        source = 'odometry' if self.odom_poses else 'world defaults'
        self.get_logger().info(
            'planning scene ready: table + %s (%s)' % (names, source))


def main():
    rclpy.init()
    node = ArmBenchPlanningScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
