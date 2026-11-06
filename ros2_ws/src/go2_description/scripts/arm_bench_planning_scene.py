#!/usr/bin/env python3
"""Populate the MoveIt planning scene for interactive arm bench planning.

Adds the table and the four tabletop objects in base_link coordinates so the
MotionPlanning panel checks collisions against the simulated world.  Object
poses come from Gazebo odometry when available; otherwise the known world
layout is used.
"""
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive

BASE_FRAME = 'base_link'

OBJECTS = {
    'target_ball': {
        'model': 'target_ball', 'shape': 'sphere', 'dims': [0.035],
        'default': (0.22, 0.0, 0.035),
    },
    'blue_cube': {
        'model': 'blue_cube', 'shape': 'box', 'dims': [0.07, 0.07, 0.07],
        'default': (0.26, -0.16, 0.035),
    },
    'green_cylinder': {
        'model': 'green_cylinder', 'shape': 'cylinder', 'dims': [0.10, 0.04],
        'default': (0.14, 0.18, 0.05),
    },
    'yellow_box': {
        'model': 'yellow_box', 'shape': 'box',
        'dims': [0.10, 0.075, 0.07],
        'default': (0.25, 0.16, 0.035),
    },
}


def _pose(xyz, quat=None):
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    if quat is None:
        pose.orientation.w = 1.0
    else:
        pose.orientation = quat
    return pose


def _collision_object(name, shape, dims, pose):
    obj = CollisionObject()
    obj.header.frame_id = BASE_FRAME
    obj.id = name
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    if shape == 'sphere':
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(dims[0])]
    elif shape == 'cylinder':
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(dims[0]), float(dims[1])]
    else:
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(v) for v in dims]
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj


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
            for name, cfg in OBJECTS.items():
                self.create_subscription(
                    Odometry, '/model/%s/odometry' % cfg['model'],
                    lambda msg, key=name: self._on_odom(key, msg), 10)
        self.timer = self.create_timer(1.0, self._publish)

    def _on_odom(self, name, msg):
        p = msg.pose.pose.position
        self.odom_poses[name] = _pose(
            (p.x, p.y, p.z - self.base_z_offset),
            msg.pose.pose.orientation)

    def _waiting_for_odom(self):
        if not self.use_odom:
            return False
        if len(self.odom_poses) == len(OBJECTS):
            return False
        return time.monotonic() - self.started < self.grace_period

    def _build_scene(self):
        objects = [
            _collision_object('table', 'box', [0.6, 0.6, 0.6],
                              _pose((0.0, 0.0, -0.3)))
        ]
        for name, cfg in OBJECTS.items():
            pose = self.odom_poses.get(name)
            if pose is None:
                pose = _pose(cfg['default'])
            objects.append(_collision_object(
                cfg['model'], cfg['shape'], cfg['dims'], pose))
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        return scene

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
