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
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints,
                             OrientationConstraint, PlanningScene,
                             PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray
from arm_bench_object_detector import HSV_RANGES
from visualization_msgs.msg import Marker

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
GRIPPER_JOINTS = ['gripper_finger1_joint', 'gripper_finger2_joint']
DOWN_Q = (1.0, 0.0, 0.0, 0.0)   # 180 deg about X: tool +Z points down
DEFAULT_OBSERVE = (0.34, 0.0, 0.32)
SCAN_POSES = [
    (0.34, 0.16, 0.33),
    (0.34, -0.16, 0.33),
    (0.24, 0.20, 0.34),
    (0.24, -0.20, 0.34),
    (0.16, 0.00, 0.36),
    DEFAULT_OBSERVE,
]
SCAN_POSES_2 = [
    (0.30, 0.08, 0.32),
    (0.30, -0.08, 0.32),
    (0.20, 0.12, 0.35),
    (0.20, -0.12, 0.35),
]

BASE_FRAME = 'base_link'
TIP_LINK = 'Link7'
CAMERA_FRAME = 'camera_depth_optical_frame'

# Geometry is in metres and matches worlds/arm_bench.sdf.
OBJECTS = {
    'red_ball': {
        'model': 'target_ball', 'shape': 'sphere',
        'dims': [0.035], 'half_height': 0.035, 'radius': 0.035,
        'grip_offset': 0.17, 'close': 0.0, 'open': 0.05,
    },
    'blue_cube': {
        'model': 'blue_cube', 'shape': 'box',
        'dims': [0.07, 0.07, 0.07], 'half_height': 0.035, 'radius': 0.0,
        'grip_offset': 0.17, 'close': 0.0, 'open': 0.04,
    },
    'green_cylinder': {
        'model': 'green_cylinder', 'shape': 'cylinder',
        'dims': [0.10, 0.04], 'half_height': 0.05, 'radius': 0.0,
        'grip_offset': 0.15, 'close': 0.028, 'open': 0.04,
    },
    'yellow_box': {
        'model': 'yellow_box', 'shape': 'box',
        'dims': [0.10, 0.075, 0.07], 'half_height': 0.035, 'radius': 0.0,
        'grip_offset': 0.17, 'close': 0.025, 'open': 0.04,
    },
}


class VisualGrasp(Node):

    def __init__(self):
        super().__init__(
            'arm_bench_visual_grasp',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('target_class', 'red_ball')
        self.declare_parameter('detections_topic', '/arm/vision/detections')
        self.declare_parameter('ball_radius', 0.035)
        self.declare_parameter('grip_offset', -1.0)
        self.declare_parameter('pre_offset', 0.10)
        self.declare_parameter('lift_offset', 0.18)
        self.declare_parameter('open', 0.06)
        self.declare_parameter('close', -1.0)
        self.declare_parameter('detect_only', False)
        self.declare_parameter('place_class', '')
        self.declare_parameter('place_mode', 'on_top')
        self.declare_parameter('place_offset', 0.03)
        self.declare_parameter('place_clearance', 0.0)
        self.declare_parameter('scan_only', False)

        self.target_class = str(self.get_parameter('target_class').value)
        if self.target_class not in OBJECTS:
            raise RuntimeError(
                'unknown target_class "%s", choose one of %s' %
                (self.target_class, ', '.join(OBJECTS)))
        self.cfg = OBJECTS[self.target_class]
        self.cfg['radius'] = float(self.get_parameter('ball_radius').value) \
            if self.target_class == 'red_ball' else self.cfg['radius']

        grip_param = float(self.get_parameter('grip_offset').value)
        self.grip_offset = grip_param if grip_param >= 0.0 \
            else self.cfg['grip_offset']
        self.pre_offset = float(self.get_parameter('pre_offset').value)
        self.lift_offset = float(self.get_parameter('lift_offset').value)
        open_param = float(self.get_parameter('open').value)
        self.open_q = open_param if open_param >= 0.0 \
            else self.cfg['open']
        close_param = float(self.get_parameter('close').value)
        self.close_q = close_param if close_param >= 0.0 \
            else self.cfg['close']
        self.detect_only = bool(self.get_parameter('detect_only').value)
        self.scan_only = bool(self.get_parameter('scan_only').value)
        self.place_class = str(self.get_parameter('place_class').value)
        self.place_mode = str(self.get_parameter('place_mode').value)
        self.place_offset = float(self.get_parameter('place_offset').value)
        self.place_clearance = float(
            self.get_parameter('place_clearance').value)
        self.place_cfg = None
        if self.place_class:
            if self.place_class not in OBJECTS:
                raise RuntimeError(
                    'unknown place_class "%s"' % self.place_class)
            if self.place_class == self.target_class:
                raise RuntimeError('place_class must differ from target_class')
            if self.place_mode not in ('on_top', 'beside'):
                raise RuntimeError('place_mode must be on_top or beside')
            self.place_cfg = OBJECTS[self.place_class]

        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.camera_info = None
        self.detections = None
        self.seen = {}
        self.first_seen = {}
        self.reference_seen = {}
        self._observations = {}
        self.joint_positions = {}
        self._last_detect_warn = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Image, '/arm/camera/image', self.on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/arm/camera/depth_image', self.on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/arm/camera/camera_info', self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.on_detections, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)

        self.debug_pub = self.create_publisher(
            Image, '/arm/visual_grasp/debug_image', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/arm/visual_grasp/marker', 10)

        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/rm_group_controller/follow_joint_trajectory')
        self.get_logger().info(
            'target_class=%s (model %s, %s)' %
            (self.target_class, self.cfg['model'], self.cfg['shape']))

    def on_color(self, msg):
        self.color = msg

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.camera_info = msg

    def on_detections(self, msg):
        self.detections = msg

    def on_joint_state(self, msg):
        self.joint_positions.update(dict(zip(msg.name, msg.position)))

    def _log_geometry(self, tag):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TIP_LINK, Time(),
                timeout=Duration(seconds=0.2))
            f = self.tf_buffer.lookup_transform(
                BASE_FRAME, 'gripper_finger1_link', Time(),
                timeout=Duration(seconds=0.2))
            g = self.tf_buffer.lookup_transform(
                BASE_FRAME, 'gripper_finger2_link', Time(),
                timeout=Duration(seconds=0.2))
            self.get_logger().info(
                '%s Link7=(%.3f,%.3f,%.3f) f1=(%.3f,%.3f,%.3f) '
                'f2=(%.3f,%.3f,%.3f)' %
                (tag, t.transform.translation.x, t.transform.translation.y,
                 t.transform.translation.z,
                 f.transform.translation.x, f.transform.translation.y,
                 f.transform.translation.z,
                 g.transform.translation.x, g.transform.translation.y,
                 g.transform.translation.z))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('geometry TF failed: %s' % exc)

    def _pose(self, p, q=DOWN_Q):
        pose = Pose()
        pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def send(self, client, goal, success_val):
        if not client.wait_for_server(timeout_sec=20):
            return False
        future = client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        deadline = time.time() + 60.0
        while time.time() < deadline and not result.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not result.done():
            return False
        code = result.result().result.error_code
        return (code.val == success_val) if hasattr(code, 'val') \
            else (code == success_val)

    def move_to(self, target):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'rm_group'
        req.num_planning_attempts = 20
        req.allowed_planning_time = 8.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        region.primitives.append(sphere)
        region.primitive_poses.append(self._pose(target))
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = TIP_LINK
        pc.constraint_region = region
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = TIP_LINK
        oc.orientation.x, oc.orientation.y = DOWN_Q[0], DOWN_Q[1]
        oc.orientation.z, oc.orientation.w = DOWN_Q[2], DOWN_Q[3]
        for axis in ('absolute_x_axis_tolerance',
                     'absolute_y_axis_tolerance',
                     'absolute_z_axis_tolerance'):
            setattr(oc, axis, 0.05)
        oc.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)
        return self.send(self.move, goal, 1)

    def cartesian(self, waypoints, avoid_collisions=True):
        cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        if not cli.wait_for_service(timeout_sec=10):
            return False, 0.0
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = 'rm_group'
        req.link_name = TIP_LINK
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        req.waypoints = [self._pose(w) for w in waypoints]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        res = future.result()
        if res is None:
            return False, 0.0
        traj = res.solution.joint_trajectory
        self.get_logger().info(
            'cartesian fraction %.2f, points %d' %
            (res.fraction, len(traj.points)))
        if res.fraction < 0.85:
            return False, res.fraction
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(traj.joint_names)
        for point in traj.points:
            goal.trajectory.points.append(point)
        goal.trajectory.header = traj.header
        return self.send(self.arm, goal, 0), res.fraction

    def _wait_tcp(self, target, timeout=10.0, tol=0.015):
        last_err = float('inf')
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    BASE_FRAME, TIP_LINK, Time(),
                    timeout=Duration(seconds=0.1))
            except Exception:  # noqa: BLE001
                continue
            p = t.transform.translation
            last_err = math.sqrt((p.x - target[0]) ** 2 +
                                 (p.y - target[1]) ** 2 +
                                 (p.z - target[2]) ** 2)
            if last_err < tol:
                return True
        self.get_logger().warn(
            'TCP settle failed: target %s, error %.3f m' %
            (target, last_err))
        return False

    def grip_to(self, q):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [q, q]
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        return self.send(self.grip, goal, 0)

    def _collision_object(self, name, shape, dims, center):
        obj = CollisionObject()
        obj.header.frame_id = BASE_FRAME
        obj.id = name
        obj.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        if shape == 'sphere':
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [dims[0]]
        elif shape == 'cylinder':
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [dims[0], dims[1]]
        else:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(dims)
        obj.primitives.append(primitive)
        pose = Pose()
        pose.position = Point(x=float(center[0]), y=float(center[1]),
                              z=float(center[2]))
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        return obj

    def set_scene(self):
        cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not cli.wait_for_service(timeout_sec=10):
            return False
        scene = PlanningScene()
        scene.is_diff = True

        table = CollisionObject()
        table.header.frame_id = BASE_FRAME
        table.id = 'table'
        table.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.6, 0.6, 0.6]
        table.primitives.append(box)
        table_pose = Pose()
        table_pose.position.z = -0.3
        table_pose.orientation.w = 1.0
        table.primitive_poses.append(table_pose)

        objects = [table]
        for name, cfg in OBJECTS.items():
            center = self.seen.get(name)
            if center is None:
                continue
            objects.append(self._collision_object(
                cfg['model'], cfg['shape'], cfg['dims'], center))

        scene.world.collision_objects = objects
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = cli.call_async(req)
        deadline = time.time() + 5.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def remove_object(self, name):
        cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not cli.wait_for_service(timeout_sec=5):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        obj = CollisionObject()
        obj.header.frame_id = BASE_FRAME
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [obj]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = cli.call_async(req)
        deadline = time.time() + 3.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def clear_scene(self):
        cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not cli.wait_for_service(timeout_sec=5):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        objects = []
        for name, cfg in OBJECTS.items():
            obj = CollisionObject()
            obj.header.frame_id = BASE_FRAME
            obj.id = cfg['model']
            obj.operation = CollisionObject.REMOVE
            objects.append(obj)
        table = CollisionObject()
        table.header.frame_id = BASE_FRAME
        table.id = 'table'
        table.operation = CollisionObject.REMOVE
        objects.append(table)
        scene.world.collision_objects = objects
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = cli.call_async(req)
        deadline = time.time() + 3.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    @staticmethod
    def _footprint_radius(cfg):
        if cfg['shape'] == 'sphere':
            return cfg['radius']
        if cfg['shape'] == 'cylinder':
            return cfg['dims'][1]
        return max(cfg['dims']) / 2.0

    def _tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(
                BASE_FRAME, TIP_LINK, Time(),
                timeout=Duration(seconds=0.2))
            return t.transform.translation
        except Exception:  # noqa: BLE001
            return None

    def _place_center(self):
        return (self.reference_seen.get(self.place_class) or
                self.first_seen.get(self.place_class) or
                self.seen.get(self.place_class))

    def _free_beside_spot(self, center):
        distance = (self._footprint_radius(self.place_cfg) +
                    self._footprint_radius(self.cfg) + self.place_offset)
        for deg in (0, 90, -90, 45, -45, 135, -135, 180):
            angle = math.radians(deg)
            x = center[0] + distance * math.cos(angle)
            y = center[1] + distance * math.sin(angle)
            held_r = self._footprint_radius(self.cfg)
            if not (0.10 <= x <= 0.29 - held_r and
                    -0.29 + held_r <= y <= 0.29 - held_r):
                continue
            free = True
            for name, p in self.seen.items():
                if name in (self.place_class, self.target_class):
                    continue
                need = (self._footprint_radius(OBJECTS[name]) +
                        self._footprint_radius(self.cfg) + 0.02)
                if math.hypot(x - p[0], y - p[1]) < need:
                    free = False
                    break
            if free:
                return (x, y)
        return None

    def place_object(self):
        center = self._place_center()
        if center is None:
            self.get_logger().error(
                'place target %s is not in the vision scan' %
                self.place_class)
            return False
        if self.place_mode == 'on_top':
            if self.place_cfg['shape'] != 'box':
                self.get_logger().error(
                    'on_top placement needs a box/cube target; use beside')
                return False
            x, y = center[0], center[1]
            top = center[2] + self.place_cfg['half_height']
            release_z = (top + self.cfg['half_height'] +
                         self.grip_offset + self.place_clearance)
            expected_z = top + self.cfg['half_height']
            desc = '%s top' % self.place_class
        else:
            spot = self._free_beside_spot(center)
            if spot is None:
                self.get_logger().error('no free space beside %s'
                                        % self.place_class)
                return False
            x, y = spot
            release_z = (self.cfg['half_height'] +
                         self.grip_offset + self.place_clearance)
            expected_z = self.cfg['half_height']
            desc = '%s beside' % self.place_class
        cur = self._tcp()
        if cur is None:
            self.get_logger().error('no TCP transform for placing')
            return False
        safe_z = max(cur.z, release_z + 0.06, 0.36)
        release = (x, y, release_z)
        self.get_logger().info(
            'placing %s -> %s at base (%.3f, %.3f, %.3f)' %
            (self.target_class, desc, x, y, release_z))
        ok, fraction = self.cartesian([
            (cur.x, cur.y, safe_z), (x, y, safe_z), release])
        if not ok:
            self.get_logger().warn(
                'Cartesian place approach %.2f, fallback MoveIt' % fraction)
            if not self.move_to((x, y, safe_z)):
                self.get_logger().error('place pre-pose failed')
                return False
            ok, fraction = self.cartesian([release])
        if not ok or not self._wait_tcp(release, timeout=10.0, tol=0.015):
            self.get_logger().error(
                'arm did not reach place pose; object still held')
            return False
        if self.cfg['shape'] == 'sphere':
            held_width = 2.0 * self.cfg['radius']
        elif self.cfg['shape'] == 'cylinder':
            held_width = 2.0 * self.cfg['dims'][1]
        else:
            held_width = max(self.cfg['dims'][0], self.cfg['dims'][1])
        release_q = max(0.0, (held_width - 0.012) / 2.0)
        self.get_logger().info(
            'opening gripper to release (q=%.3f, width %.3f)'
            % (release_q, held_width))
        self.grip_to(release_q)
        time.sleep(0.8)
        lift = (x, y, release_z + 0.10)
        ok_lift, fraction = self.cartesian([lift])
        if not ok_lift:
            self.get_logger().warn(
                'place lift Cartesian %.2f, fallback MoveIt' % fraction)
            self.move_to(lift)
        self._wait_tcp(lift, timeout=8.0, tol=0.02)
        time.sleep(1.0)
        self._fresh_frame()
        placed = self.detect_label(self.target_class, table_prior=False,
                                   publish_debug=True)
        if placed is None:
            self.get_logger().warn(
                'placed object not visible here, moving to verify pose')
            if self.move_to(SCAN_POSES[0]) and self._fresh_frame():
                placed = self.detect_label(self.target_class,
                                           table_prior=False)
        if placed is None:
            self.get_logger().error(
                'PLACE unverified: %s not visible after release'
                % self.target_class)
            return False
        p = placed[0]
        err_z = abs(p[2] - expected_z)
        err_xy = math.hypot(p[0] - x, p[1] - y)
        success = err_z < 0.025 and err_xy < 0.035
        self.get_logger().info(
            'PLACE %s %s -> %s: visual base (%.3f, %.3f, %.3f), '
            'expected z=%.3f, err_xy=%.3f' %
            ('SUCCESS' if success else 'FAILED', self.target_class,
             self.place_class, p[0], p[1], p[2], expected_z, err_xy))
        return success

    def _publish_marker(self, p):
        marker = Marker()
        marker.header.frame_id = BASE_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'visual_grasp'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self._pose(p, (0.0, 0.0, 0.0, 1.0))
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        self.marker_pub.publish(marker)

    def _select_detection(self, label):
        if self.detections is None:
            return None
        best = None
        best_area = -1.0
        for det in self.detections.detections:
            if not det.results:
                continue
            if det.results[0].hypothesis.class_id != label:
                continue
            area = det.bbox.size_x * det.bbox.size_y
            if area > best_area:
                best, best_area = det, area
        return best

    def detect_label(self, label, table_prior=True, publish_debug=False):
        if self.color is None or self.depth is None \
                or self.camera_info is None or self.detections is None:
            return None
        det = self._select_detection(label)
        if det is None:
            return None
        det_stamp = Time.from_msg(self.detections.header.stamp)
        img_stamp = Time.from_msg(self.color.header.stamp)
        if abs((det_stamp - img_stamp).nanoseconds) > 3e8:
            return None

        cfg = OBJECTS[label]
        u = int(round(det.bbox.center.position.x))
        v = int(round(det.bbox.center.position.y))
        bw, bh = det.bbox.size_x, det.bbox.size_y
        depth = self.bridge.imgmsg_to_cv2(self.depth, '32FC1')

        h, w = depth.shape
        u0 = max(int(u - bw / 2.0), 0)
        u1 = min(int(u + bw / 2.0) + 1, w)
        v0 = max(int(v - bh / 2.0), 0)
        v1 = min(int(v + bh / 2.0) + 1, h)
        patch = depth[v0:v1, u0:u1]
        finite = np.isfinite(patch) & (patch > 0.05) & (patch < 3.0)
        # Restrict to this label's colour: on a stack the bbox of the
        # support object can contain the object on top, and the top-face
        # depth must not be taken from the occluder.
        bgr = self.bridge.imgmsg_to_cv2(self.color, 'bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in HSV_RANGES.get(label, []):
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        finite &= mask[v0:v1, u0:u1] > 0
        if not np.any(finite):
            return None

        if cfg['shape'] == 'sphere':
            z = float(np.median(patch[finite]))
            u_c, v_c = float(u), float(v)
        else:
            # Box/cylinder: nearest-depth region is the top face.
            d = patch[finite]
            zmin = float(np.percentile(d, 5))
            sel = finite & (patch <= zmin + 0.015)
            vv, uu = np.mgrid[v0:v1, u0:u1]
            u_c = float(uu[sel].mean())
            v_c = float(vv[sel].mean())
            z = float(np.median(patch[sel]))

        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        x = (u_c - cx) * z / fx
        y = (v_c - cy) * z / fy
        if cfg['shape'] == 'sphere':
            norm = math.sqrt(x * x + y * y + z * z)
            r = cfg['radius']
            if norm > 1e-6:
                x += r * x / norm
                y += r * y / norm
                z += r * z / norm

        point = PointStamped()
        point.header.stamp = self.depth.header.stamp
        point.header.frame_id = CAMERA_FRAME
        point.point.x = x
        point.point.y = y
        point.point.z = z
        try:
            transform = self.tf_buffer.lookup_transform(
                BASE_FRAME, CAMERA_FRAME, Time(),
                timeout=Duration(seconds=0.2))
            base = tf2_geometry_msgs.do_transform_point(point, transform)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('TF failed: %s' % exc,
                                   throttle_duration_sec=2.0)
            return None

        base_point = base.point
        if table_prior:
            # Objects on the table: class dimensions and the table plane are
            # known priors; x/y come from the camera.
            base_point.z = cfg['half_height']
        elif cfg['shape'] != 'sphere':
            # Verification (e.g. object on top of another): use the
            # measured top surface minus the class half height.
            base_point.z = base.point.z - cfg['half_height']

        if publish_debug:
            color = bgr
            x1, y1 = int(u - bw / 2), int(v - bh / 2)
            x2, y2 = int(u + bw / 2), int(v + bh / 2)
            cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(color, '%s %.2fm' % (label, z), (x1, y1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            debug = self.bridge.cv2_to_imgmsg(color, 'bgr8')
            debug.header = self.color.header
            self.debug_pub.publish(debug)
            self._publish_marker((base_point.x, base_point.y, base_point.z))

        return ((base_point.x, base_point.y, base_point.z),
                (u, v, z), bw * bh)

    def detect(self):
        return self.detect_label(self.target_class, table_prior=True,
                                 publish_debug=True)

    def _detect_all(self):
        if self.detections is None:
            return {}
        labels = {d.results[0].hypothesis.class_id
                  for d in self.detections.detections if d.results}
        result = {}
        for label in labels:
            if label not in OBJECTS:
                continue
            # Balls use the table-plane prior (their centre is one radius
            # above the support); boxes/cylinders use the measured top face
            # so objects stacked on another object still work.
            res = self.detect_label(
                label, table_prior=(OBJECTS[label]['shape'] == 'sphere'))
            if res is not None:
                result[label] = res
        return result

    def _fuse_observations(self):
        for label, observations in self._observations.items():
            if not observations:
                continue
            xs = [item[0][0] for item in observations]
            ys = [item[0][1] for item in observations]
            zs = [item[0][2] for item in observations]
            self.seen[label] = (float(np.median(xs)),
                                float(np.median(ys)),
                                float(np.median(zs)))
            first = [item for item in observations if item[2] == 0]
            if first and label not in self.first_seen:
                self.first_seen[label] = first[0][0]

    def _fresh_frame(self, timeout=2.5):
        self.color = self.depth = self.camera_info = self.detections = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(item is not None for item in
                   (self.color, self.depth, self.camera_info,
                    self.detections)):
                for _ in range(3):
                    rclpy.spin_once(self, timeout_sec=0.02)
                return True
        return False

    def _scan_pass(self, poses, first_pass=False):
        for pose_index, pose in enumerate(poses):
            if not self.move_to(pose):
                self.get_logger().warn('scan pose %s unreachable' % (pose,))
                continue
            if not self._fresh_frame():
                self.get_logger().warn('no fresh camera frame at %s' % (pose,))
                continue
            detections = self._detect_all()
            for label, result in detections.items():
                point, _, area = result
                self._observations.setdefault(label, []).append(
                    (point, area, pose_index if first_pass else 99))
                if pose == DEFAULT_OBSERVE:
                    self.reference_seen[label] = point
            self._fuse_observations()
            if detections:
                self.set_scene()
            self.get_logger().info(
                'scan pose %s: %s' %
                (pose, ','.join(sorted(detections)) if detections else 'none'))

    def scan_scene(self, required=None):
        self.get_logger().info('scanning tabletop ...')
        self._scan_pass(SCAN_POSES, first_pass=True)
        missing = [label for label in (required or [])
                   if label not in self.seen]
        if missing:
            self.get_logger().warn(
                'not found in first pass: %s; looking again'
                % ','.join(missing))
            self._scan_pass(SCAN_POSES_2)
        missing = [label for label in (required or [])
                   if label not in self.seen]
        if missing:
            self.get_logger().error(
                'objects not in view after scanning: %s' % ','.join(missing))
            return False
        return True

    def run(self):
        if not self.move.wait_for_server(timeout_sec=30):
            self.get_logger().error('/move_action not available, start '
                                    'arm_moveit.launch.py first')
            return 1
        if not self.grip.wait_for_server(timeout_sec=10):
            self.get_logger().error('gripper action not available')
            return 1

        self.clear_scene()
        if not self.set_scene():
            self.get_logger().warn('could not load table collision box')

        required = [self.target_class]
        if self.place_class:
            required.append(self.place_class)
        scan_required = None if (self.scan_only or self.detect_only) \
            else required
        if not self.scan_scene(scan_required):
            visible = ','.join(sorted(self.seen)) or 'none'
            self.get_logger().error('visible objects: %s' % visible)
            return 1

        if self.scan_only or self.detect_only:
            for name in sorted(self.seen):
                p = (self.reference_seen.get(name) or
                     self.first_seen.get(name) or self.seen[name])
                self.get_logger().info(
                    'VISIBLE %s base=(%.3f, %.3f, %.3f)' %
                    (name, p[0], p[1], p[2]))
            return 0

        target = (self.reference_seen.get(self.target_class) or
                  self.first_seen.get(self.target_class) or
                  self.seen[self.target_class])
        self.get_logger().info(
            'TARGET %s (camera) base=(%.3f, %.3f, %.3f)' %
            (self.target_class, target[0], target[1], target[2]))
        if self.place_class:
            pc = (self.reference_seen.get(self.place_class) or
                  self.first_seen.get(self.place_class) or
                  self.seen.get(self.place_class))
            if pc is None:
                self.get_logger().error(
                    'place target %s is not in the vision scan'
                    % self.place_class)
                return 1
            self.get_logger().info(
                'PLACE TARGET %s (camera) base=(%.3f, %.3f, %.3f)' %
                (self.place_class, pc[0], pc[1], pc[2]))

        self.set_scene()
        self.get_logger().info('opening gripper')
        self.grip_to(self.open_q)
        pre = (target[0], target[1],
               target[2] + self.grip_offset + self.pre_offset)
        grasp = (target[0], target[1], target[2] + self.grip_offset)
        lift = (target[0], target[1],
                target[2] + self.grip_offset + self.lift_offset)

        self.get_logger().info('moving to pre-grasp %s' % (pre,))
        if not self.move_to(pre):
            self.get_logger().error('pre-grasp failed')
            return 1
        if not self._wait_tcp(pre, timeout=8.0, tol=0.02):
            self.get_logger().warn(
                'pre-grasp did not settle, descending from actual pose')

        # Remove only the target from the collision scene so the final
        # camera/gripper approach is checked against the arm, table and the
        # other objects but not against the object being grasped.
        self.remove_object(self.cfg['model'])
        self.get_logger().info('descending to %s' % (grasp,))
        ok, fraction = self.cartesian([grasp])
        if not ok:
            self.get_logger().warn('Cartesian descent %.2f, fallback MoveIt'
                                   % fraction)
            ok = self.move_to(grasp)
        if not ok:
            self.get_logger().error('descent failed')
            return 1
        if not self._wait_tcp(grasp, timeout=8.0, tol=0.015):
            self.get_logger().error(
                'arm did not settle at the grasp pose; aborting before '
                'closing the gripper')
            return 1

        self._log_geometry('at grasp')
        self.get_logger().info('closing gripper')
        self.get_logger().info('close action: %s' % self.grip_to(self.close_q))
        time.sleep(0.5)
        self._log_geometry('after close')

        self.get_logger().info('lifting to %s' % (lift,))
        ok, fraction = self.cartesian([lift])
        if not ok:
            self.get_logger().warn('Cartesian lift %.2f, fallback MoveIt'
                                   % fraction)
            ok = self.move_to(lift)
        if not ok:
            self.get_logger().error('lift failed')
            return 1
        self._wait_tcp(lift, timeout=10.0, tol=0.02)
        time.sleep(0.8)

        # Pure-vision grasp verification: look for the object above the
        # table, near the TCP.  No odometry is used.
        self._fresh_frame()
        held = self.detect_label(self.target_class, table_prior=False,
                                 publish_debug=True)
        if held is None:
            self.get_logger().warn(
                'held object not visible at lift pose; moving to verify pose')
            if self.move_to(SCAN_POSES[0]) and self._fresh_frame():
                held = self.detect_label(self.target_class, table_prior=False)
        fingers = [round(self.joint_positions.get(j, -1.0), 4)
                   for j in GRIPPER_JOINTS]
        if held is None:
            self.get_logger().warn(
                'held object not directly visible; relying on the place '
                'visual verification; fingers=%s' % fingers)
        else:
            p = held[0]
            tcp = self._tcp()
            if tcp is not None:
                dist = math.sqrt((p[0] - tcp.x) ** 2 +
                                 (p[1] - tcp.y) ** 2 +
                                 (p[2] - tcp.z) ** 2)
            else:
                dist = 999.0
            success = p[2] > 0.10 and dist < 0.30
            self.get_logger().info(
                'GRASP %s %s: visual base=(%.3f, %.3f, %.3f), '
                'tcp_dist=%.3f, fingers=%s' %
                ('VERIFIED' if success else 'UNVERIFIED', self.target_class,
                 p[0], p[1], p[2], dist, fingers))
            if not success:
                return 1
        if not self.place_class:
            return 0
        return 0 if self.place_object() else 1



def main():
    rclpy.init()
    node = VisualGrasp()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
