#!/usr/bin/env python3
"""Simple eye-in-hand visual grasp on the standalone arm bench.

Flow:
    move to observation pose -> detect red ball from RGB-D -> compute 3D
    position in base_link -> MoveIt to pre-grasp -> Cartesian descend ->
    close gripper -> Cartesian lift -> verify ball odometry z.

Requires:
    arm_bench.launch.py + arm_moveit.launch.py

Usage:
    ros2 run go2_description arm_bench_visual_grasp.py
    ros2 run go2_description arm_bench_visual_grasp.py --ros-args -p detect_only:=true
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
from visualization_msgs.msg import Marker

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
GRIPPER_JOINTS = ['gripper_finger1_joint', 'gripper_finger2_joint']
DOWN_Q = (1.0, 0.0, 0.0, 0.0)   # 180 deg about X: tool +Z points down
BASE_FRAME = 'base_link'
TIP_LINK = 'Link7'
CAMERA_FRAME = 'camera_depth_optical_frame'


class VisualGrasp(Node):

    def __init__(self):
        super().__init__(
            'arm_bench_visual_grasp',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('observe_xyz', '0.34 0 0.30')
        self.declare_parameter('ball_radius', 0.035)
        self.declare_parameter('grip_offset', 0.18)
        self.declare_parameter('pre_offset', 0.08)
        self.declare_parameter('lift_offset', 0.18)
        self.declare_parameter('open', 0.09)
        self.declare_parameter('close', 0.0)
        self.declare_parameter('detect_only', False)

        self.ball_radius = float(self.get_parameter('ball_radius').value)
        self.grip_offset = float(self.get_parameter('grip_offset').value)
        self.pre_offset = float(self.get_parameter('pre_offset').value)
        self.lift_offset = float(self.get_parameter('lift_offset').value)
        self.open_q = float(self.get_parameter('open').value)
        self.close_q = float(self.get_parameter('close').value)
        self.detect_only = bool(self.get_parameter('detect_only').value)

        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.camera_info = None
        self.ball_odom = None
        self.initial_ball_z = None
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
            Odometry, '/model/target_ball/odometry', self.on_ball_odom, 10)
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

    def on_color(self, msg):
        self.color = msg

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.camera_info = msg

    def on_ball_odom(self, msg):
        self.ball_odom = msg
        if self.initial_ball_z is None:
            self.initial_ball_z = msg.pose.pose.position.z

    def on_joint_state(self, msg):
        self.joint_positions.update(dict(zip(msg.name, msg.position)))

    def _log_ball(self, tag):
        rclpy.spin_once(self, timeout_sec=0.2)
        if self.ball_odom is not None:
            p = self.ball_odom.pose.pose.position
            self.get_logger().info(
                '%s ball xyz=(%.3f, %.3f, %.3f)' % (tag, p.x, p.y, p.z))
        fingers = [round(self.joint_positions.get(j, -1.0), 4)
                   for j in GRIPPER_JOINTS]
        self.get_logger().info('%s fingers=%s' % (tag, fingers))

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
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
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
            setattr(oc, axis, 0.15)
        oc.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)
        return self.send(self.move, goal, 1)

    def cartesian(self, waypoints):
        cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        if not cli.wait_for_service(timeout_sec=10):
            return False, 0.0
        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_FRAME
        req.group_name = 'rm_group'
        req.link_name = TIP_LINK
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        req.waypoints = [self._pose(w) for w in waypoints]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        res = future.result()
        if res is None or res.fraction < 0.85:
            return False, res.fraction if res else 0.0
        traj = res.solution.joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(traj.joint_names)
        for point in traj.points:
            goal.trajectory.points.append(point)
        goal.trajectory.header = traj.header
        return self.send(self.arm, goal, 0), res.fraction

    def grip_to(self, q):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [q, q]
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        return self.send(self.grip, goal, 0)

    def set_scene(self, ball_base):
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

        ball = CollisionObject()
        ball.header.frame_id = BASE_FRAME
        ball.id = 'target_ball'
        ball.operation = CollisionObject.ADD
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.ball_radius]
        ball.primitives.append(sphere)
        ball_pose = Pose()
        ball_pose.position.x = float(ball_base[0])
        ball_pose.position.y = float(ball_base[1])
        ball_pose.position.z = float(ball_base[2])
        ball_pose.orientation.w = 1.0
        ball.primitive_poses.append(ball_pose)

        scene.world.collision_objects = [table, ball]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = cli.call_async(req)
        deadline = time.time() + 5.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

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

    def detect(self):
        if self.color is None or self.depth is None \
                or self.camera_info is None:
            return None

        color = self.bridge.imgmsg_to_cv2(self.color, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(self.depth, '32FC1')
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 80, 50), (10, 255, 255))
        mask |= cv2.inRange(hsv, (170, 80, 50), (180, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 30:
            return None
        (u, v), _radius = cv2.minEnclosingCircle(contour)
        u, v = int(round(u)), int(round(v))

        patch = depth[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > 0.05) & (valid < 3.0)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))

        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        norm = math.sqrt(x * x + y * y + z * z)
        if norm > 1e-6:
            x += self.ball_radius * x / norm
            y += self.ball_radius * y / norm
            z += self.ball_radius * z / norm

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

        cv2.circle(color, (u, v), 5, (0, 255, 0), 2)
        cv2.putText(color, 'ball', (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)
        debug = self.bridge.cv2_to_imgmsg(color, 'bgr8')
        debug.header = self.color.header
        self.debug_pub.publish(debug)

        p = (base.point.x, base.point.y, base.point.z)
        self._publish_marker(p)
        return p, (u, v, z)

    def wait_detection(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            result = self.detect()
            if result is not None:
                return result
        return None

    def run(self):
        if not self.move.wait_for_server(timeout_sec=30):
            self.get_logger().error('/move_action not available, start '
                                    'arm_moveit.launch.py first')
            return 1
        if not self.grip.wait_for_server(timeout_sec=10):
            self.get_logger().error('gripper action not available')
            return 1

        observe = [float(v) for v in
                   self.get_parameter('observe_xyz').value.split()]
        self.get_logger().info('opening gripper')
        self.grip_to(self.open_q)
        self.get_logger().info('moving to observation pose %s' % observe)
        if not self.move_to(observe):
            self.get_logger().error('observation pose failed')
            return 1

        self.get_logger().info('detecting red ball ...')
        result = self.wait_detection(15.0)
        if result is None:
            self.get_logger().error('ball not detected')
            return 1
        ball, pixel = result
        self.get_logger().info(
            'BALL_BASE x=%.3f y=%.3f z=%.3f (pixel %d,%d depth=%.3f)' %
            (ball[0], ball[1], ball[2], pixel[0], pixel[1], pixel[2]))
        if self.detect_only:
            return 0

        self.set_scene(ball)
        self.grip_to(self.open_q)
        pre = (ball[0], ball[1], ball[2] + self.grip_offset + self.pre_offset)
        grasp = (ball[0], ball[1], ball[2] + self.grip_offset)
        lift = (ball[0], ball[1],
                ball[2] + self.grip_offset + self.lift_offset)

        self.get_logger().info('moving to pre-grasp %s' % (pre,))
        if not self.move_to(pre):
            self.get_logger().error('pre-grasp failed')
            return 1

        self.get_logger().info('descending to %s' % (grasp,))
        ok, fraction = self.cartesian([grasp])
        if not ok:
            self.get_logger().warn('Cartesian descent %.2f, fallback MoveIt'
                                   % fraction)
            ok = self.move_to(grasp)
        if not ok:
            self.get_logger().error('descent failed')
            return 1

        self.get_logger().info('closing gripper')
        self.get_logger().info('close action: %s' % self.grip_to(self.close_q))
        time.sleep(0.5)
        self._log_ball('after close')

        self.get_logger().info('lifting to %s' % (lift,))
        ok, fraction = self.cartesian([lift])
        if not ok:
            self.get_logger().warn('Cartesian lift %.2f, fallback MoveIt'
                                   % fraction)
            ok = self.move_to(lift)
        time.sleep(1.0)
        self._log_ball('after lift')
        rclpy.spin_once(self, timeout_sec=0.2)

        if self.ball_odom is None or self.initial_ball_z is None:
            self.get_logger().warn('no ball odometry, cannot verify')
            return 0 if ok else 1
        lifted_z = self.ball_odom.pose.pose.position.z
        success = lifted_z > self.initial_ball_z + 0.03
        self.get_logger().info(
            'GRASP %s (ball z %.3f -> %.3f)' %
            ('SUCCESS' if success else 'FAILED', self.initial_ball_z,
             lifted_z))
        return 0 if success else 1


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
