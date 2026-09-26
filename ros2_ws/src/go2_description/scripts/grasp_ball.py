#!/usr/bin/env python3
"""简单抓取演示:MoveIt 规划到预抓取,笛卡尔直线下探/抬起,夹爪开合.

前提:仿真里 arm:=true(含简化夹爪),MoveIt(move_group)已启动;
目标为台面上的小球 grasp_ball(r=0.05).
"""
import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, OrientationConstraint,
                             PositionConstraint, BoundingVolume)
from moveit_msgs.srv import GetCartesianPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

BALL_GOAL = (0.35, 0.25, 0.25)
GRIP_OFFSET = 0.10
OPEN = 0.09
CLOSE = 0.0
DOWN_Q = (1.0, 0.0, 0.0, 0.0)


class Grasp(Node):
    def __init__(self):
        super().__init__('grasp_ball')
        self.ball = None
        self.dog = None
        self.yaw = 0.0
        self.create_subscription(Odometry, '/model/grasp_ball/odometry', self._ball, 10)
        self.create_subscription(Odometry, '/model/go2/odometry', self._dog, 10)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/rm_group_controller/follow_joint_trajectory')

    def _ball(self, m):
        p = m.pose.pose.position
        self.ball = (p.x, p.y, p.z)

    def _dog(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.dog = (p.x, p.y, p.z)
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def to_base(self, point):
        dx, dy = point[0]-self.dog[0], point[1]-self.dog[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (dx*c+dy*s, -dx*s+dy*c, point[2]-(self.dog[2]+0.065))

    def ball_base(self):
        return self.to_base(self.ball)

    def set_scene(self, ball_base, table_base):
        from geometry_msgs.msg import Pose as MsgPose
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not cli.wait_for_service(timeout_sec=10):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        ball = CollisionObject()
        ball.header.frame_id = 'base_link'
        ball.id = 'grasp_ball'
        ball.operation = CollisionObject.ADD
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.07]
        ball.primitives.append(sp)
        pose = MsgPose()
        pose.position.x, pose.position.y, pose.position.z = ball_base
        pose.orientation.w = 1.0
        ball.primitive_poses.append(pose)
        table = CollisionObject()
        table.header.frame_id = 'base_link'
        table.id = 'pedestal'
        table.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.3, 0.3, 0.25]
        table.primitives.append(box)
        tpose = MsgPose()
        tpose.position.x, tpose.position.y, tpose.position.z = table_base
        tpose.orientation.w = 1.0
        table.primitive_poses.append(tpose)
        scene.world.collision_objects = [ball, table]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        f = cli.call_async(req)
        deadline = time.time() + 5
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return f.done()

    @staticmethod
    def _pose(p, q=DOWN_Q):
        pose = Pose()
        pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def send(self, client, goal, success_val):
        if not client.wait_for_server(timeout_sec=20):
            return False
        f = client.send_goal_async(goal)
        deadline = time.time() + 10
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        h = f.result() if f.done() else None
        if h is None or not h.accepted:
            return False
        r = h.get_result_async()
        deadline = time.time() + 60
        while time.time() < deadline and not r.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not r.done():
            return False
        code = r.result().result.error_code
        return (code.val == success_val) if hasattr(code, 'val') else (code == success_val)

    def move_to(self, target):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'rm_group'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        region = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        region.primitives.append(sp)
        region.primitive_poses.append(self._pose(target))
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'Link7'
        pc.constraint_region = region
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'Link7'
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = DOWN_Q
        for axis in ('absolute_x_axis_tolerance', 'absolute_y_axis_tolerance',
                     'absolute_z_axis_tolerance'):
            setattr(oc, axis, 0.15)
        oc.weight = 1.0
        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)
        return self.send(self.move, goal, 1)

    def grip_to(self, q):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_finger1_joint', 'gripper_finger2_joint']
        p = JointTrajectoryPoint()
        p.positions = [q, q]
        p.time_from_start.sec = 3
        goal.trajectory.points = [p]
        return self.send(self.grip, goal, 0)

    def cartesian(self, waypoints):
        cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        if not cli.wait_for_service(timeout_sec=10):
            return False, 0.0
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'rm_group'
        req.link_name = 'Link7'
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        req.waypoints = [self._pose(w) for w in waypoints]
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10)
        res = f.result()
        if res is None or res.fraction < 0.85:
            return False, res.fraction if res else 0.0
        traj = res.solution.joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(traj.joint_names)
        for pt in traj.points:
            goal.trajectory.points.append(pt)
        goal.trajectory.header = traj.header
        ok = self.send(self.arm, goal, 0)
        return ok, res.fraction


    def finger_positions(self):
        from sensor_msgs.msg import JointState
        holder = {}
        sub = self.create_subscription(
            JointState, '/joint_states',
            lambda m: holder.update(dict(zip(m.name, m.position))), 10)
        deadline = time.time() + 3
        while time.time() < deadline and not holder:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return {k: round(holder.get(k, -1), 3)
                for k in ('gripper_finger1_joint', 'gripper_finger2_joint')}


def main():
    rclpy.init()
    node = Grasp()
    print('homing arm ...')
    home = FollowJointTrajectory.Goal()
    home.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4',
                                   'joint5', 'joint6', 'joint7']
    hp = JointTrajectoryPoint()
    hp.positions = [0.0] * 7
    hp.time_from_start.sec = 4
    home.trajectory.points = [hp]
    move_client = ActionClient(node, FollowJointTrajectory,
                               '/rm_group_controller/follow_joint_trajectory')
    node.send(move_client, home, 0)
    time.sleep(1.0)
    deadline = time.time() + 15
    while time.time() < deadline and (node.ball is None or node.dog is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.ball is None or node.dog is None:
        print('no ball/dog odometry data')
        return 1
    print('ball at', tuple(round(v, 3) for v in node.ball))

    def log(tag):
        rclpy.spin_once(node, timeout_sec=0.1)
        print('  [%s] ball=%s' % (tag, tuple(round(v, 3) for v in node.ball)))

    bb = node.ball_base()
    table_base = node.to_base((0.35, 0.25, 0.125))
    print('planning scene objects set:', node.set_scene(bb, table_base))
    grasp = (bb[0], bb[1], bb[2] + GRIP_OFFSET)
    pre = (bb[0], bb[1], bb[2] + GRIP_OFFSET + 0.08)
    lift = (bb[0], bb[1], bb[2] + GRIP_OFFSET + 0.18)
    print('grasp=%s pre=%s' % (tuple(round(v, 3) for v in grasp),
                               tuple(round(v, 3) for v in pre)))
    print('open:', node.grip_to(OPEN)); log('open')
    print('move pre:', node.move_to(pre)); log('pre')
    bb2 = node.ball_base()
    grasp = (bb2[0], bb2[1], bb2[2] + GRIP_OFFSET)
    ok, frac = node.cartesian([grasp])
    if not ok:
        print('  cartesian descent frac=%.2f -> fallback MoveIt' % frac)
        ok = node.move_to(grasp)
    print('descent:', ok); log('descent')
    print('close:', node.grip_to(CLOSE)); log('close')
    fingers = node.finger_positions()
    print('  finger joints after close:', fingers)
    ok, frac = node.cartesian([lift])
    if not ok:
        print('  cartesian lift frac=%.2f -> fallback MoveIt' % frac)
        ok = node.move_to(lift)
    print('lift:', ok); log('lift')
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.1)
    lifted = node.ball[2] > BALL_GOAL[2] + 0.06
    print('GRASP %s (ball z %.3f -> %.3f)' % (
        'SUCCESS' if lifted else 'FAILED', BALL_GOAL[2], node.ball[2]))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if lifted else 1


if __name__ == '__main__':
    sys.exit(main())
