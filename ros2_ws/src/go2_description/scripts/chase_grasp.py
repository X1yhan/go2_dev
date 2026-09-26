#!/usr/bin/env python3
"""追球抓取:狗走向动态小球(仿真真值坐标),行进中机械臂折叠为监控姿态,
到位停稳后规划抓取(带球速预测).

依赖:gazebo(arm:=true walk:=true auto_forward:=false)+ move_group;
小球由 target_director 驱动(真值 odometry 提供位置/速度).
"""
import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Twist
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, OrientationConstraint,
                             PositionConstraint, BoundingVolume)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

MONITOR_POSE = [0.0, 1.57, -1.2, 0.4, 0.0, 0.0, 0.0]   # 折叠监控视角
STOP_DIST = 0.42          # 与球的停靠距离(world)
GRIP_OFFSET = 0.10        # 法兰到夹持中心
OPEN = 0.09
CLOSE = 0.0
DOWN_Q = (1.0, 0.0, 0.0, 0.0)
BALL_CENTER = (1.5, 0.55, 0.30)   # world, 浮空小球中心
BALL_AMP = 0.35                   # y 方向摆动幅度
BALL_SPEED = 0.06                 # 峰值速度 m/s


def clamp(v, lim):
    return max(-lim, min(lim, v))


class ChaseGrasp(Node):
    def __init__(self):
        super().__init__('chase_grasp')
        self.ball = None
        self.ball_stamp = None
        self.ball_vel = (0.0, 0.0)
        self.dog = None
        self.yaw = 0.0
        self.create_subscription(Odometry, '/model/grasp_ball/odometry',
                                 self._on_ball, 10)
        self.create_subscription(Odometry, '/model/go2/odometry',
                                 self._on_dog, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 '/gripper_controller/follow_joint_trajectory')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/rm_group_controller/follow_joint_trajectory')
        self.set_pose = self.create_client(
            SetEntityPose, '/world/go2_sim/set_pose')
        self.pending_pose = None
        self.hold_active = False
        self.hold_y = BALL_CENTER[1]
        self.create_timer(0.05, self._hold_timer)

    # ---------- 状态 ----------
    def _on_ball(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.ball is not None and self.ball_stamp is not None:
            dt = stamp - self.ball_stamp
            if 1e-3 < dt < 0.5:
                self.ball_vel = ((p.x - self.ball[0]) / dt,
                                 (p.y - self.ball[1]) / dt)
        self.ball = (p.x, p.y, p.z)
        self.ball_stamp = stamp

    def _on_dog(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.dog = (p.x, p.y, p.z)
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def to_base(self, point):
        dx, dy = point[0]-self.dog[0], point[1]-self.dog[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (dx*c+dy*s, -dx*s+dy*c, point[2]-(self.dog[2]+0.065))

    def ball_base(self):
        return self.to_base(self.ball)

    def ball_base_velocity(self):
        vx, vy = self.ball_vel
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (vx*c + vy*s, -vx*s + vy*c)

    # ---------- 动作用户端 ----------
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
        return (code.val == success_val) if hasattr(code, 'val') \
            else (code == success_val)

    def arm_to(self, joints):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4',
                                       'joint5', 'joint6', 'joint7']
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in joints]
        p.time_from_start.sec = 3
        goal.trajectory.points = [p]
        return self.send(self.arm, goal, 0)

    def grip_to(self, q):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_finger1_joint',
                                       'gripper_finger2_joint']
        p = JointTrajectoryPoint()
        p.positions = [float(q), float(q)]
        p.time_from_start.sec = 3
        goal.trajectory.points = [p]
        return self.send(self.grip, goal, 0)

    @staticmethod
    def _pose(p, q=DOWN_Q):
        pose = Pose()
        pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

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

    def cartesian(self, waypoints):
        cli = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        if not cli.wait_for_service(timeout_sec=10):
            return False
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = 'rm_group'
        req.link_name = 'Link7'
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False
        req.waypoints = [self._pose(w) for w in waypoints]
        f = cli.call_async(req)
        deadline = time.time() + 10
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = f.result() if f.done() else None
        if res is None or res.fraction < 0.85:
            return False
        traj = res.solution.joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(traj.joint_names)
        goal.trajectory.points = list(traj.points)
        goal.trajectory.header = traj.header
        return self.send(self.arm, goal, 0)

    def set_scene(self, ball_base):
        from geometry_msgs.msg import Pose as MsgPose
        from moveit_msgs.msg import CollisionObject, PlanningScene
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
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in ball_base]
        pose.orientation.w = 1.0
        ball.primitive_poses.append(pose)
        scene.world.collision_objects = [ball]
        req = ApplyPlanningScene.Request()
        req.scene = scene
        f = cli.call_async(req)
        deadline = time.time() + 5
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return f.done()

    # ---------- 行为 ----------
    def _hold_timer(self):
        if self.hold_active:
            self._teleport(BALL_CENTER[0], self.hold_y)

    def _teleport(self, x, y):
        if not self.set_pose.service_is_ready():
            return
        if self.pending_pose is not None and not self.pending_pose.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = 'grasp_ball'
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = BALL_CENTER[2]
        req.pose.orientation.w = 1.0
        self.pending_pose = self.set_pose.call_async(req)

    def teleport_ball(self, t):
        if not self.set_pose.service_is_ready():
            return
        if self.pending_pose is not None and not self.pending_pose.done():
            return
        omega = BALL_SPEED / BALL_AMP
        y = BALL_CENTER[1] + BALL_AMP * math.sin(omega * t)
        self._teleport(BALL_CENTER[0], y)

    def wait_data(self, timeout=20):
        deadline = time.time() + timeout
        while time.time() < deadline and (self.ball is None or self.dog is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ball is not None and self.dog is not None

    def chase(self, move_ball=True):
        print('chasing ball ...')
        t0 = time.time()
        last_teleport = 0.0
        while rclpy.ok() and time.time() - t0 < 60:
            rclpy.spin_once(self, timeout_sec=0.05)
            if move_ball and time.time() - last_teleport > 0.05:
                self.teleport_ball(time.time() - t0)
                last_teleport = time.time()
            if self.ball is None or self.dog is None:
                continue
            dx = self.ball[0] - self.dog[0]
            dy = self.ball[1] - self.dog[1]
            dist = math.hypot(dx, dy)
            if dist <= STOP_DIST + 0.03:
                break
            bearing = math.atan2(dy, dx) - self.yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            cmd = Twist()
            cmd.angular.z = clamp(1.0 * bearing, 0.8)
            if abs(bearing) > 1.0:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = clamp(0.8 * (dist - STOP_DIST), 0.3)
            self.cmd_pub.publish(cmd)
        self.cmd_pub.publish(Twist())
        if self.ball is not None:
            self.hold_y = self.ball[1]
        self.hold_active = True
        print('arrived, dist=%.2f m, settling ...' % math.hypot(
            self.ball[0]-self.dog[0], self.ball[1]-self.dog[1]))
        deadline = time.time() + 2.0
        while time.time() < deadline:
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

    def ball_z(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        return self.ball[2] if self.ball else -1.0

    def grasp_once(self, predict=1.2):
        bb = self.ball_base()
        vb = self.ball_base_velocity()
        aim = (bb[0] + clamp(vb[0]*predict, 0.08),
               bb[1] + clamp(vb[1]*predict, 0.08),
               bb[2])
        self.set_scene(bb)
        z0 = self.ball_z()
        print('aim=(%.3f, %.3f, %.3f) ball_base=(%.3f, %.3f, %.3f) v=(%.3f, %.3f)'
              % (aim + bb + vb))
        self.grip_to(OPEN)
        pre = (aim[0], aim[1], aim[2] + GRIP_OFFSET + 0.08)
        grasp = (aim[0], aim[1], aim[2] + GRIP_OFFSET)
        lift = (aim[0], aim[1], aim[2] + GRIP_OFFSET + 0.10)
        if not self.move_to(pre):
            print('  pre-grasp plan failed')
            return False
        self.hold_active = False
        deadline = time.time() + 0.3
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        ok = self.cartesian([grasp])
        if not ok:
            print('  cartesian descent failed -> MoveIt')
            if not self.move_to(grasp):
                return False
        self.grip_to(CLOSE)
        ok = self.cartesian([lift])
        if not ok:
            ok = self.move_to(lift)
        deadline = time.time() + 2
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        z1 = self.ball_z()
        print('  ball z: %.3f -> %.3f' % (z0, z1))
        return z1 > z0 + 0.05

    def run(self):
        if not self.wait_data():
            print('no ball/dog odometry')
            return False
        print('arm to monitor pose:', self.arm_to(MONITOR_POSE))
        self.chase(move_ball=True)
        for attempt in range(2):
            print('grasp attempt %d ...' % (attempt + 1))
            if self.grasp_once():
                return True
            print('  failed, retry')
            self.grip_to(OPEN)
            rclpy.spin_once(self, timeout_sec=0.5)
        return False


def main():
    rclpy.init()
    node = ChaseGrasp()
    ok = node.run()
    print('CHASE-GRASP %s' % ('SUCCESS' if ok else 'FAILED'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
