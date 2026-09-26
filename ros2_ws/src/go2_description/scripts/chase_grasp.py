#!/usr/bin/env python3
"""追球抓取(动态版):球全程运动,狗走过去,机械臂预测拦截 + 快速闭合抓取.

- 球:浮空(z=0.30,无重力+速度衰减),由本脚本 20Hz 遥操作沿正弦摆动
- 狗:监控姿态折叠 → 追球(真值坐标)→ 停在 STOP_DIST
- 抓:预测 PLAN_T 秒后球的拦截点 → 快速移到该点(爪张开,等球进来)
  → 球过拦截点 ±CROSS_TOL 时 0.4s 快速闭合 → 停遥控 → 抬起
依赖:gazebo(arm:=true walk:=true auto_forward:=false)+ move_group.
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
from moveit_msgs.srv import GetCartesianPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

MONITOR_POSE = [0.0, 1.57, -1.2, 0.4, 0.0, 0.0, 0.0]
BALL_CENTER = (1.5, 0.55, 0.30)
BALL_AMP = 0.35
BALL_SPEED = 0.06
STOP_DIST = 0.42
GRIP_OFFSET = 0.10
OPEN = 0.09
CLOSE = 0.0
DOWN_Q = (1.0, 0.0, 0.0, 0.0)
PLAN_T = 3.0           # 拦截提前量 s
MOVE_SCALE = 0.3       # 机械臂速度缩放(抓动态球要快)
CLOSE_TIME = 0.4       # 快速闭合时间 s
CROSS_TOL = 0.02       # 球过拦截点判定 m


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
        self.ball_t0 = time.time()
        self.ball_active = False
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
        self.create_timer(0.05, self._ball_timer)

    # ---------- 小球运动 ----------
    def _ball_y(self, t):
        omega = BALL_SPEED / BALL_AMP
        return BALL_CENTER[1] + BALL_AMP * math.sin(omega * t)

    def _ball_timer(self):
        if not self.ball_active or not self.set_pose.service_is_ready():
            return
        if self.pending_pose is not None and not self.pending_pose.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = 'grasp_ball'
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(BALL_CENTER[0])
        req.pose.position.y = float(self._ball_y(time.time() - self.ball_t0))
        req.pose.position.z = float(BALL_CENTER[2])
        req.pose.orientation.w = 1.0
        self.pending_pose = self.set_pose.call_async(req)

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

    # ---------- 动作 ----------
    def send(self, client, goal, success_val):
        if not client.wait_for_server(timeout_sec=20):
            return False
        f = client.send_goal_async(goal)
        deadline = time.time() + 10
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        h = f.result() if f.done() else None
        if h is None or not h.accepted:
            return False
        r = h.get_result_async()
        deadline = time.time() + 60
        while time.time() < deadline and not r.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not r.done():
            return False
        code = r.result().result.error_code
        return (code.val == success_val) if hasattr(code, 'val') \
            else (code == success_val)

    def arm_to(self, joints, sec=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4',
                                       'joint5', 'joint6', 'joint7']
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in joints]
        p.time_from_start.sec = int(sec)
        goal.trajectory.points = [p]
        return self.send(self.arm, goal, 0)

    def grip_to(self, q, sec=CLOSE_TIME):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['gripper_finger1_joint',
                                       'gripper_finger2_joint']
        p = JointTrajectoryPoint()
        p.positions = [float(q), float(q)]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = int(sec * 1e9)
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

    def move_to(self, target, scale=MOVE_SCALE):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'rm_group'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
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

    def cartesian(self, waypoints, scale=MOVE_SCALE):
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
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale
        req.waypoints = [self._pose(w) for w in waypoints]
        f = cli.call_async(req)
        deadline = time.time() + 10
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        res = f.result() if f.done() else None
        if res is None or res.fraction < 0.85:
            return False
        traj = res.solution.joint_trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(traj.joint_names)
        goal.trajectory.points = list(traj.points)
        goal.trajectory.header = traj.header
        return self.send(self.arm, goal, 0)

    # ---------- 行为 ----------
    def wait_data(self, timeout=20):
        deadline = time.time() + timeout
        while time.time() < deadline and (self.ball is None or self.dog is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ball is not None and self.dog is not None

    def chase(self):
        print('chasing ball ...')
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 60:
            rclpy.spin_once(self, timeout_sec=0.05)
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
        print('arrived, dist=%.2f m, settle 2s (ball keeps moving)'
              % math.hypot(self.ball[0]-self.dog[0],
                           self.ball[1]-self.dog[1]))
        deadline = time.time() + 2.0
        while time.time() < deadline:
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

    def ball_z(self):
        rclpy.spin_once(self, timeout_sec=0.02)
        return self.ball[2] if self.ball else -1.0

    def grasp_dynamic(self):
        z0 = self.ball_z()
        for attempt in range(3):
            self.ball_active = True
            t_now = time.time() - self.ball_t0
            y_target = self._ball_y(t_now + PLAN_T)
            aim = self.to_base((BALL_CENTER[0], y_target, BALL_CENTER[2]))
            print('attempt %d: y_target=%.3f aim_base=(%.3f, %.3f, %.3f)'
                  % (attempt + 1, y_target, aim[0], aim[1], aim[2]))
            self.grip_to(OPEN, 0.5)
            pre = (aim[0], aim[1], aim[2] + GRIP_OFFSET + 0.06)
            if not self.move_to(pre):
                print('  pre-grasp plan failed')
                continue
            grasp = (aim[0], aim[1], aim[2] + GRIP_OFFSET)
            if not self.cartesian([grasp]):
                if not self.move_to(grasp):
                    print('  descent failed')
                    continue
            print('  waiting ball crossing ...')
            t_end = time.time() + 3.0
            crossed = False
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.02)
                if abs(self.ball[1] - y_target) < CROSS_TOL:
                    crossed = True
                    break
            if not crossed:
                print('  no crossing in 3s -> re-predict')
                continue
            print('  crossed -> close')
            self.ball_active = False
            self.grip_to(CLOSE, CLOSE_TIME)
            lift = (aim[0], aim[1], aim[2] + GRIP_OFFSET + 0.12)
            if not self.cartesian([lift]):
                self.move_to(lift)
            deadline = time.time() + 2
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            z1 = self.ball_z()
            print('  ball z: %.3f -> %.3f' % (z0, z1))
            if z1 > z0 + 0.05:
                return True
            print('  missed, retry')
            self.grip_to(OPEN, 0.5)
        return False

    def run(self):
        if not self.wait_data():
            print('no ball/dog odometry')
            return False
        self.ball_t0 = time.time()
        self.ball_active = True
        print('arm to monitor pose:', self.arm_to(MONITOR_POSE))
        self.chase()
        return self.grasp_dynamic()


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
