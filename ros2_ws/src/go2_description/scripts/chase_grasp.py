#!/usr/bin/env python3
"""直线运动小球的拦截抓取.

策略(重视成功率):
  1. 球沿 x=1.0 的直线以 BALL_SPEED 向 +y 匀速运动(脚本 20Hz 遥操作,捕获前不停)
  2. 狗走到球轨迹旁边 WAIT_OFFSET 处、正对轨迹等待(身体离球线 > 头长,不挤球)
  3. 机械臂把爪口放到预测拦截点(球前方 BALL_SPEED*PLAN_T),球进爪口时 0.4s 快速闭合
  4. 球跑过了就重新到前方等待(最多 3 次),保证成功率
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
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

MONITOR_POSE = [0.0, 1.45, -2.1, 1.35, 0.0, -0.25, 0.0]   # 收拢折叠,防倾覆
BALL_START = (1.0, -2.0, 0.30)     # world, 直线起点
BALL_SPEED = 0.07                  # m/s, 沿 +y
LINE_X = BALL_START[0]
SIDE_OFFSET = 0.30                 # 狗侧向站位(到球线), 臂只横伸这么远
WAIT_LEAD = 18.0                    # 等待点选在球前方(秒)
YAW_ALONG = 1.5708                 # 狗朝向 +y(平行球线)
GRIP_OFFSET = 0.10
OPEN = 0.09
CLOSE = 0.0
DOWN_Q = (1.0, 0.0, 0.0, 0.0)
PLAN_T = 3.0                       # 拦截提前量 s
MOVE_SCALE = 0.5
CLOSE_TIME = 0.4
CROSS_TOL = 0.05


def clamp(v, lim):
    return max(-lim, min(lim, v))


class InterceptGrasp(Node):
    def __init__(self):
        super().__init__('intercept_grasp')
        self.ball = None
        self.ball_stamp = None
        self.ball_vel = (0.0, 0.0)
        self.dog = None
        self.quat = None
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

    # ---------- 小球直线运动 ----------
    def _ball_y(self, t):
        return BALL_START[1] + BALL_SPEED * t

    def _ball_timer(self):
        if not self.ball_active or not self.set_pose.service_is_ready():
            return
        if self.pending_pose is not None and not self.pending_pose.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = 'grasp_ball'
        req.entity.type = Entity.MODEL
        req.pose.position.x = float(BALL_START[0])
        req.pose.position.y = float(self._ball_y(time.time() - self.ball_t0))
        req.pose.position.z = float(BALL_START[2])
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
        self.quat = (q.x, q.y, q.z, q.w)
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def to_base(self, point):
        dx, dy = point[0]-self.dog[0], point[1]-self.dog[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (dx*c+dy*s, -dx*s+dy*c, point[2]-(self.dog[2]+0.065))

    def ball_base(self):
        dx, dy = self.ball[0]-self.dog[0], self.ball[1]-self.dog[1]
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (dx*c+dy*s, -dx*s+dy*c, self.ball[2]-(self.dog[2]+0.065))

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

    def move_to(self, target):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'rm_group'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = MOVE_SCALE
        req.max_acceleration_scaling_factor = MOVE_SCALE
        region = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.012]
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
        req.max_velocity_scaling_factor = MOVE_SCALE
        req.max_acceleration_scaling_factor = MOVE_SCALE
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

    def set_scene(self, ball_base):
        from geometry_msgs.msg import Pose as MsgPose
        from moveit_msgs.msg import CollisionObject, PlanningScene
        cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not cli.wait_for_service(timeout_sec=10):
            return False
        scene = PlanningScene()
        scene.is_diff = True
        objs = []
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
        objs.append(ball)
        body = CollisionObject()
        body.header.frame_id = 'base_link'
        body.id = 'dog_body'
        body.operation = CollisionObject.ADD
        b = SolidPrimitive()
        b.type = SolidPrimitive.BOX
        b.dimensions = [0.53, 0.10, 0.14]
        body.primitives.append(b)
        p = MsgPose()
        p.position.x, p.position.y, p.position.z = 0.075, 0.0, 0.01
        p.orientation.w = 1.0
        body.primitive_poses.append(p)
        objs.append(body)
        ground = CollisionObject()
        ground.header.frame_id = 'base_link'
        ground.id = 'ground'
        ground.operation = CollisionObject.ADD
        g = SolidPrimitive()
        g.type = SolidPrimitive.BOX
        g.dimensions = [4.0, 4.0, 0.1]
        ground.primitives.append(g)
        gp = MsgPose()
        gp.position.x, gp.position.y, gp.position.z = 0.0, 0.0, -0.41
        gp.orientation.w = 1.0
        ground.primitive_poses.append(gp)
        objs.append(ground)
        scene.world.collision_objects = objs
        req = ApplyPlanningScene.Request()
        req.scene = scene
        f = cli.call_async(req)
        deadline = time.time() + 5
        while time.time() < deadline and not f.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return f.done()

    def check_tip(self):
        if self.quat is None or self.dog is None:
            return
        x, y, z, w = self.quat
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        if abs(roll) > 0.5 or abs(pitch) > 0.5 or self.dog[2] < 0.20:
            self.cmd_pub.publish(Twist())
            print('!! TIP-OVER DETECTED (roll=%.2f pitch=%.2f z=%.2f) -> abort'
                  % (roll, pitch, self.dog[2]))
            self.ball_active = False
            raise SystemExit(3)

    # ---------- 行为 ----------
    def wait_data(self, timeout=20):
        deadline = time.time() + timeout
        while time.time() < deadline and (self.ball is None or self.dog is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ball is not None and self.dog is not None

    def goto(self, target_world, yaw_target):
        print('  goto (%.2f, %.2f) yaw=%.2f' % (target_world[0], target_world[1],
                                               yaw_target))
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 40:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.check_tip()
            if self.dog is None:
                continue
            dx = target_world[0] - self.dog[0]
            dy = target_world[1] - self.dog[1]
            dist = math.hypot(dx, dy)
            if dist < 0.12:
                break
            bearing = math.atan2(dy, dx)
            err = math.atan2(math.sin(bearing - self.yaw),
                             math.cos(bearing - self.yaw))
            cmd = Twist()
            cmd.angular.z = clamp(1.2 * err, 0.8)
            cmd.linear.x = clamp(0.5 * dist, 0.3) if abs(err) < 1.0 else 0.0
            self.cmd_pub.publish(cmd)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        # 原地转到目标朝向
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 5:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.check_tip()
            err = math.atan2(math.sin(yaw_target - self.yaw),
                             math.cos(yaw_target - self.yaw))
            if abs(err) < 0.05:
                break
            cmd = Twist()
            cmd.angular.z = clamp(1.2 * err, 0.5)
            self.cmd_pub.publish(cmd)
        self.cmd_pub.publish(Twist())
        deadline = time.time() + 1.0
        while time.time() < deadline:
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_beside_line(self):
        # 侧向等待点: 球线旁 0.38m, 球前方(按秒 lead), 狗平行球线
        ball_y = self.ball[1]
        target = (LINE_X - SIDE_OFFSET,
                  ball_y + BALL_SPEED * WAIT_LEAD + 0.3)
        print('  wait point (%.2f, %.2f) ball_y=%.2f' % (target[0], target[1], ball_y))
        self.goto(target, YAW_ALONG)

    def grasp_intercept(self):
        z0 = self.ball[2]
        base_body = (0.075, 0.0, 0.01)
        for attempt in range(3):
            self.ball_active = True
            self.set_scene(self.to_base(self.ball))
            self.grip_to(OPEN, 0.5)
            # 等球接近到前方 0.02~0.20m(即将经过), 否则继续等
            t_end = time.time() + 30
            bb = self.ball_base()
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.02)
                self.check_tip()
                bb = self.ball_base()
                if 0.02 < bb[0] < 0.10:
                    break
                if bb[0] > 0.35:
                    bb = None
                    break
            else:
                print('  ball did not approach -> reposition')
                self.wait_beside_line()
                continue
            if bb is None:
                print('  ball passed -> reposition')
                self.wait_beside_line()
                continue
            # 自适应预判: 用球在 base 系的速度外推 PLAN_T 秒
            pass
            vx, vy = self.ball_vel
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            vb = (vx*c + vy*s, -vx*s + vy*c)
            aim = (bb[0] + vb[0]*PLAN_T, bb[1] + vb[1]*PLAN_T, bb[2])
            if abs(aim[0]) > 0.45 or abs(aim[1]) > 0.45:
                print('  aim (%.2f, %.2f) out of safe reach -> reposition'
                      % (aim[0], aim[1]))
                self.wait_beside_line()
                continue
            pre = (aim[0], aim[1], aim[2] + GRIP_OFFSET + 0.05)
            grasp = (aim[0], aim[1], aim[2] + GRIP_OFFSET)
            liftz = aim[2] + GRIP_OFFSET + 0.12
            print('attempt %d: aim=(%.2f, %.2f, %.2f) bb=(%.2f, %.2f) vb=(%.2f, %.2f)'
                  % ((attempt + 1,) + aim + (bb[0], bb[1], vb[0], vb[1])))
            if not self.move_to(pre):
                print('  pre-grasp plan failed')
                self.wait_beside_line()
                continue
            if not self.cartesian([grasp]):
                if not self.move_to(grasp):
                    print('  descent failed')
                    continue
            print('  waiting ball crossing (prediction)...')
            t_end = time.time() + 8.0
            crossed = False
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.02)
                self.check_tip()
                bb2 = self.ball_base()
                d = math.hypot(bb2[0]-aim[0], bb2[1]-aim[1])
                if d < 0.04:
                    crossed = True
                    break
            if not crossed:
                print('  missed crossing -> walk ahead')
                self.wait_beside_line()
                continue
            print('  crossed -> close')
            self.ball_active = False
            self.grip_to(CLOSE, CLOSE_TIME)
            if not self.cartesian([(aim[0], aim[1], liftz)]):
                self.move_to((aim[0], aim[1], liftz))
            deadline = time.time() + 2
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            z1 = self.ball[2]
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
        self.wait_beside_line()
        return self.grasp_intercept()


def main():
    rclpy.init()
    node = InterceptGrasp()
    ok = node.run()
    print('INTERCEPT-GRASP %s' % ('SUCCESS' if ok else 'FAILED'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
