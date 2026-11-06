"""The robot's verbs: plan, move, descend, grip.

Layer L4.  Every MoveIt and controller action call in the arm bench stack
lives here, so `grasp.py` can read like a task script and `vision.py` can
stay ignorant of the planner.  All of these return plain bools; the caller
turns them into reasons.
"""
import math
import subprocess
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint,
                             OrientationConstraint, PositionConstraint,
                             PositionIKRequest, RobotState)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_bench.constants import (ARM_JOINTS, BASE_FRAME, DOWN_Q,
                                 GRIPPER_JOINTS, TIP_LINK)

MOVE_ACTION = '/move_action'
GRIPPER_ACTION = '/gripper_controller/follow_joint_trajectory'
ARM_ACTION = '/rm_group_controller/follow_joint_trajectory'

# Planning is deliberately slow: 5 % velocity on a bench arm.
PLANNING_ATTEMPTS = 20
PLANNING_TIME = 8.0
SCALING = 0.05
ORIENTATION_TOLERANCE = 0.05
MIN_CARTESIAN_FRACTION = 0.85

# IK seeds: current posture plus a few perturbations, so the solver can
# return a nearby branch instead of a far wrist/elbow flip.
IK_SEED_DELTAS = (0.3, -0.3, 0.8, -0.8)
FORWARD_SEED = [0.0, 0.8395, 0.0, 1.4442, 0.0, 0.8513, 0.0]


class ArmSkills:

    def __init__(self, node, tf_buffer, joint_positions, open_q=0.0):
        self.node = node
        self.tf_buffer = tf_buffer
        self.joint_positions = joint_positions
        self.open_q = open_q
        self.move = ActionClient(node, MoveGroup, MOVE_ACTION)
        self.grip = ActionClient(node, FollowJointTrajectory, GRIPPER_ACTION)
        self.arm = ActionClient(node, FollowJointTrajectory, ARM_ACTION)

    # -- helpers ---------------------------------------------------------
    def pose(self, position, quat=DOWN_Q):
        msg = Pose()
        msg.position = Point(x=float(position[0]), y=float(position[1]),
                             z=float(position[2]))
        msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2],
                                     w=quat[3])
        return msg

    def send(self, client, goal, success_val):
        """Wait for an action server, send, wait for the result code."""
        if not client.wait_for_server(timeout_sec=20):
            return False
        future = client.send_goal_async(goal)
        deadline = time.time() + 10.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        deadline = time.time() + 60.0
        while time.time() < deadline and not result.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not result.done():
            return False
        code = result.result().result.error_code
        return (code.val == success_val) if hasattr(code, 'val') \
            else (code == success_val)

    def current_joints(self):
        return [float(self.joint_positions.get(joint, 0.0))
                for joint in ARM_JOINTS]

    def log_geometry(self, tag):
        """Log the TCP and both fingers in base_link (diagnostics)."""
        links = (('Link7', 'Link7'),
                 ('gripper_finger1_link', 'f1'),
                 ('gripper_finger2_link', 'f2'))
        parts = []
        try:
            for link, label in links:
                transform = self.tf_buffer.lookup_transform(
                    BASE_FRAME, link, Time(), timeout=Duration(seconds=0.2))
                t = transform.transform.translation
                parts.append('%s=(%.3f,%.3f,%.3f)' % (label, t.x, t.y, t.z))
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn('geometry TF failed: %s' % exc)
            return
        self.node.get_logger().info('%s %s' % (tag, ' '.join(parts)))

    # -- motion ----------------------------------------------------------
    def move_to(self, target):
        """Free-space plan to a tool pose (tool pointing down)."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'rm_group'
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.allowed_planning_time = PLANNING_TIME
        req.max_velocity_scaling_factor = SCALING
        req.max_acceleration_scaling_factor = SCALING
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        region.primitives.append(sphere)
        region.primitive_poses.append(self.pose(target))
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_FRAME
        position_constraint.link_name = TIP_LINK
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = BASE_FRAME
        orientation.link_name = TIP_LINK
        orientation.orientation.x = DOWN_Q[0]
        orientation.orientation.y = DOWN_Q[1]
        orientation.orientation.z = DOWN_Q[2]
        orientation.orientation.w = DOWN_Q[3]
        for axis in ('absolute_x_axis_tolerance',
                     'absolute_y_axis_tolerance',
                     'absolute_z_axis_tolerance'):
            setattr(orientation, axis, ORIENTATION_TOLERANCE)
        orientation.weight = 1.0
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation)
        req.goal_constraints.append(constraints)
        return self.send(self.move, goal, 1)

    def move_to_joint(self, positions):
        """Free-space plan to a joint configuration of the arm."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'rm_group'
        req.num_planning_attempts = PLANNING_ATTEMPTS
        req.allowed_planning_time = PLANNING_TIME
        req.max_velocity_scaling_factor = SCALING
        req.max_acceleration_scaling_factor = SCALING
        constraints = Constraints()
        for name, value in zip(ARM_JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        req.goal_constraints.append(constraints)
        return self.send(self.move, goal, 1)

    def compute_ik(self, target, seeds=None):
        """Best IK solution for a tool pose, by smallest joint travel."""
        client = self.node.create_client(GetPositionIK, '/compute_ik')
        if not client.wait_for_service(timeout_sec=5):
            return None
        current = self.current_joints()
        if seeds is None:
            seeds = [current, FORWARD_SEED, [0.0] * len(ARM_JOINTS)]
            for index in range(len(ARM_JOINTS)):
                for delta in IK_SEED_DELTAS:
                    seed = list(current)
                    seed[index] += delta
                    seeds.append(seed)
        best = None
        best_cost = float('inf')
        for seed in seeds:
            request = PositionIKRequest()
            request.group_name = 'rm_group'
            request.ik_link_name = TIP_LINK
            request.avoid_collisions = False
            request.timeout.sec = 1
            pose_stamped = self._pose_stamped(target)
            request.pose_stamped = pose_stamped
            robot_state = RobotState()
            joint_state = JointState()
            joint_state.name = ARM_JOINTS + GRIPPER_JOINTS
            joint_state.position = [float(v) for v in seed] + \
                [self.open_q, self.open_q]
            robot_state.joint_state = joint_state
            request.robot_state = robot_state
            future = client.call_async(
                GetPositionIK.Request(ik_request=request))
            rclpy.spin_until_future_complete(self.node, future,
                                             timeout_sec=10)
            response = future.result()
            if response is None or not response.solution.joint_state.name:
                continue
            solution = dict(zip(response.solution.joint_state.name,
                                response.solution.joint_state.position))
            solved = [float(solution.get(joint, 0.0)) for joint in ARM_JOINTS]
            cost = 0.0
            for index in range(len(ARM_JOINTS)):
                diff = solved[index] - current[index]
                if ARM_JOINTS[index] == 'joint7':
                    diff = (diff + math.pi) % (2.0 * math.pi) - math.pi
                cost += abs(diff)
            if cost < best_cost:
                best, best_cost = solved, cost
        if best is not None:
            self.node.get_logger().info(
                'IK pre-grasp joint delta=%.3f rad' % best_cost)
        return best

    def _pose_stamped(self, position):
        stamped = PoseStamped()
        stamped.header.frame_id = BASE_FRAME
        stamped.pose = self.pose(position)
        return stamped

    def joint_move(self, positions, seconds=4.0):
        """Direct joint trajectory to the arm controller (no planning)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(seconds)
        goal.trajectory.points = [point]
        return self.send(self.arm, goal, 0)

    def cartesian(self, waypoints, avoid_collisions=True,
                  start_positions=None):
        """Straight-line TCP path; returns (ok, fraction)."""
        client = self.node.create_client(GetCartesianPath,
                                         '/compute_cartesian_path')
        if not client.wait_for_service(timeout_sec=10):
            return False, 0.0
        request = GetCartesianPath.Request()
        request.header.frame_id = BASE_FRAME
        request.group_name = 'rm_group'
        request.link_name = TIP_LINK
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = SCALING
        request.max_acceleration_scaling_factor = SCALING
        request.waypoints = [self.pose(point) for point in waypoints]
        if start_positions is not None:
            robot_state = RobotState()
            joint_state = JointState()
            joint_state.name = ARM_JOINTS + GRIPPER_JOINTS
            joint_state.position = [float(v) for v in start_positions] + \
                [self.open_q, self.open_q]
            robot_state.joint_state = joint_state
            request.start_state = robot_state
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        response = future.result()
        if response is None:
            return False, 0.0
        trajectory = response.solution.joint_trajectory
        self.node.get_logger().info(
            'cartesian fraction %.2f, points %d' %
            (response.fraction, len(trajectory.points)))
        if response.fraction < MIN_CARTESIAN_FRACTION:
            return False, response.fraction
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(trajectory.joint_names)
        for point in trajectory.points:
            goal.trajectory.points.append(point)
        goal.trajectory.header = trajectory.header
        return self.send(self.arm, goal, 0), response.fraction

    def wait_tcp(self, target, timeout=10.0, tol=0.015):
        """Block until the TCP is within `tol` of `target` (or time out)."""
        last_error = float('inf')
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    BASE_FRAME, TIP_LINK, Time(),
                    timeout=Duration(seconds=0.1))
            except Exception:  # noqa: BLE001
                continue
            point = transform.transform.translation
            last_error = math.sqrt((point.x - target[0]) ** 2 +
                                   (point.y - target[1]) ** 2 +
                                   (point.z - target[2]) ** 2)
            if last_error < tol:
                return True
        self.node.get_logger().warn(
            'TCP settle failed: target %s, error %.3f m' %
            (target, last_error))
        return False

    # -- gripper ---------------------------------------------------------
    def grip_to(self, q):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [q, q]
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        return self.send(self.grip, goal, 0)

    def detach(self, action):
        """Attach/detach the simulated ball to the gripper (Gazebo only)."""
        topic = '/target_ball/%s' % action
        try:
            subprocess.run(
                ['ign', 'topic', '-t', topic,
                 '-m', 'ignition.msgs.Empty', '-p', 'unused: true'],
                check=True, capture_output=True, timeout=5)
            return True
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(
                'detachable %s failed: %s' % (action, exc))
            return False
