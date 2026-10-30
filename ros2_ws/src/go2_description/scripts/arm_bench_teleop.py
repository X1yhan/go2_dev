#!/usr/bin/env python3
"""Keyboard jog for the standalone arm bench (teach-pendant style).

Streams joint position trajectories to rm_group_controller, so the arm can
be moved interactively without writing a program per motion.  Requires
arm_bench.launch.py to be running.

Usage:
    ros2 run go2_description arm_bench_teleop.py
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
GRIPPER_JOINTS = ['gripper_finger1_joint', 'gripper_finger2_joint']
ARM_LIMITS = [(-3.1, 3.1), (-2.268, 2.268), (-3.1, 3.1), (-2.355, 2.355),
              (-3.1, 3.1), (-2.233, 2.233), (-6.28, 6.28)]
ARM_TOPIC = '/rm_group_controller/joint_trajectory'
GRIPPER_TOPIC = '/gripper_controller/joint_trajectory'
OPEN = 0.09
CLOSE = 0.0
ZERO = [0.0] * 7
FORWARD = [0.0, 0.8395, 0.0, 1.4442, 0.0, 0.8513, 0.0]

HELP = """
arm_bench_teleop 键盘示教(实时下发到 rm_group_controller)
  1..7      选择关节(默认 joint1)
  w / s     选中关节正向 / 负向转动(按住可连续)
  + / -     步长 x2 / /2(默认 0.03 rad)
  g         夹爪开 / 合切换
  0         所有关节回零位(竖立姿态)
  f         forward 准备姿态
  p         打印当前关节角
  h         显示本帮助
  q / Ctrl+C 退出
"""


class ArmTeleop(Node):

    def __init__(self):
        super().__init__('arm_bench_teleop')
        self.declare_parameter('step', 0.03)
        self.declare_parameter('publish_rate', 20.0)
        self.step = float(self.get_parameter('step').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.target = None
        self.current = None
        self.gr = None
        self.gr_closed = False
        self.selected = 0
        self._last_print = 0.0

        self.arm_pub = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.gr_pub = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint_state, 10)
        self.create_timer(1.0 / rate, self.publish_arm)

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            arm = [float(pos[j]) for j in ARM_JOINTS]
            self.current = arm
            if self.target is None:
                self.target = list(arm)
                self.get_logger().info(
                    'connected, current arm joints: %s' %
                    ' '.join('%.3f' % v for v in arm))
        if self.gr is None and all(j in pos for j in GRIPPER_JOINTS):
            self.gr = [float(pos[j]) for j in GRIPPER_JOINTS]

    def _trajectory(self, joints, positions, seconds):
        traj = JointTrajectory()
        traj.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
        traj.points = [point]
        return traj

    def publish_arm(self):
        if self.target is not None:
            self.arm_pub.publish(
                self._trajectory(ARM_JOINTS, self.target, 0.2))

    def publish_gripper(self):
        if self.gr is not None:
            self.gr_pub.publish(
                self._trajectory(GRIPPER_JOINTS, self.gr, 0.4))

    def _print(self, text):
        now = time.monotonic()
        if now - self._last_print > 0.1:
            self._last_print = now
            print('\r' + text + '   ', end='', flush=True)

    def jog(self, sign):
        if self.target is None:
            print('\r还没有收到 /joint_states,等仿真启动完成再操作   ',
                  end='', flush=True)
            return
        lo, hi = ARM_LIMITS[self.selected]
        value = self.target[self.selected] + sign * self.step
        value = min(max(value, lo), hi)
        self.target[self.selected] = value
        self._print('关节%d = %.3f rad (步长 %.3f)' %
                    (self.selected + 1, value, self.step))

    def toggle_gripper(self):
        if self.gr is None:
            print('\r夹爪未连接(模型可能没装夹爪)   ',
                  end='', flush=True)
            return
        self.gr_closed = not self.gr_closed
        value = CLOSE if self.gr_closed else OPEN
        self.gr = [value, value]
        self.publish_gripper()
        self._print('夹爪 %s' % ('闭合' if self.gr_closed else '张开'))

    def set_pose(self, pose, name):
        if self.target is None:
            return
        self.target = list(pose)
        self._print('目标姿态: %s' % name)

    def print_state(self):
        if self.current is None:
            print('\r还没有收到 /joint_states   ', end='', flush=True)
            return
        pretty = ' '.join('j%d=%.2f' % (i + 1, v)
                          for i, v in enumerate(self.current))
        print('\r当前: %s   ' % pretty, end='', flush=True)

    def handle_key(self, key):
        if key in '1234567':
            self.selected = int(key) - 1
            self._print('已选择 关节%d' % (self.selected + 1))
        elif key == 'w':
            self.jog(+1.0)
        elif key == 's':
            self.jog(-1.0)
        elif key in ('+', '='):
            self.step = min(self.step * 2.0, 0.4)
            self._print('步长 %.3f rad' % self.step)
        elif key == '-':
            self.step = max(self.step / 2.0, 0.005)
            self._print('步长 %.3f rad' % self.step)
        elif key == 'g':
            self.toggle_gripper()
        elif key == '0':
            self.set_pose(ZERO, '零位')
        elif key == 'f':
            self.set_pose(FORWARD, 'forward')
        elif key == 'p':
            self.print_state()
        elif key == 'h':
            print(HELP)
        elif key in ('q', '\x03'):
            return False
        return True


def main():
    rclpy.init()
    node = ArmTeleop()

    if not sys.stdin.isatty():
        print('arm_bench_teleop 需要交互式终端(直接运行,不要重定向输入)')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(HELP)
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                if not node.handle_key(sys.stdin.read(1)):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        print('\n退出。')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
