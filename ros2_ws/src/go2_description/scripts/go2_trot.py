#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

L1 = 0.213
L2 = 0.213

LEGS = ('FL', 'FR', 'RL', 'RR')
PHASE = {'FL': 0.0, 'RR': 0.0, 'FR': 0.5, 'RL': 0.5}
HIP_XY = {
    'FL': (0.1934, 0.0465),
    'FR': (0.1934, -0.0465),
    'RL': (-0.1934, 0.0465),
    'RR': (-0.1934, -0.0465),
}
SWING = 0.4


def leg_ik(x, z):
    d = math.hypot(x, z)
    d = min(d, L1 + L2 - 1e-6)
    c = max(-1.0, min(1.0, (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)))
    calf = -math.acos(c)
    delta = math.atan2(L2 * math.sin(calf), L1 + L2 * math.cos(calf))
    thigh = math.atan2(-x, -z) - delta
    return thigh, calf


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Go2Trot(Node):
    def __init__(self):
        super().__init__('go2_trot')
        self.frequency = self.declare_parameter('frequency', 1.2).value
        self.stride = self.declare_parameter('stride', 0.10).value
        self.lift = self.declare_parameter('lift', 0.05).value
        self.height = self.declare_parameter('stand_height', 0.275).value
        self.steer_kp = self.declare_parameter('steer_kp', 1.0).value
        self.steer_kd = self.declare_parameter('steer_kd', 0.15).value
        self.vx_gain = self.declare_parameter('vx_gain', 1.6).value
        self.vy_gain = self.declare_parameter('vy_gain', 1.6).value
        self.max_step = self.declare_parameter('max_step', 0.18).value
        self.max_vx = self.declare_parameter('max_vx', 0.6).value
        self.max_vy = self.declare_parameter('max_vy', 0.3).value
        self.max_wz = self.declare_parameter('max_wz', 1.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.auto_forward = self.declare_parameter(
            'auto_forward', True).value
        rate = self.declare_parameter('publish_rate', 100.0).value

        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.desired_yaw = 0.0
        self.have_pose = False
        self.last_yaw = None
        self.last_stamp = None
        self.last_time = None
        self.teleop = False
        self.cmd = Twist()
        self.cmd_stamp = None

        self.start = self.get_clock().now()
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands', 10)
        self.create_subscription(Odometry, '/model/go2/odometry',
                                 self._on_pose, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_yaw is not None and stamp > self.last_stamp:
            delta = yaw - self.last_yaw
            delta = math.atan2(math.sin(delta), math.cos(delta))
            self.yaw += delta
            dt = stamp - self.last_stamp
            if dt > 0.0:
                meas = delta / dt
                self.yaw_rate = 0.9 * self.yaw_rate + 0.1 * meas
            self.have_pose = True
        else:
            self.yaw = yaw
            self.desired_yaw = yaw
        self.last_yaw = yaw
        self.last_stamp = stamp

    def _on_cmd(self, msg):
        self.cmd = msg
        self.cmd_stamp = self._now()
        self.teleop = True

    def _velocity_command(self, now):
        if not self.teleop:
            if not self.auto_forward:
                return 0.0, 0.0, 0.0
            return self.stride * self.frequency, 0.0, 0.0
        if self.cmd_stamp is not None and now - self.cmd_stamp > self.cmd_timeout:
            return 0.0, 0.0, 0.0
        return (clamp(self.cmd.linear.x, self.max_vx),
                clamp(self.cmd.linear.y, self.max_vy),
                clamp(self.cmd.angular.z, self.max_wz))

    def _stand(self):
        thigh, calf = leg_ik(0.0, -self.height)
        return [0.0, thigh, calf] * len(LEGS)

    def _foot(self, phase, dx, dy):
        if phase < SWING:
            s = phase / SWING
            return (-0.5 * dx + dx * s,
                    -0.5 * dy + dy * s,
                    -self.height + self.lift * math.sin(math.pi * s))
        s = (phase - SWING) / (1.0 - SWING)
        return (0.5 * dx - dx * s,
                0.5 * dy - dy * s,
                -self.height)

    def _on_timer(self):
        now = self._now()
        dt = 0.0 if self.last_time is None else max(0.0, now - self.last_time)
        self.last_time = now
        vx, vy, wz = self._velocity_command(now)
        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
            if self.have_pose:
                self.desired_yaw = self.yaw
            self.pub.publish(Float64MultiArray(data=self._stand()))
            return

        self.desired_yaw += wz * dt
        wz_eff = clamp(wz + self.steer_kp * (self.desired_yaw - self.yaw)
                       + self.steer_kd * (wz - self.yaw_rate),
                       self.max_wz + 0.5)
        t_stance = (1.0 - SWING) / self.frequency
        vx_eff = vx * self.vx_gain
        vy_eff = vy * self.vy_gain

        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        command = []
        for leg in LEGS:
            hx, hy = HIP_XY[leg]
            dx = clamp((vx_eff - wz_eff * hy) * t_stance, self.max_step)
            dy = clamp((vy_eff + wz_eff * hx) * t_stance, self.max_step)
            phase = (self.frequency * t + PHASE[leg]) % 1.0
            x, y, z = self._foot(phase, dx, dy)
            hip = math.atan2(y, -z)
            thigh, calf = leg_ik(x, -math.hypot(y, z))
            command.extend([hip, thigh, calf])
        self.pub.publish(Float64MultiArray(data=command))


def main():
    rclpy.init()
    node = Go2Trot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
