#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

L1 = 0.213
L2 = 0.213

LEGS = ('FL', 'FR', 'RL', 'RR')
PHASE = {'FL': 0.0, 'RR': 0.0, 'FR': 0.5, 'RL': 0.5}
LEFT_LEGS = ('FL', 'RL')
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


class Go2Trot(Node):
    def __init__(self):
        super().__init__('go2_trot')
        self.frequency = self.declare_parameter('frequency', 1.2).value
        self.stride = self.declare_parameter('stride', 0.10).value
        self.lift = self.declare_parameter('lift', 0.05).value
        self.height = self.declare_parameter('stand_height', 0.275).value
        self.steer_kp = self.declare_parameter('steer_kp', 0.8).value
        self.steer_kd = self.declare_parameter('steer_kd', 0.15).value
        max_corr = self.declare_parameter('max_steer_correction', 0.5).value
        rate = self.declare_parameter('publish_rate', 100.0).value

        self.max_corr = max_corr
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.have_pose = False
        self.last_yaw = None
        self.last_stamp = None

        self.start = self.get_clock().now()
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands', 10)
        self.create_subscription(PoseStamped, '/model/go2/pose',
                                 self._on_pose, 10)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

    def _on_pose(self, msg):
        yaw = yaw_from_quaternion(msg.pose.orientation)
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
        self.last_yaw = yaw
        self.last_stamp = stamp

    def _steer(self):
        if not self.have_pose:
            return 0.0
        corr = self.steer_kp * self.yaw + self.steer_kd * self.yaw_rate
        return max(-self.max_corr, min(self.max_corr, corr))

    def _foot(self, phase, stride):
        if phase < SWING:
            s = phase / SWING
            return (-0.5 * stride + stride * s,
                    -self.height + self.lift * math.sin(math.pi * s))
        s = (phase - SWING) / (1.0 - SWING)
        return (0.5 * stride - stride * s, -self.height)

    def _on_timer(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        corr = self._steer()
        command = []
        for leg in LEGS:
            mult = (1.0 + corr) if leg in LEFT_LEGS else (1.0 - corr)
            phase = (self.frequency * t + PHASE[leg]) % 1.0
            x, z = self._foot(phase, self.stride * mult)
            thigh, calf = leg_ik(x, z)
            command.extend([0.0, thigh, calf])
        msg = Float64MultiArray()
        msg.data = command
        self.pub.publish(msg)


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
