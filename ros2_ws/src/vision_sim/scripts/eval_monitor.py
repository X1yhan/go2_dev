#!/usr/bin/env python3
"""Accuracy monitor: compares the estimated target position against ground
truth and reports error statistics.

    /target/ground_truth   (from target_director, world frame)
    /target/position_world (from target_localizer, world frame)

Outputs:
    /target/error        std_msgs/Float64        instantaneous error [m]
    /target/error_stats  std_msgs/Float64MultiArray [mean, max, rms]
"""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class EvalMonitor(Node):

    def __init__(self):
        super().__init__('eval_monitor')

        self.declare_parameter('ground_truth_topic', '/target/ground_truth')
        self.declare_parameter('estimate_topic', '/target/position_world')
        self.declare_parameter('error_topic', '/target/error')
        self.declare_parameter('stats_topic', '/target/error_stats')
        self.declare_parameter('window', 300)
        self.declare_parameter('max_stamp_dt', 0.5)
        self.declare_parameter('report_period', 5.0)

        self.window = int(self.get_parameter('window').value)
        self.max_dt = float(self.get_parameter('max_stamp_dt').value)
        self.report_period = float(self.get_parameter('report_period').value)

        self.gt = deque(maxlen=200)
        self.errors = deque(maxlen=self.window)
        self.count = 0

        self.error_pub = self.create_publisher(
            Float64, self.get_parameter('error_topic').value, 10)
        self.stats_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('stats_topic').value, 10)

        self.create_subscription(
            PoseStamped, self.get_parameter('ground_truth_topic').value,
            self.on_gt, 10)
        self.create_subscription(
            PointStamped, self.get_parameter('estimate_topic').value,
            self.on_estimate, 10)
        self.create_timer(self.report_period, self.report)

        self.get_logger().info('eval_monitor up')

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_gt(self, msg):
        self.gt.append(msg)

    def on_estimate(self, msg):
        if not self.gt:
            return
        t = self._stamp_seconds(msg.header.stamp)
        gt = min(self.gt,
                 key=lambda g: abs(self._stamp_seconds(g.header.stamp) - t))
        dt = abs(self._stamp_seconds(gt.header.stamp) - t)
        if dt > self.max_dt:
            return

        dx = msg.point.x - gt.pose.position.x
        dy = msg.point.y - gt.pose.position.y
        dz = msg.point.z - gt.pose.position.z
        err = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.errors.append(err)
        self.count += 1

        self.error_pub.publish(Float64(data=err))

    def report(self):
        if len(self.errors) < 10:
            return
        n = len(self.errors)
        mean = sum(self.errors) / n
        rms = math.sqrt(sum(e * e for e in self.errors) / n)
        worst = max(self.errors)
        stats = Float64MultiArray()
        stats.data = [mean, worst, rms]
        self.stats_pub.publish(stats)
        self.get_logger().info(
            f'{self.count} samples | error mean={mean*100:.2f} cm, '
            f'rms={rms*100:.2f} cm, max={worst*100:.2f} cm')


def main():
    rclpy.init()
    node = EvalMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
