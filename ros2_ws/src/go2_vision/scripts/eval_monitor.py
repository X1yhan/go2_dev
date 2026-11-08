#!/usr/bin/env python3
"""评估:把融合定位(/target/position_world)与真值(/target/ground_truth)对账."""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from go2_vision.msg import TargetObservation


def _stats(errors):
    n = len(errors)
    if n == 0:
        return None
    xy = math.sqrt(sum(e[0] ** 2 + e[1] ** 2 for e in errors) / n)
    rms = math.sqrt(sum(e[0] ** 2 + e[1] ** 2 + e[2] ** 2 for e in errors) / n)
    mx = max(math.sqrt(e[0] ** 2 + e[1] ** 2 + e[2] ** 2) for e in errors)
    mean_z = sum(e[2] for e in errors) / n
    return xy, rms, mx, mean_z


class EvalMonitor(Node):
    def __init__(self):
        super().__init__('eval_monitor')
        self.gt_topic = self.declare_parameter(
            'ground_truth_topic', '/target/ground_truth').value
        self.gt_odom_topic = self.declare_parameter(
            'ground_truth_odom_topic', '/model/red_ball/odometry').value
        self.obs_topic = self.declare_parameter(
            'observation_topic', '/target/position_world').value
        self.max_match_dt = self.declare_parameter('max_stamp_dt', 0.3).value
        self.report_period = self.declare_parameter('report_period', 5.0).value
        self.window = int(self.declare_parameter('window', 300).value)

        self.gt_buffer = deque(maxlen=200)
        self.gt_odom_buffer = deque(maxlen=200)
        self.errors = deque(maxlen=self.window)
        self.errors_cmd = deque(maxlen=self.window)
        self.methods = {}
        self.quality_sum = 0.0
        self.quality_n = 0

        self.create_subscription(PoseStamped, self.gt_topic, self._on_gt, 10)
        if self.gt_odom_topic:
            self.create_subscription(Odometry, self.gt_odom_topic,
                                     self._on_gt_odom, 10)
        self.create_subscription(TargetObservation, self.obs_topic,
                                 self._on_obs, 10)
        self.create_timer(self.report_period, self._report)
        self.get_logger().info(
            f'eval_monitor up: {self.obs_topic} vs {self.gt_topic}'
            + (f' / {self.gt_odom_topic}' if self.gt_odom_topic else ''))

    def _on_gt(self, msg):
        self.gt_buffer.append(msg)

    def _on_gt_odom(self, msg):
        self.gt_odom_buffer.append(msg)

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _match(self, buffer, stamp, get_position):
        best, best_dt = None, self.max_match_dt
        for gt in buffer:
            dt = abs(self._stamp(gt) - stamp)
            if dt < best_dt:
                best_dt, best = dt, gt
        return None if best is None else get_position(best)

    def _on_obs(self, msg):
        stamp = self._stamp(msg)
        actual = self._match(
            self.gt_odom_buffer, stamp,
            lambda m: (m.pose.pose.position.x, m.pose.pose.position.y,
                       m.pose.pose.position.z))
        if actual is not None:
            self.errors.append((msg.position.x - actual[0],
                                msg.position.y - actual[1],
                                msg.position.z - actual[2]))
        commanded = self._match(
            self.gt_buffer, stamp,
            lambda m: (m.pose.position.x, m.pose.position.y,
                       m.pose.position.z))
        if commanded is not None:
            self.errors_cmd.append((msg.position.x - commanded[0],
                                    msg.position.y - commanded[1],
                                    msg.position.z - commanded[2]))
        name = {TargetObservation.METHOD_LIDAR_POINTS: 'lidar_points',
                TargetObservation.METHOD_LIDAR_GROUND: 'lidar_ground',
                TargetObservation.METHOD_MONOCULAR: 'monocular'}.get(
                    msg.method, 'unknown')
        self.methods[name] = self.methods.get(name, 0) + 1
        self.quality_sum += msg.quality
        self.quality_n += 1

    def _report(self):
        if not self.errors and not self.errors_cmd:
            self.get_logger().info('no matched observations yet')
            return
        methods = ' '.join(f'{k}={v}' for k, v in sorted(self.methods.items()))
        quality = self.quality_sum / max(1, self.quality_n)
        parts = []
        if self.errors:
            xy, rms, mx, mean_z = _stats(list(self.errors))
            parts.append(f'vs_actual xy={xy * 100:.1f}cm rms={rms * 100:.1f}cm '
                         f'max={mx * 100:.1f}cm dz={mean_z * 100:.1f}cm')
        if self.errors_cmd:
            xy, rms, mx, _ = _stats(list(self.errors_cmd))
            parts.append(f'vs_cmd xy={xy * 100:.1f}cm')
        self.get_logger().info(
            f'n={len(self.errors)} ' + ' | '.join(parts)
            + f' quality={quality:.2f} [{methods}]')
        self.errors.clear()
        self.errors_cmd.clear()
        self.methods.clear()
        self.quality_sum = 0.0
        self.quality_n = 0


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
