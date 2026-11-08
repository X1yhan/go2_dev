#!/usr/bin/env python3
"""视觉伺服(follow-me):用 /target/position 闭环控制底盘跟随目标.

控制律(每周期):
    e_yaw = atan2(y, x)              -> wz = clamp(kp_yaw * e_yaw)
    e_dist = hypot(x, y) - d_dock    -> vx = clamp(kp_dist * e_dist)
目标偏得越多前进越慢;距离/方位进入死区则停。
目标丢失 > lost_timeout 时进入 SEARCH(朝最后方位慢转),再超时则停。
输出 /cmd_vel,可随时用键盘遥控接管(别同时开).
"""
import math

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from visualization_msgs.msg import Marker

from go2_vision.msg import TargetDetection, TargetObservation


def clamp(value, limit):
    return max(-limit, min(limit, value))


class VisualServo(Node):
    def __init__(self):
        super().__init__('visual_servo')
        self.obs_topic = self.declare_parameter(
            'observation_topic', '/target/position').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/target/detections').value
        self.image_cx = self.declare_parameter('image_cx', 640.0).value
        self.image_fx = self.declare_parameter('image_fx', 864.4).value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/target/marker').value
        self.d_dock = self.declare_parameter('d_dock', 1.2).value
        self.d_safe = self.declare_parameter('d_safe', 1.05).value
        self.kp_yaw = self.declare_parameter('kp_yaw', 1.2).value
        self.kp_dist = self.declare_parameter('kp_dist', 0.6).value
        self.vx_max = self.declare_parameter('vx_max', 0.30).value
        self.wz_max = self.declare_parameter('wz_max', 0.8).value
        self.deadband = self.declare_parameter('dist_deadband', 0.08).value
        self.yaw_deadband = self.declare_parameter('yaw_deadband', 0.03).value
        self.lost_timeout = self.declare_parameter('lost_timeout', 0.6).value
        self.det_timeout = self.declare_parameter('det_timeout', 0.4).value
        self.search_timeout = self.declare_parameter(
            'search_timeout', 5.0).value
        self.smooth = self.declare_parameter('smoothing', 0.3).value
        rate = self.declare_parameter('rate', 30.0).value

        self.cmd = Twist()
        self.last_pos = None
        self.last_dist = None
        self.last_bearing = 0.0
        self.last_seen = None
        self.last_det_u = None
        self.last_det_seen = None
        self.search_until = None

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.create_subscription(TargetObservation, self.obs_topic,
                                 self._on_obs, 10)
        self.create_subscription(TargetDetection, self.detections_topic,
                                 self._on_det, 10)
        self.create_timer(1.0 / rate, self._control)
        self.create_timer(0.2, self._publish_marker)
        self.get_logger().info(
            f'visual_servo up: dock={self.d_dock} m, '
            f'kp_yaw={self.kp_yaw}, kp_dist={self.kp_dist}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_obs(self, msg):
        self.last_pos = (msg.position.x, msg.position.y, msg.position.z)
        self.last_seen = self._now()
        self.search_until = None

    def _on_det(self, msg):
        self.last_det_u = msg.center_u
        self.last_det_seen = self._now()

    def _control(self):
        now = self._now()
        last_seen = 0.0
        if self.last_seen is not None:
            last_seen = now - self.last_seen

        if self.last_pos is None or last_seen > self.lost_timeout:
            if self.last_det_seen is not None \
                    and now - self.last_det_seen <= self.det_timeout:
                e_yaw = -math.atan((self.last_det_u - self.image_cx)
                                   / self.image_fx)
                self.last_bearing = e_yaw
                wz = clamp(self.kp_yaw * e_yaw, self.wz_max)
                if abs(e_yaw) < self.yaw_deadband:
                    wz = 0.0
                self._publish(0.0, wz)
                return
            self._search(now)
            return

        x, y, _ = self.last_pos
        dist = math.hypot(x, y)
        self.last_dist = dist
        self.last_bearing = math.atan2(y, x)

        e_yaw = self.last_bearing
        e_dist = dist - self.d_dock

        wz = clamp(self.kp_yaw * e_yaw, self.wz_max)
        if abs(e_yaw) > 1.2:
            vx = 0.0
        else:
            vx = clamp(self.kp_dist * e_dist, self.vx_max) * math.cos(e_yaw)
        if abs(e_dist) < self.deadband:
            vx = 0.0
        if dist < self.d_safe:
            vx = -self.vx_max
        if abs(e_yaw) < self.yaw_deadband:
            wz = 0.0

        self._publish(vx, wz)

    def _search(self, now):
        if self.last_pos is None:
            self._publish(0.0, 0.0)
            return
        if self.search_until is None:
            self.search_until = now + self.search_timeout
        if now >= self.search_until:
            self._publish(0.0, 0.0)
            return
        if self.last_dist is not None and self.last_dist < self.d_safe + 0.2:
            self._publish(-0.6 * self.vx_max, 0.0)
            return
        direction = 1.0 if self.last_bearing >= 0.0 else -1.0
        self._publish(0.0, 0.35 * direction)

    def _publish(self, vx, wz):
        a = self.smooth
        self.cmd.linear.x += a * (vx - self.cmd.linear.x)
        self.cmd.angular.z += a * (wz - self.cmd.angular.z)
        self.cmd.linear.x = clamp(self.cmd.linear.x, self.vx_max)
        self.cmd.angular.z = clamp(self.cmd.angular.z, self.wz_max)
        self.pub.publish(self.cmd)

    def _publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'base'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.a = 0.8
        marker.color.g = 1.0
        if self.last_pos is not None:
            marker.pose.position.x = self.last_pos[0]
            marker.pose.position.y = self.last_pos[1]
            marker.pose.position.z = self.last_pos[2]
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = VisualServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
