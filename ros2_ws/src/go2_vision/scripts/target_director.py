#!/usr/bin/env python3
"""运动导演:让目标沿脚本轨迹移动,并发布真值(teleport,无物理不确定性).

Trajectories (world frame, ball on the ground):
    static | line | sine | circle
Ported from vision_sim for the Go2 vision bench.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class TargetDirector(Node):

    def __init__(self):
        super().__init__('target_director')

        self.declare_parameter('target_name', 'red_ball')
        self.declare_parameter('world_name', 'go2_sim')
        self.declare_parameter('trajectory', 'sine')  # static|line|sine|circle
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('speed', 0.5)        # m/s
        self.declare_parameter('amplitude', 1.0)    # m (line/sine)
        self.declare_parameter('radius', 1.0)       # m (circle)
        self.declare_parameter('center_x', 1.5)     # m
        self.declare_parameter('center_y', 0.0)     # m
        self.declare_parameter('height', 0.15)      # m (ball radius)
        self.declare_parameter('publish_ground_truth', True)

        self.target_name = self.get_parameter('target_name').value
        self.world_name = self.get_parameter('world_name').value
        self.trajectory = self.get_parameter('trajectory').value
        self.speed = float(self.get_parameter('speed').value)
        self.amplitude = float(self.get_parameter('amplitude').value)
        self.radius = float(self.get_parameter('radius').value)
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.height = float(self.get_parameter('height').value)
        self.publish_gt = bool(self.get_parameter('publish_ground_truth').value)
        rate = float(self.get_parameter('rate').value)

        self.set_cli = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')
        self.gt_pub = self.create_publisher(
            PoseStamped, '/target/ground_truth', 10)

        self.t0 = None
        self.pending = None
        self.service_warned = False

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'target_director up: target={self.target_name}, '
            f'trajectory={self.trajectory}, speed={self.speed} m/s')

    def trajectory_position(self, t):
        cx, cy = self.center_x, self.center_y
        if self.trajectory == 'static' or self.speed <= 0.0:
            return cx, cy
        if self.trajectory == 'line':
            period = 2.0 * self.amplitude / self.speed
            p = (t % period) / period
            tri = 4.0 * abs(p - 0.5) - 1.0
            return cx, cy + self.amplitude * tri
        if self.trajectory == 'range':
            period = 2.0 * self.amplitude / self.speed
            p = (t % period) / period
            tri = 4.0 * abs(p - 0.5) - 1.0
            return cx + self.amplitude * tri, cy
        if self.trajectory == 'sine':
            period = 2.0 * math.pi * self.amplitude / self.speed
            return cx, cy + self.amplitude * math.sin(2.0 * math.pi * t / period)
        if self.trajectory == 'circle':
            omega = self.speed / self.radius
            return (cx + self.radius * math.cos(omega * t),
                    cy + self.radius * math.sin(omega * t))
        self.get_logger().warn(
            f'unknown trajectory "{self.trajectory}", using static')
        return cx, cy

    def tick(self):
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9
        x, y = self.trajectory_position(t)

        if self.publish_gt:
            msg = PoseStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'world'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = self.height
            msg.pose.orientation.w = 1.0
            self.gt_pub.publish(msg)

        if self.trajectory == 'static':
            return
        if not self.set_cli.service_is_ready():
            if not self.service_warned:
                self.get_logger().warn(
                    'set_pose service not ready yet; is Gazebo running?')
                self.service_warned = True
            return
        self.service_warned = False
        if self.pending is not None and not self.pending.done():
            return
        req = SetEntityPose.Request()
        req.entity.name = self.target_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = self.height
        req.pose.orientation.w = 1.0
        self.pending = self.set_cli.call_async(req)


def main():
    rclpy.init()
    node = TargetDirector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
