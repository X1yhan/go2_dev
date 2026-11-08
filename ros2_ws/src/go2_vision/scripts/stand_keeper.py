#!/usr/bin/env python3
"""保持站立:持续发零速度,让步态节点(walk:=true)稳住站姿.

视觉测试用:步态节点把狗从 spawn 高度收到标准站姿,相机高度稳定、目标在画面内.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class StandKeeper(Node):
    def __init__(self):
        super().__init__('stand_keeper')
        self.rate = self.declare_parameter('rate', 10.0).value
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info('stand_keeper up: zero /cmd_vel @ %.0f Hz'
                               % self.rate)

    def _tick(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = StandKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
