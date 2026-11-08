#!/usr/bin/env python3
"""HSV red-ball detector -> go2_vision/TargetDetection + debug image."""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from go2_vision.msg import TargetDetection


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.image_topic = self.declare_parameter(
            'image_topic', '/go2/camera/image').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/target/detections').value
        self.debug_topic = self.declare_parameter(
            'debug_image_topic', '/target/debug_image').value
        self.class_id = self.declare_parameter('class_id', 'red_ball').value
        self.min_area = self.declare_parameter('min_area', 80.0).value
        self.publish_debug = self.declare_parameter(
            'publish_debug_image', True).value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            TargetDetection, self.detections_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.get_logger().info('ball_detector up: %s' % self.image_topic)

    def _on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 110, 60), (10, 255, 255)) | \
            cv2.inRange(hsv, (170, 110, 60), (180, 255, 255))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_area:
            return
        u0, v0, bw, bh = cv2.boundingRect(contour)
        det = TargetDetection()
        det.header = msg.header
        det.class_id = self.class_id
        det.score = 1.0
        det.x_min = float(u0)
        det.y_min = float(v0)
        det.x_max = float(u0 + bw)
        det.y_max = float(v0 + bh)
        det.center_u = float(u0 + bw / 2.0)
        det.center_v = float(v0 + bh / 2.0)
        det.area = float(area)
        self.pub.publish(det)
        if self.publish_debug:
            cv2.rectangle(bgr, (u0, v0), (u0 + bw, v0 + bh), (0, 255, 0), 2)
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))


def main():
    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
