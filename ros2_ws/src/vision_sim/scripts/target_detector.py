#!/usr/bin/env python3
"""Color-based target detector (development stage of the vision bench).

Subscribes the simulated D435i color image and publishes a TargetDetection
with the largest matching blob. HSV thresholds are parameters, so the
same node works for other target colors; the detector is meant to be
swapped for a learned model later (same message interface).

Also publishes an annotated debug image.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_sim.msg import TargetDetection


class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector')

        self.declare_parameter('image_topic', '/viz/camera/image')
        self.declare_parameter('detections_topic', '/target/detections')
        self.declare_parameter('debug_image_topic', '/target/debug_image')
        self.declare_parameter('class_id', 'target_ball')
        # default: red ball, two hue ranges (red wraps around 0/180)
        self.declare_parameter('hsv_lower_1', [0, 120, 70])
        self.declare_parameter('hsv_upper_1', [10, 255, 255])
        self.declare_parameter('hsv_lower_2', [170, 120, 70])
        self.declare_parameter('hsv_upper_2', [180, 255, 255])
        self.declare_parameter('min_area', 100.0)     # px^2
        self.declare_parameter('max_area', 0.0)       # 0 = no limit
        self.declare_parameter('morph_kernel', 5)     # 0 = disabled
        self.declare_parameter('publish_debug_image', True)

        image_topic = self.get_parameter('image_topic').value
        det_topic = self.get_parameter('detections_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value

        lo1 = np.array(self.get_parameter('hsv_lower_1').value, dtype=np.uint8)
        up1 = np.array(self.get_parameter('hsv_upper_1').value, dtype=np.uint8)
        lo2 = np.array(self.get_parameter('hsv_lower_2').value, dtype=np.uint8)
        up2 = np.array(self.get_parameter('hsv_upper_2').value, dtype=np.uint8)
        self.ranges = [(lo1, up1), (lo2, up2)]
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area = float(self.get_parameter('max_area').value)
        self.kernel = int(self.get_parameter('morph_kernel').value)
        self.publish_debug = bool(
            self.get_parameter('publish_debug_image').value)
        self.class_id = self.get_parameter('class_id').value

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(TargetDetection, det_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(Image, image_topic, self.on_image, 10)

        self.frames = 0
        self.detected = 0
        self.get_logger().info(
            f'target_detector up: {image_topic} -> {det_topic}')

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')
            return

        self.frames += 1
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = None
        for lo, up in self.ranges:
            m = cv2.inRange(hsv, lo, up)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        if self.kernel > 0:
            k = np.ones((self.kernel, self.kernel), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            if self.max_area > 0.0 and area > self.max_area:
                continue
            if area > best_area:
                best_area = area
                best = contour

        debug = image.copy() if self.publish_debug else None

        if best is not None:
            u, v, w, h = cv2.boundingRect(best)
            det = TargetDetection()
            det.header = msg.header
            det.class_id = self.class_id
            det.score = 1.0
            det.x_min = float(u)
            det.y_min = float(v)
            det.x_max = float(u + w)
            det.y_max = float(v + h)
            det.center_u = float(u + w / 2.0)
            det.center_v = float(v + h / 2.0)
            det.area = float(best_area)
            self.det_pub.publish(det)
            self.detected += 1
            if debug is not None:
                cv2.rectangle(debug, (u, v), (u + w, v + h), (0, 255, 0), 2)
                cv2.circle(debug, (int(det.center_u), int(det.center_v)),
                           3, (255, 0, 0), -1)

        if debug is not None:
            try:
                self.debug_pub.publish(
                    self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
            except Exception:  # noqa: BLE001
                pass

        if self.frames % 300 == 0:
            self.get_logger().info(
                f'{self.frames} frames, {self.detected} detections')


def main():
    rclpy.init()
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
