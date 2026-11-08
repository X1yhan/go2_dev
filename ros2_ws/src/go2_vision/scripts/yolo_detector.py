#!/usr/bin/env python3
"""YOLO common-object detector -> Detection2DArray (depth in pose.position.z) + annotated image."""
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        default_model = os.path.expanduser('~/go2_dev/models/yolo11n.pt')
        self.model_path = self.declare_parameter('model', default_model).value
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/d435i/color/image_raw').value
        self.depth_topic = self.declare_parameter(
            'depth_topic',
            '/camera/d435i/aligned_depth_to_color/image_raw').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/vision/detections').value
        self.debug_topic = self.declare_parameter(
            'debug_image_topic', '/vision/image_annotated').value
        self.conf = self.declare_parameter('confidence', 0.35).value
        self.device = str(self.declare_parameter('device', '0').value)
        self.imgsz = self.declare_parameter('imgsz', 640).value
        self.classes = self.declare_parameter('classes', ['']).value
        self.publish_debug = self.declare_parameter(
            'publish_annotated', True).value
        self.use_depth = self.declare_parameter('use_depth', True).value
        self.max_depth = self.declare_parameter('max_depth', 10.0).value
        self.min_depth_px = self.declare_parameter('min_depth_px', 30).value

        from ultralytics import YOLO
        self.model = YOLO(self.model_path)
        self.names = self.model.names
        self.keep = [c for c in self.classes if c]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.bridge = CvBridge()
        self.latest_depth = None
        self.pub = self.create_publisher(
            Detection2DArray, self.detections_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, qos)
        if self.use_depth:
            self.create_subscription(
                Image, self.depth_topic, self._on_depth, qos)
        self._n = 0
        self._t0 = time.time()
        self.get_logger().info(
            f'yolo_detector up: {self.image_topic} <- {self.model_path}'
            + (f', depth: {self.depth_topic}' if self.use_depth else ''))

    def _on_depth(self, msg):
        self.latest_depth = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1'))

    def _object_depth(self, depth, x1, y1, x2, y2):
        h, w = depth.shape
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        bw, bh = (x2 - x1) * 0.6, (y2 - y1) * 0.6
        u0 = max(int(cx - bw / 2.0), 0)
        u1 = min(int(cx + bw / 2.0) + 1, w)
        v0 = max(int(cy - bh / 2.0), 0)
        v1 = min(int(cy + bh / 2.0) + 1, h)
        limit = self.max_depth * 1000.0
        region = depth[v0:v1, u0:u1]
        valid = region[(region > 0) & (region <= limit)]
        if valid.size < self.min_depth_px:
            region = depth[max(int(y1), 0):min(int(y2) + 1, h),
                           max(int(x1), 0):min(int(x2) + 1, w)]
            valid = region[(region > 0) & (region <= limit)]
        if valid.size < self.min_depth_px:
            return 0.0, 0.0
        return float(np.median(valid)) / 1000.0, \
            valid.size / float(max(region.size, 1))

    def _on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.model.predict(
            bgr, conf=self.conf, imgsz=self.imgsz, device=self.device,
            verbose=False)[0]
        annotated = result.plot() if self.publish_debug else None

        depth = None
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.latest_depth is not None and \
                abs(self.latest_depth[0] - stamp) < 0.1:
            depth = self.latest_depth[1]

        out = Detection2DArray()
        out.header = msg.header
        info = []
        for box in result.boxes:
            name = self.names[int(box.cls)]
            if self.keep and name not in self.keep:
                continue
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            depth_m, ratio = 0.0, 0.0
            if depth is not None:
                depth_m, ratio = self._object_depth(depth, x1, y1, x2, y2)
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = float(box.conf)
            hyp.pose.pose.position.z = depth_m
            det.results.append(hyp)
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            det.bbox = bbox
            out.detections.append(det)
            info.append(f'{name} {float(box.conf):.2f} '
                        f'{depth_m:.2f}m({ratio:.0%})')
            if annotated is not None:
                y_txt = min(int(y2) + 20, annotated.shape[0] - 5)
                text = f'{depth_m:.2f}m' if depth_m > 0 else 'no depth'
                cv2.putText(annotated, text, (int(x1) + 2, y_txt),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        self.pub.publish(out)
        if annotated is not None:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
        self._n += 1
        if self._n % 30 == 0:
            fps = self._n / (time.time() - self._t0)
            self.get_logger().info(
                f'{self._n} frames, fps={fps:.1f}, dets='
                + ('; '.join(info) if info else '0'))


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
