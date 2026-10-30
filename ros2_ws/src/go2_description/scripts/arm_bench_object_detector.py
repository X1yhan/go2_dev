#!/usr/bin/env python3
"""Multi-object detector for the standalone arm bench.

Detects the four color-coded tabletop objects, labels them
(red_ball / blue_cube / green_cylinder / yellow_box), measures their bbox
depth and publishes:

    /arm/vision/detections        vision_msgs/Detection2DArray
    /arm/vision/image_annotated   sensor_msgs/Image

The grasp script picks the target by label.  The Detection2DArray interface is
the same one used by the real-camera YOLO pipeline, so a learned detector can
replace this color/shape detector later.
"""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

HSV_RANGES = {
    'red_ball': [((0, 70, 40), (12, 255, 255)),
                 ((168, 70, 40), (180, 255, 255))],
    'blue_cube': [((95, 80, 40), (135, 255, 255))],
    'green_cylinder': [((35, 60, 40), (90, 255, 255))],
    'yellow_box': [((18, 80, 50), (38, 255, 255))],
}
DRAW_COLORS = {
    'red_ball': (0, 0, 255),
    'blue_cube': (255, 100, 0),
    'green_cylinder': (0, 200, 0),
    'yellow_box': (0, 220, 220),
}


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('arm_bench_object_detector')
        self.declare_parameter('image_topic', '/arm/camera/image')
        self.declare_parameter('depth_topic', '/arm/camera/depth_image')
        self.declare_parameter('detections_topic', '/arm/vision/detections')
        self.declare_parameter('debug_image_topic',
                               '/arm/vision/image_annotated')
        self.declare_parameter('min_area', 120)
        self.declare_parameter('max_depth', 3.0)
        self.declare_parameter('publish_debug', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        det_topic = self.get_parameter('detections_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)

        self.bridge = CvBridge()
        self.depth = None
        self.pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.create_subscription(Image, self.image_topic,
                                 self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic,
                                 self.on_depth, qos_profile_sensor_data)
        self._n = 0
        self._t0 = time.time()
        self.get_logger().info(
            'object detector up: %s + %s -> %s' %
            (self.image_topic, self.depth_topic, det_topic))

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def _depth_median(self, u, v, bw, bh):
        if self.depth is None:
            return 0.0, 0.0
        h, w = self.depth.shape
        half_w, half_h = bw * 0.3, bh * 0.3
        u0 = max(int(u - half_w), 0)
        u1 = min(int(u + half_w) + 1, w)
        v0 = max(int(v - half_h), 0)
        v1 = min(int(v + half_h) + 1, h)
        region = self.depth[v0:v1, u0:u1]
        valid = region[np.isfinite(region)]
        valid = valid[(valid > 0.05) & (valid < self.max_depth)]
        if valid.size == 0:
            return 0.0, 0.0
        return float(np.median(valid)), valid.size / float(max(region.size, 1))

    def _detect(self, bgr, out, annotated):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        detections = []
        for name, ranges in HSV_RANGES.items():
            mask = None
            for lo, hi in ranges:
                part = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            u, v = x + bw / 2.0, y + bh / 2.0
            depth_m, ratio = self._depth_median(u, v, bw, bh)

            det = Detection2D()
            det.header = out.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = min(1.0, area / 4000.0)
            hyp.pose.pose.position.z = depth_m
            det.results.append(hyp)
            bbox = BoundingBox2D()
            bbox.center.position.x = u
            bbox.center.position.y = v
            bbox.center.theta = 0.0
            bbox.size_x = float(bw)
            bbox.size_y = float(bh)
            det.bbox = bbox
            detections.append(det)

            if annotated is not None:
                cv2.rectangle(annotated, (x, y), (x + bw, y + bh),
                              DRAW_COLORS[name], 2)
                cv2.circle(annotated, (int(u), int(v)), 3,
                           DRAW_COLORS[name], -1)
                text = '%s %.2fm' % (name, depth_m) if depth_m > 0 \
                    else '%s no-depth' % name
                cv2.putText(annotated, text, (x, max(y - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            DRAW_COLORS[name], 2)
                self.get_logger().debug(
                    '%s area=%.0f depth=%.3f valid=%.0f%%' %
                    (name, area, depth_m, ratio * 100.0))
        detections.sort(key=lambda d: d.bbox.size_x * d.bbox.size_y,
                        reverse=True)
        out.detections = detections
        return [d.results[0].hypothesis.class_id for d in detections]

    def on_image(self, msg):
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        out = Detection2DArray()
        out.header = msg.header
        annotated = bgr.copy() if self.publish_debug else None
        names = self._detect(bgr, out, annotated)
        self.pub.publish(out)
        if annotated is not None:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        self._n += 1
        if self._n % 30 == 0:
            fps = self._n / max(time.time() - self._t0, 1e-6)
            self.get_logger().info(
                '%d frames, fps=%.1f, objects=%s' %
                (self._n, fps, ','.join(names) if names else 'none'))


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
