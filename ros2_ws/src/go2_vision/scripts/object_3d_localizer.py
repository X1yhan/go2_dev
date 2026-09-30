#!/usr/bin/env python3
"""Deproject Detection2D bbox center + depth to 3D (Detection3DArray + markers)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (Detection2DArray, Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray


class Object3DLocalizer(Node):
    def __init__(self):
        super().__init__('object_3d_localizer')
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/vision/detections').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/d435i/color/camera_info').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/vision/detections_3d').value
        self.markers_topic = self.declare_parameter(
            'markers_topic', '/vision/detections_markers').value
        self.publish_markers = self.declare_parameter(
            'publish_markers', True).value
        self.min_depth = self.declare_parameter('min_depth', 0.1).value
        self.max_depth = self.declare_parameter('max_depth', 10.0).value

        self.fx = self.fy = self.cx = self.cy = None
        self.frame_id = None
        self.pub = self.create_publisher(
            Detection3DArray, self.output_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.markers_topic, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_info, qos)
        self.create_subscription(
            Detection2DArray, self.detections_topic, self._on_dets, 10)
        self._n = 0
        self._skipped = 0
        self.get_logger().info(
            f'object_3d_localizer up: {self.detections_topic} -> '
            f'{self.output_topic}')

    def _on_info(self, msg):
        if self.fx is None:
            self.get_logger().info(
                f'camera_info: {msg.width}x{msg.height} '
                f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.frame_id = msg.header.frame_id

    def _on_dets(self, msg):
        if self.fx is None:
            return
        out = Detection3DArray()
        out.header = msg.header
        markers = MarkerArray()
        idx = 0
        for det in msg.detections:
            z = det.results[0].pose.pose.position.z
            if z <= self.min_depth or z > self.max_depth:
                self._skipped += 1
                continue
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            name = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

            d3 = Detection3D()
            d3.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = score
            hyp.pose.pose.position.x = x
            hyp.pose.pose.position.y = y
            hyp.pose.pose.position.z = z
            hyp.pose.pose.orientation.w = 1.0
            d3.results.append(hyp)
            d3.bbox.center.position.x = x
            d3.bbox.center.position.y = y
            d3.bbox.center.position.z = z
            d3.bbox.center.orientation.w = 1.0
            d3.bbox.size.x = det.bbox.size_x * z / self.fx
            d3.bbox.size.y = det.bbox.size_y * z / self.fy
            d3.bbox.size.z = 0.05
            out.detections.append(d3)

            if self.publish_markers:
                cube = Marker()
                cube.header = out.header
                cube.ns = 'objects'
                cube.id = idx * 2
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose = d3.bbox.center
                cube.scale = d3.bbox.size
                cube.color.r = 0.1
                cube.color.g = 1.0
                cube.color.b = 0.2
                cube.color.a = 0.5
                label = Marker()
                label.header = out.header
                label.ns = 'objects'
                label.id = idx * 2 + 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = d3.bbox.center.position.x
                label.pose.position.y = d3.bbox.center.position.y
                label.pose.position.z = (d3.bbox.center.position.z
                                         + d3.bbox.size.y / 2.0 + 0.05)
                label.scale.z = 0.06
                label.color.r = label.color.g = label.color.b = 1.0
                label.color.a = 1.0
                label.text = f'{name} {z:.2f}m'
                markers.markers.extend([cube, label])
            idx += 1

        self.pub.publish(out)
        if self.publish_markers:
            self.marker_pub.publish(markers)
        self._n += 1
        if self._n % 60 == 0:
            self.get_logger().info(
                f'{self._n} frames, {len(out.detections)} objects, '
                f'{self._skipped} skipped (no/invalid depth)')


def main():
    rclpy.init()
    node = Object3DLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
