#!/usr/bin/env python3
"""3D localization of the detected target.

Fuses the 2D detection with the D435i depth image:

    bbox -> robust median depth -> deprojection with camera intrinsics
         -> 3D point in camera_depth_optical_frame
         -> optional transform to the world frame via TF

Outputs:
    /target/position_camera  geometry_msgs/PointStamped (optical frame)
    /target/position_world   geometry_msgs/PointStamped (world frame)
    /target/marker           visualization_msgs/Marker
"""
import math

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker
from vision_sim.msg import TargetDetection


class TargetLocalizer(Node):

    def __init__(self):
        super().__init__('target_localizer')

        self.declare_parameter('detections_topic', '/target/detections')
        self.declare_parameter('depth_topic', '/viz/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/viz/camera/camera_info')
        self.declare_parameter('depth_frame', 'camera_depth_optical_frame')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('position_camera_topic', '/target/position_camera')
        self.declare_parameter('position_world_topic', '/target/position_world')
        self.declare_parameter('marker_topic', '/target/marker')
        self.declare_parameter('min_depth', 0.05)     # m
        self.declare_parameter('max_depth', 10.0)     # m
        self.declare_parameter('patch_fraction', 0.5)  # central bbox fraction
        self.declare_parameter('max_stamp_dt', 0.2)   # s, detection <-> depth
        self.declare_parameter('publish_world', True)
        # for spherical targets the depth image sees the front surface; add
        # the radius along the viewing ray to recover the center. 0 = off.
        self.declare_parameter('target_radius', 0.0)  # m

        self.depth_frame = self.get_parameter('depth_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.patch_fraction = float(self.get_parameter('patch_fraction').value)
        self.max_stamp_dt = float(self.get_parameter('max_stamp_dt').value)
        self.publish_world = bool(self.get_parameter('publish_world').value)
        self.target_radius = float(self.get_parameter('target_radius').value)

        self.bridge = CvBridge()
        self.depth = None
        self.depth_stamp = None
        self.camera_info = None
        self.missed = 0
        self.located = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            TargetDetection, self.get_parameter('detections_topic').value,
            self.on_detection, 10)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.on_depth, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, 10)

        self.pos_cam_pub = self.create_publisher(
            PointStamped, self.get_parameter('position_camera_topic').value, 10)
        self.pos_world_pub = self.create_publisher(
            PointStamped, self.get_parameter('position_world_topic').value, 10)
        self.marker_pub = self.create_publisher(
            Marker, self.get_parameter('marker_topic').value, 10)

        self.get_logger().info('target_localizer up')

    def on_depth(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            self.depth_stamp = msg.header.stamp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'depth conversion failed: {exc}')

    def on_camera_info(self, msg):
        self.camera_info = msg

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_detection(self, det):
        if self.depth is None or self.camera_info is None:
            return

        dt = abs(self._stamp_seconds(det.header.stamp)
                 - self._stamp_seconds(self.depth_stamp))
        if dt > self.max_stamp_dt:
            self.missed += 1
            self.get_logger().warn(
                f'depth/detection stamp mismatch {dt:.3f}s, skipped',
                throttle_duration_sec=2.0)
            return

        depth = self.depth
        h, w = depth.shape[:2]
        u0 = max(0, min(w - 1, int(round(det.x_min))))
        u1 = max(0, min(w, int(round(det.x_max))))
        v0 = max(0, min(h - 1, int(round(det.y_min))))
        v1 = max(0, min(h, int(round(det.y_max))))
        if u1 <= u0 or v1 <= v0:
            return

        # central patch of the bbox: less likely to contain background pixels
        if self.patch_fraction < 1.0:
            cu = 0.5 * (u0 + u1)
            cv = 0.5 * (v0 + v1)
            du = 0.5 * (u1 - u0) * self.patch_fraction
            dv = 0.5 * (v1 - v0) * self.patch_fraction
            u0, u1 = int(round(cu - du)), int(round(cu + du))
            v0, v1 = int(round(cv - dv)), int(round(cv + dv))

        patch = depth[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > self.min_depth) & (valid < self.max_depth)]
        if valid.size == 0:
            self.missed += 1
            self.get_logger().warn('no valid depth in bbox',
                                   throttle_duration_sec=2.0)
            return

        z = float(np.median(valid))

        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        u = det.center_u
        v = det.center_v
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # depth sees the front surface of a sphere: offset to its center
        if self.target_radius > 0.0:
            norm = math.sqrt(x * x + y * y + z * z)
            if norm > 1e-6:
                x += self.target_radius * x / norm
                y += self.target_radius * y / norm
                z += self.target_radius * z / norm

        msg = PointStamped()
        msg.header = det.header
        msg.header.frame_id = self.depth_frame
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.pos_cam_pub.publish(msg)

        marker = Marker()
        marker.header = msg.header
        marker.ns = 'target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b = 0.0, 0.8, 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

        self.located += 1
        if self.publish_world:
            self.publish_in_world(msg)

        if self.located % 100 == 0:
            self.get_logger().info(
                f'located {self.located} samples, {self.missed} misses, '
                f'depth={z:.3f} m')

    def publish_in_world(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.depth_frame, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:  # noqa: BLE001
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        # rotation matrix from quaternion
        xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
        xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
        wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
        rot = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ])
        p_cam = np.array([msg.point.x, msg.point.y, msg.point.z])
        p_world = rot @ p_cam + np.array([t.x, t.y, t.z])

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame
        out.point.x = float(p_world[0])
        out.point.y = float(p_world[1])
        out.point.z = float(p_world[2])
        self.pos_world_pub.publish(out)


def main():
    rclpy.init()
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
