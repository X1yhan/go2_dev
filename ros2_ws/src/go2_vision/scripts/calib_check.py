#!/usr/bin/env python3
"""检查相机-雷达-地面几何是否标定齐(仿真/真机通用).

- 用 TF 把点云投到图像上,叠加保存(看对齐)
- 从点云拟合地面平面
- HSV 检测红球,取 bbox 底边中点发射线,与地面求交 -> 球心
- 与真值(参数给定)比较,打印像素/米制残差

Usage:
    ros2 run go2_vision calib_check.py
    ros2 run go2_vision calib_check.py --ros-args -p ball_x:=2.0
"""
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

R_SENSOR_OPTICAL = None  # filled in _rot_sensor_optical


def _rot_sensor_optical():
    roll, pitch, yaw = -math.pi / 2.0, 0.0, -math.pi / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _tf_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    mat = np.eye(4)
    mat[:3, :3] = rot
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def fit_plane(points, thresh=0.02, iters=300, seed=0):
    n = len(points)
    if n < 50:
        return None
    rng = np.random.default_rng(seed)
    best = None
    best_count = 0
    for _ in range(iters):
        idx = rng.integers(0, n, 3)
        p1, p2, p3 = points[idx]
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -float(normal @ p1)
        count = int(np.sum(np.abs(points @ normal + d) < thresh))
        if count > best_count:
            best_count = count
            best = (normal, d)
    if best is None:
        return None
    normal, d = best
    inliers = points[np.abs(points @ normal + d) < thresh]
    centroid = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - centroid)
    normal = vh[2] / np.linalg.norm(vh[2])
    d = -float(normal @ centroid)
    if normal[2] < 0:
        normal, d = -normal, -d
    return normal, d, best_count


class CalibCheck(Node):
    def __init__(self):
        super().__init__('calib_check')
        self.image_topic = self.declare_parameter(
            'image_topic', '/go2/camera/image').value
        self.info_topic = self.declare_parameter(
            'camera_info_topic', '/go2/camera/camera_info').value
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/go2/lidar/points').value
        self.base_frame = self.declare_parameter('base_frame', 'base').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'go2/base/front_camera').value
        self.lidar_frame = self.declare_parameter(
            'lidar_frame', 'go2/base/utlidar_lidar').value
        self.ball_x = self.declare_parameter('ball_x', 1.5).value
        self.ball_y = self.declare_parameter('ball_y', 0.0).value
        self.ball_z = self.declare_parameter('ball_z', 0.15).value
        self.ball_radius = self.declare_parameter('ball_radius', 0.15).value
        self.output = self.declare_parameter(
            'output', '/tmp/opencode/calib_overlay.png').value
        self.duration = self.declare_parameter('duration', 5.0).value

        self.bridge = CvBridge()
        self.image = None
        self.info = None
        self.cloud = None
        self._rot_so = _rot_sensor_optical()

        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 10)
        self.create_subscription(PointCloud2, self.cloud_topic,
                                 self._on_cloud, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.get_logger().info('collecting data ...')

    def _on_image(self, msg):
        self.image = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_cloud(self, msg):
        self.cloud = msg

    def _lookup(self, parent, child):
        for _ in range(50):
            try:
                return _tf_matrix(self.buffer.lookup_transform(
                    parent, child, rclpy.time.Time()))
            except Exception:
                time.sleep(0.1)
        raise RuntimeError(f'no TF {parent} -> {child}')

    def _detect_ball(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 110, 60), (10, 255, 255)) | \
            cv2.inRange(hsv, (170, 110, 60), (180, 255, 255))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 100:
            return None
        return cv2.boundingRect(contour)

    def process(self):
        if self.image is None or self.info is None or self.cloud is None:
            self.get_logger().error('missing image / camera_info / cloud')
            return False
        bgr = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        h, w = bgr.shape[:2]

        k = np.array(self.info.k).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]

        t_base_cam = self._lookup(self.base_frame, self.camera_frame)
        t_base_lidar = self._lookup(self.base_frame, self.lidar_frame)

        cloud_xyz = np.array([
            (p[0], p[1], p[2]) for p in point_cloud2.read_points(
                self.cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        ], dtype=np.float32)
        finite = cloud_xyz[np.isfinite(cloud_xyz).all(axis=1)]
        pts_base = (t_base_lidar[:3, :3] @ finite.T).T + t_base_lidar[:3, 3]

        r_bc = t_base_cam[:3, :3]
        t_bc = t_base_cam[:3, 3]
        pts_cam = (r_bc.T @ (pts_base - t_bc).T).T
        pts_opt = (self._rot_so.T @ pts_cam.T).T

        overlay = bgr.copy()
        vis = pts_opt[(pts_opt[:, 2] > 0.2) & (pts_opt[:, 2] < 20.0)]
        u = (fx * vis[:, 0] / vis[:, 2] + cx).astype(int)
        v = (fy * vis[:, 1] / vis[:, 2] + cy).astype(int)
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        for uu, vv, dd in zip(u[inside], v[inside], vis[inside, 2]):
            color = (0, 255, 0) if dd < 2.0 else (0, 200, 255)
            cv2.circle(overlay, (int(uu), int(vv)), 1, color, -1)

        plane = fit_plane(pts_base[np.abs(pts_base[:, 2]) < 1.0], thresh=0.02)
        if plane is None:
            self.get_logger().error('ground plane fit failed')
            return False
        normal, d, inliers = plane

        bbox = self._detect_ball(bgr)
        lines = [
            'plane n=(%.3f %.3f %.3f) d=%.3f inliers=%d'
            % (normal[0], normal[1], normal[2], d, inliers),
        ]
        # world (ground z=0) -> base frame via the fitted ground plane
        z_ground_base = -(normal[0] * self.ball_x
                          + normal[1] * self.ball_y + d) / normal[2]
        gt_base = np.array([self.ball_x, self.ball_y,
                            self.ball_z + z_ground_base])

        if bbox is not None:
            u0, v0, bw, bh = bbox
            uc, vc = u0 + bw / 2.0, v0 + bh / 2.0
            v_bottom = v0 + bh
            cv2.rectangle(overlay, (u0, v0), (u0 + bw, v0 + bh),
                          (0, 255, 0), 2)
            o_base = t_bc
            p_opt = np.array([(uc - cx) / fx, (v_bottom - cy) / fy, 1.0])
            d_base = r_bc @ (self._rot_so @ p_opt)
            denom = float(normal @ d_base)
            if abs(denom) > 1e-6:
                s = -(float(normal @ o_base) + d) / denom
                contact = o_base + s * d_base
                center = contact + np.array([0.0, 0.0, self.ball_radius])
                err = float(np.linalg.norm(center - gt_base))
                lines.append('ball bbox=(%d,%d,%d,%d) center_px=(%.1f,%.1f)'
                             % (u0, v0, bw, bh, uc, vc))
                lines.append('ray(bottom)->contact=(%.3f %.3f) err_xy=%.3f m'
                             % (contact[0], contact[1],
                                math.hypot(contact[0] - self.ball_x,
                                           contact[1] - self.ball_y)))
                lines.append('fused center=(%.3f %.3f %.3f) err=%.3f m'
                             % (center[0], center[1], center[2], err))
        else:
            lines.append('ball NOT detected')

        gt_opt = self._rot_so.T @ (r_bc.T @ (gt_base - t_bc))
        if gt_opt[2] > 0.1:
            gu = fx * gt_opt[0] / gt_opt[2] + cx
            gv = fy * gt_opt[1] / gt_opt[2] + cy
            r_px = fx * self.ball_radius / gt_opt[2]
            cv2.circle(overlay, (int(gu), int(gv)), int(r_px), (255, 0, 255), 2)
            lines.append('GT center_px=(%.1f,%.1f) radius=%.1f px'
                         % (gu, gv, r_px))
            if bbox is not None:
                du, dv = uc - gu, vc - gv
                lines.append('pixel residual=(%.1f,%.1f) |%.1f| px'
                             % (du, dv, math.hypot(du, dv)))

        for i, text in enumerate(lines):
            cv2.putText(overlay, text, (8, 24 + 22 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
        cv2.imwrite(self.output, overlay)
        print('\n'.join('[calib] ' + t for t in lines))
        print('[calib] overlay saved: %s' % self.output)
        return True


def main():
    rclpy.init()
    node = CalibCheck()
    deadline = time.time() + node.duration
    ok = False
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.image is not None and node.cloud is not None \
                and node.info is not None:
            time.sleep(1.0)
            ok = node.process()
            break
    if not ok:
        print('[calib] failed (no data collected)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
