#!/usr/bin/env python3
"""相机 + 雷达融合定位:检测框 + 点云 + 地面平面 -> 目标 3D.

方法优先级:
  1. bbox 内雷达点(球面圆内)取中位数
  2. bbox 底边射线 x 雷达地面平面,再按目标半径补到球心(默认优先,红球场景)
  3. 单目尺寸测距兜底
输出 alpha-beta 滤波后的 base/world 系位置(go2_vision/TargetObservation).
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from go2_vision.msg import TargetDetection, TargetObservation


def rot_sensor_optical():
    roll, pitch, yaw = -math.pi / 2.0, 0.0, -math.pi / 2.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def tf_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    mat = np.eye(4)
    mat[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def fit_plane(points, thresh=0.02, iters=300, seed=0):
    n = len(points)
    if n < 50:
        return None
    rng = np.random.default_rng(seed)
    best, best_count = None, 0
    for _ in range(iters):
        idx = rng.integers(0, n, 3)
        p1, p2, p3 = points[idx]
        normal = np.cross(p2 - p1, p3 - p1)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        d = -float(normal @ p1)
        count = int(np.sum(np.abs(points @ normal + d) < thresh))
        if count > best_count:
            best_count, best = count, (normal, d)
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


class TargetLocalizer(Node):
    def __init__(self):
        super().__init__('target_localizer')
        self.detections_topic = self.declare_parameter(
            'detections_topic', '/target/detections').value
        self.info_topic = self.declare_parameter(
            'camera_info_topic', '/go2/camera/camera_info').value
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/go2/lidar/points').value
        self.odom_topic = self.declare_parameter(
            'odom_topic', '/model/go2/odometry').value
        self.base_frame = self.declare_parameter('base_frame', 'base').value
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'go2/base/front_camera').value
        self.lidar_frame = self.declare_parameter(
            'lidar_frame', 'go2/base/utlidar_lidar').value
        self.target_radius = self.declare_parameter(
            'target_radius', 0.15).value
        self.prefer_ground = self.declare_parameter(
            'prefer_ground', True).value
        self.allow_clipped = self.declare_parameter(
            'allow_clipped', False).value
        self.min_points = int(self.declare_parameter('min_points', 4).value)
        self.min_depth = self.declare_parameter('min_depth', 0.3).value
        self.max_depth = self.declare_parameter('max_depth', 20.0).value
        self.max_match_dt = self.declare_parameter(
            'max_match_dt', 0.3).value
        self.alpha = self.declare_parameter('filter_alpha', 0.55).value
        self.beta = self.declare_parameter('filter_beta', 0.12).value
        self.plane_period = self.declare_parameter('plane_period', 1.0).value

        self.bridge_rot = rot_sensor_optical()
        self.info = None
        self.cloud_stamp = None
        self.cloud_base = None
        self.cloud_lidar = None
        self.odom = None
        self.odom_stamp = None
        self.plane = None
        self.plane_stamp = None
        self.t_base_cam = None
        self.t_base_lidar = None
        self.filter_pos = None
        self.filter_vel = np.zeros(3)
        self.filter_stamp = None

        self.create_subscription(CameraInfo, self.info_topic, self._on_info, 10)
        self.create_subscription(PointCloud2, self.cloud_topic,
                                 self._on_cloud, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(TargetDetection, self.detections_topic,
                                 self._on_detection, 10)
        self.pub_base = self.create_publisher(
            TargetObservation, '/target/position', 10)
        self.pub_world = self.create_publisher(
            TargetObservation, '/target/position_world', 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(1.0 / self.plane_period, self._update_plane)
        self.get_logger().info('target_localizer up')

    def _on_info(self, msg):
        self.info = msg

    def _on_odom(self, msg):
        self.odom = msg
        self.odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_cloud(self, msg):
        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        arr = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32)
        arr = arr[np.isfinite(arr).all(axis=1)]
        self.cloud_lidar = arr
        self.cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _lookup(self, parent, child):
        for _ in range(50):
            try:
                return tf_matrix(self.buffer.lookup_transform(
                    parent, child, rclpy.time.Time()))
            except Exception:
                time.sleep(0.1)
        raise RuntimeError('no TF %s -> %s' % (parent, child))

    def _transforms(self):
        if self.t_base_cam is None:
            self.t_base_cam = self._lookup(self.base_frame, self.camera_frame)
        if self.t_base_lidar is None:
            self.t_base_lidar = self._lookup(self.base_frame, self.lidar_frame)
        return self.t_base_cam, self.t_base_lidar

    def _update_plane(self):
        if self.cloud_lidar is None:
            return
        try:
            _, t_base_lidar = self._transforms()
        except RuntimeError:
            return
        pts = (t_base_lidar[:3, :3] @ self.cloud_lidar.T).T + t_base_lidar[:3, 3]
        pts = pts[np.abs(pts[:, 2]) < 1.0]
        if len(pts) > 4000:
            pts = pts[np.linspace(0, len(pts) - 1, 4000).astype(int)]
        plane = fit_plane(pts)
        if plane is not None and plane[2] > 200:
            self.plane = (plane[0], plane[1])
            self.plane_stamp = time.time()

    def _ground_candidate(self, det, t_base_cam, fx, fy, cx, cy):
        if self.plane is None or time.time() - self.plane_stamp > 5.0:
            return None
        normal, d = self.plane
        r_bc, t_bc = t_base_cam[:3, :3], t_base_cam[:3, 3]
        u = det.center_u
        v = det.y_max
        p_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_base = r_bc @ (self.bridge_rot @ p_opt)
        denom = float(normal @ d_base)
        if abs(denom) < 1e-6:
            return None
        s = -(float(normal @ t_bc) + d) / denom
        if s < self.min_depth or s > self.max_depth:
            return None
        contact = t_bc + s * d_base
        return contact + self.target_radius * normal, 0.9

    def _points_candidate(self, det, t_base_cam, t_base_lidar,
                          fx, fy, cx, cy):
        if self.cloud_base is None:
            return None
        r_bc, t_bc = t_base_cam[:3, :3], t_base_cam[:3, 3]
        pts_cam = (r_bc.T @ (self.cloud_base - t_bc).T).T
        pts_opt = (self.bridge_rot.T @ pts_cam.T).T
        vis = pts_opt[:, 2] > self.min_depth
        vis &= pts_opt[:, 2] < self.max_depth
        if not np.any(vis):
            return None
        p = pts_opt[vis]
        u = fx * p[:, 0] / p[:, 2] + cx
        v = fy * p[:, 1] / p[:, 2] + cy
        radius = 0.5 * min(det.x_max - det.x_min, det.y_max - det.y_min)
        inside = (u - det.center_u) ** 2 + (v - det.center_v) ** 2 < radius ** 2
        count = int(np.count_nonzero(inside))
        if count < self.min_points:
            return None
        selected = self.cloud_base[vis][inside]
        return np.median(selected, axis=0), min(1.0, count / 12.0)

    def _monocular_candidate(self, det, t_base_cam, fx, fy, cx, cy):
        w, h = self.info.width, self.info.height
        width_px = det.x_max - det.x_min
        height_px = det.y_max - det.y_min
        if det.x_min > 1.0 and det.x_max < w - 1.0:
            r_px = 0.5 * width_px
            cu = 0.5 * (det.x_min + det.x_max)
        elif det.y_min > 1.0 and det.y_max < h - 1.0:
            r_px = 0.5 * height_px
            cu = det.center_u
        else:
            return None
        if r_px < 4.0:
            return None
        if det.y_min > 1.0:
            cv = det.y_min + r_px
        elif det.y_max < h - 1.0:
            cv = det.y_max - r_px
        else:
            return None
        if det.y_max >= h - 1.0 and det.y_min <= 1.0:
            return None
        alpha = math.atan(r_px / fx)
        z = self.target_radius / math.sin(alpha)
        if z < self.min_depth or z > self.max_depth:
            return None
        r_bc, t_bc = t_base_cam[:3, :3], t_base_cam[:3, 3]
        p_opt = np.array([(cu - cx) / fx, (cv - cy) / fy, 1.0])
        d_base = r_bc @ (self.bridge_rot @ p_opt)
        d_base = d_base / np.linalg.norm(d_base)
        return t_bc + z * d_base, 0.3

    def _filter(self, meas, stamp):
        if self.filter_pos is None or self.filter_stamp is None \
                or not (0.0 < stamp - self.filter_stamp < 0.5):
            self.filter_pos = meas.copy()
            self.filter_vel = np.zeros(3)
        else:
            dt = stamp - self.filter_stamp
            pred = self.filter_pos + self.filter_vel * dt
            residual = meas - pred
            self.filter_pos = pred + self.alpha * residual
            self.filter_vel = self.filter_vel + (self.beta / dt) * residual
        self.filter_stamp = stamp

    def _publish(self, det, method, quality, stamp):
        obs = TargetObservation()
        obs.header.stamp = det.header.stamp
        obs.header.frame_id = self.base_frame
        obs.position = Point(x=float(self.filter_pos[0]),
                             y=float(self.filter_pos[1]),
                             z=float(self.filter_pos[2]))
        obs.velocity = Vector3(x=float(self.filter_vel[0]),
                               y=float(self.filter_vel[1]),
                               z=float(self.filter_vel[2]))
        obs.method = method
        obs.quality = float(quality)
        self.pub_base.publish(obs)

        if self.odom is None:
            return
        q = self.odom.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        t = self.odom.pose.pose.position
        world = rot @ self.filter_pos + np.array([t.x, t.y, t.z])
        obs_w = TargetObservation(header=obs.header,
                                  position=Point(x=float(world[0]),
                                                 y=float(world[1]),
                                                 z=float(world[2])),
                                  velocity=obs.velocity,
                                  method=method,
                                  quality=float(quality))
        obs_w.header.frame_id = 'world'
        self.pub_world.publish(obs_w)

    def _on_detection(self, det):
        if self.info is None or self.cloud_lidar is None:
            return
        try:
            t_base_cam, t_base_lidar = self._transforms()
        except RuntimeError:
            return
        k = np.array(self.info.k).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        self.cloud_base = (t_base_lidar[:3, :3] @ self.cloud_lidar.T).T \
            + t_base_lidar[:3, 3]

        stamp = det.header.stamp.sec + det.header.stamp.nanosec * 1e-9
        horiz_clip = (det.x_min <= 1.0
                      or det.x_max >= self.info.width - 1.0)
        bottom_clip = det.y_max >= self.info.height - 1.0
        if bottom_clip and det.y_min <= 1.0:
            return
        candidates = []
        if horiz_clip:
            ground, points = None, None
        else:
            ground = None if bottom_clip else self._ground_candidate(
                det, t_base_cam, fx, fy, cx, cy)
            points = None if bottom_clip else self._points_candidate(
                det, t_base_cam, t_base_lidar, fx, fy, cx, cy)
        mono = self._monocular_candidate(det, t_base_cam, fx, fy, cx, cy)
        if self.prefer_ground:
            order = [(ground, TargetObservation.METHOD_LIDAR_GROUND),
                     (points, TargetObservation.METHOD_LIDAR_POINTS),
                     (mono, TargetObservation.METHOD_MONOCULAR)]
        else:
            order = [(points, TargetObservation.METHOD_LIDAR_POINTS),
                     (ground, TargetObservation.METHOD_LIDAR_GROUND),
                     (mono, TargetObservation.METHOD_MONOCULAR)]
        for cand, method in order:
            if cand is not None:
                candidates = (cand, method)
                break
        if not candidates:
            return
        position, quality = candidates[0]
        method = candidates[1]
        self._filter(position, stamp)
        self._publish(det, method, quality, stamp)


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
