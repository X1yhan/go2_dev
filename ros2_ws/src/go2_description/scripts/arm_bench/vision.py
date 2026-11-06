"""Camera and TF adapter: ROS messages in, base_link positions out.

This is layer L2.  It owns the latest sensor messages, converts the pixel
measurement from `detection.py` into a camera-frame point, applies the
table prior, and hands back base_link coordinates.  It must not import
moveit_msgs: if a change here needs the planner, the change is in the
wrong layer.
"""
import time

import rclpy
import tf2_geometry_msgs
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from arm_bench import detection, geometry
from arm_bench.constants import BASE_FRAME, CAMERA_FRAME
from arm_bench.objects import OBJECTS

# A detection older than this relative to the colour frame is stale.
MAX_STAMP_SKEW_NS = 3e8
MARKER_SCALE = 0.06


def _pose(point, quat):
    """geometry_msgs/Pose; duplicated from world.py to keep L2 free of L3."""
    msg = Pose()
    msg.position = Point(x=float(point[0]), y=float(point[1]),
                         z=float(point[2]))
    msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    return msg


class VisionFrontend:

    def __init__(self, node, tf_buffer, hsv_ranges, objects=None,
                 base_frame=BASE_FRAME, camera_frame=CAMERA_FRAME,
                 debug_publisher=None, marker_publisher=None):
        self.node = node
        self.tf_buffer = tf_buffer
        self.hsv_ranges = hsv_ranges
        self.objects = objects if objects is not None else OBJECTS
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.debug_publisher = debug_publisher
        self.marker_publisher = marker_publisher
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.camera_info = None
        self.detections = None

    # -- sensor cache ----------------------------------------------------
    def on_color(self, msg):
        self.color = msg

    def on_depth(self, msg):
        self.depth = msg

    def on_info(self, msg):
        self.camera_info = msg

    def on_detections(self, msg):
        self.detections = msg

    def reset(self):
        self.color = None
        self.depth = None
        self.camera_info = None
        self.detections = None

    def ready(self):
        return all(item is not None for item in
                   (self.color, self.depth, self.camera_info,
                    self.detections))

    def fresh(self, timeout=2.5):
        """Wait for one complete, current frame set."""
        self.reset()
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.ready():
                for _ in range(3):
                    rclpy.spin_once(self.node, timeout_sec=0.02)
                return True
        return False

    # -- detection -------------------------------------------------------
    def select(self, label):
        """Largest detection whose label matches."""
        if self.detections is None:
            return None
        best = None
        best_area = -1.0
        for det in self.detections.detections:
            if not det.results:
                continue
            if det.results[0].hypothesis.class_id != label:
                continue
            area = det.bbox.size_x * det.bbox.size_y
            if area > best_area:
                best, best_area = det, area
        return best

    def label_mask(self, bgr, label):
        return detection.label_mask(bgr, self.hsv_ranges.get(label, []))

    def detect_label(self, label, table_prior=True, publish_debug=False):
        """One detection -> ((x, y, z) in base_link, (u, v, depth), area)."""
        if not self.ready():
            return None
        det = self.select(label)
        if det is None:
            return None
        det_stamp = Time.from_msg(self.detections.header.stamp)
        image_stamp = Time.from_msg(self.color.header.stamp)
        if abs((det_stamp - image_stamp).nanoseconds) > MAX_STAMP_SKEW_NS:
            return None

        cfg = self.objects[label]
        box = (det.bbox.center.position.x, det.bbox.center.position.y,
               det.bbox.size_x, det.bbox.size_y)
        depth = self.bridge.imgmsg_to_cv2(self.depth, '32FC1')
        bgr = self.bridge.imgmsg_to_cv2(self.color, 'bgr8')
        # Restrict to this label's colour: on a stack the bbox of the
        # support object can contain the object on top, and the top-face
        # depth must not be taken from the occluder.
        mask = self.label_mask(bgr, label)
        measured = detection.measure(depth, box, cfg['shape'], mask)
        if measured is None:
            return None
        u, v, z = measured

        x, y, z = geometry.deproject(u, v, z, self.camera_info.k)
        if cfg['shape'] == 'sphere':
            x, y, z = geometry.on_sphere(x, y, z, cfg['radius'])

        point = PointStamped()
        point.header.stamp = self.depth.header.stamp
        point.header.frame_id = self.camera_frame
        point.point.x, point.point.y, point.point.z = x, y, z
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time(),
                timeout=Duration(seconds=0.2))
            base_point = tf2_geometry_msgs.do_transform_point(
                point, transform).point
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn('TF failed: %s' % exc,
                                        throttle_duration_sec=2.0)
            return None

        if table_prior:
            # Objects on the table: class dimensions and the table plane
            # are known priors; x/y come from the camera.
            base_point.z = cfg['half_height']
        elif cfg['shape'] != 'sphere':
            # Verification (e.g. object on top of another): use the
            # measured top surface minus the class half height.
            base_point.z = base_point.z - cfg['half_height']

        if publish_debug:
            detection.draw_detection(
                bgr, (u, v, box[2], box[3]), '%s %.2fm' % (label, z))
            debug = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
            debug.header = self.color.header
            if self.debug_publisher is not None:
                self.debug_publisher.publish(debug)
            self.publish_marker((base_point.x, base_point.y, base_point.z))

        return ((base_point.x, base_point.y, base_point.z),
                (u, v, z), box[2] * box[3])

    def detect(self, label):
        return self.detect_label(label, table_prior=True, publish_debug=True)

    def detect_all(self):
        """Every known label currently in view."""
        if self.detections is None:
            return {}
        labels = {det.results[0].hypothesis.class_id
                  for det in self.detections.detections if det.results}
        found = {}
        for label in labels:
            if label not in self.objects:
                continue
            is_sphere = self.objects[label]['shape'] == 'sphere'
            result = self.detect_label(label, table_prior=is_sphere)
            if result is not None:
                found[label] = result
        return found

    def publish_marker(self, point):
        if self.marker_publisher is None:
            return
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = 'visual_grasp'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = _pose(point, (0.0, 0.0, 0.0, 1.0))
        marker.scale.x = marker.scale.y = marker.scale.z = MARKER_SCALE
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        self.marker_publisher.publish(marker)
