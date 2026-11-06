"""The arm_bench_visual_grasp node: parameters, scan, pick, place.

This is the composition root.  It declares the ROS interface (topics,
actions, parameters) and runs the task as a linear script; the work itself
lives in the layered modules:

    vision.py   camera + TF       -> base_link points
    scene.py    fused observations
    world.py    collision scene
    skills.py   motion verbs

The node name, every topic name and every parameter name are unchanged
from the single-file version, so `start_arm_bench.sh`, the language shell
and Robot Studio keep working untouched.
"""
import math
import sys
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker

from arm_bench import geometry, world
from arm_bench.constants import GRIPPER_JOINTS
from arm_bench.objects import OBJECTS, held_width
from arm_bench.scene import SCAN_RANK, SceneModel
from arm_bench.skills import ArmSkills
from arm_bench.vision import VisionFrontend
from arm_bench_object_detector import HSV_RANGES

COLOR_TOPIC = '/arm/camera/image'
DEPTH_TOPIC = '/arm/camera/depth_image'
INFO_TOPIC = '/arm/camera/camera_info'
JOINT_STATE_TOPIC = '/joint_states'
DEBUG_IMAGE_TOPIC = '/arm/visual_grasp/debug_image'
MARKER_TOPIC = '/arm/visual_grasp/marker'

DEFAULT_OBSERVE = (0.22, 0.0, 0.38)
ABOVE_Z = 0.38
PLACE_AREA_LOOK = (0.24, 0.20, 0.34)
# Scan viewpoints tried in order when an object is missing from the
# default observation pose.
LOOK_POSES = [
    (0.34, 0.16, 0.33),
    (0.34, -0.16, 0.33),
    (0.24, 0.20, 0.34),
    (0.24, -0.20, 0.34),
    (0.16, 0.00, 0.36),
]
LOOK_POSES_2 = [
    (0.30, 0.08, 0.32),
    (0.30, -0.08, 0.32),
    (0.20, 0.12, 0.35),
    (0.20, -0.12, 0.35),
]

# Tolerances on the post-place visual check.
PLACE_TOL_Z = 0.025
PLACE_TOL_XY = 0.035


class VisualGrasp(Node):

    def __init__(self):
        super().__init__(
            'arm_bench_visual_grasp',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('target_class', 'red_ball')
        self.declare_parameter('detections_topic', '/arm/vision/detections')
        self.declare_parameter('ball_radius', 0.035)
        self.declare_parameter('grip_offset', -1.0)
        self.declare_parameter('pre_offset', 0.06)
        self.declare_parameter('lift_offset', 0.18)
        self.declare_parameter('open', 0.06)
        self.declare_parameter('close', -1.0)
        self.declare_parameter('detect_only', False)
        self.declare_parameter('place_class', '')
        self.declare_parameter('place_mode', 'on_top')
        self.declare_parameter('place_offset', 0.05)
        self.declare_parameter('place_clearance', 0.0)
        self.declare_parameter('scan_only', False)

        self.target_class = str(self.get_parameter('target_class').value)
        if self.target_class not in OBJECTS:
            raise RuntimeError(
                'unknown target_class "%s", choose one of %s' %
                (self.target_class, ', '.join(OBJECTS)))
        # Copy, so a -p ball_radius does not leak into the shared table.
        self.cfg = dict(OBJECTS[self.target_class])
        if self.target_class == 'red_ball':
            self.cfg['radius'] = float(
                self.get_parameter('ball_radius').value)

        grip_param = float(self.get_parameter('grip_offset').value)
        self.grip_offset = grip_param if grip_param >= 0.0 \
            else self.cfg['grip_offset']
        self.pre_offset = float(self.get_parameter('pre_offset').value)
        self.lift_offset = float(self.get_parameter('lift_offset').value)
        open_param = float(self.get_parameter('open').value)
        self.open_q = open_param if open_param >= 0.0 \
            else self.cfg['open']
        close_param = float(self.get_parameter('close').value)
        self.close_q = close_param if close_param >= 0.0 \
            else self.cfg['close']
        self.detect_only = bool(self.get_parameter('detect_only').value)
        self.scan_only = bool(self.get_parameter('scan_only').value)
        self.place_class = str(self.get_parameter('place_class').value)
        self.place_mode = str(self.get_parameter('place_mode').value)
        self.place_offset = float(self.get_parameter('place_offset').value)
        self.place_clearance = float(
            self.get_parameter('place_clearance').value)
        self.place_cfg = None
        if self.place_class:
            if self.place_class not in OBJECTS:
                raise RuntimeError(
                    'unknown place_class "%s"' % self.place_class)
            if self.place_class == self.target_class:
                raise RuntimeError('place_class must differ from target_class')
            if self.place_mode not in ('on_top', 'beside'):
                raise RuntimeError('place_mode must be on_top or beside')
            self.place_cfg = dict(OBJECTS[self.place_class])

        self.joint_positions = {}
        self._trace_fingers = False
        self._finger_trace = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.debug_pub = self.create_publisher(Image, DEBUG_IMAGE_TOPIC, 10)
        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.vision = VisionFrontend(
            self, self.tf_buffer, HSV_RANGES,
            debug_publisher=self.debug_pub, marker_publisher=self.marker_pub)
        self.scene = SceneModel()
        self.skills = ArmSkills(self, self.tf_buffer, self.joint_positions,
                                open_q=self.open_q)
        self.scene_client = world.SceneClient(self)

        self.create_subscription(
            Image, COLOR_TOPIC, self.vision.on_color, qos_profile_sensor_data)
        self.create_subscription(
            Image, DEPTH_TOPIC, self.vision.on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, INFO_TOPIC, self.vision.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, self.get_parameter('detections_topic').value,
            self.vision.on_detections, 10)
        self.create_subscription(
            JointState, JOINT_STATE_TOPIC, self.on_joint_state, 10)

        self.get_logger().info(
            'target_class=%s (model %s, %s)' %
            (self.target_class, self.cfg['model'], self.cfg['shape']))

    # -- plumbing --------------------------------------------------------
    def on_joint_state(self, msg):
        self.joint_positions.update(dict(zip(msg.name, msg.position)))
        if self._trace_fingers:
            self._finger_trace.append((
                time.time(),
                *[self.joint_positions.get(joint, -1.0)
                  for joint in GRIPPER_JOINTS]))

    def _step(self, message):
        self.get_logger().info('[STEP] %s' % message)

    def _settle(self, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    # -- collision scene -------------------------------------------------
    def set_scene(self):
        """Table plus every object the scan has located."""
        objects = [world.table_object()]
        for label, cfg in OBJECTS.items():
            center = self.scene.seen.get(label)
            if center is None:
                continue
            objects.append(world.collision_object(
                cfg['model'], cfg['shape'], cfg['dims'], world.pose(center)))
        return self.scene_client.apply(world.diff_scene(objects),
                                       timeout=5.0, wait=10.0)

    def remove_object(self, name):
        return self.scene_client.apply(
            world.diff_scene([world.removal(name)]), timeout=3.0, wait=5.0)

    def clear_scene(self):
        objects = [world.removal(cfg['model'])
                   for cfg in OBJECTS.values()]
        objects.append(world.removal(world.TABLE_ID))
        return self.scene_client.apply(world.diff_scene(objects),
                                       timeout=3.0, wait=5.0)

    # -- scanning --------------------------------------------------------
    def _scan_pass(self, poses, first_pass=False):
        for index, pose in enumerate(poses):
            if not self.skills.move_to(pose):
                self.get_logger().warn('scan pose %s unreachable' % (pose,))
                continue
            if not self.vision.fresh():
                self.get_logger().warn('no fresh camera frame at %s'
                                       % (pose,))
                continue
            found = self.vision.detect_all()
            rank = index if first_pass else SCAN_RANK
            for label, result in found.items():
                point = result[0]
                self.scene.add(label, point, result[2], rank)
                if pose == DEFAULT_OBSERVE:
                    self.scene.set_reference(label, point)
            self.scene.fuse()
            if found:
                self.set_scene()
            self.get_logger().info(
                'scan pose %s: %s' %
                (pose, ','.join(sorted(found)) if found else 'none'))

    def _capture_current(self):
        if not self.vision.fresh():
            return {}
        found = self.vision.detect_all()
        for label, result in found.items():
            point = result[0]
            self.scene.add(label, point, result[2], 0)
            self.scene.set_reference(label, point)
        self.scene.fuse()
        if found:
            self.set_scene()
        return found

    def scan_scene(self, required=None):
        # Lazy scan: look at the default observation first.  Only move to
        # the other viewpoints when a required object is missing.
        self.get_logger().info('looking at default observation ...')
        if self.skills.move_to(DEFAULT_OBSERVE):
            found = self._capture_current()
            self.get_logger().info(
                'default view: %s' %
                (','.join(sorted(found)) if found else 'none'))
            if required is None:
                if found:
                    return True
            elif self.scene.known(required):
                if (not (self.scan_only or self.detect_only) and
                        self.place_mode == 'beside'):
                    self.get_logger().info(
                        'checking place area for nearby obstacles ...')
                    self._scan_pass([PLACE_AREA_LOOK], first_pass=True)
                return True
        else:
            self.get_logger().warn(
                'default observation unreachable; looking around')
        self.get_logger().info(
            'required object(s) missing at default, looking around ...')
        self._scan_pass(LOOK_POSES, first_pass=True)
        missing = [label for label in (required or [])
                   if label not in self.scene.seen]
        if missing:
            self.get_logger().warn(
                'not found yet: %s; looking again' % ','.join(missing))
            self._scan_pass(LOOK_POSES_2)
        missing = [label for label in (required or [])
                   if label not in self.scene.seen]
        if missing:
            self.get_logger().error(
                'objects not in view after scanning: %s' %
                ','.join(missing))
            return False
        return True

    # -- placing ---------------------------------------------------------
    def place_object(self):
        """Carry the held object to a release pose.

        Returns (ok, reason); the reason is short and human readable so the
        language layer can show it without scraping logs.
        """
        center = self.scene.best(self.place_class)
        if center is None:
            self.get_logger().error(
                'place target %s is not in the vision scan' %
                self.place_class)
            return False, 'place target not in vision scan'
        if self.place_mode == 'on_top':
            if self.place_cfg['shape'] != 'box':
                self.get_logger().error(
                    'on_top placement needs a box/cube target; use beside')
                return False, 'on_top needs a box target'
            x, y = center[0], center[1]
            release_z, expected_z = geometry.place_on_top(
                center, self.cfg, self.place_cfg, self.grip_offset,
                self.place_clearance)
            desc = '%s top' % self.place_class
        else:
            spot = geometry.free_beside_spot(
                center, self.scene.seen, self.cfg, self.place_cfg,
                self.place_class, self.target_class, self.place_offset)
            if spot is None:
                self.get_logger().error('no free space beside %s'
                                        % self.place_class)
                return False, 'no free space beside target'
            (x, y), nearest, distance = spot
            self.get_logger().info(
                'beside spot (%.3f, %.3f), clearance to %s %.3f m' %
                (x, y, nearest or 'none', distance or 0.0))
            release_z, expected_z = geometry.place_beside(
                self.cfg, self.grip_offset, self.place_clearance)
            desc = '%s beside' % self.place_class

        release = (x, y, release_z)
        above = (x, y, ABOVE_Z)
        self._step('搬运到放置点上方 %s' % (above,))
        self.get_logger().info('moving above place %s' % (above,))
        self._finger_trace.clear()
        self._trace_fingers = True
        try:
            above_ok = self.skills.move_to(above)
        finally:
            self._trace_fingers = False
        if self._finger_trace:
            f1 = [sample[1] for sample in self._finger_trace]
            f2 = [sample[2] for sample in self._finger_trace]
            self._step('搬运中手指: f1=%.4f..%.4f, f2=%.4f..%.4f' %
                       (min(f1), max(f1), min(f2), max(f2)))
        if not above_ok:
            self.get_logger().error('above-place move failed')
            return False, 'above-place move failed'
        self.skills.wait_tcp(above, timeout=8.0, tol=0.02)
        self._step('已到达放置点上方')
        self.get_logger().info(
            'placing %s -> %s at base (%.3f, %.3f, %.3f)' %
            (self.target_class, desc, x, y, release_z))
        ok, _fraction = self.skills.cartesian([release])
        if not ok or not self.skills.wait_tcp(release, timeout=10.0,
                                              tol=0.015):
            self.get_logger().error(
                'arm did not reach place pose; object still held')
            return False, 'arm did not reach place pose'

        width = held_width(self.cfg)
        open_q = geometry.release_q(width)
        self._step('夹爪释放 (q=%.3f, 宽度 %.3f)' % (open_q, width))
        self.get_logger().info(
            'opening gripper to release (q=%.3f, width %.3f)'
            % (open_q, width))
        self.skills.grip_to(open_q)
        if self.target_class == 'red_ball' and self.skills.detach('detach'):
            self._step('target_ball detach')
        time.sleep(0.8)

        lift = (x, y, ABOVE_Z)
        lifted, fraction = self.skills.cartesian([lift])
        if not lifted:
            self.get_logger().warn(
                'place lift Cartesian %.2f, fallback MoveIt' % fraction)
            self.skills.move_to(lift)
        self.skills.wait_tcp(lift, timeout=8.0, tol=0.02)
        time.sleep(1.0)

        self.vision.fresh()
        placed = self.vision.detect_label(self.target_class,
                                          table_prior=False,
                                          publish_debug=True)
        if placed is None:
            self.get_logger().warn(
                'placed object not visible here, moving to verify pose')
            if self.skills.move_to(DEFAULT_OBSERVE) and self.vision.fresh():
                placed = self.vision.detect_label(self.target_class,
                                                 table_prior=False)
        if placed is None:
            self.get_logger().error(
                'PLACE unverified: %s not visible after release'
                % self.target_class)
            return False, 'not visible after release'
        point = placed[0]
        error_z = abs(point[2] - expected_z)
        error_xy = math.hypot(point[0] - x, point[1] - y)
        success = error_z < PLACE_TOL_Z and error_xy < PLACE_TOL_XY
        self._step('放置视觉复检: %s, base=(%.3f, %.3f, %.3f), '
                   'expected_z=%.3f' % (self.target_class, point[0],
                                        point[1], point[2], expected_z))
        self.get_logger().info(
            'PLACE %s %s -> %s: visual base (%.3f, %.3f, %.3f), '
            'expected z=%.3f, err_xy=%.3f' %
            ('SUCCESS' if success else 'FAILED', self.target_class,
             self.place_class, point[0], point[1], point[2], expected_z,
             error_xy))
        return success, 'ok' if success else 'visual check failed'

    # -- the task --------------------------------------------------------
    def run(self):
        if not self.skills.move.wait_for_server(timeout_sec=30):
            self.get_logger().error('/move_action not available, start '
                                    'arm_moveit.launch.py first')
            return 1
        if not self.skills.grip.wait_for_server(timeout_sec=10):
            self.get_logger().error('gripper action not available')
            return 1

        self.clear_scene()
        if not self.set_scene():
            self.get_logger().warn('could not load table collision box')

        required = [self.target_class]
        if self.place_class:
            required.append(self.place_class)
        scan_required = None if (self.scan_only or self.detect_only) \
            else required
        if not self.scan_scene(scan_required):
            self.get_logger().error(
                'visible objects: %s' % (','.join(self.scene.labels())
                                         or 'none'))
            return 1

        if self.scan_only or self.detect_only:
            for name in self.scene.labels():
                point = self.scene.best(name)
                self.get_logger().info(
                    'VISIBLE %s base=(%.3f, %.3f, %.3f)' %
                    (name, point[0], point[1], point[2]))
            return 0

        target = self.scene.best(self.target_class)
        if target is None:
            self.get_logger().error(
                'target %s vanished from the scene model'
                % self.target_class)
            return 1
        self._step('观察位扫描完成, 可见物体: %s' % ','.join(self.scene.labels()))
        self.get_logger().info(
            'TARGET %s (camera) base=(%.3f, %.3f, %.3f)' %
            (self.target_class, target[0], target[1], target[2]))
        if self.place_class:
            drop = self.scene.best(self.place_class)
            if drop is None:
                self.get_logger().error(
                    'place target %s is not in the vision scan'
                    % self.place_class)
                return 1
            self.get_logger().info(
                'PLACE TARGET %s (camera) base=(%.3f, %.3f, %.3f)' %
                (self.place_class, drop[0], drop[1], drop[2]))

        self._step('夹爪打开 (q=%.3f)' % self.open_q)
        self.get_logger().info('opening gripper')
        self.skills.grip_to(self.open_q)
        pre, grasp = geometry.grasp_pose(target, self.cfg, self.grip_offset,
                                         self.pre_offset)
        lift = (target[0], target[1], ABOVE_Z)

        # From the observation pose go straight down to the pre-grasp with
        # one Cartesian move (the target is already removed), then pause and
        # do the short vertical descent.  Avoid a long free-space MoveIt plan
        # that circles around the target.
        self.remove_object(self.cfg['model'])
        self.get_logger().info('moving to pre-grasp %s' % (pre,))
        ok, fraction = self.skills.cartesian([pre])
        if not ok:
            self.get_logger().warn(
                'Cartesian pre-grasp %.2f, fallback MoveIt' % fraction)
            if not self.skills.move_to(pre):
                self.get_logger().error('pre-grasp move failed')
                return 1
        self.skills.wait_tcp(pre, timeout=8.0, tol=0.02)
        self._step('到达 pre-grasp (%.3f, %.3f, %.3f)' % pre)

        self.get_logger().info('descending to %s' % (grasp,))
        ok, fraction = self.skills.cartesian([grasp])
        if not ok:
            self.get_logger().warn('Cartesian descent %.2f, fallback MoveIt'
                                   % fraction)
            ok = self.skills.move_to(grasp)
        if not ok:
            self.get_logger().error('descent failed')
            return 1
        if not self.skills.wait_tcp(grasp, timeout=8.0, tol=0.015):
            self.get_logger().error(
                'arm did not settle at the grasp pose; aborting before '
                'closing the gripper')
            return 1
        self._step('到达抓取位 (%.3f, %.3f, %.3f)' % grasp)

        self.skills.log_geometry('at grasp')
        self._step('夹爪闭合 (q=%.3f)' % self.close_q)
        self.get_logger().info('closing gripper')
        self.get_logger().info('close action: %s' % self.skills.grip_to(
            self.close_q))
        self._settle(0.5)
        self.skills.log_geometry('after close')
        fingers = [round(self.joint_positions.get(joint, -1.0), 4)
                   for joint in GRIPPER_JOINTS]
        self._step('抓取校验: 手指=%s (物体应夹在指间)' % fingers)
        if self.target_class == 'red_ball' and self.skills.detach('attach'):
            self._step('target_ball attach')

        self.get_logger().info('lifting to %s' % (lift,))
        ok, fraction = self.skills.cartesian([lift])
        if not ok:
            self.get_logger().warn('Cartesian lift %.2f, fallback MoveIt'
                                   % fraction)
            ok = self.skills.move_to(lift)
        if not ok:
            self.get_logger().error('lift failed')
            return 1
        self.skills.wait_tcp(lift, timeout=10.0, tol=0.02)
        self._settle(0.5)
        fingers = [round(self.joint_positions.get(joint, -1.0), 4)
                   for joint in GRIPPER_JOINTS]
        self._step('已抬起 (%.3f, %.3f, %.3f), 手指=%s'
                   % (lift + (fingers,)))
        if not self.place_class:
            return 0
        placed, _reason = self.place_object()
        return 0 if placed else 1


def main():
    rclpy.init()
    node = VisualGrasp()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
