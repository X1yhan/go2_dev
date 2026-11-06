"""The MoveIt view of the bench: collision objects and the planning scene.

This is the only place outside `skills.py` that may import moveit_msgs.
Everything here is a pure message builder except `SceneClient`, which owns
the single `/apply_planning_scene` connection.

Both the grasp node and `arm_bench_planning_scene.py` build their scenes
through these helpers, so the table box and the primitive conversion exist
exactly once.
"""
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from arm_bench.constants import BASE_FRAME

TABLE_ID = 'table'
TABLE_DIMS = [0.6, 0.6, 0.6]
TABLE_CENTRE = (0.0, 0.0, -0.3)

APPLY_SERVICE = '/apply_planning_scene'


def pose(xyz, quat=None):
    """geometry_msgs/Pose from a position tuple and optional quaternion."""
    msg = Pose()
    msg.position.x = float(xyz[0])
    msg.position.y = float(xyz[1])
    msg.position.z = float(xyz[2])
    if quat is None:
        msg.orientation.w = 1.0
    else:
        msg.orientation = quat
    return msg


def collision_object(obj_id, shape, dims, pose_msg, frame=BASE_FRAME):
    """One ADD collision object built from a shape name and dimensions."""
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = obj_id
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    if shape == 'sphere':
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(dims[0])]
    elif shape == 'cylinder':
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(dims[0]), float(dims[1])]
    else:
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in dims]
    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose_msg)
    return obj


def table_object(frame=BASE_FRAME):
    """The bench top: its surface is z = 0 in base_link."""
    return collision_object(TABLE_ID, 'box', TABLE_DIMS,
                            pose(TABLE_CENTRE), frame)


def removal(obj_id, frame=BASE_FRAME):
    """One REMOVE collision object."""
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = obj_id
    obj.operation = CollisionObject.REMOVE
    return obj


def diff_scene(objects):
    """Incremental planning scene from a list of collision objects."""
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = list(objects)
    return scene


class SceneClient:
    """Single connection to the /apply_planning_scene service."""

    def __init__(self, node, service=APPLY_SERVICE):
        self.node = node
        self.client = node.create_client(ApplyPlanningScene, service)

    def apply(self, scene, timeout=5.0, wait=10.0):
        """Send one diff scene; True if the call completed."""
        if not self.client.wait_for_service(timeout_sec=wait):
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return future.done()
