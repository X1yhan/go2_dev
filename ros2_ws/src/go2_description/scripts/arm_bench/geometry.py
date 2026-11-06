"""Pure geometry for the arm bench: camera rays, place poses, free spots.

Everything here takes and returns plain numbers or tuples.  No ROS, no
images, so all of it can be exercised offline:

    python3 -c "from arm_bench.geometry import free_beside_spot"
"""
import math

from arm_bench.objects import (GRIPPER_SAFETY_R, OBJECTS, TABLE_X_MAX,
                               TABLE_X_MIN, TABLE_Y_LIMIT, footprint_radius)

# Candidate directions when placing beside another object, tried in order.
BESIDE_ANGLES_DEG = (0, 90, -90, 45, -45, 135, -135, 180)
# Minimum edge-to-edge gap demanded between the gripper and a neighbour.
BESIDE_PAD = 0.02


def deproject(u, v, z, k):
    """Pixel + depth -> camera-frame (x, y, z), k = CameraInfo.k."""
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return (x, y, z)


def on_sphere(x, y, z, radius):
    """Push a point on the sphere silhouette back to the sphere surface."""
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-6:
        return (x, y, z)
    return (x + radius * x / norm,
            y + radius * y / norm,
            z + radius * z / norm)


def grasp_pose(target, cfg, grip_offset, pre_offset):
    """(pre-grasp, grasp, lift) above a target centre, in base_link."""
    grasp = (target[0], target[1], target[2] + grip_offset)
    pre = (target[0], target[1],
           target[2] + grip_offset + pre_offset)
    return pre, grasp


def place_on_top(center, cfg, place_cfg, grip_offset, clearance):
    """Stack `cfg` on top of `place_cfg`; returns the release/expected z."""
    top = center[2] + place_cfg['half_height']
    release_z = top + cfg['half_height'] + grip_offset + clearance
    expected_z = top + cfg['half_height']
    return release_z, expected_z


def place_beside(cfg, grip_offset, clearance):
    """Release above the table for a beside placement."""
    release_z = cfg['half_height'] + grip_offset + clearance
    expected_z = cfg['half_height']
    return release_z, expected_z


def release_q(held, gap=0.012):
    """Finger angle that opens the gripper to just wider than `held`."""
    return max(0.0, (held - gap) / 2.0)


def free_beside_spot(center, seen, cfg, place_cfg, place_class, target_class,
                     place_offset, objects=None, safety_r=GRIPPER_SAFETY_R):
    """Find a table spot beside `center` where the gripper clears everything.

    `seen` maps label -> (x, y, z) for every object the scan found.
    Returns (spot, nearest_label, nearest_distance) or None.
    """
    table = objects if objects is not None else OBJECTS
    held_r = max(footprint_radius(cfg), safety_r)
    distance = footprint_radius(place_cfg) + held_r + place_offset
    for angle_deg in BESIDE_ANGLES_DEG:
        angle = math.radians(angle_deg)
        x = center[0] + distance * math.cos(angle)
        y = center[1] + distance * math.sin(angle)
        if not (TABLE_X_MIN <= x <= TABLE_X_MAX - held_r and
                -TABLE_Y_LIMIT + held_r <= y <= TABLE_Y_LIMIT - held_r):
            continue
        nearest = None
        nearest_d = None
        free = True
        for name, point in seen.items():
            if name in (place_class, target_class):
                continue
            other = table.get(name)
            if other is None:
                continue
            need = footprint_radius(other) + safety_r + BESIDE_PAD
            d = math.hypot(x - point[0], y - point[1])
            if nearest_d is None or d < nearest_d:
                nearest_d, nearest = d, name
            if d < need:
                free = False
                break
        if free:
            return ((x, y), nearest or 'none', nearest_d or 0.0)
    return None
