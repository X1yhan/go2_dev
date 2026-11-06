"""The object table: one source of truth for names, shapes and sizes.

Geometry is in metres and matches worlds/arm_bench.sdf.  This module is
plain data plus two size helpers, so the language layer and the planning
scene node can both use it without importing MoveIt or OpenCV.
"""

# The fingers reach about +/-0.075 m from the TCP.  Keep the whole gripper,
# not just the held object, clear of other objects when choosing a spot.
GRIPPER_SAFETY_R = 0.08

# Table surface half extents, used when looking for a free place spot.
TABLE_X_MIN = 0.10
TABLE_X_MAX = 0.29
TABLE_Y_LIMIT = 0.29

OBJECTS = {
    'red_ball': {
        'model': 'target_ball', 'shape': 'sphere',
        'dims': [0.035], 'half_height': 0.035, 'radius': 0.035,
        'grip_offset': 0.17, 'close': 0.0, 'open': 0.05,
        'default': (0.22, 0.0, 0.035),
    },
    'blue_cube': {
        'model': 'blue_cube', 'shape': 'box',
        'dims': [0.07, 0.07, 0.07], 'half_height': 0.035, 'radius': 0.0,
        'grip_offset': 0.17, 'close': 0.0, 'open': 0.04,
        'default': (0.26, -0.16, 0.035),
    },
    'green_cylinder': {
        'model': 'green_cylinder', 'shape': 'cylinder',
        'dims': [0.10, 0.04], 'half_height': 0.05, 'radius': 0.0,
        'grip_offset': 0.15, 'close': 0.028, 'open': 0.04,
        'default': (0.14, 0.18, 0.05),
    },
    'yellow_box': {
        'model': 'yellow_box', 'shape': 'box',
        'dims': [0.10, 0.075, 0.07], 'half_height': 0.035, 'radius': 0.0,
        'grip_offset': 0.17, 'close': 0.025, 'open': 0.04,
        'default': (0.25, 0.16, 0.035),
    },
}

LABEL_CN = {
    'red_ball': '红球',
    'blue_cube': '蓝方块',
    'green_cylinder': '绿圆柱',
    'yellow_box': '黄盒子',
}

SHAPE_SPHERE = 'sphere'
SHAPE_BOX = 'box'
SHAPE_CYLINDER = 'cylinder'


def footprint_radius(cfg):
    """Radius of the circle the object sweeps on the table."""
    if cfg['shape'] == SHAPE_SPHERE:
        return cfg['radius']
    if cfg['shape'] == SHAPE_CYLINDER:
        return cfg['dims'][1]
    return max(cfg['dims']) / 2.0


def held_width(cfg):
    """Width of the object across the gripper fingers."""
    if cfg['shape'] == SHAPE_SPHERE:
        return 2.0 * cfg['radius']
    if cfg['shape'] == SHAPE_CYLINDER:
        return 2.0 * cfg['dims'][1]
    return max(cfg['dims'][0], cfg['dims'][1])


def with_radius(cfg, radius):
    """Copy of `cfg` with the sphere radius overridden."""
    out = dict(cfg)
    out['radius'] = float(radius)
    return out
