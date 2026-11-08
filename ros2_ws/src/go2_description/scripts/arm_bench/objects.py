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
TABLE_X_MAX = 0.34
TABLE_Y_LIMIT = 0.32

OBJECTS = {
    'red_ball': {
        'model': 'target_ball', 'shape': 'sphere',
        'dims': [0.035], 'half_height': 0.035, 'radius': 0.035,
        'grip_offset': 0.17, 'close': 0.0, 'open': 0.05,
        'default': (0.215, 0.0, 0.035),
    },
    'blue_cube': {
        'model': 'blue_cube', 'shape': 'box',
        'dims': [0.07, 0.07, 0.07], 'half_height': 0.035, 'radius': 0.0,
        'grip_offset': 0.17, 'close': 0.0, 'open': 0.04,
        'default': (0.265, -0.175, 0.035),
    },
    'green_cylinder': {
        'model': 'green_cylinder', 'shape': 'cylinder',
        'dims': [0.10, 0.04], 'half_height': 0.05, 'radius': 0.0,
        'grip_offset': 0.15, 'close': 0.028, 'open': 0.04,
        'default': (0.135, 0.215, 0.05),
    },
    'yellow_box': {
        'model': 'yellow_box', 'shape': 'box',
        'dims': [0.10, 0.075, 0.07], 'half_height': 0.035, 'radius': 0.0,
        'grip_offset': 0.17, 'close': 0.025, 'open': 0.04,
        'default': (0.265, 0.175, 0.035),
    },
    # Household props.  `grip_offset` is chosen so the flange ends up about
    # 13.5 cm above the object top -- the distance at which the finger tips
    # straddle the object instead of stubbing into it.
    'milk_carton': {
        'model': 'milk_carton', 'shape': 'box',
        'dims': [0.065, 0.065, 0.075], 'half_height': 0.0375, 'radius': 0.0,
        'grip_offset': 0.1675, 'close': 0.0, 'open': 0.045,
        'default': (0.135, -0.115, 0.0375),
    },
    'water_cup': {
        'model': 'water_cup', 'shape': 'cylinder',
        'dims': [0.075, 0.035], 'half_height': 0.0375, 'radius': 0.0,
        'grip_offset': 0.1675, 'close': 0.006, 'open': 0.045,
        'default': (0.145, 0.09, 0.0375),
    },
    'tissue_box': {
        'model': 'tissue_box', 'shape': 'box',
        'dims': [0.14, 0.08, 0.06], 'half_height': 0.03, 'radius': 0.0,
        'grip_offset': 0.165, 'close': 0.012, 'open': 0.045,
        # The gripper closes across the 8 cm side, not across the 14 cm one.
        'grip_width': 0.08,
        'default': (0.09, -0.235, 0.03),
    },
}

LABEL_CN = {
    'red_ball': '红球',
    'blue_cube': '蓝方块',
    'green_cylinder': '绿圆柱',
    'yellow_box': '黄盒子',
    'milk_carton': '牛奶盒',
    'water_cup': '水杯',
    'tissue_box': '纸巾盒',
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
    if 'grip_width' in cfg:
        return cfg['grip_width']
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
