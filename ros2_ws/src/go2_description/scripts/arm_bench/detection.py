"""Image + depth -> one 3D measurement, on plain numpy arrays.

No ROS types cross this boundary: `measure()` takes an HxW float depth
image and a pixel box, and returns pixels plus metres.  That is what makes
"image + depth -> object pose" testable without a running simulator.
"""
import cv2
import numpy as np

DEPTH_MIN = 0.05
DEPTH_MAX = 3.0
# Anything within this many metres of the nearest surface counts as "the
# top face" of a box or cylinder.
TOP_FACE_BAND = 0.015
TOP_FACE_PERCENTILE = 5


def label_mask(bgr, ranges, kernel_size=5):
    """uint8 mask of the pixels matching this label's HSV ranges."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def bbox_slices(bbox, shape):
    """(u, v, bw, bh) + image shape -> clamped (u0, u1, v0, v1)."""
    u, v, bw, bh = bbox
    height, width = shape[:2]
    u0 = max(int(u - bw / 2.0), 0)
    u1 = min(int(u + bw / 2.0) + 1, width)
    v0 = max(int(v - bh / 2.0), 0)
    v1 = min(int(v + bh / 2.0) + 1, height)
    return u0, u1, v0, v1


def measure(depth, bbox, shape_kind, mask=None,
            depth_min=DEPTH_MIN, depth_max=DEPTH_MAX):
    """Pixel centre and depth of one detection.

    Spheres report the bbox centre and the median depth: their centre sits
    one radius above the table, so depth alone locates them.  Boxes and
    cylinders report the centroid of their nearest-depth band, which is the
    top face; taking the bbox centre would hit the side wall instead.

    Returns (u, v, z) or None when nothing usable is in the box.
    """
    u, v = int(round(bbox[0])), int(round(bbox[1]))
    bw, bh = bbox[2], bbox[3]
    u0, u1, v0, v1 = bbox_slices((u, v, bw, bh), depth.shape)
    patch = depth[v0:v1, u0:u1]
    usable = np.isfinite(patch) & (patch > depth_min) & (patch < depth_max)
    if mask is not None:
        usable &= mask[v0:v1, u0:u1] > 0
    if not np.any(usable):
        return None

    if shape_kind == 'sphere':
        return (float(u), float(v), float(np.median(patch[usable])))

    depths = patch[usable]
    nearest = float(np.percentile(depths, TOP_FACE_PERCENTILE))
    on_top = usable & (patch <= nearest + TOP_FACE_BAND)
    vv, uu = np.mgrid[v0:v1, u0:u1]
    return (float(uu[on_top].mean()),
            float(vv[on_top].mean()),
            float(np.median(patch[on_top])))


def draw_detection(bgr, bbox, text, colour=(0, 255, 0)):
    """Annotate one detection in place; returns the same image."""
    u, v, bw, bh = bbox
    x1, y1 = int(u - bw / 2), int(v - bh / 2)
    x2, y2 = int(u + bw / 2), int(v + bh / 2)
    cv2.rectangle(bgr, (x1, y1), (x2, y2), colour, 2)
    cv2.putText(bgr, text, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                colour, 1)
    return bgr
