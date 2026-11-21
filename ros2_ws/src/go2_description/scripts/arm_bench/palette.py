"""Colour signatures for the bench objects.  Pure data: no ROS, no OpenCV.

The detector node and the grasp node both read this table, so the two can
never disagree about what "blue_cube" looks like.

Hue is OpenCV's 0-180 scale.  The scene holds two objects (a blue cube on a
green table, and a yellow box that receives it), so two slots:

    95-135          blue_cube
    20-34           yellow_box

A new object needs one interval here, one entry in `objects.OBJECTS`, and a
model in `worlds/arm_bench.sdf`.  Keep the hue slot clear of its neighbours.
The green table sits around hue 55-65, well away from both slots, so the
table itself is never picked up as an object.  The yellow slot starts at
saturation 140 on purpose: wood/tan surfaces sit near hue 15-20 but are much
less saturated, so the threshold keeps them out without a distance gate.
"""

HSV_RANGES = {
    'blue_cube': [((95, 80, 40), (135, 255, 255))],
    'yellow_box': [((20, 140, 120), (34, 255, 255))],
}

# Per-class upper bound on the measured bbox depth, in metres.  Empty today:
# only a white class that is not separable by hue would need it (the grey
# floor is about a metre away while bench objects are 0.25-0.5 m from the
# camera).  The hook stays so adding one does not reopen detection.py.
MAX_DEPTH = {}

# BGR outline colours used on the annotated image.
DRAW_COLORS = {
    'blue_cube': (255, 100, 0),
    'yellow_box': (0, 200, 255),
}
