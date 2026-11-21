"""Colour signatures for the bench objects.  Pure data: no ROS, no OpenCV.

The detector node and the grasp node both read this table, so the two can
never disagree about what "yellow_box" looks like.

Hue is OpenCV's 0-180 scale.  The classes sit in separate hue slots:

    0-12, 168-180   red_ball      (wraps around, so two intervals)
    18-38           yellow_box
    45-78           green_cylinder
    95-135          blue_cube

A new object needs one interval here, one entry in `objects.OBJECTS`, and a
model in `worlds/arm_bench.sdf`.  Keep the hue slot clear of its neighbours:
`green_cylinder` used to span 35-90, which collided with anything cyan.
"""

HSV_RANGES = {
    'red_ball': [((0, 70, 40), (12, 255, 255)),
                 ((168, 70, 40), (180, 255, 255))],
    'yellow_box': [((18, 80, 50), (38, 255, 255))],
    'green_cylinder': [((45, 60, 40), (78, 255, 255))],
    'blue_cube': [((95, 80, 40), (135, 255, 255))],
}

# Per-class upper bound on the measured bbox depth, in metres.  Empty today:
# only a white class that is not separable by hue would need it (the grey
# floor is about a metre away while bench objects are 0.25-0.5 m from the
# camera).  The hook stays so adding one does not reopen detection.py.
MAX_DEPTH = {}

# BGR outline colours used on the annotated image.
DRAW_COLORS = {
    'red_ball': (0, 0, 255),
    'blue_cube': (255, 100, 0),
    'green_cylinder': (0, 200, 0),
    'yellow_box': (0, 220, 220),
}
