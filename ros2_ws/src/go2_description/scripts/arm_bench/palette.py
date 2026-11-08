"""Colour signatures for the bench objects.  Pure data: no ROS, no OpenCV.

The detector node and the grasp node both read this table, so the two can
never disagree about what "milk_carton" looks like.

Hue is OpenCV's 0-180 scale.  The classes sit in separate hue slots:

    0-12, 168-180   red_ball
    18-38           yellow_box
    45-78           green_cylinder
    79-93           water_cup
    95-135          blue_cube
    136-165         tissue_box
    saturation<55   milk_carton   (white, needs the depth gate below)

The white class cannot be separated from the grey floor by colour, so it is
bounded by distance instead: bench objects are 0.25-0.5 m from the wrist
camera, the floor beyond the table edge is about a metre away.
"""

HSV_RANGES = {
    'red_ball': [((0, 70, 40), (12, 255, 255)),
                 ((168, 70, 40), (180, 255, 255))],
    'yellow_box': [((18, 80, 50), (38, 255, 255))],
    'green_cylinder': [((45, 60, 40), (78, 255, 255))],
    'water_cup': [((79, 70, 60), (93, 255, 255))],
    'blue_cube': [((95, 80, 40), (135, 255, 255))],
    'tissue_box': [((136, 60, 40), (165, 255, 255))],
    'milk_carton': [((0, 0, 150), (180, 55, 255))],
}

# Per-class upper bound on the measured bbox depth, in metres.  Only the
# white class needs one, but keeping it general costs nothing.
MAX_DEPTH = {'milk_carton': 0.65}

# BGR outline colours used on the annotated image.
DRAW_COLORS = {
    'red_ball': (0, 0, 255),
    'blue_cube': (255, 100, 0),
    'green_cylinder': (0, 200, 0),
    'yellow_box': (0, 220, 220),
    'milk_carton': (150, 150, 150),
    'water_cup': (200, 180, 0),
    'tissue_box': (200, 60, 160),
}
