# 2D detection of a target in an image
std_msgs/Header header

string class_id          # e.g. "target_ball"

float32 score            # confidence in [0, 1]

# bounding box in pixels
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# bbox center in pixels
float32 center_u
float32 center_v

# bbox area in pixels
float32 area
