"""Names and numbers that both the vision side and the motion side need.

Pure literals only: no ROS imports, so every layer may depend on this
module without dragging in a message package.
"""

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4',
              'joint5', 'joint6', 'joint7']
GRIPPER_JOINTS = ['gripper_finger1_joint', 'gripper_finger2_joint']

BASE_FRAME = 'base_link'
TIP_LINK = 'Link7'
CAMERA_FRAME = 'camera_depth_optical_frame'

# Tool points down, rotated 180 deg about the tool axis.  The camera is on
# the side away from Link2; yaw search showed 135/180/225/270/315 are
# collision-free, but only 180 keeps all four objects in the default view.
DOWN_Q = (0.0, -1.0, 0.0, 0.0)
