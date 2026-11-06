"""Shared pieces of the arm bench stack, split by layer.

Module map (see docs/architecture.md):

    constants.py  frame names, joint names, tool orientation   (L4/L2 both)
    objects.py    object table + size helpers                  (data)
    geometry.py   pure maths: deprojection, place maths         (L2)
    detection.py  ndarray -> measurement, no ROS at all         (L2)
    model.py      fused observation state                       (L3)
    world.py      MoveIt collision scene adapters               (L3)
    vision.py     camera/TF adapter around detection.py         (L2)
    skills.py     MoveIt motion primitives                      (L4)
    grasp.py      the VisualGrasp node and its orchestration    (L4)
    language.py   Chinese command parsing, no ROS at all        (L5)

Nothing in this package may import `rclpy.node.Node`.  Only `world.py`
and `skills.py` may import `moveit_msgs`.
"""
