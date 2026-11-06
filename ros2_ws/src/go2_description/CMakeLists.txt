cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config dae urdf xacro rviz launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/go2_trot.py scripts/go2_teleop.py scripts/build_arm_model.py scripts/grasp_ball.py scripts/chase_grasp.py scripts/arm_bench_home.py scripts/arm_bench_visual_grasp.py scripts/arm_bench_teleop.py scripts/arm_bench_object_detector.py scripts/arm_bench_language.py scripts/arm_bench_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

# Support package for the arm bench nodes above: pure python, no entry
# points of its own.  With --symlink-install this installs a symlink to the
# source directory, so edits take effect without a rebuild.
install(
  DIRECTORY scripts/arm_bench
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
