cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config dae urdf xacro rviz launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/go2_trot.py scripts/go2_teleop.py scripts/build_arm_model.py scripts/grasp_ball.py scripts/chase_grasp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
