cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config dae urdf xacro rviz launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/go2_trot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
