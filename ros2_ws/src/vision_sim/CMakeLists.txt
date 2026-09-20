cmake_minimum_required(VERSION 3.8)
project(vision_sim)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetDetection.msg"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY urdf worlds config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/target_director.py
  scripts/target_detector.py
  scripts/target_localizer.py
  scripts/eval_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
