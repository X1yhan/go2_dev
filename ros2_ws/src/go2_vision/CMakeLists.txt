cmake_minimum_required(VERSION 3.8)
project(go2_vision)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetDetection.msg"
  "msg/TargetObservation.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(
  PROGRAMS
    scripts/calib_check.py
    scripts/ball_detector.py
    scripts/target_localizer.py
    scripts/target_director.py
    scripts/eval_monitor.py
    scripts/stand_keeper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
