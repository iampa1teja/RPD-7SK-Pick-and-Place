cmake_minimum_required(VERSION 3.8)
project(msgs_pkg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/MotionStep.msg"
  "action/MotionSequence.action"
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()