cmake_minimum_required(VERSION 3.8)
project(moveit_7sk)

# find all packages your code depends on
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

# generate service headers from your .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToNamedPose.srv"
  "srv/GoToPose.srv"
  "srv/SetGripper.srv"
  "srv/AddNamedPose.srv"
  DEPENDENCIES geometry_msgs
)

# declare the executable
add_executable(moveit_7sk_node src/moveit_7sk_node.cpp)

# link ROS2/MoveIt dependencies to the executable
ament_target_dependencies(moveit_7sk_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# link yaml-cpp (non-ROS library, different syntax)
target_link_libraries(moveit_7sk_node yaml-cpp)

# link the generated service headers to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit_7sk_node ${cpp_typesupport_target})

# install the executable so ros2 run can find it
install(TARGETS moveit_7sk_node
  DESTINATION lib/${PROJECT_NAME}
)

# install config folder
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()