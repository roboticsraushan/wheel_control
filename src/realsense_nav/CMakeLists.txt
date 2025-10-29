cmake_minimum_required(VERSION 3.8)
project(realsense_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviorState.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Behavior Tree C++ executable
add_executable(behavior_tree_cpp_node 
  src/behavior_tree_cpp_node.cpp
  src/bt_actions.cpp
)

ament_target_dependencies(behavior_tree_cpp_node
  rclcpp
  geometry_msgs
  std_msgs
  behaviortree_cpp_v3
)

rosidl_target_interfaces(behavior_tree_cpp_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install C++ executables
install(TARGETS
  behavior_tree_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  realsense_nav/segmentation_node.py
  realsense_nav/color_tuner.py
  realsense_nav/rgb_segmentation_node.py
  realsense_nav/rgb_color_tuner.py
  realsense_nav/goal_detection_node.py
  realsense_nav/pure_pursuit_controller.py
  realsense_nav/nav_monitor.py
  realsense_nav/behavior_tree_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install config files (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# Install behavior tree XML files
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
