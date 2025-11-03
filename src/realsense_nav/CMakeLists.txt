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
find_package(rosidl_default_generators REQUIRED)

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviorState.msg"
  "msg/JunctionInfo.msg"
  "msg/NavigationTask.msg"
  "msg/JunctionPath.msg"
  "msg/JunctionDetection.msg"
  "srv/RecordJunction.srv"
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/GetLastPosition.srv"
  "action/NavigateToNode.action"
  "action/NavigateToJunction.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include directories (no longer needed for C++ only)
# include_directories(include)

# Install Python executables
install(PROGRAMS
  realsense_nav/segmentation_node.py
  realsense_nav/color_tuner.py
  realsense_nav/rgb_color_tuner.py
  realsense_nav/goal_detection_node.py
  realsense_nav/pure_pursuit_controller.py
  realsense_nav/nav_monitor.py
  realsense_nav/behavior_tree_controller.py
  realsense_nav/scene_graph_node.py
  realsense_nav/vision_safety_node.py
  realsense_nav/scene_graph_viz.py
  realsense_nav/llm_goal_detection.py
  realsense_nav/nl_interpreter.py
  realsense_nav/topo_planner.py
  realsense_nav/training/junction_manager_node.py
  realsense_nav/training/scene_graph_recorder.py
  realsense_nav/training/enhanced_scene_graph_recorder.py
  realsense_nav/training/voice_command_node.py
  realsense_nav/training/topo_map_builder.py
  realsense_nav/planning/voice_nav_parser.py
  realsense_nav/planning/graph_path_planner.py
  realsense_nav/behavior_tree_node.py
  realsense_nav/navigation/junction_recognizer.py
  realsense_nav/navigation/junction_navigator.py
  realsense_nav/navigation/obstacle_aware_navigator.py
  realsense_nav/navigation/junction_visibility_monitor.py
  realsense_nav/navigation/simple_odometry.py
  realsense_nav/navigation/localization_manager.py
  realsense_nav/task/destination_verifier.py
  scripts/monitor_segmentation.py
  scripts/motor_pwm_monitor.py
  scripts/test_cameras.py
  scripts/tune_yellow_zone.py
  scripts/view_segmentation.py
  
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
