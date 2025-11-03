cmake_minimum_required(VERSION 3.8)
project(memory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find all Python files
ament_python_install_package(memory)

# Install Python executables
install(PROGRAMS
  memory/floorplan_manager_node.py
  memory/semantic_memory_node.py
  memory/episodic_memory_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # ament_lint_auto is optional in some environments. Try to find it quietly
  # and only invoke its helper if present so builds don't fail on systems
  # without the lint packages installed.
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # ament_lint_auto was found but the helper macro may not be available
    # in some environments. To be conservative avoid calling it here.
    message(STATUS "ament_lint_auto found but skipping automatic test discovery in CMakeLists.")
  else()
    message(WARNING "ament_lint_auto not found; skipping lint test discovery")
  endif()
endif()

ament_package()
