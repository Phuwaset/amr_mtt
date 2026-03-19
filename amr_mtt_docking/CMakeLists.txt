cmake_minimum_required(VERSION 3.5)
project(amr_mtt_docking)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python docking module (used as library import)
install(
  DIRECTORY scripts/docking
  DESTINATION lib/python3.10/site-packages
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables to lib/${PROJECT_NAME}/
# (required for ros2 launch Node(executable=...) to find them)
install(
  PROGRAMS
    scripts/docking/enhanced_docking_node.py
    scripts/docking/pid_controller.py
    scripts/docking/docking_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
