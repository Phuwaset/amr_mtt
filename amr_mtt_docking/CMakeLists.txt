cmake_minimum_required(VERSION 3.5)
project(amr_mtt_docking)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python docking module
install(
  DIRECTORY scripts/docking
  DESTINATION lib/python3.10/site-packages
  FILES_MATCHING PATTERN "*.py"
)

# Install marker image
install(
  FILES aruco_marker_0.png
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
