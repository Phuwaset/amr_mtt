cmake_minimum_required(VERSION 3.5)
project(amr_mtt_bot)

# ===============================
# Find main build system
# ===============================
find_package(ament_cmake REQUIRED)

# ===============================
# Find dependencies
# ===============================
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)

# ===============================
# Install files
# ===============================
install(
  DIRECTORY launch meshes models urdf worlds rviz map robotiq_description
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Python scripts (only remapper for basic bot)
install(
  PROGRAMS 
    scripts/remapper.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===============================
# Export package
# ===============================
ament_package()