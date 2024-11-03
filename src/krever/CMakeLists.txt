cmake_minimum_required(VERSION 3.8)
project(krever)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ignition-gazebo6 REQUIRED)  # Assuming you're using Ignition Gazebo version 6
find_package(ros_gz_bridge REQUIRED)

# Create the executable for the node
add_executable(krever_node src/krever_node.cpp)  # Ensure the file exists or replace with actual source

# Link dependencies to the executable
ament_target_dependencies(krever_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  ignition-gazebo6
  ros_gz_bridge
)

# Install the executable
install(TARGETS krever_node
  DESTINATION lib/${PROJECT_NAME})

# Install directories (e.g., launch files, URDF, etc.)
install(DIRECTORY
  launch
  urdf
  worlds
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
