cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(gz_env_pkg)

# Required for ROS 2 packages
find_package(ament_cmake REQUIRED)

# ROS 2 Core
find_package(rclcpp REQUIRED)

# ROS 2 Message types for publishing/subscribing
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Gazebo Harmonic (8) dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED) 
find_package(gz-msgs10 REQUIRED) 
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-transport13 REQUIRED)

# Sezione per SDF (Harmonic usa sdformat14)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDFORMAT REQUIRED sdformat14)

# Per la parte ros_gz_sim (il bridge ROS 2 <-> Gazebo)
find_package(ros_gz_sim REQUIRED)

# Add the plugin library
add_library(peg_pose_gz_plugin SHARED
  src/peg_pose_gz_plugin.cpp
)

# Set standard C++ flags for the plugin
target_compile_features(peg_pose_gz_plugin PUBLIC cxx_std_17)
target_compile_options(peg_pose_gz_plugin PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

## Directory di Inclusione
target_include_directories(peg_pose_gz_plugin  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Directory di inclusione per ROS 2 core e messaggi
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  # Directory di inclusione per sdformat14
  ${SDFORMAT_INCLUDE_DIRS}
  # Directory di inclusione per Gazebo Harmonic
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
)

## Librerie di Linkage
ament_target_dependencies(peg_pose_gz_plugin
  rclcpp
  gz-math7
  gz-msgs10
  gz-plugin2
  gz-common5
  gz-transport13
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install the plugin so Gazebo can find it
install(TARGETS peg_pose_gz_plugin
  DESTINATION lib/${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
)

# Install models and worlds
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Export targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Mark as ament package
ament_package()
