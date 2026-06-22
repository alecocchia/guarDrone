cmake_minimum_required(VERSION 3.5)
project(fd_haptic_joy)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(fd_haptic_joy_node src/fd_haptic_joy_node.cpp)
target_include_directories(fd_haptic_joy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fd_haptic_joy_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
target_link_libraries(fd_haptic_joy_node Eigen3::Eigen)
install(TARGETS
  fd_haptic_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
