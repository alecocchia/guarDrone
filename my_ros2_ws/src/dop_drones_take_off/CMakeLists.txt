cmake_minimum_required(VERSION 3.8)
project(dop_drones_take_off)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
# TODO: installa qpOASES prima di abilitare controll_velocita_tko
# find_library(QPOASES_LIB qpOASES HINTS /usr/local/lib REQUIRED)
# set(QPOASES_INCLUDE_DIR /usr/local/include)

add_executable(take_off src/take_off.cpp)

target_include_directories(take_off PRIVATE
  ${EIGEN3_INCLUDE_DIR}       # /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${px4_ros_com_INCLUDE_DIRS}
)

target_link_libraries(take_off
  Eigen3::Eigen               # target moderno CMake
)

ament_target_dependencies(take_off
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  px4_ros_com  
)

# TODO: controll_velocita_tko (CBF + qpOASES) - da abilitare quando qpOASES è installato
# add_executable(controll_velocita_tko src/offboard_control_velocita_takeoff.cpp
#   include/cbf.cpp             
#   include/UtilitiesDrones.cpp )
#
# target_include_directories(controll_velocita_tko PRIVATE
#   ${QPOASES_INCLUDE_DIR}
#   ${EIGEN3_INCLUDE_DIR}
#   ${CMAKE_CURRENT_SOURCE_DIR}/include 
#   ${px4_ros_com_INCLUDE_DIRS}
# )
#
# target_link_libraries(controll_velocita_tko
#   ${QPOASES_LIB}
#   Eigen3::Eigen
# )
#
# ament_target_dependencies(controll_velocita_tko
#   rclcpp
#   px4_msgs
#   geometry_msgs
#   std_msgs
#   px4_ros_com  
# )

add_executable(bt_executor src/bt_executor.cpp)
target_include_directories(bt_executor PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${px4_ros_com_INCLUDE_DIRS}
)
ament_target_dependencies(bt_executor
  rclcpp
  std_msgs
  behaviortree_cpp_v3
  ament_index_cpp
)
install(TARGETS bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Installa il file XML nella cartella share del package
install(FILES src/drone_bt.xml
  DESTINATION share/${PROJECT_NAME}
)



install(TARGETS take_off
  # controll_velocita_tko  # TODO: ri-abilitare con qpOASES
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
