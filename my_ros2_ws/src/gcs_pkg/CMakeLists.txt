cmake_minimum_required(VERSION 3.8)
project(gcs_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# =========================================================
# SEZIONE PYTHON
# =========================================================

# 1. Installa l'intera cartella come modulo Python 
# (permette gli import "from gcs_pkg.common import *")
ament_python_install_package(${PROJECT_NAME})

# 2. Rendi disponibili i nodi eseguibili a "ros2 run" e ai launch file
install(PROGRAMS
  ${PROJECT_NAME}/supervisor.py
  ${PROJECT_NAME}/logger.py
  ${PROJECT_NAME}/keyboard_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installa i file di launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Installa eventuali configurazioni (se hai copiato anche la cartella config qui)
 install(DIRECTORY config
   DESTINATION share/${PROJECT_NAME}
 )

ament_package()
