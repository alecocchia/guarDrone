cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(drone_mpc_pkg)

# Requisiti base ROS 2
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # Fondamentale per i pacchetti ibridi

# Dipendenze
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)

# =========================================================
# SEZIONE PYTHON
# =========================================================

# 1. Installa l'intera cartella come modulo Python 
# (permette gli import "from drone_mpc_pkg.common import *")
ament_python_install_package(${PROJECT_NAME})

# 2. Rendi disponibili i nodi eseguibili a "ros2 run" e ai launch file
install(PROGRAMS
  ${PROJECT_NAME}/human_goal_node.py
  ${PROJECT_NAME}/MPC_planner_node.py
  ${PROJECT_NAME}/offboard_trajectory_planner.py
  ${PROJECT_NAME}/offboard_admittance_planner.py
  ${PROJECT_NAME}/logger.py
  ${PROJECT_NAME}/geometric_controller.py
  ${PROJECT_NAME}/PID_controller.py
  ${PROJECT_NAME}/supervisor.py
  ${PROJECT_NAME}/teleop_peg.py
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

# =========================================================
# SEZIONE C++ (Futura)
# =========================================================
# Quando aggiungerai file in src/, scriverai qui i tuoi:
# add_executable(...)
# ament_target_dependencies(...)
# install(TARGETS ... DESTINATION lib/${PROJECT_NAME})


ament_package()
