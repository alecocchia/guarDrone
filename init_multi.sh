#!/bin/bash

# Definizione colori ANSI
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

echo -e "${CYAN}======================================================${NC}"
echo -e "${GREEN}   H-CoRE Multi-Robot Simulation Container Init${NC}" 
echo -e "${CYAN}======================================================${NC}" 

# Setup ROS2 environment
echo -e "${BLUE}🔧 UGV simulation build...${NC}" 
cd /home/user/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_description_pkg
source install/setup.bash
colcon build --packages-select rover_description_pkg rover_gazebo
source install/setup.bash

# Setup ROS2 environment
echo -e "${BLUE}🔧 PTZ simulation build...${NC}" 
cd /root/ptz_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select axis_msgs
source install/setup.bash
colcon build --packages-select axis_camera ptz_manager ptz_action_server_msgs camera_info_manager_py ptz_gz_sim
source install/setup.bash

# Setup PX4
echo -e "${BLUE}🚁 Setup PX4 environment...${NC}"
cd ~/ros2_ws
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=17

echo -e "${GREEN}✅ Build completato con successo!${NC}"
echo -e "${CYAN}======================================================${NC}"
echo -e "${WHITE}🚀 HOW TO RUN THE SIMULATION:${NC}"
echo -e "${YELLOW}  - $ tmuxp load src/pkg/babyk_drone_manager/utils/multi_simulation.yml${NC}"
echo -e "${CYAN}======================================================${NC}"
echo -e "${PURPLE}📝 NB: You need to run the UGV motion stack in a separate container with the same ROS_DOMAIN_ID${NC}"
echo -e "${WHITE}🛑 How to stop the simulation:${NC}"
echo -e "${YELLOW}  - $ tmux kill-server${NC}"

# Avvia bash interattivo
exec bash