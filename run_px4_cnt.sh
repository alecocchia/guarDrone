
# docker exec -itu 0 px4-ros2 bash  // run root

# enable access to xhost from the container
xhost +


#Trova la cartella PX4-Autopilot (Firmware)
HOST_PX4_DIR=$(find "/home/${USER}" -type d -name "PX4-Autopilot" -print -quit 2>/dev/null)
if [ -z "$HOST_PX4_DIR" ]; then
    echo "[ERROR] Impossibile trovare la cartella 'PX4-Autopilot'."
    exit 1
else
    echo "[INFO] Trovata cartella PX4 in: $HOST_PX4_DIR"
fi

#Trova la cartella ros2_ws-src
HOST_ROS2_WS_SRC=$(find "/home/${USER}" -type d -name "ros2_ws-src" -print -quit 2>/dev/null)

if [ -z "$HOST_ROS2_WS_SRC" ]; then
    echo "[ERROR] Impossibile trovare la cartella 'ros2_ws-src'."
    exit 1
else
    echo "[INFO] Trovata cartella workspace in: $HOST_ROS2_WS_SRC"
fi

#Trova la mia cartella my_ros2_ws
HOST_MY_ROS2_WS=$(find "/home/${USER}" -type d -name "my_ros2_ws" -print -quit 2>/dev/null)

#Trova lo script di inizializzazione init_drone.sh
HOST_INIT_SCRIPT=$(find "/home/${USER}" -type f -name "init_drone.sh" -print -quit 2>/dev/null)

if [ -z "$HOST_INIT_SCRIPT" ]; then
    echo "[ERROR] Impossibile trovare il file 'init_drone.sh'."
    exit 1
else
    echo "[INFO] Trovato script di init in: $HOST_INIT_SCRIPT"
fi

GZ_ENVIRONMENT_PKG=gz_env_pkg	#cambiare se cambia il nome del package per il setup dell'environment

echo "---------------------------------------------------"

# Run docker and open bash shell 
docker run --rm -it --privileged \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v "/dev:/dev" \
-v "${HOST_PX4_DIR}:/root/PX4-Autopilot:rw" \
-v "${HOST_ROS2_WS_SRC}/pkg:/root/ros2_ws/src/pkg:rw" \
-v "${HOST_ROS2_WS_SRC}/px4_ros_com:/root/px4_ws/src/px4_ros_com:rw" \
-v "${HOST_MY_ROS2_WS}/src:/root/my_ros2_ws/src:rw" \
-v "${HOST_MY_ROS2_WS}/SimulationScripts:/root/my_ros2_ws/SimulationScripts:rw" \
-v "${HOST_INIT_SCRIPT}:/root/init_drone.sh:rw" \
--env="DISPLAY=$DISPLAY" \
-e ROS_DOMAIN_ID=14 \
-e XDG_RUNTIME_DIR="/tmp/runtime-root" \
-e PX4_GZ_MODELS="/root/PX4-Autopilot/Tools/simulation/gz/models" \
-e PX4_GZ_WORLDS="/root/PX4-Autopilot/Tools/simulation/gz/worlds" \
-e GZ_SIM_RESOURCE_PATH="/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds:root/my_ros2_ws/src/$GZ_ENVIRONMENT_PKG/models:/root/my_ros2_ws/install/$GZ_ENVIRONMENT_PKG/share/$GZ_ENVIRONMENT_PKG/models" \
-e GZ_SIM_SYSTEM_PLUGIN_PATH="/root/my_ros2_ws/install/$GZ_ENVIRONMENT_PKG/lib/$GZ_ENVIRONMENT_PKG" \
-e LD_LIBRARY_PATH=/opt/acados/lib \
-w /root/my_ros2_ws \
--network host \
--name=px4-cnt px4-img bash
