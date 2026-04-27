#!/bin/bash

# docker exec -itu 0 px4-ros2 bash  // run root

# enable access to xhost from the container (Abilita GPU e GUI)
xhost +local:docker

# Trova la cartella root GuarDRONE (case insensitive per sicurezza)
HOST_GUARDRONE_DIR=$(find "/home/${USER}" -maxdepth 4 -type d -iname "guarDrone" -print -quit 2>/dev/null)

if [ -z "$HOST_GUARDRONE_DIR" ]; then
    echo "[ERROR] Impossibile trovare la cartella 'guarDrone'."
    exit 1
else
    echo "[INFO] Trovata cartella guarDrone in: $HOST_GUARDRONE_DIR"
fi

GZ_ENVIRONMENT_PKG=gz_env_pkg	# cambiare se cambia il nome del package per il setup dell'environment

echo "---------------------------------------------------"

# Run docker and open bash shell
docker run --rm -it --privileged \
--gpus all \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v "/dev:/dev" \
-v "${HOST_GUARDRONE_DIR}:/root/guarDrone:rw" \
--env="DISPLAY=$DISPLAY" \
--env="NVIDIA_VISIBLE_DEVICES=all" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
-e ROS_DOMAIN_ID=14 \
-e XDG_RUNTIME_DIR="/tmp/runtime-root" \
-e PX4_GZ_MODELS="/root/PX4-Autopilot/Tools/simulation/gz/models" \
-e PX4_GZ_WORLDS="/root/PX4-Autopilot/Tools/simulation/gz/worlds" \
-e GZ_SIM_RESOURCE_PATH="/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds:/root/guarDrone/my_ros2_ws/src/$GZ_ENVIRONMENT_PKG/models:/root/guarDrone/models:/root/guarDrone/worlds" \
-e GZ_SIM_SYSTEM_PLUGIN_PATH="/root/guarDrone/my_ros2_ws/install/$GZ_ENVIRONMENT_PKG/lib" \
-e LD_LIBRARY_PATH=/opt/acados/lib \
-w /root/guarDrone/my_ros2_ws \
--network host \
--name=px4-cnt px4-img bash
