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

HOST_FIRMWARE_DIR=$(find "/home/${USER}" -maxdepth 4 -type d -iname "PX4-Autopilot" -print -quit 2>/dev/null)

if [ -z "$HOST_FIRMWARE_DIR" ]; then
    echo "[ERROR] Impossibile trovare la cartella 'PX4-Autopilot'."
    exit 1
else
    echo "[INFO] Trovata cartella PX4-Autopilot in: $HOST_FIRMWARE_DIR"
fi

GZ_ENVIRONMENT_PKG=gz_env_pkg	# cambiare se cambia il nome del package per il setup dell'environment

echo "---------------------------------------------------"

# === INIZIO BLOCCO CONTROLLO INTELLIGENTE GPU ===
GPU_FLAGS=""
if command -v nvidia-smi &> /dev/null; then
    echo "[INFO] GPU NVIDIA rilevata. Abilito il supporto hardware in Docker."
    GPU_FLAGS="--gpus all --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all"
else
    echo "[INFO] Nessuna GPU NVIDIA rilevata o driver mancanti. Avvio in modalità CPU."
fi
# === FINE BLOCCO CONTROLLO INTELLIGENTE GPU ===

echo "---------------------------------------------------"

# Run docker and open bash shell
docker run --rm -it --privileged \
$GPU_FLAGS \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v "/dev:/dev" \
-v "${HOST_GUARDRONE_DIR}/.git:/root/.git:ro" \
-v "${HOST_FIRMWARE_DIR}:/root/PX4-Autopilot:rw" \
-v "${HOST_GUARDRONE_DIR}/my_ros2_ws/src:/root/my_ros2_ws/src:rw" \
-v "${HOST_GUARDRONE_DIR}/my_ros2_ws/SimulationScripts:/root/my_ros2_ws/SimulationScripts:rw" \
--env="DISPLAY=$DISPLAY" \
-e ROS_DOMAIN_ID=14 \
-e XDG_RUNTIME_DIR="/tmp/runtime-root" \
-e PX4_GZ_MODELS="/root/PX4-Autopilot/Tools/simulation/gz/models" \
-e PX4_GZ_WORLDS="/root/PX4-Autopilot/Tools/simulation/gz/worlds" \
-e GZ_SIM_RESOURCE_PATH="/root/PX4-Autopilot/Tools/simulation/gz/models:/root/PX4-Autopilot/Tools/simulation/gz/worlds:/root/my_ros2_ws/src/$GZ_ENVIRONMENT_PKG/models:/root/my_ros2_ws/models:/root/my_ros2_ws/src/$GZ_ENVIRONMENT_PKG/worlds" \
-e GZ_SIM_SYSTEM_PLUGIN_PATH="/root/my_ros2_ws/install/$GZ_ENVIRONMENT_PKG/lib/$GZ_ENVIRONMENT_PKG" \
-e LD_LIBRARY_PATH=/opt/acados/lib \
-w /root/my_ros2_ws \
--network host \
--name=px4-cnt guardrone_img bash