#!/bin/bash
# =============================================================================
# run_guardrone_cnt.sh — Avvia il container Docker per il drone GuarDrone (REAL)
#
# Prerequisiti sul PC del drone (LattePanda 3 Delta):
#   1. Docker installato
#   2. Repo clonato con sparse-checkout:
#        git clone --depth 1 --filter=blob:none --sparse \
#            -b master https://github.com/alecocchia/guarDrone.git ~/guarDrone
#        cd ~/guarDrone
#        git sparse-checkout set \
#            my_ros2_ws/src/guardrone_pkg \
#            my_ros2_ws/src/utils_pkg \
#            docker
#
#   3. Per aggiornare i sorgenti: cd ~/guarDrone && git pull
#
# Uso:
#   ./run_guardrone_cnt.sh
# =============================================================================

set -e

# === CONFIGURAZIONE ===
CONTAINER_NAME="guardrone-cnt"
IMAGE_NAME="guardrone_img"
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-22} # POCHO

# === TROVA LA CARTELLA guarDrone (sparse-checkout) ===
HOST_GUARDRONE_DIR=$(find "/home/${USER}" -maxdepth 4 -type d -iname "guarDrone" -print -quit 2>/dev/null)

if [ -z "$HOST_GUARDRONE_DIR" ]; then
    echo "[ERROR] Impossibile trovare la cartella 'guarDrone'."
    echo "        Eseguire prima il clone sparse:"
    echo "          git clone --depth 1 --filter=blob:none --sparse \\"
    echo "              -b master https://github.com/alecocchia/guarDrone.git ~/guarDrone"
    echo "          cd ~/guarDrone && git sparse-checkout set \\"
    echo "              my_ros2_ws/src/guardrone_pkg my_ros2_ws/src/utils_pkg"
    exit 1
else
    echo "[INFO] Trovata cartella guarDrone in: $HOST_GUARDRONE_DIR"
fi

# === VERIFICA CHE I PKG ESISTANO (sparse-checkout) ===
for PKG in guardrone_pkg utils_pkg; do
    if [ ! -d "${HOST_GUARDRONE_DIR}/my_ros2_ws/src/${PKG}" ]; then
        echo "[ERROR] Pacchetto '${PKG}' non trovato in ${HOST_GUARDRONE_DIR}/my_ros2_ws/src/"
        echo "        Verificare il sparse-checkout: cd ${HOST_GUARDRONE_DIR} && git sparse-checkout list"
        exit 1
    fi
done
echo "[INFO] Pacchetti guardrone_pkg e utils_pkg trovati."

echo "---------------------------------------------------"

# === GPU INTEL (Mesa) — accesso a /dev/dri per accelerazione grafica ===
DRI_FLAGS=""
if [ -d "/dev/dri" ]; then
    echo "[INFO] GPU Intel (Mesa) rilevata. Abilito accesso a /dev/dri."
    DRI_FLAGS="--device=/dev/dri"
else
    echo "[WARN] /dev/dri non trovato. Nessuna accelerazione grafica hardware."
fi

echo "---------------------------------------------------"

# === ABILITA GUI (X11) ===
xhost +local:docker 2>/dev/null || echo "[WARN] xhost non disponibile, GUI potrebbe non funzionare."

echo "---------------------------------------------------"
echo "[INFO] Avvio container '${CONTAINER_NAME}' dall'immagine '${IMAGE_NAME}'"
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[INFO] Volume mount: guardrone_pkg, utils_pkg"
echo "---------------------------------------------------"

# === RUN CONTAINER ===
docker run --rm -it --privileged \
    ${DRI_FLAGS} \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "/dev:/dev" \
    -v "${HOST_GUARDRONE_DIR}/my_ros2_ws/src/guardrone_pkg:/root/my_ros2_ws/src/guardrone_pkg:rw" \
    -v "${HOST_GUARDRONE_DIR}/my_ros2_ws/src/utils_pkg:/root/my_ros2_ws/src/utils_pkg:rw" \
    --env="DISPLAY=${DISPLAY}" \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e XDG_RUNTIME_DIR="/tmp/runtime-root" \
    -e LD_LIBRARY_PATH=/opt/acados/lib \
    -w /root/my_ros2_ws \
    --name=${CONTAINER_NAME} \
    ${IMAGE_NAME} bash
