#!/bin/bash
# =============================================================================
# Script di Replay Offline Esperimenti Hardware in RViz2
# GuarDrone + Interaction Drone
# =============================================================================
# Uso:
#   ./replay_exp.sh                     -> Esegue il replay dell'ultimo test registrato
#   ./replay_exp.sh <path_esperimento>  -> Esegue il replay di una cartella specifica
#   ./replay_exp.sh <path> --loop       -> Replay ciclico
#   ./replay_exp.sh <path> -r 0.5       -> Replay a velocita' dimezzata
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ambiente ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERRORE] ROS 2 Humble non trovato in /opt/ros/humble/setup.bash"
    exit 1
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

BAGS_DIR="${SCRIPT_DIR}/bag_files"

TARGET_INPUT=""
EXTRA_ARGS=()

# Parsing argomenti
while [[ $# -gt 0 ]]; do
    case "$1" in
        --loop|-l|-r|--rate|--start-offset|-s|--clock)
            EXTRA_ARGS+=("$1")
            shift
            if [[ "$1" =~ ^[0-9] ]] || [[ "$1" =~ ^[a-zA-Z] ]]; then
                EXTRA_ARGS+=("$1")
                shift
            fi
            ;;
        -h|--help)
            echo "Uso: $0 [path_esperimento_o_rosbag] [opzioni_ros2_bag_play]"
            echo ""
            echo "Opzioni:"
            echo "  --loop, -l       Riproduzione ciclica"
            echo "  -r, --rate RATE  Fattore velocita' (es. -r 0.5 per 50%, -r 2.0 per 2x)"
            echo "  -h, --help       Mostra questo messaggio di aiuto"
            exit 0
            ;;
        *)
            if [ -z "$TARGET_INPUT" ]; then
                TARGET_INPUT="$1"
            else
                EXTRA_ARGS+=("$1")
            fi
            shift
            ;;
    esac
done

BAG_PATH=""

if [ -n "$TARGET_INPUT" ]; then
    # Se e' un percorso diretto a una cartella con metadata.yaml
    if [ -f "${TARGET_INPUT}/metadata.yaml" ]; then
        BAG_PATH="${TARGET_INPUT}"
    elif [ -f "${TARGET_INPUT}/rosbag/metadata.yaml" ]; then
        BAG_PATH="${TARGET_INPUT}/rosbag"
    elif [ -d "${BAGS_DIR}/${TARGET_INPUT}/rosbag" ] && [ -f "${BAGS_DIR}/${TARGET_INPUT}/rosbag/metadata.yaml" ]; then
        BAG_PATH="${BAGS_DIR}/${TARGET_INPUT}/rosbag"
    elif [ -d "${BAGS_DIR}/exp_${TARGET_INPUT}/rosbag" ] && [ -f "${BAGS_DIR}/exp_${TARGET_INPUT}/rosbag/metadata.yaml" ]; then
        BAG_PATH="${BAGS_DIR}/exp_${TARGET_INPUT}/rosbag"
    else
        echo "[ERRORE] Impossibile trovare un rosbag valido in: $TARGET_INPUT"
        exit 1
    fi
else
    echo "[INFO] Nessun esperimento specificato. Ricerca dell'ultimo rosbag in: ${BAGS_DIR}..."
    # Trova la cartella exp_* piu' recente con rosbag/metadata.yaml
    LATEST_BAG=$(find "${BAGS_DIR}" -mindepth 2 -maxdepth 3 -type f -name "metadata.yaml" 2>/dev/null | sort -r | head -n 1 || true)
    if [ -n "$LATEST_BAG" ]; then
        BAG_PATH="$(dirname "$LATEST_BAG")"
    fi
fi

if [ -z "$BAG_PATH" ] || [ ! -f "${BAG_PATH}/metadata.yaml" ]; then
    echo "[ERRORE] Nessun rosbag con metadata.yaml trovato in ${BAGS_DIR}!"
    echo "Assicurati di aver eseguito almeno un test con 'record_rosbag: true' in hw_config.yaml."
    exit 1
fi

RVIZ_CONFIG="${WS_DIR}/src/gcs_pkg/config/rviz_config_file_hw.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "[WARNING] Configurazione RViz non trovata in $RVIZ_CONFIG, avvio RViz default..."
    RVIZ_CONFIG=""
fi

echo "============================================================================="
echo " REPLAY ESPERIMENTO OFFLINE"
echo " Rosbag : $BAG_PATH"
echo " RViz   : $RVIZ_CONFIG"
echo " Extra  : ${EXTRA_ARGS[*]}"
echo "============================================================================="

# Pulizia all'uscita (Ctrl+C o termine)
cleanup() {
    echo ""
    echo "[INFO] Chiusura replay..."
    if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
        kill "$RVIZ_PID" 2>/dev/null || true
    fi
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Avvio RViz2 con use_sim_time sincronizzato su /clock del rosbag
if [ -n "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
else
    rviz2 --ros-args -p use_sim_time:=true &
fi
RVIZ_PID=$!

sleep 2

echo "[INFO] Avvio riproduzione rosbag..."
ros2 bag play "$BAG_PATH" --clock "${EXTRA_ARGS[@]}"

echo "[INFO] Riproduzione completata."
echo "Premi Ctrl+C per chiudere RViz..."
wait "$RVIZ_PID" 2>/dev/null || true
