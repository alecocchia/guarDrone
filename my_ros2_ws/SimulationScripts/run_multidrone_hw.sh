#!/bin/bash
# =============================================================================
# run_multidrone_hw.sh — Esecuzione Multi-Drone su HARDWARE REALE
#
# Questo script gira sul PC GCS:
# 1. Avvia i nodi GCS locali (Agent, Supervisor, Haptic, Keyboard)
# 2. Si connette via SSH ai droni e lancia i nodi ROS2 all'interno dei 
#    container Docker (che devono essere GIA' in esecuzione sui droni).
#
# Prerequisiti:
# - Autenticazione SSH con chiave pubblica configurata per i droni
# - Container Docker avviati sui droni (es. con run_guardrone_cnt.sh)
# =============================================================================

SESSION_NAME="drone_hw"

FAKE_PUBLISHER="true" # set "false" when using the real supervisor

# =============================================================================
# CONFIGURAZIONE RETE E SSH
# =============================================================================
# Modifica questi parametri con gli IP e gli username corretti dei LattePanda
GD_USER="dummy" # utente del LattePanda guardrone
GD_IP="192.168.3.22"
GD_CONTAINER="guardrone-cnt"

ID_USER="bho" # Sostituire con l'utente reale del drone interaction
ID_IP="192.168.1.Y"
ID_CONTAINER="boh" # TODO: Aggiornare col nome del container del secondo drone


# Comando di chiusura sessione locale
KILL_ALIAS="alias aaa='tmux list-panes -s -F \"#{pane_id}\" | grep -v \$(tmux display-message -p \"#{pane_id}\") | xargs -I {} tmux send-keys -t {} C-c && echo \"Attendendo 5s...\" && sleep 5 && tmux kill-server'"

# =============================================================================
# 1. CREA SESSIONE TMUX
# =============================================================================
# Nota: Usa la conf tmux locale del GCS, ma senza /root/ siccome siamo sull'host
TMUX_CONF="${HOME}/guarDrone/my_ros2_ws/SimulationScripts/tmux.conf"
if [ -f "$TMUX_CONF" ]; then
    tmux -f "$TMUX_CONF" new-session -d -s $SESSION_NAME -n 'gcs'
else
    tmux new-session -d -s $SESSION_NAME -n 'gcs'
fi

tmux set-option -g mouse on
tmux set-option -t $SESSION_NAME pane-border-status top

# =============================================================================
# 2. CREAZIONE FINESTRE
# =============================================================================
tmux new-window -t $SESSION_NAME -n 'guardrone'
tmux new-window -t $SESSION_NAME -n 'drone_interaction'

# =============================================================================
# 3. LAYOUT PANE
# =============================================================================
# --- Layout GCS: 4 pane (2x2) ---
tmux split-window -h -t $SESSION_NAME:gcs.0          
tmux split-window -v -t $SESSION_NAME:gcs.0          
tmux split-window -v -t $SESSION_NAME:gcs.2          
tmux select-layout -t $SESSION_NAME:gcs tiled        

# --- Layout Droni: 3 pane (Agent, Launch, Shell) ---
for WIN in 'guardrone' 'drone_interaction'; do
    tmux split-window -h -t $SESSION_NAME:$WIN.0
    tmux split-window -v -t $SESSION_NAME:$WIN.0
done

# =============================================================================
# 4. FINESTRA 0 — GCS (LOCALE)
# =============================================================================
# I path locali presumono che il ws sia in ~/guarDrone/my_ros2_ws
WS_DIR="${HOME}/my_ros2_ws"
LOCAL_SOURCE="source /opt/ros/humble/setup.bash && [ -f ${WS_DIR}/install/setup.bash ] && source ${WS_DIR}/install/setup.bash"

# Pane 0: GCS Launch (supervisor + logger)
# NOTA: Per testare SOLO il drone MPC senza l'interaction drone reale, usa:
# ros2 launch gcs_pkg gcs_hw.launch.py use_fake_supervisor:=true
tmux select-pane -T '0: GCS Launch' -t $SESSION_NAME:gcs.0
tmux send-keys -t $SESSION_NAME:gcs.0 "cd ${WS_DIR} && ${LOCAL_SOURCE}" C-m
tmux send-keys -t $SESSION_NAME:gcs.0 "ros2 launch gcs_pkg gcs_hw.launch.py use_fake_supervisor:=${FAKE_PUBLISHER}" C-m

# Pane 1: Haptic
tmux select-pane -T '1: Haptic' -t $SESSION_NAME:gcs.1
tmux send-keys -t $SESSION_NAME:gcs.1 "cd ${WS_DIR} && ${LOCAL_SOURCE}" C-m
tmux send-keys -t $SESSION_NAME:gcs.1 "ros2 launch fd_haptic_joy haptic_sim.launch.py" C-m

# Pane 2: Keyboard Client
tmux select-pane -T '2: Keyboard Client' -t $SESSION_NAME:gcs.2
tmux send-keys -t $SESSION_NAME:gcs.2 "cd ${WS_DIR} && ${LOCAL_SOURCE} && clear" C-m
tmux send-keys -t $SESSION_NAME:gcs.2 "sleep 2 && ros2 run gcs_pkg keyboard_client.py" C-m

# Pane 3: Spare / Kill
tmux select-pane -T '3: Spare / Kill' -t $SESSION_NAME:gcs.3
tmux send-keys -t $SESSION_NAME:gcs.3 "cd ${WS_DIR} && ${LOCAL_SOURCE} && $KILL_ALIAS && clear" C-m
tmux send-keys -t $SESSION_NAME:gcs.3 "echo 'Usa il comando aaa per killare tutto in sicurezza.'" C-m

# =============================================================================
# 5. FINESTRA 1 — GUARDRONE (SSH persistente)
# =============================================================================
# Strategia: ogni pane apre una sessione SSH stabile sul drone.
# I send-keys con sleep girano in background (&) cosi' tmux si apre subito
# e i comandi arrivano ai pane mentre l'utente guarda gia' la sessione.

# --- Pane 0: Avvio container + MicroXRCE-DDS Agent ---
tmux select-pane -T '0: Container + MicroAgent' -t $SESSION_NAME:guardrone.0
(
    tmux send-keys -t $SESSION_NAME:guardrone.0 "ssh dummy@${GD_IP}" C-m
    sleep 3
    tmux send-keys -t $SESSION_NAME:guardrone.0 "bash ~/guarDrone/docker/run_guardrone_cnt.sh -d" C-m
    sleep 10
    tmux send-keys -t $SESSION_NAME:guardrone.0 "docker exec -it ${GD_CONTAINER} bash" C-m
    sleep 2
    # TODO: Controllare la porta seriale esatta (/dev/ttyUSB0 o /dev/ttyACM0 o /dev/ttyS0) e il baudrate
    tmux send-keys -t $SESSION_NAME:guardrone.0 "MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600" C-m
) &

# --- Pane 1: Launch nodi ROS2 del GuarDrone ---
tmux select-pane -T '1: GuarDrone Launch' -t $SESSION_NAME:guardrone.1
(
    tmux send-keys -t $SESSION_NAME:guardrone.1 "ssh dummy@${GD_IP}" C-m
    # Aspetta che il container sia avviato dal Pane 0:
    # Pane 0: sleep 3 (SSH) + sleep 10 (container start) + margine = ~15s totali
    sleep 15
    tmux send-keys -t $SESSION_NAME:guardrone.1 "docker exec -it ${GD_CONTAINER} bash" C-m
    sleep 2
    tmux send-keys -t $SESSION_NAME:guardrone.1 "colcon build && source /opt/ros/humble/setup.bash && source /root/my_ros2_ws/install/setup.bash" C-m
    sleep 1
    tmux send-keys -t $SESSION_NAME:guardrone.1 "ros2 launch guardrone_pkg guardrone_hw.launch.py" C-m
) &

# --- Pane 2: Shell interattiva nel container (per debug manuale) ---
tmux select-pane -T '2: Interactive Shell' -t $SESSION_NAME:guardrone.2
(
    tmux send-keys -t $SESSION_NAME:guardrone.2 "ssh dummy@${GD_IP}" C-m
    # Stesso wait del Pane 1
    sleep 15
    tmux send-keys -t $SESSION_NAME:guardrone.2 "docker exec -it ${GD_CONTAINER} bash" C-m
) &


# =============================================================================
# 6. FINESTRA 2 — DRONE INTERACTION (SSH)
# =============================================================================
# Pane 0: MicroXRCE-DDS Agent via SSH -> Docker
tmux select-pane -T '0: MicroAgent (SSH)' -t $SESSION_NAME:drone_interaction.0
tmux send-keys -t $SESSION_NAME:drone_interaction.0 "ssh -t ${ID_USER}@${ID_IP} 'docker exec -it ${ID_CONTAINER} MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'" C-m

# Pane 1: Launch Interaction via SSH -> Docker
tmux select-pane -T '1: Interaction Launch (SSH)' -t $SESSION_NAME:drone_interaction.1
tmux send-keys -t $SESSION_NAME:drone_interaction.1 "ssh -t ${ID_USER}@${ID_IP} 'docker exec -it ${ID_CONTAINER} bash'" C-m # TODO: creare interaction_drone_hw.launch.py e usare stessa architettura del guardrone

# Pane 2: Terminale interattivo nel container via SSH
tmux select-pane -T '2: Interactive Shell (SSH)' -t $SESSION_NAME:drone_interaction.2
tmux send-keys -t $SESSION_NAME:drone_interaction.2 "ssh -t ${ID_USER}@${ID_IP} 'docker exec -it ${ID_CONTAINER} bash'" C-m

# =============================================================================
# 7. ATTACH
# =============================================================================
tmux select-window -t $SESSION_NAME:gcs
tmux attach-session -t $SESSION_NAME

