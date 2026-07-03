#!/bin/bash
# =============================================================================
# run_multidrone_sim.sh — Simulazione Multi-Drone (GuaDrone + Interaction)
#
# Layout tmux a 3 finestre:
#   Finestra 0 — gcs              : MicroAgent | GCS Launch   | Spare/Kill
#   Finestra 1 — guardrone        : PX4 Drone1 | GuaDrone Launch | Spare
#   Finestra 2 — drone_interaction: PX4 Drone2 | Interaction Launch | Spare
#
# Uso:
#   ./run_multidrone_sim.sh [--headless]
# =============================================================================

SESSION_NAME="drone_sim"

# --- Modalità headless (Gazebo senza GUI) ---
HEADLESS_PREFIX=""
if [[ "$1" == "--headless" ]]; then
    echo "Avvio in modalità HEADLESS (Gazebo senza GUI)"
    HEADLESS_PREFIX="HEADLESS=1 "
fi

# =============================================================================
# CONFIGURAZIONE POSE E MODELLI
# =============================================================================

# Drone 1 — GuaDrone (MPC + Camera): avvia Gazebo
DRONE1_X=${DRONE1_X:--4.0}
DRONE1_Y=${DRONE1_Y:--53.0}
DRONE1_Z=${DRONE1_Z:-4.52}
DRONE1_YAW=${DRONE1_YAW:-0.0}
DRONE1_MODEL_NAME=${DRONE1_MODEL_NAME:-"x500_depth"}

# Drone 2 — Interaction Drone (ammettenza): standalone, si aggancia a Gazebo
DRONE2_X=${DRONE2_X:--1.0}
DRONE2_Y=${DRONE2_Y:--55.0}
DRONE2_Z=${DRONE2_Z:-4.52}
DRONE2_YAW=${DRONE2_YAW:-0.0}
DRONE2_MODEL_NAME=${DRONE2_MODEL_NAME:-"x500_interaction"}

WORLD_NAME=${WORLD_NAME:-"bridge_inspection_gazebo"}

# Comando di source ROS2 + workspace (usato in ogni pane)
SOURCE_CMD="source /opt/ros/humble/setup.bash && [ -f /root/my_ros2_ws/install/setup.bash ] && source /root/my_ros2_ws/install/setup.bash"

# Comando di chiusura sessione (alias 'aaa'): invia C-c a tutti i pane tranne quello corrente,
# aspetta 5 secondi per la chiusura pulita di Gazebo e ROS, poi termina il server tmux.
KILL_ALIAS="alias aaa='tmux list-panes -s -F \"#{pane_id}\" | grep -v \$(tmux display-message -p \"#{pane_id}\") | xargs -I {} tmux send-keys -t {} C-c && echo \"Attendendo 5s per chiusura pulita...\" && sleep 5 && tmux kill-server'"

# =============================================================================
# 1. CREA SESSIONE TMUX
# =============================================================================
tmux -f /root/my_ros2_ws/SimulationScripts/tmux.conf new-session -d -s $SESSION_NAME -n 'gcs'
tmux set-option -g mouse on
tmux set-option -t $SESSION_NAME pane-border-status top

# =============================================================================
# 2. CREAZIONE FINESTRE
# =============================================================================
# La finestra 0 'gcs' è già stata creata con new-session.
tmux new-window -t $SESSION_NAME -n 'guardrone'
tmux new-window -t $SESSION_NAME -n 'drone_interaction'

# =============================================================================
# 3. LAYOUT PANE
#
# Finestra GCS (4 pane):
#  ┌──────────────────────┬──────────────────────┐
#  │  Pane 0: MicroAgent  │  Pane 1: GCS Launch  │
#  ├──────────────────────┼──────────────────────┤
#  │  Pane 2: Haptic      │                      │
#  ├──────────────────────┴──────────────────────┤
#  │  Pane 3: Spare / Kill terminal              │
#  └─────────────────────────────────────────────┘
#
# Finestre Drone (3 pane):
#  ┌──────────────────────┬──────────────────────┐
#  │  Pane 0: PX4 / Agent │  Pane 1: Launch ROS2 │
#  ├──────────────────────┴──────────────────────┤
#  │  Pane 2: Spare                              │
#  └─────────────────────────────────────────────┘
# =============================================================================

# --- Layout GCS: 2x2 + 1 full-width in basso ---
tmux split-window -h -t $SESSION_NAME:gcs.0          # Pane 0 (sx) | Pane 1 (dx)
tmux split-window -v -t $SESSION_NAME:gcs.0          # Pane 0 (sx-top) | Pane 2 (sx-bot)
tmux select-layout -t $SESSION_NAME:gcs tiled        # Equilibra i 4 riquadri in griglia 2x2
tmux select-pane -t $SESSION_NAME:gcs.0
tmux split-window -v -f -t $SESSION_NAME:gcs         # Pane 3: full-width in basso
tmux resize-pane -t $SESSION_NAME:gcs.3 -y 10

# --- Layout finestre drone: 2 affiancati + 1 full-width in basso ---
for WIN in 'guardrone' 'drone_interaction'; do
    tmux split-window -h -t $SESSION_NAME:$WIN.0       # Pane 0 (sx) | Pane 1 (dx)
    tmux select-pane -t $SESSION_NAME:$WIN.0
    tmux split-window -v -f -t $SESSION_NAME:$WIN      # Pane 2: full-width in basso
    tmux resize-pane -t $SESSION_NAME:$WIN.2 -y 10
done

# =============================================================================
# 4. FINESTRA 0 — GCS
# =============================================================================

# --- Pane 0: MicroXRCE-DDS Agent ---
# Un singolo agente UDP gestisce entrambi i droni (multi-client).
tmux select-pane -T '0: MicroAgent' -t $SESSION_NAME:gcs.0
tmux send-keys -t $SESSION_NAME:gcs.0 "MicroXRCEAgent udp4 -p 8888" C-m

# --- Pane 1: GCS Launch (supervisor + logger) ---
tmux select-pane -T '1: GCS Launch' -t $SESSION_NAME:gcs.1
tmux send-keys -t $SESSION_NAME:gcs.1 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:gcs.1 "$SOURCE_CMD" C-m
tmux send-keys -t $SESSION_NAME:gcs.1 "sleep 15 && ros2 launch gcs_pkg gcs_sim.launch.py \
    drone_x:=${DRONE1_X} drone_y:=${DRONE1_Y} drone_z:=${DRONE1_Z} \
    peg_x:=${DRONE2_X}   peg_y:=${DRONE2_Y}   peg_z:=${DRONE2_Z}" C-m

# --- Pane 2: Haptic (Falcon Force Dimension — opzionale) ---
# Se il dispositivo non è collegato il launch fallisce silenziosamente;
# il resto della simulazione continua normalmente.
tmux select-pane -T '2: Haptic' -t $SESSION_NAME:gcs.2
tmux send-keys -t $SESSION_NAME:gcs.2 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:gcs.2 "$SOURCE_CMD" C-m
tmux send-keys -t $SESSION_NAME:gcs.2 "sleep 5 && ros2 launch fd_haptic_joy haptic_sim.launch.py" C-m

# --- Pane 3: Spare / Kill ---
tmux select-pane -T '3: Spare' -t $SESSION_NAME:gcs.3
tmux send-keys -t $SESSION_NAME:gcs.3 "cd /root/my_ros2_ws && $SOURCE_CMD && $KILL_ALIAS && clear" C-m

# =============================================================================
# 5. FINESTRA 1 — GUARDRONE (Drone 1 — MPC + Camera)
# =============================================================================

# --- Pane 0: PX4 SITL Drone 1 — avvia anche il mondo Gazebo ---
tmux select-pane -T '0: PX4 GuaDrone' -t $SESSION_NAME:guardrone.0
tmux send-keys -t $SESSION_NAME:guardrone.0 "cd /root/PX4-Autopilot" C-m
tmux send-keys -t $SESSION_NAME:guardrone.0 "$SOURCE_CMD" C-m
tmux send-keys -t $SESSION_NAME:guardrone.0 \
    "sleep 2 && PX4_GZ_WORLD=${WORLD_NAME} \
    PX4_GZ_MODEL_POSE='${DRONE1_X},${DRONE1_Y},${DRONE1_Z},0,0,${DRONE1_YAW}' \
    ${HEADLESS_PREFIX}make px4_sitl gz_${DRONE1_MODEL_NAME}" C-m

# --- Pane 1: GuaDrone Launch (MPC planner + bridge + rviz) ---
tmux select-pane -T '1: GuaDrone Launch' -t $SESSION_NAME:guardrone.1
tmux send-keys -t $SESSION_NAME:guardrone.1 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:guardrone.1 "$SOURCE_CMD" C-m
tmux send-keys -t $SESSION_NAME:guardrone.1 "sleep 15 && ros2 launch guardrone_pkg guardrone_sim.launch.py \
    model:=${DRONE1_MODEL_NAME} \
    drone_x:=${DRONE1_X} drone_y:=${DRONE1_Y} drone_z:=${DRONE1_Z} drone_yaw:=${DRONE1_YAW} \
    peg_x:=${DRONE2_X}   peg_y:=${DRONE2_Y}   peg_z:=${DRONE2_Z}" C-m

# --- Pane 2: Spare ---
tmux select-pane -T '2: Spare' -t $SESSION_NAME:guardrone.2
tmux send-keys -t $SESSION_NAME:guardrone.2 "cd /root/my_ros2_ws && $SOURCE_CMD && clear" C-m

# =============================================================================
# 6. FINESTRA 2 — DRONE INTERACTION (Drone 2 — Ammettenza)
# =============================================================================

# --- Pane 0: PX4 SITL Drone 2 — STANDALONE (si aggancia a Gazebo esistente) ---
# PX4_GZ_STANDALONE=1: non lancia un nuovo Gazebo.
# -i 1: istanza PX4 separata (evita conflitti con Drone 1).
# -w /tmp/px4_sitl_1: working directory separata (evita lock file).
tmux select-pane -T '0: PX4 Interaction' -t $SESSION_NAME:drone_interaction.0
tmux send-keys -t $SESSION_NAME:drone_interaction.0 "cd /root/PX4-Autopilot" C-m
tmux send-keys -t $SESSION_NAME:drone_interaction.0 "$SOURCE_CMD" C-m
tmux send-keys -t $SESSION_NAME:drone_interaction.0 \
    "echo 'Aspetto 10s per Gazebo e Drone1...' && sleep 10 && \
    PX4_SIM_MODEL=gz_${DRONE2_MODEL_NAME} \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD=${WORLD_NAME} \
    PX4_GZ_MODEL_POSE='${DRONE2_X},${DRONE2_Y},${DRONE2_Z},0,0,${DRONE2_YAW}' \
    build/px4_sitl_default/bin/px4 -i 1" C-m

# --- Pane 1: Interaction Launch (admittance planner) ---
tmux select-pane -T '1: Interaction Launch' -t $SESSION_NAME:drone_interaction.1
tmux send-keys -t $SESSION_NAME:drone_interaction.1 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:drone_interaction.1 "$SOURCE_CMD" C-m
tmux send-keys -t $SESSION_NAME:drone_interaction.1 "sleep 15 && ros2 launch interaction_drone_pkg interaction_drone_sim.launch.py \
    peg_x:=${DRONE2_X} peg_y:=${DRONE2_Y} peg_z:=${DRONE2_Z}" C-m

# --- Pane 2: Spare ---
tmux select-pane -T '2: Spare' -t $SESSION_NAME:drone_interaction.2
tmux send-keys -t $SESSION_NAME:drone_interaction.2 "cd /root/my_ros2_ws && $SOURCE_CMD && clear" C-m

# =============================================================================
# 7. ATTACH: apre sulla finestra gcs
# =============================================================================
tmux select-window -t $SESSION_NAME:gcs
tmux attach-session -t $SESSION_NAME
