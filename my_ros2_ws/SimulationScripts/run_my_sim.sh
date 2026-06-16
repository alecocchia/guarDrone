#!/bin/bash

# Nome della sessione
SESSION_NAME="drone_sim"

# Gestione argomenti
HEADLESS_PREFIX=""
if [[ "$1" == "--headless" ]]; then
    echo "Avvio in modalità HEADLESS (Gazebo senza GUI)"
    HEADLESS_PREFIX="HEADLESS=1 "
fi

# --- Configurazione Posa Iniziale Drone MPC ---
DRONE1_X=${DRONE1_X:--4.0}
DRONE1_Y=${DRONE1_Y:--52.0}
DRONE1_Z=${DRONE1_Z:-4.52}
DRONE1_YAW=${DRONE1_YAW:-0.0}

# --- Configurazione Posa Iniziale Drone di Interazione ---
DRONE2_X=${DRONE2_X:--1.0}
DRONE2_Y=${DRONE2_Y:--54.0}
DRONE2_Z=${DRONE2_Z:-4.52}
DRONE2_YAW=${DRONE2_YAW:-0.0}

WORLD_NAME=${WORLD_NAME:-"bridge_inspection_gazebo"}
DRONE1_MODEL_NAME=${DRONE1_MODEL_NAME:-"x500_depth"}       # Drone MPC con camera
DRONE2_MODEL_NAME=${DRONE2_MODEL_NAME:-"x500_interaction"}  # Drone di interazione

# 1. Crea la sessione in background e nomina la finestra 'dashboard'
tmux -f /root/my_ros2_ws/SimulationScripts/tmux.conf new-session -d -s $SESSION_NAME -n 'dashboard'

# 2. CONFIGURAZIONE MODERNA
tmux set-option -g mouse on
tmux set-option -t $SESSION_NAME pane-border-status top

# 3. CREAZIONE LAYOUT: 2x2 in alto + 1 orizzontale piena larghezza in basso
#
#  ┌─────────────────────┬─────────────────────┐
#  │  0: MicroAgent      │  1: PX4 Drone1      │
#  ├─────────────────────┼─────────────────────┤
#  │  2: PX4 Drone2      │  3: ROS2 Launch     │
#  ├─────────────────────┴─────────────────────┤
#  │  4: Spare Terminal (full width)           │
#  └───────────────────────────────────────────┘
#
# Strategia tmux: creare prima la finestra principale, poi dividere in 2x2,
# poi creare un riquadro a piena larghezza in basso con split-window -v sulla finestra.
tmux split-window -h -t $SESSION_NAME:0.0      # Pane 0 (sx) | Pane 1 (dx)
tmux split-window -v -t $SESSION_NAME:0.0      # Pane 0 (sx-top) | Pane 2 (sx-bot)
tmux split-window -v -t $SESSION_NAME:0.1      # Pane 1 (dx-top) | Pane 3 (dx-bot)
# Ora seleziona tutto il gruppo sinistra+destra con un resize per bilanciare
tmux select-layout -t $SESSION_NAME:0 tiled    # Equilibra i 4 riquadri in griglia 2x2
# Ora aggiunge il 5° riquadro in fondo, a tutta larghezza
tmux select-pane -t $SESSION_NAME:0.0          # Torna al pane 0
tmux split-window -v -f -t $SESSION_NAME:0     # -f = full-width split (da tutta la finestra)
tmux resize-pane -t $SESSION_NAME:0.4 -y 10    # Altezza del riquadro in fondo: 10 righe
# Il pane 4 è ora in basso a piena larghezza

# 4. AVVIO PROCESSI E TITOLI

# --- Riquadro 0: Micro-XRCE-DDS Agent ---
# Un solo agente è sufficiente per entrambi i droni (è multi-client).
tmux select-pane -T '0: MicroAgent' -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m

# --- Riquadro 1: PX4 SITL Drone 1 (MPC) + avvio Gazebo ---
# Non usa PX4_GZ_STANDALONE: è lui che avvia il mondo Gazebo.
tmux select-pane -T '1: PX4 Drone1 (MPC+Gazebo)' -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME:0.1 "cd /root/PX4-Autopilot" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash && [ -f /root/my_ros2_ws/install/setup.bash ] && source /root/my_ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "sleep 2 &&PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL_POSE='${DRONE1_X},${DRONE1_Y},${DRONE1_Z},0,0,${DRONE1_YAW}' ${HEADLESS_PREFIX}make px4_sitl gz_${DRONE1_MODEL_NAME}" C-m

# --- Riquadro 2: PX4 SITL Drone 2 (Interazione) - STANDALONE ---
# Usiamo il binario px4 direttamente con -i 1 per evitare conflitti con l'istanza 0.
# PX4_GZ_STANDALONE=1: non lancia un nuovo Gazebo, si aggancia a quello esistente.
# UXRCE_DDS_NS=px4_1: namespace DDS separato per non sovrascrivere i topic del Drone 1.
# -w /tmp/px4_sitl_1: directory di lavoro separata (evita lock file in conflitto).
tmux select-pane -T '2: PX4 Drone2 (Interaction)' -t $SESSION_NAME:0.2
tmux send-keys -t $SESSION_NAME:0.2 "cd /root/PX4-Autopilot" C-m
tmux send-keys -t $SESSION_NAME:0.2 "source /opt/ros/humble/setup.bash && [ -f /root/my_ros2_ws/install/setup.bash ] && source /root/my_ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Aspetto 10s per Gazebo e Drone1...' && sleep 10 && PX4_SIM_MODEL=gz_${DRONE2_MODEL_NAME} PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL_POSE='${DRONE2_X},${DRONE2_Y},${DRONE2_Z},0,0,${DRONE2_YAW}' build/px4_sitl_default/bin/px4 -i 1 " C-m

# --- Riquadro 3: ROS 2 Launch (Bridge + MPC Planner + peg_planner + RViz) ---
tmux select-pane -T '3: ROS2 Nodes' -t $SESSION_NAME:0.3
tmux send-keys -t $SESSION_NAME:0.3 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:0.3 "colcon build && source /opt/ros/humble/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "sleep 10 && ros2 launch drone_mpc_pkg mpc_sim.launch.py model:=${DRONE1_MODEL_NAME} drone_x:=${DRONE1_X} drone_y:=${DRONE1_Y} drone_z:=${DRONE1_Z} drone_yaw:=${DRONE1_YAW} peg_x:=${DRONE2_X} peg_y:=${DRONE2_Y} peg_z:=${DRONE2_Z}" C-m

# --- Riquadro 4: Terminale Libero (full width, in fondo) ---
tmux select-pane -T '4: Spare Terminal' -t $SESSION_NAME:0.4
tmux send-keys -t $SESSION_NAME:0.4 "cd /root/my_ros2_ws && colcon build && source /opt/ros/humble/setup.bash && source install/setup.bash && alias aaa='tmux list-panes -s -F \"#{pane_id}\" | grep -v \$(tmux display-message -p \"#{pane_id}\") | xargs -I {} tmux send-keys -t {} C-c && echo \"Attendendo 5 secondi per la chiusura pulita di Gazebo e ROS...\" && sleep 5 && tmux kill-server' && clear" C-m

# 5. ATTACCO ALLA SESSIONE
tmux attach-session -t $SESSION_NAME\
