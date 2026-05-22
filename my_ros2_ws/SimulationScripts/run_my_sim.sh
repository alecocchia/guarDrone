#!/bin/bash

# Nome della sessione
SESSION_NAME="drone_sim"

# Gestione argomenti
HEADLESS_PREFIX=""
if [[ "$1" == "--headless" ]]; then
    echo "Avvio in modalità HEADLESS (Gazebo senza GUI)"
    HEADLESS_PREFIX="HEADLESS=1 "
fi

# Configurazione Posa Iniziale Drone (Default: Origine)
DRONE_X=${DRONE_X:-0.0}
DRONE_Y=${DRONE_Y:-0.0}
DRONE_Z=${DRONE_Z:-0.0}
DRONE_YAW=${DRONE_YAW:-0.0}
WORLD_NAME=${WORLD_NAME:-"default"} # Default a "default"
MODEL_NAME=${MODEL_NAME:-"x500_depth"} # Default a "x500_depth"

# 1. Crea la sessione in background e nomina la finestra 'dashboard'
tmux -f /root/my_ros2_ws/SimulationScripts/tmux.conf new-session -d -s $SESSION_NAME -n 'dashboard'

# 2. CONFIGURAZIONE MODERNA (Nessun errore di 'mouse-select-pane')
# Abilita il mouse per selezionare i riquadri e ridimensionarli
tmux set-option -g mouse on
# Attiva la barra dei titoli in cima a ogni riquadro
tmux set-option -t $SESSION_NAME pane-border-status top

# 3. CREAZIONE DELLA GRIGLIA 2x2
# Dividiamo la finestra in 4 parti
tmux split-window -h -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.2

# 4. AVVIO PROCESSI E TITOLI

# --- Riquadro 0: Micro-XRCE-DDS Agent ---
tmux select-pane -T '0: MicroAgent' -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.0 "MicroXRCEAgent udp4 -p 8888" C-m

# --- Riquadro 1: PX4 SITL / Gazebo ---
tmux select-pane -T '1: PX4 SITL' -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME:0.1 "cd /root/PX4-Autopilot" C-m
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash && [ -f /root/my_ros2_ws/install/setup.bash ] && source /root/my_ros2_ws/install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "PX4_GZ_WORLD=${WORLD_NAME} PX4_GZ_MODEL_POSE='${DRONE_X},${DRONE_Y},${DRONE_Z},0,0,${DRONE_YAW}' ${HEADLESS_PREFIX}make px4_sitl gz_${MODEL_NAME}" C-m

# --- Riquadro 2: ROS 2 Launch ---
tmux select-pane -T '2: ROS 2 MPC' -t $SESSION_NAME:0.2
tmux send-keys -t $SESSION_NAME:0.2 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:0.2 "colcon build" C-m
tmux send-keys -t $SESSION_NAME:0.2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 launch drone_mpc_pkg mpc_sim.launch.py model:=${MODEL_NAME} world:=${WORLD_NAME}.sdf drone_x:=${DRONE_X} drone_y:=${DRONE_Y} drone_z:=${DRONE_Z} drone_yaw:=${DRONE_YAW}" C-m

# --- Riquadro 3: Terminale Vuoto (Pronto per ROS 2) ---
tmux select-pane -T '3: Manual Control' -t $SESSION_NAME:0.3
tmux send-keys -t $SESSION_NAME:0.3 "cd /root/my_ros2_ws" C-m
tmux send-keys -t $SESSION_NAME:0.3 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear" C-m

# 5. BILANCIAMENTO E ATTACCO
# Bilancia il layout per 3 riquadri
tmux select-layout -t $SESSION_NAME:0 tiled
# Entra nella sessione
tmux attach-session -t $SESSION_NAME
