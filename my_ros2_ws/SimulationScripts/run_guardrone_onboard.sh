#!/bin/bash
# =============================================================================
# run_guardrone_onboard.sh — Esecuzione HARDWARE REALE dal PC di bordo
#
# Questo script va eseguito DIRETTAMENTE dal terminale del LattePanda 
# (senza SSH, oppure via SSH locale ma girando direttamente sulla board).
#
# Prerequisiti:
# - Il container Docker 'guardrone-cnt' deve essere GIA' in esecuzione.
# =============================================================================

SESSION_NAME="gd_onboard"
GD_CONTAINER="guardrone-cnt"

# Comando di chiusura sessione locale
KILL_ALIAS="alias aaa='tmux list-panes -s -F \"#{pane_id}\" | grep -v \$(tmux display-message -p \"#{pane_id}\") | xargs -I {} tmux send-keys -t {} C-c && echo \"Attendendo 5s...\" && sleep 5 && tmux kill-server'"

# =============================================================================
# 1. CREA SESSIONE TMUX
# =============================================================================
tmux new-session -d -s $SESSION_NAME -n 'guardrone'
tmux set-option -g mouse on
tmux set-option -t $SESSION_NAME pane-border-status top

# =============================================================================
# 2. LAYOUT PANE
# =============================================================================
tmux split-window -h -t $SESSION_NAME:guardrone.0
tmux split-window -v -t $SESSION_NAME:guardrone.0
tmux split-window -v -t $SESSION_NAME:guardrone.2
tmux select-layout -t $SESSION_NAME:guardrone tiled

# =============================================================================
# 3. POPOLAMENTO PANE (Tutti via docker exec locali)
# =============================================================================
# Variabile helper per semplificare i comandi dentro docker
DOCKER_ROS_SETUP="source /opt/ros/humble/setup.bash && source /root/my_ros2_ws/install/setup.bash"

# Pane 0: MicroXRCE-DDS Agent
tmux select-pane -T '0: MicroAgent' -t $SESSION_NAME:guardrone.0
tmux send-keys -t $SESSION_NAME:guardrone.0 "docker exec -it ${GD_CONTAINER} MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600" C-m

# Pane 1: Launch GuaDrone HW
tmux select-pane -T '1: GuaDrone Launch' -t $SESSION_NAME:guardrone.1
tmux send-keys -t $SESSION_NAME:guardrone.1 "docker exec -it ${GD_CONTAINER} bash -c \"${DOCKER_ROS_SETUP} && ros2 launch guardrone_pkg guardrone_hw.launch.py\"" C-m

# Pane 2: Terminale interattivo nel container
tmux select-pane -T '2: Interactive Shell' -t $SESSION_NAME:guardrone.2
tmux send-keys -t $SESSION_NAME:guardrone.2 "docker exec -it ${GD_CONTAINER} bash -c \"${DOCKER_ROS_SETUP} && exec bash\"" C-m

# Pane 3: Spare / Kill Host
tmux select-pane -T '3: Spare / Kill' -t $SESSION_NAME:guardrone.3
tmux send-keys -t $SESSION_NAME:guardrone.3 "$KILL_ALIAS && clear" C-m
tmux send-keys -t $SESSION_NAME:guardrone.3 "echo 'Sei sul LattePanda (Host). Usa il comando aaa per killare tutto in sicurezza.'" C-m

# =============================================================================
# 4. ATTACH
# =============================================================================
tmux select-window -t $SESSION_NAME:guardrone
tmux attach-session -t $SESSION_NAME
