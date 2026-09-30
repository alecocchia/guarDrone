#!/bin/bash
# =============================================================================
# spawn_sphere.sh — Spawna una sfera statica volante a mezz'aria in Gazebo Sim
#
# Uso posizionale (NON digitare le parentesi quadre):
#   create_sphere X Y Z [raggio] [colore]
#
# Esempi:
#   create_sphere                               -> (-2.0, -50.0, 5.0), raggio 0.25, red
#   create_sphere -1.5 -51 9 0.6 red           -> sfera a (-1.5, -51, 9), raggio 0.6m, rossa
#   create_sphere -2 -50 6                      -> quota 6m
#   create_sphere -1 -49 4.5 0.3 yellow        -> giallo, r=0.3m
#   create_sphere -z 7 -c blue                  -> quota 7m, blu
# =============================================================================

# Valori di default (default occlusione inspection_start)
X=1.5
Y=-54.8
Z=9.5
RADIUS=0.6
COLOR="red"

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Uso dinamico davanti a GuarDrone: create_sphere front [distanza_m] [raggio] [colore]"
    echo "  Esempio: create_sphere front                  -> a 1.2m davanti al GuarDrone, raggio 0.5m, rosso"
    echo "  Esempio: create_sphere front 1.5 0.6 yellow   -> a 1.5m, raggio 0.6m, giallo"
    echo ""
    echo "Uso posizionale fisso:          create_sphere X Y Z [raggio] [colore]"
    echo "Uso con flag:                    create_sphere [-x X] [-y Y] [-z Z] [-r raggio] [-c colore]"
    echo "Colori disponibili: red, yellow, green, blue, white, orange"
    exit 0
fi

# Modalità dinamica 'front': calcola la posizione in tempo reale davanti a GuarDrone
if [ "$1" = "front" ] || [ "$1" = "--front" ] || [ "$1" = "-f" ]; then
    DIST="${2:-1.2}"
    RADIUS="${3:-0.5}"
    COLOR="${4:-red}"

    # Carica ambiente ROS2 se necessario
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash >/dev/null 2>&1
    [ -f /root/my_ros2_ws/install/setup.bash ] && source /root/my_ros2_ws/install/setup.bash >/dev/null 2>&1

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    echo "⏳ Calcolo posizione dinamica davanti a GuarDrone (distanza=${DIST}m)..."
    COORDS=$(python3 "${SCRIPT_DIR}/get_front_coord.py" "$DIST" 2>&1)
    RET=$?
    if [ $RET -ne 0 ]; then
        echo "❌ $COORDS"
        exit 1
    fi
    X=$(echo "$COORDS" | awk '{print $1}')
    Y=$(echo "$COORDS" | awk '{print $2}')
    Z=$(echo "$COORDS" | awk '{print $3}')
    echo "🎯 Coordinate calcolate: X=${X}, Y=${Y}, Z=${Z}"

# Parsing argomenti: distingue flag (-x, -r, etc.) da coordinate numeriche (es. -2.0)
elif [[ "$1" =~ ^-[a-zA-Z] ]]; then
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -x|--x) X="$2"; shift 2 ;;
            -y|--y) Y="$2"; shift 2 ;;
            -z|--z) Z="$2"; shift 2 ;;
            -r|--radius) RADIUS="$2"; shift 2 ;;
            -c|--color) COLOR="$2"; shift 2 ;;
            *) shift ;;
        esac
    done
else
    [ -n "$1" ] && X="$1"
    [ -n "$2" ] && Y="$2"
    [ -n "$3" ] && Z="$3"
    [ -n "$4" ] && RADIUS="$4"
    [ -n "$5" ] && COLOR="$5"
fi

# Rimuovi eventuali parentesi quadre o virgole digitate accidentalmente (es. [-1.5 o red])
X=$(echo "$X" | tr -d '[],')
Y=$(echo "$Y" | tr -d '[],')
Z=$(echo "$Z" | tr -d '[],')
RADIUS=$(echo "$RADIUS" | tr -d '[],')
COLOR=$(echo "$COLOR" | tr -d '[],')

NAME="floating_sphere_$(date +%s%N | cut -b14-18)"

case "$COLOR" in
    yellow) RGBA="1.0 0.8 0.0 1.0" ;;
    red)    RGBA="0.95 0.15 0.15 1.0" ;;
    blue)   RGBA="0.15 0.5 1.0 1.0" ;;
    white)  RGBA="0.95 0.95 0.95 1.0" ;;
    orange) RGBA="1.0 0.5 0.0 1.0" ;;
    *)      RGBA="0.0 0.85 0.15 1.0" ;; # default: green

esac

SDF="<?xml version=\"1.0\" ?><sdf version=\"1.8\"><model name=\"${NAME}\"><static>true</static><pose>${X} ${Y} ${Z} 0 0 0</pose><link name=\"link\"><visual name=\"v\"><geometry><sphere><radius>${RADIUS}</radius></sphere></geometry><material><ambient>${RGBA}</ambient><diffuse>${RGBA}</diffuse><specular>0.4 0.4 0.4 1.0</specular></material></visual></link></model></sdf>"

# Esegui il service call Gazebo
RESPONSE=$(gz service -s /world/bridge_inspection_gazebo/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req "sdf: \"$(echo "$SDF" | sed 's/"/\\"/g')\"" 2>&1)

sleep 0.2
if gz model --list 2>/dev/null | grep -q "$NAME"; then
    echo "Sfera volante creata: '${NAME}' in (${X}, ${Y}, ${Z}) | raggio=${RADIUS}m | colore=${COLOR}"
    echo "  -> Spostabile nella GUI di Gazebo cliccandola e premendo il tasto 'T' (Translate)."
elif echo "$RESPONSE" | grep -q "data: true"; then
    echo "Sfera volante creata: '${NAME}' in (${X}, ${Y}, ${Z}) | raggio=${RADIUS}m | colore=${COLOR}"
    echo "  -> Spostabile nella GUI di Gazebo cliccandola e premendo il tasto 'T' (Translate)."
else
    echo "Errore nello spawn della sfera: $RESPONSE"
fi
