#!/bin/bash
# =============================================================================
# delete_sphere.sh — Rimuove una o tutte le sfere spawnate da Gazebo Sim
#
# Uso:
#   ./delete_sphere.sh [nome_modello | 'all']
# =============================================================================

TARGET=${1:-"all"}

if [ "$TARGET" = "all" ]; then
    MODELS=$(gz model --list 2>/dev/null | grep -E "floating_sphere|sphere_" | tr -d ' -')
    if [ -z "$MODELS" ]; then
        echo "Nessuna sfera volante trovata da eliminare."
        exit 0
    fi
    for M in $MODELS; do
        gz service -s /world/bridge_inspection_gazebo/remove \
          --reqtype gz.msgs.Entity \
          --reptype gz.msgs.Boolean \
          --timeout 2000 \
          --req "name: \"${M}\", type: 2" >/dev/null 2>&1
        echo "Rimossa: ${M}"
    done
else
    gz service -s /world/bridge_inspection_gazebo/remove \
      --reqtype gz.msgs.Entity \
      --reptype gz.msgs.Boolean \
      --timeout 2000 \
      --req "name: \"${TARGET}\", type: 2" >/dev/null 2>&1
    echo "Rimossa: ${TARGET}"
fi
