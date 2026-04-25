#!/bin/bash

# Script per il build del Docker con aggiornamento automatico del repository mpc_uav

GITHUB_TOKEN=""
DOCKERFILE="px4_humble_harmonic_dockerfile.txt"
IMAGE_TAG="px4-img"
REPO_URL="https://github.com/Simone-DAngelo/mpc_uav.git"
BRANCH="2025_fix"

echo "🔍 Controllo dell'ultimo commit del repository mpc_uav..."

# Ottieni l'SHA dell'ultimo commit dal repository remoto
REMOTE_SHA=$(git ls-remote ${REPO_URL} ${BRANCH} 2>/dev/null | cut -f1)

if [ -z "$REMOTE_SHA" ]; then
    echo "❌ Impossibile ottenere l'SHA del repository remoto. Uso timestamp come cache buster."
    CACHE_BUST=$(date +%s)
else
    echo "📋 Ultimo commit remoto: $REMOTE_SHA"
    # Usa l'SHA come cache buster
    CACHE_BUST="$REMOTE_SHA"
fi

echo "🐳 Esecuzione del docker build..."

# Esegui il build con i build arguments necessari
docker build \
    --build-arg GITHUB_TOKEN="$GITHUB_TOKEN" \
    --build-arg MPC_UAV_CACHE_BUST="$CACHE_BUST" \
    -t "$IMAGE_TAG" \
    -f "$DOCKERFILE" \
    .

if [ $? -eq 0 ]; then
    echo "✅ Build completato con successo!"
    echo "🏷️  Immagine creata: $IMAGE_TAG"
else
    echo "❌ Build fallito!"
    exit 1
fi
