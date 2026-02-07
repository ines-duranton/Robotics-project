#!/bin/bash
###############################################################################
# start.sh – Auto-detect GPU and launch the SAFE_ROS2 Docker container
#
# Usage:
#   ./start.sh          # build + start + open shell
#   ./start.sh build    # rebuild the image
#   ./start.sh down     # stop the container
###############################################################################
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ---------- Detect GPU ----------
COMPOSE_FILES="-f docker-compose.yml"

if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    echo "[GPU] NVIDIA GPU detected"
    COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.nvidia.yml"
elif [ -d /dev/dri ]; then
    echo "[GPU] AMD/Intel GPU detected (/dev/dri)"
else
    echo "[GPU] No GPU detected, using software rendering"
fi

COMPOSE_CMD="docker compose $COMPOSE_FILES"

# ---------- Handle commands ----------
case "${1:-run}" in
    build)
        $COMPOSE_CMD build
        ;;
    down|stop)
        $COMPOSE_CMD down
        ;;
    shell)
        $COMPOSE_CMD exec ros bash
        ;;
    *)
        # Allow X11 access
        xhost +local:docker 2>/dev/null || true

        # Start container if not already running
        if ! docker ps --format '{{.Names}}' | grep -q safe_ros2_foxy; then
            $COMPOSE_CMD up -d
        fi

        # Open a shell
        $COMPOSE_CMD exec ros bash
        ;;
esac
