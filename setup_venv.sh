#!/bin/bash
###############################################################################
# setup_venv.sh – Create a Python venv with all SAFE_ROS2 pip dependencies
#
# Usage:
#   chmod +x setup_venv.sh
#   ./setup_venv.sh
#   source .venv/bin/activate
#
# Note: This creates the venv in .venv/ at the project root.
#       ROS 2 system packages (rclpy, etc.) are inherited via --system-site-packages.
###############################################################################
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
WHEEL="${SCRIPT_DIR}/HOST_setup/linux/pyspline-1.5.2-py3-none-linux_x86_64.whl"

echo "=== Creating venv at ${VENV_DIR} ==="
python3 -m venv --system-site-packages "${VENV_DIR}"

echo "=== Activating venv ==="
source "${VENV_DIR}/bin/activate"

echo "=== Upgrading pip ==="
pip install --upgrade pip setuptools wheel

echo "=== Installing pip dependencies ==="
pip install -r "${SCRIPT_DIR}/requirements.txt"

echo "=== Installing pyspline from wheel ==="
if [ -f "${WHEEL}" ]; then
    pip install "${WHEEL}"
else
    echo "WARNING: pyspline wheel not found at ${WHEEL}"
    echo "         You may need to install it manually."
fi

echo "=== Patching transforms3d (np.float -> float) ==="
TRANSFORMS3D_DIR=$(python3 -c "import transforms3d, os; print(os.path.dirname(transforms3d.__file__))" 2>/dev/null) || true
if [ -n "${TRANSFORMS3D_DIR}" ] && [ -d "${TRANSFORMS3D_DIR}" ]; then
    find "${TRANSFORMS3D_DIR}" -name '*.py' -exec \
        sed -i 's/\bnp\.float\b/float/g' {} +
    echo "         Patched transforms3d at ${TRANSFORMS3D_DIR}"
else
    echo "WARNING: transforms3d not found, skipping patch."
fi

echo ""
echo "=== Done! Activate with: source ${VENV_DIR}/bin/activate ==="
