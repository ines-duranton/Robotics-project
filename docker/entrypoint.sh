#!/bin/bash
set -e

# Suppress Mesa/libGL warnings (Foxy's drivers are too old for modern GPUs,
# but software rendering works fine for rviz2/rqt)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Source ROS 2 Foxy
source /opt/ros/foxy/setup.bash

# If the workspace has been built, source the overlay
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

exec "$@"
