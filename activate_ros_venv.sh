#!/usr/bin/env bash
# Helper to combine ROS 2 Jazzy + workspace install + project Python venv cleanly.
# Usage: source /home/quique/Mobile-Robots-2027-1/activate_ros_venv.sh

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "${_SCRIPT_DIR}/ros2_ws/install/setup.bash" ]; then
    source "${_SCRIPT_DIR}/ros2_ws/install/setup.bash"
fi

if [ -f "${_SCRIPT_DIR}/venv/bin/activate" ]; then
    source "${_SCRIPT_DIR}/venv/bin/activate"
    export PYTHONPATH="${VIRTUAL_ENV}/lib/python3.12/site-packages:${PYTHONPATH}"
fi
