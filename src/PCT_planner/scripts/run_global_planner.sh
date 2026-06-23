#!/usr/bin/env bash
# =============================================================================
# PCT Global Planner — shell launcher
#
# Sets up library paths, sources ROS2, and runs the online planning node.
#
# Usage:
#   # Direct shell launch (dev):
#   ./run_global_planner.sh --ros-args --params-file params/pct_global_planner.yaml
#
#   # Recommended: ros2 launch (after colcon build):
#   ros2 launch pct_planner pct_global_planner.launch.py
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${SCRIPT_DIR}"
LIB_ROOT="${ROOT}/planner/lib"

# ── ROS2 environment ────────────────────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: /opt/ros/humble/setup.bash not found. Is ROS2 Humble installed?"
    exit 1
fi

# Also source workspace if built
if [ -f "${ROOT}/../../install/setup.bash" ]; then
    source "${ROOT}/../../install/setup.bash"
fi

# ── Library path for GTSAM + smoothing ──────────────────────────────────────
export LD_LIBRARY_PATH="${LIB_ROOT}/3rdparty/gtsam-4.1.1/install/lib:${LIB_ROOT}/build/src/common/smoothing:${LD_LIBRARY_PATH:-}"

# ── Run the global planner ──────────────────────────────────────────────────
exec python3 "${ROOT}/scripts/run_ros2_global_planner" "$@"
