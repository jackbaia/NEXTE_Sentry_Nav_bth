#!/usr/bin/env bash

# Strict mode for safety
set -euo pipefail
IFS=$'\n\t'

# -----------------------------------------------------------------------------
# Workspace-aware build script for ROS1 (Noetic) catkin workspace
# - Builds livox_ros_driver2 FIRST using its custom script
# - Then builds the rest of the workspace with catkin_make
# - Ensures proper ROS environment and provides informative logging
# -----------------------------------------------------------------------------

# Timestamped logger
log() {
  echo "[$(date '+%F %T')] [build_all] $*"
}

# Resolve workspace root (this script's directory)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="${WS_DIR}/src"
LIVOX_DIR="${SRC_DIR}/livox_ros_driver2"

# Default build parallelism and type (can be overridden by env)
: "${CATKIN_BUILD_TYPE:=Release}"
: "${CATKIN_JOBS:=$(nproc)}"

log "Workspace: ${WS_DIR}"

# 1) Source ROS1 environment if available (Noetic typical path)
if [ -f "/opt/ros/noetic/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/noetic/setup.bash"
  log "Sourced ROS Noetic environment from /opt/ros/noetic/setup.bash"
else
  log "WARNING: /opt/ros/noetic/setup.bash not found. Ensure ROS1 is sourced in your shell."
fi

# 1.1) Prepend local tool wrappers to PATH (e.g., catkin_make wrapper)
export PATH="${WS_DIR}/.tools:${PATH}"
log "Prepended PATH with ${WS_DIR}/.tools"

# Verify catkin_make existence after sourcing
if ! command -v catkin_make >/dev/null 2>&1; then
  log "ERROR: catkin_make not found in PATH. Source your ROS1 setup.bash first."
  exit 1
fi

# 2) Build livox_ros_driver2 FIRST via its custom script
if [ ! -d "${LIVOX_DIR}" ]; then
  log "ERROR: Directory not found: ${LIVOX_DIR}"
  exit 1
fi
if [ ! -f "${LIVOX_DIR}/build.sh" ]; then
  log "ERROR: Missing build script: ${LIVOX_DIR}/build.sh"
  exit 1
fi

# Ensure the script is executable
chmod +x "${LIVOX_DIR}/build.sh" || true

log "Building livox_ros_driver2 first using its custom script (ROS1 mode)..."
pushd "${LIVOX_DIR}" >/dev/null
# NOTE: livox build.sh will cleanup ../../build|devel|install and run catkin_make itself
./build.sh ROS1
popd >/dev/null
log "livox_ros_driver2 custom build completed."

# 3) Build (or re-build) the entire workspace with catkin_make
#    Use the same ROS_EDITION define to be consistent with livox script.
log "Building entire workspace with catkin_make (type=${CATKIN_BUILD_TYPE}, jobs=${CATKIN_JOBS})..."
pushd "${WS_DIR}" >/dev/null
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE="${CATKIN_BUILD_TYPE}" -j"${CATKIN_JOBS}"
log "catkin_make build finished."

# Optionally source the devel setup for immediate usage
if [ -f "${WS_DIR}/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/devel/setup.bash"
  log "Sourced workspace devel/setup.bash"
fi
popd >/dev/null

log "All builds completed successfully."
