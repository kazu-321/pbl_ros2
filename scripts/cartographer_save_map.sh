#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <bag_path>"
    exit 1
fi

BAG_PATH_REL=$1
BAG_PATH=$(realpath "$BAG_PATH_REL")

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

if [ -f /opt/ros/jazzy/setup.bash ]; then
    # Make sure ros2 and xacro are available even from a plain shell.
    # shellcheck disable=SC1091
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

PBSTREAM_FILE="${PBSTREAM_FILE:-$REPO_ROOT/pbl_launch/map/map.pbstream}"
CONFIG_DIR="${CONFIG_DIR:-$REPO_ROOT/pbl_launch/config}"
CONFIG_FILE="${CONFIG_FILE:-cartographer_map_save.lua}"
URDF_XACRO="${URDF_XACRO:-$REPO_ROOT/pbl_launch/urdf/pbl.urdf.xacro}"

TMP_URDF_FILE=$(mktemp "${TMPDIR:-/tmp}/pbl_urdf.XXXXXX.urdf")
cleanup() {
    rm -f "$TMP_URDF_FILE"
}
trap cleanup EXIT

# Save the current Cartographer state first.
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: $PBSTREAM_FILE}"

# cartographer_assets_writer expects a URDF, so expand the xacro first.
xacro "$URDF_XACRO" > "$TMP_URDF_FILE"

# Write the exported point cloud into rosbag/pbl/.
cd "$REPO_ROOT/pbl_launch/map"
ros2 run cartographer_ros cartographer_assets_writer \
    -configuration_directory "$CONFIG_DIR" \
    -configuration_basename "$CONFIG_FILE" \
    -urdf_filename "$TMP_URDF_FILE" \
    -bag_filenames "$BAG_PATH" \
    -pose_graph_filename "$PBSTREAM_FILE"
