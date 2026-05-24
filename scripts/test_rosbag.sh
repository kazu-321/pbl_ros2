#!/bin/bash
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <bag_path>"
    exit 1
fi

BAG_PATH=$1

cleanup() {
    if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

ros2 launch pbl_launch rosbag_test.launch.xml &
LAUNCH_PID=$!

sleep 1

ros2 bag play "$BAG_PATH" \
    --clock \
    --topics \
        /joint_states \
        /unilidar/cloud \
        /unilidar/imu \
    -r 1.5
