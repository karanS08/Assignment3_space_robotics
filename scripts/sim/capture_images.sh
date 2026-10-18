#!/bin/bash
# Start the simulator, run an image-capture script against it, and shut everything down afterwards.
# usage: capture_images.sh WORKSPACE CAPTURE_SCRIPT OUTPUT_DIR [SEED] [EXTRA ARGUMENTS FOR THE SCRIPT]
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$1; SCRIPT=$2; OUT=$3; SEED=${4:-0}
source "$HERE/env.sh" "$WS"
"$HERE/cleanup.sh" > /dev/null 2>&1
mkdir -p ~/space_robotics_sim/runs
ros2 launch cave_explorer cave_explorer_startup.launch.py > ~/space_robotics_sim/runs/capture_startup.log 2>&1 &
ok=0; for i in $(seq 1 30); do timeout 6 ros2 topic echo /camera/image --once --field width >/dev/null 2>&1 && { ok=1; break; }; sleep 2; done
[ $ok = 1 ] || { echo "simulator did not start"; "$HERE/cleanup.sh" > /dev/null 2>&1; exit 1; }
echo "simulator up at $(date +%H:%M:%S)"
cd "$(dirname "$SCRIPT")" && python3 "$SCRIPT" "$OUT" "$SEED" "${@:5}" 2>&1 | grep --line-buffered -E "walls|viewpoints|^[0-9]+/|done|rror"
"$HERE/cleanup.sh" > /dev/null 2>&1
echo "shut down at $(date +%H:%M:%S)"
