#!/bin/bash
# Run one part of the system in the simulator for a fixed time and record it. Logs are saved under OUTPUT_DIR.
# usage: run_part.sh OUTPUT_DIR WORKSPACE LAUNCH_FILE SECONDS [ENV_ASSIGNMENT_FOR_THE_NODE]
# The optional last argument is a VAR=value setting given only to the node, for example PYTHONNOUSERSITE=1.
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; WS=$2; LAUNCH=$3; SECS=$4; NODE_ENV=${5:-NODE_ENV_UNUSED=1}
source "$HERE/env.sh" "$WS"
rm -rf "$OUT"; mkdir -p "$OUT"
"$HERE/cleanup.sh" > /dev/null 2>&1
ros2 launch cave_explorer cave_explorer_startup.launch.py > "$OUT/startup.log" 2>&1 &
ok=0; for i in $(seq 1 25); do timeout 6 ros2 topic echo /scan --once >/dev/null 2>&1 && { ok=1; break; }; sleep 2; done
[ $ok = 1 ] || { echo "[$OUT] simulator did not start"; "$HERE/cleanup.sh" >/dev/null 2>&1; exit 1; }
ros2 launch cave_explorer cave_explorer_navigation.launch.py > "$OUT/nav.log" 2>&1 &
for i in $(seq 1 60); do grep -q "Managed nodes are active" "$OUT/nav.log" && timeout 6 ros2 topic echo /map --once --field info.width >/dev/null 2>&1 && break; sleep 3; done
echo "[$OUT] simulator and navigation up at $(date +%H:%M:%S)"
python3 "$HERE/../run_recorder.py" "$OUT" "$SECS" > "$OUT/recorder.log" 2>&1 &
env $NODE_ENV ros2 launch cave_explorer "$LAUNCH" > "$OUT/node.log" 2>&1 &
sleep $((SECS + 12))
echo "[$OUT] finished after ${SECS}s: $(tail -1 "$OUT/recorder.log")"
"$HERE/cleanup.sh" > /dev/null 2>&1
echo "[$OUT] shut down, $(pgrep -f 'rviz[2]|ign gazeb[o]|lib/cave_explorer/cave_explore[r]' | wc -l) processes left"
