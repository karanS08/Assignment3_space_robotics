#!/bin/bash
# Stop every process that a simulator run starts, so one run cannot disturb the next.
# Patterns use [x] so that they never match this script's own command line.
PATTERNS=("ros2 launc[h]" "adaptive_samplin[g]" "cave_geometr[y]" "rviz[2]" "ign gazeb[o]" "g[z] sim" "ruby.*ig[n]"
          "parameter_bridg[e]" "slam_toolbo[x]" "nav2_[a-z_]*/" "ekf_nod[e]" "robot_state_publishe[r]" "creat[e] --ros-args"
          "lib/cave_explorer/cave_explore[r]")
for pat in "${PATTERNS[@]}"; do pkill -INT -f "$pat" 2>/dev/null; done
sleep 5
for pat in "${PATTERNS[@]}"; do pkill -9 -f "$pat" 2>/dev/null; done
sleep 2
echo "left: $(pgrep -f 'adaptive_samplin[g]|cave_geometr[y]|rviz[2]|ign gazeb[o]|parameter_bridg[e]|slam_toolbo[x]|ekf_nod[e]' | wc -l)"
