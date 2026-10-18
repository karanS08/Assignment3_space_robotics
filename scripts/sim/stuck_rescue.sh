#!/bin/bash
# Rescue rule for code that cannot get itself unstuck (used for the original exploration code only).
# If the robot has moved less than 0.5 m for 240 s, first cancel its goal; if it is still not moving 25 s later, reverse for 5 s and
# turn right for 3 s. Every rescue is written to RUN_DIR/interventions.txt so the report can say how much manual help was needed.
# usage: stuck_rescue.sh RUN_DIR WORKSPACE MAX_SECONDS
HERE=$(cd "$(dirname "$0")" && pwd)
RUN=$1; WS=$2; MAX=$3
source "$HERE/env.sh" "$WS"
pose() { timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -m1 Translation | sed 's/- Translation: \[\([-0-9.]*\), \([-0-9.]*\).*/\1 \2/'; }
moved() { python3 -c "import math,sys; a=[float(v) for v in sys.argv[1:5]]; print(math.hypot(a[0]-a[2], a[1]-a[3]))" $1 $2; }
STILL=0; START=$(date +%s); REF=$(pose)
while [ $(( $(date +%s) - START )) -lt "$MAX" ]; do
  sleep 10
  NOW=$(pose); [ -z "$NOW" ] && continue
  if [ -n "$REF" ] && [ "$(python3 -c "import math,sys; a=[float(v) for v in sys.argv[1:5]]; print(1 if math.hypot(a[0]-a[2],a[1]-a[3])<0.5 else 0)" $REF $NOW)" = 1 ]; then
    STILL=$((STILL + 10))
  else
    STILL=0; REF=$NOW
  fi
  if [ $STILL -ge 240 ]; then
    echo "$(date +%H:%M:%S) rescue 1: robot at ($NOW) has not moved for 240 s; cancelled its goal" >> "$RUN/interventions.txt"
    ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" > /dev/null 2>&1
    sleep 25
    AFTER=$(pose)
    if [ "$(python3 -c "import math,sys; a=[float(v) for v in sys.argv[1:5]]; print(1 if math.hypot(a[0]-a[2],a[1]-a[3])<0.5 else 0)" $NOW $AFTER)" = 1 ]; then
      echo "$(date +%H:%M:%S) rescue 2: still not moving 25 s later; reversed 5 s and turned right 3 s" >> "$RUN/interventions.txt"
      timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.8}, angular: {z: 0.0}}" > /dev/null 2>&1
      timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -1.0}}" > /dev/null 2>&1
      ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
    fi
    STILL=0; REF=$(pose)
  fi
done
