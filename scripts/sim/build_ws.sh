#!/bin/bash
# Build the cave_explorer package from a Git checkout into its own workspace.
# usage: build_ws.sh NAME PACKAGE_DIR      e.g. build_ws.sh robust ~/Development/Space_robotics/Assignment3-int2/cave_explorer
SIM=${SIM_HOME:-$HOME/space_robotics_sim}
NAME=$1; PKG=$2
[ -d "$PKG" ] || { echo "no such package directory: $PKG"; exit 1; }
mkdir -p "$SIM/ws_$NAME/src"
ln -sfn "$PKG" "$SIM/ws_$NAME/src/cave_explorer"
cd "$SIM/ws_$NAME" && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select cave_explorer 2>&1 | tail -2
