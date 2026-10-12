#!/bin/bash
# Source this to get a ROS environment for one of the simulator workspaces:  source scripts/sim/env.sh NAME
# NAME is a workspace built by build_ws.sh, for example "robust" or "baseline". It lives in ~/space_robotics_sim/ws_NAME.
SIM=${SIM_HOME:-$HOME/space_robotics_sim}
source /opt/ros/humble/setup.bash
[ -n "$1" ] && [ -f "$SIM/ws_$1/install/setup.bash" ] && source "$SIM/ws_$1/install/setup.bash"
export AMENT_PREFIX_PATH=$SIM/overlay/opt/ros/humble:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$SIM/overlay/opt/ros/humble/lib:$SIM/overlay/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=$SIM/overlay/opt/ros/humble/local/lib/python3.10/dist-packages:$PYTHONPATH
export ROS_DOMAIN_ID=42
