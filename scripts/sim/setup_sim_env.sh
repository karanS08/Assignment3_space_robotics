#!/bin/bash
# One-time setup of the simulator tools, without sudo.
# Downloads the three ROS packages that are missing on this laptop and unpacks them into ~/space_robotics_sim/overlay.
# (On a machine where they are installed with apt, this is not needed.)
SIM=${SIM_HOME:-$HOME/space_robotics_sim}
mkdir -p "$SIM/debs" "$SIM/overlay"
cd "$SIM/debs" || exit 1
apt-get download ros-humble-robot-localization ros-humble-geographic-msgs libgeographic19 \
                 ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge || exit 1
for deb in *.deb; do dpkg -x "$deb" "$SIM/overlay"; done
echo "overlay ready in $SIM/overlay"
