#!/bin/bash
set -e

cd /home/ros2_user/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build the ROS packages mounted into this workspace.
colcon build
source install/setup.bash

if [[ "$#" -eq 0 ]]; then
    set -- bash
fi
exec "$@"
