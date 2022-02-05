#!/bin/bash
set -e

# setup ros2 environment
if [ -e /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi
if [ -e $ROS_OVERLAY/install/setup.bash ]; then
    source "$ROS_OVERLAY/install/setup.bash"
fi
exec "$@"
