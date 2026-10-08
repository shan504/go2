#!/usr/bin/env bash
set -euo pipefail
case "${1:-check}" in
  check)
    sudo docker exec go2-sdk /bin/bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_sensors/check.py'
    ;;
  record)
    sudo docker exec -it go2-sdk /bin/bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 bag record -o "/opt/go2_edu/maps/3d_sensors_$(date +%Y%m%d_%H%M%S)" /point_cloud2 /odom /tf /tf_static'
    ;;
  *) echo 'Usage: bash tools.sh [check|record]' >&2; exit 2 ;;
esac
