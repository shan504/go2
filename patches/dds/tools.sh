#!/usr/bin/env bash
set -euo pipefail
case "${1:-check}" in
  check)
    sudo docker exec go2-sdk /bin/bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_edu/verify.py'
    ;;
  save)
    sudo docker exec go2-sdk /bin/bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /opt/go2_edu/maps/go2_map --ros-args -p save_map_timeout:=15.0'
    ;;
  enable|disable)
    value=false
    [[ "$1" == enable ]] && value=true
    sudo docker exec go2-sdk /bin/bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 param set /go2_edu_dds_bridge enable_control "$1"' bash "$value"
    ;;
  *) echo 'Usage: bash tools.sh [check|save|enable|disable]' >&2; exit 2 ;;
esac
