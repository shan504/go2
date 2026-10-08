#!/usr/bin/env bash
set -euo pipefail
case "${1:-check}" in
  navcheck)
    [[ $# == 3 ]] || { echo 'Usage: tools.sh navcheck GOAL_X GOAL_Y (map coordinates)' >&2; exit 2; }
    sudo docker exec -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 30 python3 /opt/go2_project/go2_3d/navcheck.py "$1" "$2"' bash "$2" "$3"
    ;;
  diagnose)
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 30 python3 /opt/go2_project/go2_3d/diagnose.py'
    ;;
  check)
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list -t && timeout 10 ros2 topic echo /localization/status --once'
    ;;
  save)
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 120 ros2 service call /save_3d_map std_srvs/srv/Trigger "{}"'
    ;;
  enable|disable)
    value=false
    [[ "$1" == enable ]] && value=true
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 param set /go2_edu_dds_bridge enable_control "$1"' bash "$value"
    ;;
  shell)
    sudo docker exec -it go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash --norc'
    ;;
  *) echo 'Usage: tools.sh [check|diagnose|navcheck GOAL_X GOAL_Y|save|enable|disable|shell]' >&2; exit 2 ;;
esac
