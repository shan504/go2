#!/usr/bin/env bash
set -euo pipefail
case "${1:-check}" in
  export-map)
    project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    mkdir -p "$project_dir/runtime"
    if sudo docker container inspect go2-3d >/dev/null 2>&1; then
      sudo docker logs --tail 500 go2-3d > "$project_dir/runtime/navigation-debug.log" 2>&1
    fi
    sudo python3 "$project_dir/go2_3d/export_map.py" --project "$project_dir"
    ;;
  repair-map|level-map)
    project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    if sudo docker container inspect go2-3d >/dev/null 2>&1; then
      echo 'Stop the navigation launcher with Ctrl+C before changing its loaded map.' >&2
      exit 1
    fi
    sudo docker run --rm --network none --entrypoint /bin/bash \
      -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
      -v "$project_dir:/opt/go2_project:ro" -v "$project_dir/maps:/maps:rw" \
      go2-3d:edu -c 'python3 "/opt/go2_project/go2_3d/$1.py"' bash "${1%-map}_map"
    ;;
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
  *) echo 'Usage: tools.sh [export-map|repair-map|level-map|check|diagnose|navcheck GOAL_X GOAL_Y|save|enable|disable|shell]' >&2; exit 2 ;;
esac
