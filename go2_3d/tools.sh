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
  repair-map|level-map|clean-map)
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
  motioncheck)
    [[ $# -le 2 ]] || { echo 'Usage: tools.sh motioncheck [SECONDS: 15..120; default 30]' >&2; exit 2; }
    project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    if [[ "$(sudo docker inspect --format '{{.State.Running}}' go2-3d 2>/dev/null || true)" != true ]]; then
      echo 'go2-3d is not running. Keep the navigation launch terminal open, then run motioncheck in a second SSH terminal.' >&2
      exit 1
    fi
    mkdir -p "$project_dir/runtime"
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 150 python3 -u /opt/go2_project/go2_3d/motioncheck.py "$1"' bash "${2:-30}" | tee "$project_dir/runtime/velocity-debug.txt"
    ;;
  inflation-5cm)
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && timeout 30 python3 /opt/go2_project/go2_3d/set_inflation.py'
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
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 10 ros2 topic pub --once /control/enable std_msgs/msg/Bool "{data: $1}"' bash "$value"
    ;;
  shell)
    sudo docker exec -it go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash --norc'
    ;;
  *) echo 'Usage: tools.sh [export-map|repair-map|level-map|clean-map|inflation-5cm|check|diagnose|navcheck GOAL_X GOAL_Y|motioncheck [SECONDS]|save|enable|disable|shell]' >&2; exit 2 ;;
esac
