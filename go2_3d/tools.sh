#!/usr/bin/env bash
set -euo pipefail
case "${1:-check}" in
  archive-map|use-map|maps)
    project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    case "$1" in
      archive-map) [[ $# == 2 || ( $# == 3 && "$3" == --all ) ]] || { echo 'Usage: tools.sh archive-map NAME [--all]' >&2; exit 2; }; action=archive ;;
      use-map)
        [[ $# == 2 || $# == 3 ]] || { echo 'Usage: tools.sh use-map NAME [VERSION]' >&2; exit 2; }
        if sudo docker container inspect go2-3d >/dev/null 2>&1; then
          echo 'Stop the navigation/mapping launcher with Ctrl+C before switching its loaded map.' >&2
          exit 1
        fi
        action=select ;;
      maps) [[ $# == 1 ]] || { echo 'Usage: tools.sh maps' >&2; exit 2; }; action=list ;;
    esac
    mkdir -p "$project_dir/maps"
    sudo docker run --rm --network none --entrypoint /bin/bash \
      -v "$project_dir:/opt/go2_project:ro" -v "$project_dir/maps:/maps:rw" \
      go2-3d:edu -c 'python3 /opt/go2_project/go2_3d/map_library.py "$@"' bash "$action" "${@:2}"
    ;;
  resume|cancel)
    [[ $# == 1 ]] || { echo 'Usage: tools.sh resume | cancel' >&2; exit 2; }
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 10 ros2 topic pub --once "/navigation/$1" std_msgs/msg/Empty "{}"' bash "$1"
    ;;
  restart-navigation)
    [[ $# -le 2 ]] || { echo 'Usage: tools.sh restart-navigation [standard|dense]' >&2; exit 2; }
    profile="${2:-dense}"
    case "$profile" in standard|dense) ;; *) echo 'Profile must be standard or dense' >&2; exit 2 ;; esac
    project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    if [[ "$(sudo docker inspect --format '{{.State.Running}}' go2-3d 2>/dev/null || true)" != true ]]; then
      echo 'Keep the current navigation container running to retain its valid map pose.' >&2
      exit 1
    fi
    bash "$project_dir/go2_3d/tools.sh" disable
    mkdir -p "$project_dir/runtime"
    pose_file="$(mktemp "$project_dir/runtime/navigation-pose.XXXXXX.json")"
    trap 'rm -f -- "$pose_file"' EXIT
    sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 10 python3 /opt/go2_project/go2_3d/snapshot_pose.py' > "$pose_file"
    # Only finite numeric fields cross from ROS into shell arguments; never
    # source a generated script or evaluate topic text as shell code.
    pose_values="$(python3 - "$pose_file" <<'PY'
import json, math, sys, time
pose = json.load(open(sys.argv[1]))
assert 0 <= time.time()-pose['captured_at'] < 30, 'Pose capture is stale'
values = [float(pose[key]) for key in ('x','y','yaw_degrees','z')]
assert all(math.isfinite(value) for value in values), 'Nonfinite pose'
print(' '.join(format(value, '.9g') for value in values))
PY
    )"
    read -r -a pose_args <<< "$pose_values"
    echo "Restarting navigation with retained map pose (x y yaw_degrees z): $pose_values; motion remains disabled. Keep the robot stationary."
    bash "$project_dir/go2_3d/run.sh" navigation "$profile" "${pose_args[@]}"
    ;;
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
  robotcheck|drivecheck|drivecheck-native|drivecheck-balanced|drivecheck-walk)
    [[ $# -le 2 ]] || { echo 'Usage: tools.sh robotcheck [SECONDS: 8..60] | drivecheck | drivecheck-native | drivecheck-balanced | drivecheck-walk' >&2; exit 2; }
    if [[ "$1" != robotcheck && $# != 1 ]]; then
      echo 'Drive diagnostics have a fixed 2s duration and accept no arguments; drivecheck-walk is 0.30m/s, others are 0.15m/s.' >&2
      exit 2
    fi
    project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
    if [[ "$(sudo docker inspect --format '{{.State.Running}}' go2-3d 2>/dev/null || true)" != true ]]; then
      echo 'go2-3d is not running. Keep the navigation launch terminal open, then run this command in a second SSH terminal.' >&2
      exit 1
    fi
    mkdir -p "$project_dir/runtime"
    if [[ "$1" == robotcheck ]]; then
      sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 70 python3 -u /opt/go2_project/go2_3d/robotcheck.py "$1"' bash "${2:-20}" | tee "$project_dir/runtime/robot-debug.txt"
    elif [[ "$1" == drivecheck ]]; then
      sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 20 python3 -u /opt/go2_project/go2_3d/drivecheck.py' | tee "$project_dir/runtime/drive-debug.txt"
    elif [[ "$1" == drivecheck-native ]]; then
      sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 20 python3 -u /opt/go2_project/go2_3d/native_drivecheck.py' | tee "$project_dir/runtime/native-drive-debug.txt"
    elif [[ "$1" == drivecheck-balanced ]]; then
      sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 20 python3 -u /opt/go2_project/go2_3d/balanced_drivecheck.py' | tee "$project_dir/runtime/balanced-drive-debug.txt"
    else
      sudo docker exec go2-3d bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && timeout 20 python3 -u /opt/go2_project/go2_3d/walking_drivecheck.py' | tee "$project_dir/runtime/walking-drive-debug.txt"
    fi
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
  *) echo 'Usage: tools.sh [archive-map NAME [--all]|use-map NAME [VERSION]|maps|resume|cancel|restart-navigation [standard|dense]|export-map|repair-map|level-map|clean-map|inflation-5cm|check|diagnose|navcheck GOAL_X GOAL_Y|motioncheck [SECONDS]|robotcheck [SECONDS]|drivecheck|drivecheck-native|drivecheck-balanced|drivecheck-walk|save|enable|disable|shell]' >&2; exit 2 ;;
esac
