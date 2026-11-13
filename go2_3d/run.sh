#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-mapping}"
profile="${2:-standard}"
if [[ "$mode" == navigation && "${GO2_DIAGNOSTICS_CAPTURE_ACTIVE:-}" != 1 ]]; then
  exec python3 "$project_dir/go2_3d/capture_navigation.py" --project "$project_dir" -- \
    bash "$project_dir/go2_3d/run.sh" "$@"
fi
case "$mode" in mapping|localization|navigation) ;; *) echo 'Usage: run.sh mapping [standard|dense] [MAP_NAME] | run.sh [localization|navigation] [standard|dense] [X Y YAW_DEGREES [Z]]' >&2; exit 2 ;; esac
case "$profile" in standard|dense) ;; *) echo 'Profile must be standard or dense' >&2; exit 2 ;; esac
loop_backend="${GO2_LOOP_BACKEND:-lidar}"
case "$loop_backend" in lidar|rtabmap) ;; *) echo 'GO2_LOOP_BACKEND must be lidar or rtabmap' >&2; exit 2 ;; esac
image=go2-3d:edu
backend_mounts=()
backend_environment=()
diagnostics_mounts=()
diagnostics_environment=()
diagnostics_labels=()
camera_calibration=''
session_directory=''
if [[ "$loop_backend" == rtabmap ]]; then
  [[ "$mode" == mapping ]] || { echo 'RTAB-Map backend is mapping-only. Unset GO2_LOOP_BACKEND for localization/navigation of the optimized saved map.' >&2; exit 2; }
  calibration_host="${GO2_CAMERA_CALIBRATION:-}"
  [[ -n "$calibration_host" && -f "$calibration_host" ]] || { echo 'Set GO2_CAMERA_CALIBRATION to measured intrinsic, extrinsic and timing YAML; no guessed calibration is supplied.' >&2; exit 2; }
  calibration_host="$(realpath -- "$calibration_host")"
  [[ "$calibration_host" != *:* ]] || { echo 'Calibration path must not contain a colon' >&2; exit 2; }
  image=go2-3d:rtabmap
  camera_calibration=/calibration/camera.yaml
  backend_mounts=(-v "$calibration_host:$camera_calibration:ro")
fi
auto_initialize=true
initial_from_map_start=true
map_name=default
initial_x=0.0; initial_y=0.0; initial_yaw=0.0; initial_z=0.0
if (( $# > 2 )); then
  if [[ "$mode" == mapping ]]; then
    [[ $# == 3 ]] || { echo 'Usage: run.sh mapping [standard|dense] MAP_NAME' >&2; exit 2; }
    map_name="$3"
    python3 - "$map_name" <<'PY'
import sys
# Validate without importing ROS or numerical libraries on the host.
import re
assert re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',sys.argv[1]), 'Invalid map name'
PY
  else
    [[ $# == 5 || $# == 6 ]] || { echo 'Usage: run.sh navigation dense X Y YAW_DEGREES [Z] (actual map pose)' >&2; exit 2; }
    initial_x="$3"; initial_y="$4"; initial_yaw="$5"; initial_z="${6:-0.0}"
    python3 -c 'import sys,math; assert all(math.isfinite(float(s)) for s in sys.argv[1:]), "Pose must be finite"' "$initial_x" "$initial_y" "$initial_yaw" "$initial_z"
    initial_from_map_start=false
  fi
fi
gicp_params=/opt/go2_project/go2_3d/gicp.yaml
[[ "$profile" == dense ]] && gicp_params=/opt/go2_project/go2_3d/gicp_dense.yaml
dds_file="${GO2_DDS_FILE:-$HOME/cyclonedds_ws/cyclonedds.xml}"
test -f "$dds_file" || { echo "Missing $dds_file" >&2; exit 1; }
if [[ "$mode" != mapping ]]; then
  for name in map.pcd nav.yaml metadata.yaml; do
    test -f "$project_dir/maps/latest/$name" || { echo "Missing maps/latest/$name; save the 3D map first." >&2; exit 1; }
  done
fi
mkdir -p "$project_dir/maps" "$project_dir/runtime"
if [[ "$mode" == navigation && -n "${GO2_DIAGNOSTICS_SESSION:-}" ]]; then
  session_host="${GO2_DIAGNOSTICS_SESSION}"
  # The host wrapper owns the session; only this subdirectory is writable by
  # the recorder. Application source/configuration stays read-only.
  session_id="$(basename -- "$session_host")"
  diagnostics_mounts=(-v "$session_host:/diagnostics:rw")
  diagnostics_environment=(-e GO2_DIAGNOSTICS_SESSION=/diagnostics
    -e "GO2_DIAGNOSTICS_UID=$(id -u)" -e "GO2_DIAGNOSTICS_GID=$(id -g)")
  diagnostics_labels=(--label "go2.diagnostics.session=$session_id")
  echo "Navigation diagnostics session: $session_host"
fi
if [[ "$mode" == navigation && -n "${GO2_DIAGNOSTICS_SESSION:-}" ]]; then
  navigation_image_id="$(sudo docker image inspect --format '{{.Id}}' "$image")"
  python3 - "$project_dir" "$session_host" "$image" "$navigation_image_id" <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path(sys.argv[1])/'go2_3d'))
try:
    from fault_store import EventStore
    EventStore(sys.argv[2]).write_manifest({'image': {'name': sys.argv[3], 'id': sys.argv[4] or None}})
except Exception as error:
    print(f'Navigation diagnostic image metadata unavailable: {error}', file=sys.stderr)
PY
else
  sudo docker image inspect "$image" >/dev/null
fi
if [[ "$mode" == navigation ]]; then
  sudo docker run --rm --network none --entrypoint /bin/bash \
    -v "$project_dir/maps:/maps:ro" -v "$project_dir:/opt/go2_project:ro" go2-3d:edu -c \
    'source /opt/ros/humble/setup.bash && if [[ -f /opt/go2_navigation/setup.bash ]]; then source /opt/go2_navigation/setup.bash; fi && python3 /opt/go2_project/go2_3d/check_nav2_plugins.py'
fi
# Validate dependencies, SDK assets and generated copies before stopping sensors.
sudo docker run --rm --network none --entrypoint /bin/bash \
  -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  -v "$project_dir:/opt/go2_project:ro" -v "$project_dir/runtime:/runtime:rw" \
  -v "$project_dir/maps:/maps:ro" \
  "$image" -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_project/go2_3d/prepare.py --mode "$1"' bash "$mode"
if [[ "$loop_backend" == rtabmap ]]; then
  # Validate real calibration, generated TF names and the installed node API
  # while the current live container is still running.
  sudo docker run --rm --network none --entrypoint /bin/bash \
    -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
    -v "$project_dir:/opt/go2_project:ro" -v "$project_dir/runtime:/runtime:ro" \
    "${backend_mounts[@]}" "$image" -c \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_project/go2_3d/check_rtabmap.py --calibration "$1" --urdf /runtime/config/go2_edu.urdf' bash "$camera_calibration"
  mkdir -p "$project_dir/maps/rtabmap_sessions"
  session_host="$(mktemp -d "$project_dir/maps/rtabmap_sessions/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"
  session_directory="/maps/rtabmap_sessions/$(basename -- "$session_host")"
  backend_environment=(-e "ROS_LOG_DIR=$session_directory/ros_logs")
  echo "RTAB-Map session (new database, journal and logs): $session_host"
fi
# Our own run uses --rm; a stopped stale container can be removed safely only if
# it was created by this launcher and has the matching label.
if sudo docker container inspect go2-3d >/dev/null 2>&1; then
  label="$(sudo docker inspect --format '{{index .Config.Labels "go2.project"}}' go2-3d)"
  [[ "$label" == shan504/go2 ]] || { echo 'Existing go2-3d container is not ours; refusing removal.' >&2; exit 1; }
  sudo docker stop --time 10 go2-3d >/dev/null
  if sudo docker container inspect go2-3d >/dev/null 2>&1; then
    sudo docker rm go2-3d >/dev/null
  fi
fi
# Keep old SDK containers intact; stop their processes to avoid duplicate DDS/TF.
if sudo docker container inspect go2-sdk >/dev/null 2>&1; then
  sudo docker stop --time 10 go2-sdk >/dev/null
fi
echo "Starting $mode ($profile); motion disabled. View registered_cloud, map_cloud and camera/image/compressed."
if [[ "$mode" != mapping && "$initial_from_map_start" == true ]]; then
  echo 'Automatic localization uses the selected map saved start pose. Place the robot at its mapping start with the same heading; no /initialpose clicks required.'
  echo 'Wait for accepted GICP localization before enabling motion. Motion stays disabled.'
fi
sudo docker run --rm -it --init --name go2-3d --label go2.project=shan504/go2 --network host \
  "${diagnostics_labels[@]}" \
  -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
  "${backend_environment[@]}" \
  "${diagnostics_environment[@]}" \
  -v "$dds_file:/etc/cyclonedds.xml:ro" \
  -v "$project_dir:/opt/go2_project:ro" \
  -v "$project_dir/runtime:/runtime:ro" -v "$project_dir/maps:/maps:rw" \
  "${backend_mounts[@]}" "${diagnostics_mounts[@]}" --entrypoint /bin/bash "$image" -c \
  'set -e
   source /opt/ros/humble/setup.bash
   source /ros2_ws/install/setup.bash
   if [[ "$1" == navigation ]]; then source /opt/go2_navigation/setup.bash; fi
   optional_args=()
   [[ -z "${11}" ]] || optional_args+=("camera_calibration:=${11}")
   [[ -z "${12}" ]] || optional_args+=("session_directory:=${12}")
   exec ros2 launch /opt/go2_project/go2_3d/stack.launch.py mode:="$1" gicp_params:="$2" auto_initialize:="$3" initial_x:="$4" initial_y:="$5" initial_yaw_degrees:="$6" initial_z:="$7" map_name:="$8" initial_from_map_start:="$9" loop_backend:="${10}" "${optional_args[@]}"' bash "$mode" "$gicp_params" "$auto_initialize" "$initial_x" "$initial_y" "$initial_yaw" "$initial_z" "$map_name" "$initial_from_map_start" "$loop_backend" "$camera_calibration" "$session_directory"
