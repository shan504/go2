#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-mapping}"
profile="${2:-standard}"
case "$mode" in mapping|localization|navigation) ;; *) echo 'Usage: run.sh [mapping|localization|navigation] [standard|dense]' >&2; exit 2 ;; esac
case "$profile" in standard|dense) ;; *) echo 'Profile must be standard or dense' >&2; exit 2 ;; esac
gicp_params=/opt/go2_project/go2_3d/gicp.yaml
[[ "$profile" == dense ]] && gicp_params=/opt/go2_project/go2_3d/gicp_dense.yaml
dds_file="$HOME/cyclonedds_ws/cyclonedds.xml"
test -f "$dds_file" || { echo "Missing $dds_file" >&2; exit 1; }
if [[ "$mode" != mapping ]]; then
  for name in map.pcd nav.yaml metadata.yaml; do
    test -f "$project_dir/maps/latest/$name" || { echo "Missing maps/latest/$name; save the 3D map first." >&2; exit 1; }
  done
fi
mkdir -p "$project_dir/maps" "$project_dir/runtime"
sudo docker image inspect go2-3d:edu >/dev/null
# Validate dependencies, SDK assets and generated copies before stopping sensors.
sudo docker run --rm --network none --entrypoint /bin/bash \
  -v "$project_dir:/opt/go2_project:ro" -v "$project_dir/runtime:/runtime:rw" \
  go2-3d:edu -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_project/go2_3d/prepare.py'
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
sudo docker run --rm -it --init --name go2-3d --label go2.project=shan504/go2 --network host \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
  -v "$dds_file:/etc/cyclonedds.xml:ro" \
  -v "$project_dir:/opt/go2_project:ro" \
  -v "$project_dir/runtime:/runtime:ro" -v "$project_dir/maps:/maps:rw" \
  --entrypoint /bin/bash go2-3d:edu -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec ros2 launch /opt/go2_project/go2_3d/stack.launch.py mode:="$1" gicp_params:="$2"' bash "$mode" "$gicp_params"
