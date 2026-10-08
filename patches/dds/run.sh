#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-mapping}"
case "$mode" in
  mapping|localization) ;;
  *) echo 'Usage: bash run.sh [mapping|localization]' >&2; exit 2 ;;
esac
dds_file="$HOME/cyclonedds_ws/cyclonedds.xml"
test -f "$dds_file" || { echo "Missing $dds_file" >&2; exit 1; }
if [[ "$mode" == localization && ! -f "$task_dir/maps/go2_map.yaml" ]]; then
  echo 'Save maps/go2_map.yaml before starting localization.' >&2
  exit 1
fi
mkdir -p "$task_dir/maps"
sudo -v
sudo docker image inspect go2-ros2-sdk:edu >/dev/null
# Prepare offline before stopping the previous SDK session.
sudo docker run --rm --network none --entrypoint /bin/bash \
  -v "$task_dir:/opt/go2_edu:rw" go2-ros2-sdk:edu -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_edu/prepare.py'
if sudo docker container inspect go2-sdk >/dev/null 2>&1; then
  sudo docker stop --time 10 go2-sdk >/dev/null
  if sudo docker container inspect go2-sdk >/dev/null 2>&1; then
    sudo docker rm go2-sdk >/dev/null
  fi
fi
echo "Starting EDU $mode; motion control disabled. Foxglove: ws://192.168.123.18:8765"
sudo docker run --rm -it --init --name go2-sdk --network host \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
  -v "$dds_file:/etc/cyclonedds.xml:ro" \
  -v "$task_dir:/opt/go2_edu:rw" \
  --entrypoint /bin/bash go2-ros2-sdk:edu -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec ros2 launch /opt/go2_edu/go2_edu.launch.py mode:="$1"' bash "$mode"
