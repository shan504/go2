#!/usr/bin/env bash
set -euo pipefail
sensor_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
existing_dir="$HOME/go2_edu_fix"
dds_file="$HOME/cyclonedds_ws/cyclonedds.xml"
for required in "$existing_dir/go2_edu_dds_bridge.py" "$existing_dir/prepare.py" "$dds_file"; do
  test -f "$required" || { echo "Missing $required" >&2; exit 1; }
done
sudo -v
sudo docker image inspect go2-ros2-sdk:edu >/dev/null
sudo docker run --rm --network none --entrypoint /bin/bash \
  -v "$existing_dir:/opt/go2_edu:rw" go2-ros2-sdk:edu -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /opt/go2_edu/prepare.py'
if sudo docker container inspect go2-sdk >/dev/null 2>&1; then
  sudo docker stop --time 10 go2-sdk >/dev/null
  if sudo docker container inspect go2-sdk >/dev/null 2>&1; then
    sudo docker rm go2-sdk >/dev/null
  fi
fi
echo 'Starting 3D point-cloud and camera preview; motion disabled. Fixed frame: odom.'
sudo docker run --rm -it --init --name go2-sdk --network host \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///etc/cyclonedds.xml \
  -v "$dds_file:/etc/cyclonedds.xml:ro" \
  -v "$existing_dir:/opt/go2_edu:rw" \
  -v "$sensor_dir:/opt/go2_sensors:ro" \
  --entrypoint /bin/bash go2-ros2-sdk:edu -c \
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec ros2 launch /opt/go2_sensors/sensors.launch.py'
