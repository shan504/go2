#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sudo docker image inspect go2-ros2-sdk:edu >/dev/null
sudo docker build -t go2-3d:edu "$project_dir"
sudo docker run --rm --network none --entrypoint /bin/bash \
  -v "$project_dir:/opt/go2_project:ro" go2-3d:edu -c \
  'cd /opt/go2_project && OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -v'
sudo docker run --rm --network none --entrypoint /bin/bash \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e CYCLONEDDS_URI= \
  -v "$project_dir:/opt/go2_project:ro" go2-3d:edu -c \
  'source /opt/ros/humble/setup.bash && cd /opt/go2_project && timeout 90 python3 tests/ros_smoke.py && timeout 90 python3 tests/ros_smoke.py --dense && timeout 45 python3 tests/navigation_gateway_smoke.py'
