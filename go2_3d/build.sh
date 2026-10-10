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
  'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /opt/go2_project && python3 tests/navigation_launch_smoke.py && python3 tests/config_smoke.py && python3 tests/dwb_velocity_smoke.py && timeout 30 python3 tests/navigation_resume_smoke.py && timeout 30 python3 tests/inflation_parameters_smoke.py && timeout 20 python3 tests/localized_control_smoke.py && timeout 45 python3 tests/motioncheck_smoke.py && timeout 35 python3 tests/robotcheck_smoke.py && timeout 65 python3 tests/drivecheck_smoke.py && timeout 65 python3 tests/walking_drivecheck_smoke.py && timeout 80 python3 tests/native_drivecheck_smoke.py && timeout 90 python3 tests/ros_smoke.py && timeout 90 python3 tests/ros_smoke.py --dense && timeout 45 python3 tests/leveled_localization_smoke.py && timeout 45 python3 tests/navigation_ready_smoke.py && timeout 45 python3 tests/navigation_gateway_smoke.py'
