#!/usr/bin/env bash
# Offline only: a NEW network-isolated container, no SDK, no Nav2, no motion.
set -euo pipefail
if [[ ${1:-} == --help || $# -lt 2 || $# -gt 4 ]]; then
  echo 'Usage: replay_kiss.sh BAG_DIRECTORY NEW_OUTPUT_DIRECTORY [kiss|lidar] [RATE=0.5]'
  echo 'Uses go2-3d:kiss-backend (override GO2_KISS_REPLAY_IMAGE); never stops existing containers.'
  [[ ${1:-} == --help ]] && exit 0 || exit 2
fi
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bag="$(realpath -- "$1")"
output="$(realpath -m -- "$2")"
backend="${3:-kiss}"
rate="${4:-0.5}"
case "$backend" in kiss|lidar) ;; *) echo 'Backend must be kiss or lidar' >&2; exit 2 ;; esac
[[ -d "$bag" && -f "$bag/metadata.yaml" ]] || { echo 'A ROS2 bag directory with metadata.yaml is required' >&2; exit 2; }
[[ ! -e "$output" ]] || { echo 'Output must be a NEW directory; existing data is never overwritten' >&2; exit 2; }
[[ "$bag" != *:* && "$output" != *:* && "$project_dir" != *:* ]] || { echo 'Docker bind paths cannot contain colons' >&2; exit 2; }
python3 - "$rate" <<'PY'
import math, sys
value=float(sys.argv[1])
if not math.isfinite(value) or not 0 < value <= 1:
    raise SystemExit('Replay rate must be finite and in (0, 1]')
PY
image="${GO2_KISS_REPLAY_IMAGE:-go2-3d:kiss-backend}"
docker image inspect "$image" >/dev/null
mkdir -p -- "$(dirname -- "$output")"
mkdir -- "$output"
echo "Offline $backend replay; output: $output"
docker run --rm --init --network none \
  --name "go2-kiss-replay-$(date -u +%Y%m%dT%H%M%S)-$$" \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=89 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e CYCLONEDDS_URI= -e ROS_LOG_DIR=/results/ros_logs \
  -e OMP_NUM_THREADS=2 -e OPENBLAS_NUM_THREADS=1 \
  -v "$project_dir:/opt/go2_project:ro" -v "$bag:/bag:ro" -v "$output:/results:rw" \
  --entrypoint /bin/bash "$image" /opt/go2_project/go2_3d/kiss_ros_replay.sh "$backend" "$rate"
