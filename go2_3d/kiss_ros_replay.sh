#!/usr/bin/env bash
# Internal entrypoint of replay_kiss.sh, not a robot launch.
set -euo pipefail
set +u  # Humble's generated setup scripts reference optional unset variables.
source /opt/ros/humble/setup.bash
set -u
backend="${1:?backend required}"
rate="${2:-0.5}"
case "$backend" in kiss|lidar) ;; *) exit 2 ;; esac
if [[ "$backend" == kiss ]]; then
  /opt/kiss-venv/bin/python3 /opt/go2_project/go2_3d/check_kiss_env.py > /results/dependencies.json
fi
ros2 bag info /bag > /results/bag-info.txt
/usr/bin/python3 - "$backend" "$rate" <<'PY'
import json,sys
from pathlib import Path
import open3d
Path('/results/run.json').write_text(json.dumps(dict(
    backend=sys.argv[1], rate=float(sys.argv[2]), frontend_open3d=open3d.__version__,
    replay_topics=['/point_cloud2','/odom'], network='none', motion_nodes=False,
    provenance='unknown', source='/bag'), indent=2)+'\n')
PY
python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_DFL); os.execvp("ros2",["ros2"]+sys.argv[1:])' \
  launch /opt/go2_project/go2_3d/kiss_replay.launch.py \
  loop_backend:="$backend" output_directory:=/results/data \
  mapper_python:=/usr/bin/python3 kiss_python:=/opt/kiss-venv/bin/python3 \
  > /results/mapping.log 2>&1 &
launch_pid=$!
cleanup() {
  trap - EXIT INT TERM
  kill -INT "$launch_pid" 2>/dev/null || true
  # A crashed/hung launch must not keep an offline container alive forever.
  for _ in {1..100}; do
    kill -0 "$launch_pid" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM "$launch_pid" 2>/dev/null || true
    sleep 1
    kill -KILL "$launch_pid" 2>/dev/null || true
  fi
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
# The helper only observes readiness and calls the mapping-save Trigger.
python3 /opt/go2_project/go2_3d/finish_kiss_replay.py --wait-ready
# Never replay /tf, /tf_static, /cmd_vel, action requests or operator controls.
ros2 bag play /bag --clock 100 --rate "$rate" --delay 2 \
  --topics /point_cloud2 /odom > /results/playback.log 2>&1
python3 /opt/go2_project/go2_3d/finish_kiss_replay.py \
  --output /results/save-result.json
