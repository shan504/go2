"""Actual installed path-alignment + DWB plugins on synthetic free costmaps.

Uses generated motion parameters, delayed dynamic TF and synthetic kinematics;
no planner, sensor, robot, motor topic or physical motion is involved.
"""
from pathlib import Path
import subprocess
import tempfile
import yaml

project = Path(__file__).resolve().parents[1]
parameters = Path('/runtime/config/nav2_3d.yaml')
values = yaml.safe_load(parameters.read_text())['controller_server']['ros__parameters']
assert values['FollowPath']['plugin'] == 'go2_navigation::PathAlignmentController'
assert values['FollowPath']['primary_controller'] == 'dwb_core::DWBLocalPlanner'
with tempfile.TemporaryDirectory() as temporary:
    build = Path(temporary)/'build'
    subprocess.run(['cmake', '-S', str(project/'tests/path_tracking'), '-B', str(build)],
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run(['cmake', '--build', str(build), '-j2'], check=True,
                   stdout=subprocess.DEVNULL)
    subprocess.run([str(build/'check_path_tracking'), '--ros-args', '--params-file',
                    str(parameters)], check=True)
