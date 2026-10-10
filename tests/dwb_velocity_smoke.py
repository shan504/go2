"""Compile the real Humble DWB iterator and feed the generated motion profile."""
import os
from pathlib import Path
import subprocess
import tempfile
import yaml
from ament_index_python.packages import get_package_prefix

project = Path(__file__).resolve().parents[1]
if os.environ.get('GO2_DWB_INCLUDE'):
    include = Path(os.environ['GO2_DWB_INCLUDE'])
else:
    prefix = Path(get_package_prefix('dwb_plugins'))
    include = next(path for path in (prefix/'include', prefix/'include/dwb_plugins')
                   if (path/'dwb_plugins/one_d_velocity_iterator.hpp').is_file())
follow = yaml.safe_load(Path('/runtime/config/nav2_3d.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
with tempfile.TemporaryDirectory() as directory:
    executable = Path(directory)/'dwb_samples'
    subprocess.run(['g++', '-std=c++17', '-I', str(include),
                    str(project/'tests/dwb_velocity/samples.cpp'), '-o', str(executable)], check=True)
    values = [follow[name] for name in ('max_vel_x', 'acc_lim_x', 'sim_time', 'vx_samples')]
    subprocess.run([str(executable), *map(str, values)], check=True)
