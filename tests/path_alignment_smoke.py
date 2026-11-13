"""Real C++ path shim/TF/footprint regression; no planner, motor or robot motion."""
import os
from pathlib import Path
import subprocess
import tempfile
import sys

project = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project/'go2_3d'))
from motion_profile import (PATH_HEADING_THRESHOLD, PATH_HEADING_RELEASE, PATH_TURN_SPEED,
                            YAW_ACCELERATION, LINEAR_SPEED)

with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    configure = ['cmake', '-S', str(project/'tests/path_alignment'), '-B', str(directory/'build')]
    env = os.environ.copy()
    for variable in ('GO2_NAV2_SOURCE', 'GO2_ANGLES_INCLUDE'):
        if env.get(variable):
            configure.append('-D'+variable+'='+env[variable])
    if env.get('GO2_NAV2_SOURCE'):
        marker = directory/'share/ament_index/resource_index/packages/nav2_core'
        marker.parent.mkdir(parents=True)
        marker.write_text('')
        package = directory/'share/nav2_core/package.xml'
        package.parent.mkdir(parents=True)
        package.write_bytes((Path(env['GO2_NAV2_SOURCE'])/'nav2_core/package.xml').read_bytes())
        env['AMENT_PREFIX_PATH'] = str(directory)+':'+env.get('AMENT_PREFIX_PATH', '')
    subprocess.run(configure, check=True, stdout=subprocess.DEVNULL, env=env)
    subprocess.run(['cmake', '--build', str(directory/'build'), '-j1'], check=True,
                   stdout=subprocess.DEVNULL, env=env)
    subprocess.run([str(directory/'build/check_path_alignment'),
                    *map(str, (PATH_HEADING_THRESHOLD, PATH_HEADING_RELEASE, 0.25,
                               PATH_TURN_SPEED, YAW_ACCELERATION, 1.0, LINEAR_SPEED))],
                   check=True, env=env)
