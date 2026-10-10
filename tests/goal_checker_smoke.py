"""Exercise the real Nav2 goal checker using the generated arrival parameters."""
import os
from pathlib import Path
import subprocess
import tempfile
import yaml

project = Path(__file__).resolve().parents[1]
params = yaml.safe_load(Path('/runtime/config/nav2_3d.yaml').read_text())
checker = params['controller_server']['ros__parameters']['general_goal_checker']
with tempfile.TemporaryDirectory() as directory:
    configure = ['cmake', '-S', str(project/'tests/goal_checker'), '-B', directory]
    for variable in ('GO2_NAV2_SOURCE', 'GO2_ANGLES_INCLUDE'):
        if os.environ.get(variable):
            configure.append('-D'+variable+'='+os.environ[variable])
    subprocess.run(configure, check=True, stdout=subprocess.DEVNULL)
    subprocess.run(['cmake', '--build', directory, '-j2'], check=True, stdout=subprocess.DEVNULL)
    values = [str(checker['xy_goal_tolerance']), str(checker['yaw_goal_tolerance']),
              str(checker['stateful']).lower()]
    subprocess.run([str(Path(directory)/'check_arrival'), *values], check=True)
