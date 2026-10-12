"""Run official Humble shim/progress code with the generated patrol parameters."""
import os
from pathlib import Path
import subprocess
import tempfile
import yaml

project = Path(__file__).resolve().parents[1]
parameters = yaml.safe_load(Path('/runtime/config/nav2_3d.yaml').read_text())['controller_server']['ros__parameters']
follow, progress = parameters['FollowPath'], parameters['progress_checker']
with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    configure = ['cmake', '-S', str(project/'tests/waypoint_turn'), '-B', str(directory/'build')]
    env = os.environ.copy()
    for variable in ('GO2_NAV2_SOURCE', 'GO2_ANGLES_INCLUDE'):
        if env.get(variable):
            configure.append('-D'+variable+'='+env[variable])
    if env.get('GO2_NAV2_SOURCE'):
        # The cloud image only lacks the Nav2 package-index registration and
        # lifecycle wrapper. Production uses the installed official packages.
        marker = directory/'share/ament_index/resource_index/packages/nav2_core'
        marker.parent.mkdir(parents=True)
        marker.write_text('')
        package = directory/'share/nav2_core/package.xml'
        package.parent.mkdir(parents=True)
        package.write_bytes((Path(env['GO2_NAV2_SOURCE'])/'nav2_core/package.xml').read_bytes())
        env['AMENT_PREFIX_PATH'] = str(directory)+':'+env.get('AMENT_PREFIX_PATH', '')
    subprocess.run(configure, check=True, stdout=subprocess.DEVNULL, env=env)
    subprocess.run(['cmake', '--build', str(directory/'build'), '-j2'], check=True,
                   stdout=subprocess.DEVNULL, env=env)
    values = [follow[key] for key in ('angular_dist_threshold', 'angular_disengage_threshold',
        'forward_sampling_distance', 'rotate_to_heading_angular_vel', 'max_angular_accel',
        'simulate_ahead_time', 'closed_loop', 'max_vel_x')]
    values += [progress[key] for key in ('required_movement_radius',
        'required_movement_angle', 'movement_time_allowance')]
    subprocess.run([str(directory/'build/check_waypoint_turn'),
                    *[str(value).lower() for value in values]], check=True, env=env)
