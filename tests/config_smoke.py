"""Validate generated navigation/URDF copies against an actual SDK checkout."""
import importlib.util
import math
import os
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

root = Path('/opt/go2_project')
spec = importlib.util.spec_from_file_location('go2_prepare',root/'go2_3d/prepare.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
sdk = Path(os.environ.get('GO2_TEST_SDK_SOURCE') or get_package_share_directory('go2_robot_sdk'))
before = {p:p.read_bytes() for p in sdk.rglob('*') if p.is_file()}
module.get_package_share_directory = lambda package: str(sdk)
module.main()
params = yaml.safe_load(Path('/runtime/config/nav2_3d.yaml').read_text())
from motion_profile import LINEAR_SPEED, YAW_SPEED, LINEAR_ACCELERATION, TRAJECTORY_TIME, ARRIVAL_RADIUS
follow = params['controller_server']['ros__parameters']['FollowPath']
smoother = params['velocity_smoother']['ros__parameters']
assert follow['max_vel_x'] == follow['max_speed_xy'] == smoother['max_velocity'][0] == LINEAR_SPEED == 0.30
assert follow['max_vel_theta'] == smoother['max_velocity'][2] == YAW_SPEED
assert follow['min_vel_x'] == smoother['min_velocity'][0] == 0.0
assert follow['min_vel_y'] == follow['max_vel_y'] == 0.0
assert follow['vx_samples'] == 2 and follow['vy_samples'] == 1
assert follow['trajectory_generator_name'] == 'dwb_plugins::StandardTrajectoryGenerator'
assert follow['limit_vel_cmd_in_traj'] is False
assert follow['sim_time'] == TRAJECTORY_TIME
assert LINEAR_ACCELERATION * TRAJECTORY_TIME >= LINEAR_SPEED
assert follow['acc_lim_x'] == smoother['max_accel'][0] == LINEAR_ACCELERATION
assert follow['decel_lim_x'] == smoother['max_decel'][0] == -LINEAR_ACCELERATION
assert follow['xy_goal_tolerance'] == params['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']
checker = params['controller_server']['ros__parameters']['general_goal_checker']
assert checker['xy_goal_tolerance'] == ARRIVAL_RADIUS == 0.30
assert checker['yaw_goal_tolerance'] == math.pi and checker['stateful'] is False
assert 'RotateToGoal' not in follow['critics']
assert params['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance'] == 10.0
assert 'amcl' not in params
for name in ('local_costmap','global_costmap'):
    cfg = params[name][name]['ros__parameters']
    assert cfg['robot_base_frame'] == 'base_footprint'
    assert cfg['global_frame'] == ('map' if name == 'global_costmap' else 'odom')
    assert cfg['obstacle_layer']['scan']['topic'] == '/scan'
    assert cfg['obstacle_layer']['scan']['inf_is_valid'] is False
    assert 'voxel_layer' not in cfg['plugins']
    assert cfg['inflation_layer']['inflation_radius'] == 0.05
    assert cfg['resolution'] == 0.05
    assert cfg['inflation_layer']['cost_scaling_factor'] == 12.0
    assert cfg['plugins'][-3:] == ['obstacle_layer','denoise_layer','inflation_layer']
    assert cfg['denoise_layer']['plugin'] == 'nav2_costmap_2d::DenoiseLayer'
    assert cfg['denoise_layer']['minimal_group_size'] == 8
    assert cfg['denoise_layer']['group_connectivity_type'] == 8
    assert cfg['footprint'] == '[[-0.40,-0.22],[-0.40,0.22],[0.40,0.22],[0.40,-0.22]]'
    assert cfg['footprint_padding'] == 0.01
    assert cfg['always_send_full_costmap'] is True
local = params['local_costmap']['local_costmap']['ros__parameters']
assert local['width'] == local['height'] == 4
assert local['resolution'] == 0.05 and local['rolling_window'] is True
assert params['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
urdf = ET.parse('/runtime/config/go2_edu.urdf').getroot()
assert not {'map','odom'} & {link.get('name') for link in urdf.findall('link')}
assert not any(j.find('child').get('link') == 'base_footprint' for j in urdf.findall('joint'))
links = {link.get('name') for link in urdf.findall('link')}
children = {j.find('child').get('link') for j in urdf.findall('joint')}
assert links-children == {'base_link'},'Generated URDF contains an orphan navigation frame'
assert params['bt_navigator']['ros__parameters']['robot_base_frame'] == 'base_footprint'
assert params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'].endswith('/go2_3d/navigate.xml')
behavior = ET.parse(root/'go2_3d/navigate.xml')
assert not list(behavior.iter('Spin')) and not list(behavior.iter('BackUp'))
assert list(behavior.iter('FollowPath'))[0].get('goal_checker_id') == 'general_goal_checker'
assert all(p.read_bytes() == content for p,content in before.items())
print('PASS SDK configuration: matched 0.30m/s envelope and forward samples, 0.30m position-only arrival with explicit goal checker, no terminal rotation, genuine stall timeout retained, protected SDK source unchanged')
