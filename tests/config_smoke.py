"""Validate generated navigation/URDF copies against an actual SDK checkout."""
import importlib.util
import os
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET

root = Path('/opt/go2_project')
spec = importlib.util.spec_from_file_location('go2_prepare',root/'go2_3d/prepare.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
sdk = Path(os.environ['GO2_TEST_SDK_SOURCE'])
before = {p:p.read_bytes() for p in sdk.rglob('*') if p.is_file()}
module.get_package_share_directory = lambda package: str(sdk)
module.main()
params = yaml.safe_load(Path('/runtime/config/nav2_3d.yaml').read_text())
assert 'amcl' not in params
for name in ('local_costmap','global_costmap'):
    cfg = params[name][name]['ros__parameters']
    assert cfg['robot_base_frame'] == 'base_footprint'
    assert cfg['global_frame'] == ('map' if name == 'global_costmap' else 'odom')
    assert cfg['obstacle_layer']['scan']['topic'] == '/scan'
    assert cfg['obstacle_layer']['scan']['inf_is_valid'] is False
    assert 'voxel_layer' not in cfg['plugins']
    assert cfg['inflation_layer']['inflation_radius'] == 0.28
    assert cfg['inflation_layer']['cost_scaling_factor'] == 12.0
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
assert all(p.read_bytes() == content for p,content in before.items())
print('PASS SDK configuration: no AMCL, correct costmap frames, protected source unchanged')
