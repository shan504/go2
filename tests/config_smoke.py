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
    assert cfg['robot_base_frame'] == 'base_link'
    assert cfg['global_frame'] == ('map' if name == 'global_costmap' else 'odom')
    assert cfg['obstacle_layer']['scan']['topic'] == '/scan'
    assert 'voxel_layer' not in cfg['plugins']
assert params['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
urdf = ET.parse('/runtime/config/go2_edu.urdf').getroot()
assert not {'map','odom'} & {link.get('name') for link in urdf.findall('link')}
assert all(p.read_bytes() == content for p,content in before.items())
print('PASS SDK configuration: no AMCL, correct costmap frames, protected source unchanged')
