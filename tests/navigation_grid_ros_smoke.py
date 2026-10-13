"""Validate navigation preflight and real ROS obstacle settings across maps."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory

project = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project/'go2_3d'))
from map_library import fingerprints
from navigation_grid import current_settings, prepare
from obstacle_cloud import ObstacleCloud
from test_free_space import sparse_corridor

spec = importlib.util.spec_from_file_location('go2_prepare', project/'go2_3d/prepare.py')
configuration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(configuration)
sdk = os.environ.get('GO2_TEST_SDK_SOURCE') or get_package_share_directory('go2_robot_sdk')
configuration.get_package_share_directory = lambda package: sdk

with tempfile.TemporaryDirectory() as tmp:
    maps, runtime = Path(tmp)/'maps', Path(tmp)/'runtime/navigation_map'
    maps.mkdir()
    source = sparse_corridor(maps)
    metadata = yaml.safe_load((source/'metadata.yaml').read_text())
    metadata['parameters'].update(obstacle_min_height=0.9, obstacle_max_height=1.0)
    (source/'metadata.yaml').write_text(yaml.safe_dump(metadata))
    before = fingerprints(source)

    def runtime_paths(root, output):
        # Only Docker mount paths are adapted; SDK preparation and the PCD
        # projection/denoising execute their actual production implementations.
        assert (root, output) == ('/maps', '/runtime/navigation_map')
        return prepare(maps, runtime)

    with patch.object(configuration, 'prepare_navigation_grid', side_effect=runtime_paths) as preflight:
        configuration.main('navigation')
    assert preflight.call_count == 1
    policy = current_settings()
    nav = yaml.safe_load((runtime/'nav.yaml').read_text())
    config = yaml.safe_load(Path('/runtime/config/nav2_3d.yaml').read_text())
    assert nav['resolution'] == policy['grid_resolution'] == 0.05
    for name in ('global_costmap', 'local_costmap'):
        assert config[name][name]['ros__parameters']['resolution'] == nav['resolution']
    for directory in (source, runtime):
        rclpy.init(args=['--ros-args', '-p', f'map_directory:={directory}'])
        node = ObstacleCloud()
        try:
            assert node.min_height == policy['obstacle_min_height'] == 0.1
            assert node.max_height == policy['obstacle_max_height'] == 1.5
            assert node.model == metadata['ground_model']
        finally:
            node.destroy_node()
            rclpy.shutdown()
    assert fingerprints(source) == before
    assert (maps/'latest').resolve() == source
print('PASS ROS shared map policy: actual navigation preflight regenerates the grid; '
      'global/local costmaps agree; live obstacles use current heights over stale map parameters; '
      'saved PCD/metadata/latest unchanged')
