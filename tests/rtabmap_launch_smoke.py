#!/usr/bin/env python3
"""Real ROS Humble launch evaluation; calibration/package checks are STUBBED.

This tests configuration wiring and ROS parameter types, not RTAB processing or
physical calibration. Run with the repository mounted at /opt/go2_project:
  source /opt/ros/humble/setup.bash
  python3 /opt/go2_project/tests/rtabmap_launch_smoke.py
"""
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


ROOT = Path('/opt/go2_project')


def load_launch(name):
    path = ROOT / 'go2_3d' / (name + '.launch.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evaluated_parameters(node, context):
    # Humble stores normalized parameters privately; evaluate them using its
    # actual launch_ros evaluator, the code which converts plain '2' to int.
    values = evaluate_parameters(context, node._Node__parameters)
    result = {}
    for value in values:
        if isinstance(value, dict):
            result.update(value)
        else:
            result.update(yaml.safe_load(Path(value).read_text())['go2_gicp']['ros__parameters'])
    return result


class LaunchChecks(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='go2-rtab-launch-test-')
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.calibration = self.directory / 'synthetic.yaml'
        self.calibration.write_text('synthetic calibration is deliberately not validated in this test\n')
        self.calibration_module = types.SimpleNamespace(
            load_calibration=lambda path: types.SimpleNamespace(optical_frame='test_camera_optical'),
            validate_input_topic=lambda *args: None)
        modules = patch.dict(sys.modules, {
            'camera_calibration': self.calibration_module,
            'check_rtabmap': types.SimpleNamespace(check_versions=lambda: {'rtabmap_slam': 'stub'})})
        modules.start()
        self.addCleanup(modules.stop)
        self.backend = load_launch('rtabmap_backend')

    def context(self, profile='standard', **overrides):
        context = LaunchContext()
        context.launch_configurations.update(
            camera_calibration=str(self.calibration), session_directory=str(self.directory / 'session'),
            gicp_params=str(ROOT / 'go2_3d' / ('gicp_dense.yaml' if profile == 'dense' else 'gicp.yaml')),
            map_name='synthetic', output_directory=str(self.directory / 'maps'),
            use_sim_time='false')
        context.launch_configurations.update(overrides)
        return context

    def test_backend_parameter_types_and_profile_inheritance(self):
        for profile, resolution, period in (('standard', .06, 2.0), ('dense', .02, 3.0)):
            with self.subTest(profile=profile):
                context = self.context(profile, use_sim_time='true', map_name='123')
                nodes = [a for a in self.backend.backend_nodes(context) if isinstance(a, Node)]
                self.assertEqual(len(nodes), 3)
                camera, adapter, rtab = [evaluated_parameters(n, context) for n in nodes]
                self.assertTrue(all(p['use_sim_time'] for p in (camera, adapter, rtab)))
                self.assertEqual(adapter['map_voxel_size'], resolution)
                self.assertEqual(adapter['map_publish_period'], period)
                self.assertEqual(adapter['calibration_file'], str(self.calibration))
                self.assertEqual(adapter['map_name'], '123')
                self.assertEqual(rtab['Reg/Strategy'], '2')
                self.assertTrue(all(isinstance(value, str) for key, value in rtab.items() if '/' in key))
                self.assertEqual(rtab['RGBD/LoopClosureIdentityGuess'], 'false')
                self.assertEqual(rtab['Rtabmap/PublishStats'], 'true')
                self.assertEqual(rtab['RGBD/Enabled'], 'true')
                self.assertEqual(rtab['Mem/BadSignaturesIgnored'], 'false')
                self.assertEqual(rtab['RGBD/OptimizeMaxError'], '3')
                self.assertEqual(rtab['Optimizer/Robust'], 'false')
                self.assertIs(rtab['publish_tf'], False)
                self.assertIs(rtab['approx_sync'], False)
                self.assertIs(rtab['gen_depth'], True)
                self.assertIs(rtab['subscribe_depth'], False)
                self.assertEqual(rtab['qos_image'], 1)

    def test_existing_database_and_empty_session_are_rejected(self):
        context = self.context()
        session = Path(context.launch_configurations['session_directory'])
        session.mkdir()
        database = session / 'rtabmap.db'
        database.write_bytes(b'pre-existing database must be preserved')
        with self.assertRaisesRegex(RuntimeError, 'reuse'):
            self.backend.backend_nodes(context)
        self.assertEqual(database.read_bytes(), b'pre-existing database must be preserved')
        context.launch_configurations['session_directory'] = ''
        with self.assertRaisesRegex(RuntimeError, 'absolute'):
            self.backend.backend_nodes(context)

    def test_invalid_calibration_prevents_session_creation(self):
        def reject(_):
            raise ValueError('unmeasured calibration')
        self.calibration_module.load_calibration = reject
        context = self.context()
        with self.assertRaisesRegex(ValueError, 'unmeasured'):
            self.backend.backend_nodes(context)
        self.assertFalse(Path(context.launch_configurations['session_directory']).exists())

    def stack(self, context):
        read_text = Path.read_text
        def with_synthetic_urdf(path, *args, **kwargs):
            if str(path) == '/runtime/config/go2_edu.urdf':
                return '<robot name="synthetic"><link name="base_link"/></robot>'
            return read_text(path, *args, **kwargs)
        with patch('ament_index_python.packages.get_package_share_directory', return_value='/tmp/stub-package'), \
                patch.object(Path, 'read_text', with_synthetic_urdf):
            stack = load_launch('stack')
            entities = list(stack.generate_launch_description().entities)
        for entity in entities:
            if isinstance(entity, DeclareLaunchArgument):
                entity.execute(context)
        nodes = [entity for entity in entities if isinstance(entity, Node)]
        for entity in entities:
            if isinstance(entity, OpaqueFunction):
                nodes.extend(entity.execute(context) or [])
        return {n._Node__node_name: evaluated_parameters(n, context)
                for n in nodes if n._Node__node_name in ('go2_gicp', 'go2_camera_sample')}

    def test_live_switch_changes_only_backend_wiring_and_optional_camera_rate(self):
        for profile in ('standard', 'dense'):
            with self.subTest(profile=profile):
                original = self.stack(self.context(profile, mode='mapping', loop_backend='lidar'))
                candidate = self.stack(self.context(profile, mode='mapping', loop_backend='rtabmap'))
                lidar = dict(original['go2_gicp'])
                rtab = dict(candidate['go2_gicp'])
                self.assertEqual(rtab.pop('mapping_backend'), 'rtabmap')
                self.assertEqual(rtab.pop('map_frame'), 'gicp_odom')
                self.assertIs(rtab.pop('loop_closure_enabled'), False)
                self.assertIs(lidar.pop('loop_closure_enabled'), True)
                self.assertEqual(lidar, rtab, 'GICP frontend/density parameters were changed')
                self.assertEqual(original['go2_camera_sample']['sample_rate'], 2.0)
                self.assertEqual(candidate['go2_camera_sample']['sample_rate'], 8.0)

    def test_navigation_and_localization_keep_original_backend(self):
        for mode in ('localization', 'navigation'):
            with self.subTest(mode=mode):
                nodes = self.stack(self.context(mode=mode, loop_backend='lidar'))
                mapper = nodes['go2_gicp']
                self.assertEqual(mapper['mode'], 'localization')
                self.assertEqual(mapper['scan_window'], .5)
                self.assertNotIn('mapping_backend', mapper)
                self.assertNotIn('map_frame', mapper)
                with self.assertRaisesRegex(RuntimeError, 'mapping-only'):
                    self.stack(self.context(mode=mode, loop_backend='rtabmap'))

    def test_replay_switch_and_simulated_clock(self):
        replay = load_launch('rtabmap_replay')
        for backend in ('lidar', 'rtabmap'):
            with self.subTest(backend=backend):
                context = self.context('dense', loop_backend=backend)
                nodes = [a for a in replay.replay_nodes(context) if isinstance(a, Node)]
                self.assertEqual(len(nodes), 2, 'Replay must not start live sensors or motion control')
                helper, mapper = [evaluated_parameters(n, context) for n in nodes]
                self.assertTrue(helper['use_sim_time'])
                self.assertTrue(mapper['use_sim_time'])
                self.assertEqual(mapper['loop_closure_enabled'], backend == 'lidar')
                self.assertEqual(mapper['map_voxel_size'], .02)


if __name__ == '__main__':
    unittest.main(verbosity=2)
