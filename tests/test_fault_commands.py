"""Use the user's shell commands to inspect/export evidence without Docker."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest

project = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project/'go2_3d'))
from fault_store import EventStore, create_session


class FaultCommandTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.checkout = Path(self.temporary.name)/'project with spaces'
        scripts = self.checkout/'go2_3d'
        scripts.mkdir(parents=True)
        for name in ('tools.sh', 'fault_cli.py', 'fault_store.py'):
            shutil.copy2(project/'go2_3d'/name, scripts/name)
        commands = Path(self.temporary.name)/'commands'
        commands.mkdir()
        forbidden = commands/'sudo'
        forbidden.write_text('#!/bin/sh\necho "Unexpected Docker/control command" >&2\nexit 99\n')
        forbidden.chmod(0o755)
        self.environment = dict(os.environ, PATH=str(commands)+os.pathsep+os.environ['PATH'])
        self.root = self.checkout/'runtime/diagnostics'
        self.session = create_session(self.root, {'source': {'commit': 'fixture-sha'}})
        (self.session/'launch.log').write_text('planner ready\nFailed to make progress\n')
        self.identity = EventStore(self.session).record_fault('Failed to make progress',
            {'route': {'current': '5', 'queued': '8'},
             'odometry': {'available': True, 'net_displacement_m': 0.01}},
            {'initial_snapshot': {'control': 'fresh command'}})

    def tearDown(self):
        self.temporary.cleanup()

    def command(self, *arguments):
        return subprocess.run(['bash', str(self.checkout/'go2_3d/tools.sh'), *arguments],
            cwd=self.temporary.name, env=self.environment, capture_output=True, text=True, timeout=10)

    def test_inspection_survives_new_idle_session_without_ros_or_docker(self):
        create_session(self.root)
        for arguments in (('faults',), ('fault-show',), ('fault-show', self.identity), ('fault-sessions',)):
            result = self.command(*arguments)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(self.session.name if arguments[0] == 'fault-sessions' else self.identity,
                          result.stdout)
        self.assertIn('控制器进度超时', self.command('fault-show').stdout)

    def test_export_produces_self_contained_bundle_at_printed_path(self):
        result = self.command('fault-export')
        self.assertEqual(result.returncode, 0, result.stderr)
        filename = result.stdout.split('诊断包已保存: ', 1)[1].splitlines()[0]
        with tarfile.open(filename) as archive:
            summary = json.load(archive.extractfile('fault/summary.json'))
            manifest = json.load(archive.extractfile('session/manifest.json'))
            self.assertEqual(summary['fault_id'], self.identity)
            self.assertEqual(manifest['source']['commit'], 'fixture-sha')
            self.assertIn('fault/launch-before.log.gz', archive.getnames())
            self.assertIn('fault/report.txt', archive.getnames())

    def test_invalid_selector_is_data_not_a_shell_command(self):
        marker = Path(self.temporary.name)/'must-not-exist'
        result = self.command('fault-show', f'../bad;touch {marker}')
        self.assertEqual(result.returncode, 2)
        self.assertFalse(marker.exists())


if __name__ == '__main__':
    unittest.main()
