"""Route files and CLI intent checks; no robot or ROS motion is involved."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
import route_cli
from route_store import RouteStore


class RouteCliTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.route = dict(schema_version=1,
            map=dict(frame_id='map', fingerprint='a' * 64, label='farm'),
            waypoints=[dict(position=[x, 0., 0.], orientation=[0., 0., 0., 1.])
                       for x in (.4, .8, 1.2)])
        self.store = RouteStore(self.directory.name)

    def run_cli(self, *arguments):
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            code = route_cli.main(['--directory', self.directory.name, *arguments])
        return code, output.getvalue(), error.getvalue()

    def test_save_complete_snapshot_and_explicit_overwrite(self):
        with patch.object(route_cli, 'exchange', return_value=dict(success=True, route=self.route)) as request:
            self.assertEqual(self.run_cli('save', 'loop')[0], 0)
            self.assertEqual(request.call_args.args[0]['command'], 'snapshot')
            self.assertEqual(self.store.load('loop'), self.route)
            self.assertEqual(self.run_cli('save', 'loop')[0], 1)
            self.assertEqual(self.run_cli('save', 'loop', '--overwrite')[0], 0)

    def test_show_and_list_work_without_ros_or_network(self):
        self.store.save('loop', self.route)
        with patch.object(route_cli, 'exchange', side_effect=AssertionError('Must stay offline')):
            code, output, _ = self.run_cli('show', 'loop')
            self.assertEqual(code, 0)
            self.assertEqual(json.loads(output), self.route)
            code, output, _ = self.run_cli('list')
            self.assertEqual(code, 0)
            self.assertIn('loop: 3 points', output)

    def test_load_and_run_have_distinct_explicit_intents(self):
        self.store.save('loop', self.route)
        with patch.object(route_cli, 'exchange', return_value=dict(success=True, message='accepted')) as request:
            self.assertEqual(self.run_cli('load', 'loop')[0], 0)
            loaded = request.call_args.args[0]
            self.assertEqual(loaded['command'], 'load')
            self.assertEqual(loaded['route'], self.route)
            code, output, _ = self.run_cli('run', 'loop')
            self.assertEqual(code, 0)
            started = request.call_args.args[0]
            self.assertEqual(started['command'], 'run')
            self.assertNotEqual(loaded['request_id'], started['request_id'])
            self.assertIn('START REQUESTED', output)

    def test_invalid_names_and_missing_files_send_no_request(self):
        with patch.object(route_cli, 'exchange') as request:
            self.assertEqual(self.run_cli('save', '../escape')[0], 1)
            self.assertEqual(self.run_cli('load', 'missing')[0], 1)
            self.assertEqual(self.run_cli('run', 'missing')[0], 1)
            request.assert_not_called()

    def test_unknown_run_outcome_is_not_retried(self):
        self.store.save('loop', self.route)
        with patch.object(route_cli, 'exchange', side_effect=TimeoutError('No receipt; outcome unknown')) as request:
            code, _, error = self.run_cli('run', 'loop')
            self.assertEqual(code, 1)
            self.assertIn('outcome unknown', error)
            request.assert_called_once()


if __name__ == '__main__':
    unittest.main()
