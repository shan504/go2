"""Reject misleading package versions and changed native backend installs."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
import check_kiss_env as env


class ProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.manifest = Path(self.directory.name) / 'kiss-provenance.json'
        self.report = dict(schema=1, upstream_commit=env.UPSTREAM_COMMIT,
                           versions=copy.deepcopy(env.CORE_VERSIONS),
                           artifacts={'kiss_slam/core.py': 'installed-source-hash',
                                      'kiss_slam/native.so': 'installed-native-hash'})

    def verify(self, report):
        if report is not None:
            self.manifest.write_text(json.dumps(report))
        with patch.object(env, 'provenance_path', return_value=self.manifest), \
             patch.object(env, 'installed_versions', return_value=env.CORE_VERSIONS), \
             patch.object(env, 'artifact_hashes', return_value=self.report['artifacts']):
            return env.verify_environment()

    def test_pypi_version_alone_is_not_accepted(self):
        with self.assertRaisesRegex(RuntimeError, 'Missing KISS source provenance'):
            self.verify(None)

    def test_different_commit_with_same_package_version_is_rejected(self):
        changed = copy.deepcopy(self.report)
        changed['upstream_commit'] = '2e1d58a'  # 0.0.2 release, not IROS25
        with self.assertRaisesRegex(RuntimeError, 'pinned IROS25'):
            self.verify(changed)

    def test_replaced_native_library_is_rejected(self):
        changed = copy.deepcopy(self.report)
        changed['artifacts']['kiss_slam/native.so'] = 'other-build'
        with self.assertRaisesRegex(RuntimeError, 'artifacts changed'):
            self.verify(changed)

    def test_changed_dependencies_are_rejected(self):
        changed = copy.deepcopy(self.report)
        changed['versions']['open3d'] = '0.14.1'
        with self.assertRaisesRegex(RuntimeError, 'dependency provenance changed'):
            self.verify(changed)

    def test_matching_record_is_accepted(self):
        self.assertEqual(self.verify(self.report), self.report)


if __name__ == '__main__':
    unittest.main()
