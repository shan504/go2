"""Synthetic persistence/identity tests; no robot, ROS or navigation execution."""
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_3d'))
from route_store import MAX_FILE_BYTES, RouteStore, map_identity, validate_route


def fixture_map(directory, label='farm_a', aligned=True):
    """Small valid ASCII PCD/grid fixtures, deliberately not sensor observations."""
    directory.mkdir(parents=True, exist_ok=True)
    z = 0.0 if aligned else -0.3
    points = [(0., 0., z), (1., 0., z), (0., 1., z)]
    pcd = ('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
           'WIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n')
    pcd += ''.join(f'{x} {y} {height}\n' for x, y, height in points)
    (directory / 'map.pcd').write_text(pcd)
    (directory / 'nav.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([254, 254, 0, 205]))
    nav = dict(image='nav.pgm', resolution=.05, origin=[0., 0., 0.], negate=0,
               occupied_thresh=.65, free_thresh=.196, mode='trinary')
    metadata = dict(frame_id='map', parameters=dict(map_name=label),
                    ground_model=dict(plane=[0., 0., 1., -z], support_points=300),
                    body_height_above_ground=.3,
                    map_start_pose=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, .3], [0, 0, 0, 1]],
                    map_from_source_map=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -z], [0, 0, 0, 1]],
                    source_map='/old/maps/farm_a', created_at='2026-11-12')
    (directory / 'nav.yaml').write_text(yaml.safe_dump(nav))
    (directory / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    return directory


def fixture_route(identity):
    return dict(schema_version=1, map=identity, waypoints=[
        dict(position=[x, y, 0.], orientation=[0., 0., 0., 1.])
        for x, y in [(0.5, 0.5), (2., 1.), (0., 0.)]])


class RouteStoreTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.map = fixture_map(self.root / 'runtime_map')
        self.identity = map_identity(self.map)
        self.route = fixture_route(self.identity)
        self.store = RouteStore(self.root / 'routes')

    def test_three_points_persist_across_store_restart_and_repeat_load(self):
        path = self.store.save('farm_loop', self.route)
        self.assertEqual(path.name, 'farm_loop.json')
        for _ in range(3):
            restarted = RouteStore(self.root / 'routes')
            loaded = restarted.load('farm_loop')
            self.assertEqual(loaded, self.route)
            self.assertEqual(validate_route(loaded, self.identity), self.route)
        self.assertEqual(self.store.list_routes(), [dict(name='farm_loop', waypoints=3, map='farm_a')])
        self.assertEqual(list((self.root / 'routes').glob('*.tmp')), [])

    def test_no_overwrite_without_explicit_flag(self):
        self.store.save('loop', self.route)
        changed = copy.deepcopy(self.route)
        changed['waypoints'][1]['position'][0] = 3.
        with self.assertRaises(FileExistsError):
            self.store.save('loop', changed)
        self.assertEqual(self.store.load('loop'), self.route)
        self.store.save('loop', changed, overwrite=True)
        self.assertEqual(self.store.load('loop'), changed)

    def test_concurrent_saves_commit_exactly_one_complete_file(self):
        other = copy.deepcopy(self.route)
        other['waypoints'][0]['position'][0] = 12.
        def save(route):
            try:
                self.store.save('concurrent', route)
                return True
            except FileExistsError:
                return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            successes = list(pool.map(save, [self.route, other]))
        self.assertEqual(sum(successes), 1)
        self.assertIn(self.store.load('concurrent'), [self.route, other])

    def test_invalid_payload_does_not_replace_existing_file(self):
        path = self.store.save('loop', self.route)
        original = path.read_bytes()
        invalid = copy.deepcopy(self.route)
        invalid['waypoints'][0]['position'][0] = float('nan')
        with self.assertRaises(ValueError):
            self.store.save('loop', invalid, overwrite=True)
        self.assertEqual(path.read_bytes(), original)

    def test_reject_invalid_numbers_quaternions_and_counts(self):
        for bad in (float('nan'), float('inf'), -float('inf'), 10**1000, True, '0.2', None):
            route = copy.deepcopy(self.route)
            route['waypoints'][0]['position'][0] = bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_route(route)
        for bad in ([0, 0, 0, 0], [0, 0, 0, 2], [0, 0, 1], [0, False, 0, 1]):
            route = copy.deepcopy(self.route)
            route['waypoints'][0]['orientation'] = bad
            with self.subTest(quaternion=bad), self.assertRaises(ValueError):
                validate_route(route)
        for count in (0, 101):
            route = copy.deepcopy(self.route)
            route['waypoints'] = [route['waypoints'][0]] * count
            with self.subTest(count=count), self.assertRaises(ValueError):
                validate_route(route)

    def test_strict_schema_types_and_frames(self):
        for field, value in [('schema_version', True), ('schema_version', 2), ('waypoints', {})]:
            route = copy.deepcopy(self.route)
            route[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                validate_route(route)
        for field, value in [('frame_id', 'odom'), ('fingerprint', 'bad'), ('label', '\n')]:
            route = copy.deepcopy(self.route)
            route['map'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_route(route)
        route = copy.deepcopy(self.route)
        route['waypoints'][0]['extra'] = 1
        with self.assertRaises(ValueError):
            validate_route(route)

    def test_names_and_symlinks_cannot_escape_store(self):
        for name in ('../outside', '/tmp/outside', 'foo/bar', '.', '', '-bad', 'x' * 65):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.store.save(name, self.route)
        self.store.directory.mkdir(exist_ok=True)
        victim = self.root / 'outside.json'
        victim.write_text('untouched')
        (self.store.directory / 'link.json').symlink_to(victim)
        with self.assertRaises(ValueError):
            self.store.load('link')
        with self.assertRaises(ValueError):
            self.store.save('link', self.route, overwrite=True)
        self.assertEqual(victim.read_text(), 'untouched')

    def test_malformed_duplicate_and_oversized_files_rejected(self):
        self.store.directory.mkdir()
        path = self.store.directory / 'bad.json'
        for content in (b'{', b'\xff', b'{"a":1,"a":2}', b' ' * (MAX_FILE_BYTES + 1)):
            path.write_bytes(content)
            with self.subTest(content=content[:20]), self.assertRaises(ValueError):
                self.store.load('bad')
        path.write_text(json.dumps(self.route).replace('0.5', 'NaN', 1))
        with self.assertRaises(ValueError):
            self.store.load('bad')

    def test_fifo_is_rejected_without_waiting_for_a_writer(self):
        self.store.directory.mkdir()
        os.mkfifo(self.store.directory / 'pipe.json')
        with self.assertRaisesRegex(ValueError, 'regular file'):
            self.store.load('pipe')

    def test_two_map_geometries_cannot_share_route(self):
        second = fixture_map(self.root / 'other_map', label='farm_b')
        with (second / 'map.pcd').open('a') as stream:
            stream.write('# Different actual map contents\n')
        with self.assertRaisesRegex(ValueError, 'map mismatch'):
            validate_route(self.route, map_identity(second))

    def test_copied_map_and_metadata_serialization_preserve_identity(self):
        copied = self.root / 'archived_map'
        shutil.copytree(self.map, copied)
        nav = yaml.safe_load((copied / 'nav.yaml').read_text())
        nav['origin'] = [0, 0, 0]
        nav['negate'] = 0.0
        nav['image'] = 'renamed.pgm'
        (copied / 'nav.pgm').rename(copied / 'renamed.pgm')
        (copied / 'nav.yaml').write_text(yaml.safe_dump(nav, sort_keys=False))
        metadata = yaml.safe_load((copied / 'metadata.yaml').read_text())
        metadata.update(source_map='/new/archive/location', created_at='2030-01-01')
        metadata['parameters']['map_name'] = 'farm_a_copy'
        metadata['ground_model']['support_points'] = 400
        (copied / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
        identity = map_identity(copied)
        self.assertEqual(identity['fingerprint'], self.identity['fingerprint'])
        validate_route(self.route, identity)

    def test_geometry_and_navigation_grid_changes_invalidate_identity(self):
        for change in ('ground', 'body_height', 'start_pose', 'transform', 'grid_origin', 'occupancy'):
            with self.subTest(change=change):
                copied = self.root / change
                shutil.copytree(self.map, copied)
                metadata = yaml.safe_load((copied / 'metadata.yaml').read_text())
                nav = yaml.safe_load((copied / 'nav.yaml').read_text())
                if change == 'ground':
                    metadata['ground_model']['plane'][3] += .1
                elif change == 'body_height':
                    metadata['body_height_above_ground'] += .1
                elif change == 'start_pose':
                    metadata['map_start_pose'][0][3] += 1.
                elif change == 'transform':
                    metadata['map_from_source_map'][0][3] += 1.
                elif change == 'grid_origin':
                    nav['origin'][0] += 1.
                else:
                    (copied / 'nav.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 254, 0, 205]))
                (copied / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
                (copied / 'nav.yaml').write_text(yaml.safe_dump(nav))
                self.assertNotEqual(map_identity(copied)['fingerprint'], self.identity['fingerprint'])

    def test_raw_and_aligned_bundles_have_stable_separate_identities(self):
        fingerprints = []
        for aligned in (False, True):
            directory = fixture_map(self.root / ('leveled' if aligned else 'raw'), aligned=aligned)
            before = {p.name: p.read_bytes() for p in directory.iterdir()}
            first = map_identity(directory)
            self.assertEqual(first, map_identity(directory))
            self.assertEqual(before, {p.name: p.read_bytes() for p in directory.iterdir()})
            fingerprints.append(first['fingerprint'])
        self.assertNotEqual(*fingerprints)

    def test_map_image_cannot_escape_bundle(self):
        nav = yaml.safe_load((self.map / 'nav.yaml').read_text())
        nav['image'] = '../elsewhere.pgm'
        (self.root / 'elsewhere.pgm').write_bytes(b'outside')
        (self.map / 'nav.yaml').write_text(yaml.safe_dump(nav))
        with self.assertRaisesRegex(ValueError, 'escapes'):
            map_identity(self.map)


if __name__ == '__main__':
    unittest.main()
