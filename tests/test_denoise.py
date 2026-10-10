import hashlib
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from denoise import clean_image, clean_directory
from map_library import archive, select, fingerprints
from clean_map import clean


def corridor():
    # 0.6m corridor: a 1- or 2-cell speckle can seal it after 0.23m
    # inscribed inflation. Continuous walls must survive the filter.
    image = np.full((17, 81), 205, dtype=np.uint8)
    image[3:14, 3:78] = 254
    image[2, 2:79] = 0
    image[14, 2:79] = 0
    image[8, 32] = 0
    image[8, 48:50] = 0
    return image


def write_map(root, image):
    source = root/'original'; source.mkdir()
    height, width = image.shape
    (source/'nav.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+image.tobytes())
    (source/'nav.yaml').write_text(yaml.safe_dump(dict(image='nav.pgm', resolution=0.05,
        origin=[-1.0, -0.4, 0.0], negate=0, occupied_thresh=0.65, free_thresh=0.196)))
    (source/'metadata.yaml').write_text(yaml.safe_dump(dict(ground_model={'plane':[0,0,1,0]},
        parameters={'grid_resolution':0.05}, body_height_above_ground=0.35)))
    (source/'map.pcd').write_bytes(b'Unmodified full 3D localization PCD fixture\n')
    (root/'latest').symlink_to(source.name)
    return source


class DenoiseTests(unittest.TestCase):
    def test_remove_one_two_cells_keep_walls_and_unknown(self):
        image = corridor()
        result, report = clean_image(image, {'minimal_group_size':3, 'group_connectivity_type':8})
        self.assertEqual(report['removed_cells'], 3)
        self.assertEqual(report['removed_groups'], 2)
        self.assertEqual(result[8, 32], 254)
        np.testing.assert_array_equal(result[2], image[2])
        np.testing.assert_array_equal(result[14], image[14])
        np.testing.assert_array_equal(result == 205, image == 205)

    def test_diagonal_cluster_and_three_cells_survive(self):
        image = np.full((8, 8), 254, dtype=np.uint8)
        image[1, 1] = image[2, 2] = image[3, 3] = 0
        result, report = clean_image(image, {'minimal_group_size':3, 'group_connectivity_type':8})
        np.testing.assert_array_equal(result, image)
        self.assertEqual(report['removed_cells'], 0)
        result, report = clean_image(image, {'minimal_group_size':4, 'group_connectivity_type':8})
        self.assertEqual(report['removed_cells'], 3)
        # The requested farm profile also removes coherent but small groups.
        result, report = clean_image(image)
        self.assertEqual(report['minimal_group_size'], 8)
        self.assertEqual(report['removed_cells'], 3)

    def test_clean_map_copies_pcd_coordinates_unknown_and_retains_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = write_map(root, corridor())
            original_pgm = (source/'nav.pgm').read_bytes()
            pcd_hash = hashlib.sha256((source/'map.pcd').read_bytes()).hexdigest()
            destination = clean(root)
            self.assertEqual((root/'latest').resolve(), destination)
            self.assertEqual((source/'nav.pgm').read_bytes(), original_pgm)
            self.assertEqual(hashlib.sha256((destination/'map.pcd').read_bytes()).hexdigest(), pcd_hash)
            self.assertEqual((source/'nav.yaml').read_bytes(), (destination/'nav.yaml').read_bytes())
            metadata = yaml.safe_load((destination/'metadata.yaml').read_text())
            self.assertEqual(metadata['body_height_above_ground'], 0.35)
            self.assertEqual(metadata['navigation_denoise']['removed_cells'], 3)
            self.assertEqual(clean_directory(destination)['removed_cells'], 0)

    def test_failure_keeps_latest_and_original(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = write_map(root, corridor())
            with patch('clean_map.clean_directory', side_effect=ValueError('bad PGM')):
                with self.assertRaises(ValueError):
                    clean(root)
            self.assertEqual((root/'latest').resolve(), source)
            self.assertEqual(sorted(p.name for p in root.iterdir()), ['latest', 'library', 'original'])
            self.assertEqual(list(root.glob('library/*/*')), [], 'Failed copy published/retained a partial version')

    def test_clean_named_map_keeps_original_archive_and_namespace(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = write_map(root,corridor())
            copied = archive(root,'indoor')
            before = fingerprints(copied)
            select(root,'indoor')
            destination = clean(root)
            self.assertEqual(destination.parent,root/'library/indoor')
            self.assertEqual(fingerprints(copied),before)
            self.assertEqual((source/'map.pcd').read_bytes(),(destination/'map.pcd').read_bytes())
            self.assertFalse((destination/'archive.yaml').exists())
            self.assertEqual((root/'library/indoor/latest').resolve(),destination)


if __name__ == '__main__':
    unittest.main()
