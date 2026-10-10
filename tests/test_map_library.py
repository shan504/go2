import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from map_library import archive, select, publish, new_directory, fingerprints, map_name, list_maps


def bundle(directory,content=b'original point cloud'):
    directory.mkdir(parents=True)
    (directory/'map.pcd').write_bytes(content)
    (directory/'nav.pgm').write_bytes(b'P5\n2 1\n255\n\x00\xfe')
    (directory/'nav.yaml').write_text(yaml.safe_dump(dict(image='nav.pgm',resolution=0.05,origin=[-1.,-2.,0.])))
    (directory/'metadata.yaml').write_text(yaml.safe_dump(dict(parameters={'floor_z':-0.4},ground_model={'plane':[0,0,1,0.4]})))
    (directory/'observed_free.npz').write_bytes(b'original observed evidence')
    return directory


class MapLibraryTests(unittest.TestCase):
    def test_archive_all_keeps_originals_and_exact_bundle_and_is_repeatable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = bundle(root/'20261008T110447.853039Z')
            current = bundle(root/'cleaned-20261010T080000.000000Z',b'current cloud')
            incomplete = root/'partial'; incomplete.mkdir()
            publish(root,current)
            originals = {path:fingerprints(path) for path in (old,current)}
            archived = archive(root,'indoor',True)
            self.assertEqual(archived,root/'library/indoor'/current.name)
            self.assertEqual((root/'latest').resolve(),current)
            self.assertEqual((root/'library/indoor/latest').resolve(),archived)
            for path,hashes in originals.items():
                self.assertEqual(fingerprints(path),hashes)
                self.assertEqual(fingerprints(root/'library/indoor'/path.name),hashes)
            self.assertFalse((root/'library/indoor/partial').exists())
            self.assertEqual(archive(root,'indoor',True),archived)
            with patch('sys.stdout',new_callable=io.StringIO) as out:
                list_maps(root)
            self.assertIn('indoor/'+current.name,out.getvalue())

    def test_farm_versions_preserve_indoor_and_switch_back(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            indoor = bundle(root/'original'); publish(root,indoor)
            copied = archive(root,'indoor')
            indoor_hashes = fingerprints(copied)
            farm = bundle(new_directory(root,'farm'),b'farm cloud')
            publish(root,farm)
            farm_two = bundle(new_directory(root,'farm'),b'new farm cloud')
            publish(root,farm_two)
            self.assertTrue(farm.exists())
            self.assertEqual((root/'library/farm/latest').resolve(),farm_two)
            self.assertEqual(fingerprints(copied),indoor_hashes)
            self.assertEqual(select(root,'indoor'),copied)
            self.assertEqual((root/'latest').resolve(),copied)
            self.assertEqual(select(root,'farm',farm.name),farm)

    def test_failed_copy_or_incomplete_selection_keeps_selected_map(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            original = bundle(root/'original'); publish(root,original)
            def incomplete_copy(source,destination):
                destination.mkdir()
                (destination/'map.pcd').write_bytes(b'partially copied PCD')
                raise OSError('disk full')
            with patch('map_library.shutil.copytree',side_effect=incomplete_copy):
                with self.assertRaises(OSError):
                    archive(root,'indoor')
            self.assertEqual((root/'latest').resolve(),original)
            self.assertFalse((root/'library/indoor/latest').exists())
            self.assertEqual(list((root/'library/indoor').iterdir()),[])
            broken = bundle(root/'library/broken/version')
            (broken/'map.pcd').unlink()
            with self.assertRaises(ValueError):
                select(root,'broken','version')
            self.assertEqual((root/'latest').resolve(),original)

    def test_invalid_names_external_paths_and_version_conflict_are_rejected(self):
        for name in ('../indoor','','farm/foo','farm;touch test','-farm'):
            with self.assertRaises(ValueError):
                map_name(name)
        with tempfile.TemporaryDirectory() as tmp,tempfile.TemporaryDirectory() as outside:
            root = Path(tmp)
            original = bundle(root/'original'); publish(root,original)
            archive(root,'indoor')
            (root/'library/indoor/original/map.pcd').write_bytes(b'changed archive')
            with self.assertRaises(ValueError):
                archive(root,'indoor')
            (root/'library/escape').symlink_to(outside)
            with self.assertRaises(ValueError):
                new_directory(root,'escape')
            with self.assertRaises(ValueError):
                select(root,'indoor','../../original')
            self.assertEqual((root/'latest').resolve(),original)


if __name__ == '__main__':
    unittest.main()
