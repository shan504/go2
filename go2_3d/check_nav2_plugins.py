#!/usr/bin/env python3
"""Check the official costmap and waypoint-turn plugins before launch."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def check():
    plugins = (
        ('nav2_costmap_2d', 'costmap_plugins.xml', 'nav2_costmap_2d::DenoiseLayer'),
        ('nav2_controller', 'plugins.xml', 'nav2_controller::PoseProgressChecker'),
        ('nav2_rotation_shim_controller', 'nav2_rotation_shim_controller.xml',
         'nav2_rotation_shim_controller::RotationShimController'),
    )
    for package, filename, required in plugins:
        try:
            xml = Path(get_package_share_directory(package))/filename
            types = {item.get('type') for item in ET.parse(xml).iter('class')}
            if required not in types:
                raise ValueError(required)
            if package == 'nav2_rotation_shim_controller':
                prefix = Path(get_package_prefix(package))/'include'
                relative = Path(package)/'nav2_rotation_shim_controller.hpp'
                headers = (prefix/relative, prefix/package/relative)
                if not any(header.is_file() and 'closed_loop_' in header.read_text()
                           for header in headers):
                    raise ValueError('RotationShimController lacks open-loop rotation support')
        except Exception as error:
            raise RuntimeError(f'Nav2 image lacks required {required} support ({error}). Run: bash go2_3d/build.sh '
                               '(updates the Humble Nav2 packages); then restart navigation.') from error
        print(f'PASS Nav2 plugin: {required} available', flush=True)


if __name__ == '__main__':
    check()
