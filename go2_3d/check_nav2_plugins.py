#!/usr/bin/env python3
"""Fail before launching navigation if this image lacks the denoise plugin."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


def check():
    xml = Path(get_package_share_directory('nav2_costmap_2d'))/'costmap_plugins.xml'
    types = {item.get('type') for item in ET.parse(xml).iter('class')}
    if 'nav2_costmap_2d::DenoiseLayer' not in types:
        raise RuntimeError('Nav2 image lacks DenoiseLayer. Run: bash go2_3d/build.sh '
                           '(updates the Humble Nav2 packages); then restart navigation.')
    print('PASS Nav2 plugin: DenoiseLayer available', flush=True)


if __name__ == '__main__':
    check()
