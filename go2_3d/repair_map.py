#!/usr/bin/env python3
"""Rebuild a saved map's navigation grid; preserve original PCD and old directory."""
import argparse
from datetime import datetime,timezone
import os
from pathlib import Path
import shutil
import numpy as np
import open3d as o3d
import yaml
from geometry import ObservedGrid
from ground import estimate_ground


def grid_stats(directory):
    cfg = yaml.safe_load((directory/'nav.yaml').read_text())
    raw = (directory/cfg['image']).read_bytes().split(b'\n',3)
    image = np.frombuffer(raw[3],dtype=np.uint8)
    return dict(free=int(np.count_nonzero(image>=250)),occupied=int(np.count_nonzero(image<=10)),
                unknown=int(np.count_nonzero((image>10)&(image<250))))


def load_old_free(source,grid):
    archive = source/'observed_free.npz'
    if archive.exists():
        with np.load(archive,allow_pickle=False) as saved:
            grid.free = set(map(tuple,saved['cells']))
        return
    nav = yaml.safe_load((source/'nav.yaml').read_text())
    raw = (source/nav['image']).read_bytes().split(b'\n',3)
    width,height = map(int,raw[1].split())
    if raw[0]!=b'P5' or raw[2]!=b'255' or len(raw[3])!=width*height:
        raise ValueError('Unsupported original PGM; refusing to guess old observed space')
    if abs(nav['origin'][2])>1e-6 or abs(nav['resolution']-grid.resolution)>1e-6:
        raise ValueError('Original grid geometry unsupported; refusing to reinterpret cells')
    image = np.frombuffer(raw[3],dtype=np.uint8).reshape(height,width)
    row,col = np.nonzero(image>=250)
    lower = np.rint(np.asarray(nav['origin'][:2])/grid.resolution).astype(int)
    grid.free = set(map(tuple,np.column_stack((col+lower[0],height-1-row+lower[1]))))


def repair(root):
    root = Path(root)
    source = (root/'latest').resolve(strict=True)
    metadata = yaml.safe_load((source/'metadata.yaml').read_text())
    points = np.asarray(o3d.io.read_point_cloud(str(source/'map.pcd')).points)
    p = metadata['parameters']
    ground = estimate_ground(points,p.get('floor_z',-0.30))
    destination = root/('repaired-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    destination.mkdir()
    try:
        grid = ObservedGrid(p.get('grid_resolution',0.10))
        load_old_free(source,grid)
        grid.export(points,destination,p.get('floor_z',-0.30),p.get('obstacle_min_height',0.10),
                    p.get('obstacle_max_height',1.5),ground)
        shutil.copy2(source/'map.pcd',destination/'map.pcd')
        stats = dict(before=grid_stats(source),after=grid_stats(destination))
        metadata.update(ground_model=ground,repaired_from=str(source),repair_grid_stats=stats,
                        repair='ground-relative projection and observed-floor free cells; original XYZ preserved')
        (destination/'metadata.yaml').write_text(yaml.safe_dump(metadata))
        link = root/'.latest-repair'
        if link.is_symlink():
            link.unlink()
        link.symlink_to(destination.name)
        os.replace(link,root/'latest')
    except Exception:
        # Never publish a partial map as latest. Keep the old directory intact.
        shutil.rmtree(destination)
        raise
    print(f'REPAIRED navigation grid: {destination}\nOriginal retained: {source}\nGround: {ground}\nGrid cells: {stats}',flush=True)
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps',default='/maps')
    repair(parser.parse_args().maps)
