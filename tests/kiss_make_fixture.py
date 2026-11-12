#!/usr/bin/env python3
"""Create explicitly synthetic offline journals; these are not farm recordings.

farm reuses the repository's 200 m toy farm and injected drifting poses.
rich isolates the native detector/ICP/g2o chain using deliberately dense,
feature-rich independent places; its extent and density are not Go2 L1 data.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from mapping_journal import MappingJournal
from kiss_backend_fixture import submap_revisit
from test_loop_closure import farm_geometry, square_route, local_scan, pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('farm', 'rich'))
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    metadata = dict(provenance='synthetic', recorded_backend='none', frontend_feedback=False,
                    pose_convention='uncorrected_gicp')
    if args.kind == 'farm':
        metadata['source'] = ('Deterministic test_loop_closure toy farm: 200 m square, '
                              'final additional 1.5 m observation, injected 5 m/-2 m/6 deg drift. '
                              'No robot data, exact idealized point scans.')
        writer = MappingJournal(args.output, metadata)
        world = farm_geometry()
        route = square_route()+[pose(1.5, 0.)]
        for index, truth in enumerate(route):
            progress = min(index/40., 1.)
            drift = pose(5.*progress, -2.*progress, np.deg2rad(6.)*progress)
            writer.append((index+1)*1_000_000_000, local_scan(world, truth), drift@truth)
        np.savez(args.output/'synthetic_ground_truth.npz', poses=np.stack(route))
    else:
        metadata['source'] = ('Dense artificial 70x70 m submaps with randomized boxes; '
                              'last place exactly repeats first with injected 3 m displacement. '
                              'Backend component fixture, not a physically consistent farm or L1 recording.')
        metadata['parameters'] = dict(map_voxel_size=.20, voxel_size=.20, grid_resolution=.20,
                                      floor_z=-.30, obstacle_min_height=.10, obstacle_max_height=1.5,
                                      kiss_submap_max_observations=1, kiss_max_iterations=40)
        writer = MappingJournal(args.output, metadata)
        for index, (points, matrix) in enumerate(submap_revisit()):
            points = points.copy()
            points[:, 2] -= .30  # Body-origin floor for shared map export.
            writer.append((index+1)*1_000_000_000, points, matrix)
    print(json.dumps({'journal': str(args.output), 'frames': writer.count,
                      'provenance': 'synthetic', 'real_robot_data': False}, indent=2))


if __name__ == '__main__':
    main()
