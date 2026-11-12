#!/usr/bin/env python3
"""Offline comparison of legacy and KISS backends on frozen GICP observations.

No ROS node, publisher, motion control, or live map is opened. Results are
internal consistency measurements, not ground-truth accuracy. The two backends
receive the same frozen observations; a complete frontend/backend A/B requires
replaying the original sensor bag independently for each mapping pipeline.
"""
import argparse
from dataclasses import asdict, fields
import json
from pathlib import Path
import sys
import time

import numpy as np

from geometry import transform_points
from loop_closure import LoopClosure, LoopConfig
from mapping_journal import load_journal
from mapping_rebuild import corrected_poses, rebuild
from map_export import save_snapshot
from registration import cloud
from rtabmap_session_report import endpoint_error, seam_distances


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (tuple, list)):
        return [json_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): json_value(item) for key, item in value.items()}
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(json_value(value), ensure_ascii=False,
                                   indent=2, allow_nan=False) + '\n')


def make_backend(name, settings=None, kiss_python=None):
    if name == 'lidar':
        config = LoopConfig(**(settings or {}))
        return LoopClosure(config), asdict(config)
    if name == 'kiss':
        from kiss_backend import KissConfig
        from kiss_backend_process import KissBackendProcess
        config = KissConfig(**(settings or {}))
        return KissBackendProcess(config, python=kiss_python or sys.executable), asdict(config)
    raise ValueError('backend must be lidar or kiss')


def replay(dataset, backend, resolutions=(.06, .15, .10), *, roi=None,
           same_place_confirmed=False):
    """Replay through the same propose/rebuild/apply contract as live mapping."""
    processed, events = [], []
    carry = np.eye(4)
    applied = 0
    map_rebuilds = 0
    candidate_store = candidate_grid = None
    started = time.monotonic()

    def process_result(result, reason):
        nonlocal processed, carry, applied, map_rebuilds, candidate_store, candidate_grid
        entry = dict(event=reason, observation_index=len(processed)-1,
                     stamp_ns=dataset.stamps_ns[len(processed)-1],
                     backend_status=json_value(dict(backend.status)),
                     optimization_proposed=result is not None, map_rebuilt=False)
        if result is not None:
            # Validate prefix association and prepare complete replacement
            # geometry before committing any backend correction.
            poses = corrected_poses(processed, result.original_poses, result.poses)
            store, grid = rebuild(processed, poses, *resolutions)
            backend.apply(result)
            processed = [item.with_pose(pose) for item, pose in zip(processed, poses)]
            candidate_store, candidate_grid = store, grid
            carry = poses[-1] @ np.linalg.inv(dataset.observations[len(processed)-1].pose)
            applied += 1
            map_rebuilds += 1
            entry.update(map_rebuilt=True, optimization_applied=True,
                         loop_edges=json_value(result.loop_edges),
                         optimization_status=json_value(result.status),
                         rebuilt_point_count=store.count,
                         rebuilt_free_cells=len(grid.free))
        events.append(entry)

    for observation in dataset.observations:
        # GICP recordings are immutable. Future local poses inherit the last
        # *accepted* backend frame correction exactly once, as in live mapping.
        current = observation.with_pose(carry @ observation.pose)
        processed.append(current)
        backend.append(current.points_body, current.pose)
        process_result(backend.detect_and_optimize(), 'observation_checked')
    if callable(getattr(backend, 'flush', None)):
        backend.flush()
        process_result(backend.detect_and_optimize(), 'final_partial_submap_checked')
    # Include all observations after the final optimization, or the entire map
    # when no loop was found. This output is never just an optimized path.
    candidate_store, candidate_grid = rebuild(processed, [item.pose for item in processed],
                                               *resolutions)
    elapsed = time.monotonic()-started
    original = dataset.observations
    report = {
        'comparison_scope': 'backend_only_on_identical_frozen_GICP_observations',
        'interpretation': 'Internal consistency, not ground-truth accuracy. '
                          'Frontend map feedback is not re-run in this comparison.',
        'dataset_sha256': dataset.sha256, 'dataset_metadata': dataset.metadata,
        'replay_runtime': {'python': sys.version.split()[0],
                           'executable': sys.executable,
                           'numpy': np.__version__,
                           'open3d': __import__('open3d').__version__,
                           'note': 'This runtime rebuilds maps and runs the lidar baseline. '
                                   'It is not necessarily the robot runtime; KISS runs in its private environment.'},
        'observations': len(processed), 'input_points': sum(len(x.points_body) for x in original),
        'first_stamp_ns': dataset.stamps_ns[0], 'last_stamp_ns': dataset.stamps_ns[-1],
        'elapsed_seconds': elapsed, 'map_voxel_size': resolutions[0],
        'registration_voxel_size': resolutions[1], 'grid_resolution': resolutions[2],
        'applied_optimizations': applied, 'loop_triggered_map_rebuilds': map_rebuilds,
        'final_map_rebuilt': True, 'final_map_points': candidate_store.count,
        'final_free_cells': len(candidate_grid.free),
        'same_physical_start_end_confirmed_by_user': bool(same_place_confirmed),
        'endpoint_measurement': ('claimed_same_place_closure_residual' if same_place_confirmed
                                 else 'start_end_separation_not_ground_truth_error'),
        'raw_start_end': endpoint_error(original[0].pose, original[-1].pose),
        'optimized_start_end': endpoint_error(processed[0].pose, processed[-1].pose),
        'backend_status': json_value(dict(backend.status)),
        'real_world_loop_success_verified': False,
        'closure_evidence': ('accepted_backend_optimization_and_map_rebuild' if applied
                             else 'NOT_VERIFIED_no_accepted_backend_optimization'),
    }
    if roi is not None:
        report['fixed_roi_seam'] = {
            'raw_gicp': seam_distances(transform_points(original[0].points_body, original[0].pose),
                                       transform_points(original[-1].points_body, original[-1].pose), roi),
            'optimized': seam_distances(transform_points(processed[0].points_body, processed[0].pose),
                                        transform_points(processed[-1].points_body, processed[-1].pose), roi),
        }
    return report, events, processed, candidate_store, candidate_grid


def write_result(directory, dataset, result, settings, export_bundles=False):
    import open3d as o3d
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    report, events, observations, store, grid = result
    report = dict(report, backend_configuration=settings)
    np.savez(directory/'trajectory.npz', stamps_ns=np.asarray(dataset.stamps_ns, dtype=np.int64),
             raw_poses=np.stack([item.pose for item in dataset.observations]),
             optimized_poses=np.stack([item.pose for item in observations]))
    np.savez(directory/'observed_free.npz', cells=np.asarray(sorted(grid.free), dtype=np.int64).reshape(-1, 2),
             resolution=float(grid.resolution))
    if not o3d.io.write_point_cloud(str(directory/'map.pcd'), cloud(store.snapshot(), None), compressed=True):
        raise RuntimeError('failed to write optimized PCD')
    write_json(directory/'map_start_pose.json', observations[0].pose)
    write_json(directory/'events.json', events)
    if export_bundles:
        params = dict(dataset.metadata.get('parameters', {}))
        params.update(output_directory=str(directory/'map_bundles'), map_name='offline_'+directory.name,
                      map_voxel_size=report['map_voxel_size'], voxel_size=report['registration_voxel_size'],
                      grid_resolution=report['grid_resolution'])
        for name, default in (('floor_z', -.30), ('obstacle_min_height', .10),
                              ('obstacle_max_height', 1.5)):
            params.setdefault(name, default)
        destination = save_snapshot(store.snapshot(), set(grid.free), observations[0].pose,
                                    params, report, 'offline_backend_'+directory.name)
        report['navigation_bundle'] = str(destination.relative_to(directory))
    write_json(directory/'report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal', type=Path)
    parser.add_argument('--backend', choices=('lidar', 'kiss', 'both'), default='both')
    parser.add_argument('--output', type=Path, required=True, help='New empty output directory; never overwritten')
    parser.add_argument('--lidar-config', type=Path, help='JSON object of legacy LoopConfig fields')
    parser.add_argument('--kiss-config', type=Path, help='JSON object of KissConfig fields')
    parser.add_argument('--kiss-python', help='Private KISS environment Python; defaults to this interpreter')
    parser.add_argument('--map-voxel-size', type=float)
    parser.add_argument('--registration-voxel-size', type=float)
    parser.add_argument('--grid-resolution', type=float)
    parser.add_argument('--assume-uncorrected-gicp', action='store_true')
    parser.add_argument('--same-place-confirmed', action='store_true')
    parser.add_argument('--roi', nargs=6, type=float, metavar=('XMIN','YMIN','ZMIN','XMAX','YMAX','ZMAX'))
    parser.add_argument('--export-bundles', action='store_true', help='Also build compatible navigation bundles inside output only')
    args = parser.parse_args(argv)
    output_created = False
    backends = {}
    try:
        dataset = load_journal(args.journal, args.assume_uncorrected_gicp)
        if args.output.exists() or args.output.is_symlink():
            raise ValueError('output already exists; choose a new experiment directory')
        params = dataset.metadata.get('parameters', {})
        resolutions = (args.map_voxel_size if args.map_voxel_size is not None else params.get('map_voxel_size', .06),
                       args.registration_voxel_size if args.registration_voxel_size is not None else params.get('voxel_size', .15),
                       args.grid_resolution if args.grid_resolution is not None else params.get('grid_resolution', .10))
        names = ('lidar', 'kiss') if args.backend == 'both' else (args.backend,)
        # Resolve configurations and dependencies before creating output.
        for name in names:
            path = getattr(args, name+'_config')
            if name == 'lidar':
                config_type, prefix = LoopConfig, 'loop_'
            else:
                from kiss_backend import KissConfig
                config_type, prefix = KissConfig, 'kiss_'
            settings = {item.name: params[prefix+item.name]
                        for item in fields(config_type) if prefix+item.name in params}
            if path:
                overrides = json.loads(path.read_text())
                if not isinstance(overrides, dict):
                    raise ValueError(f'{name} configuration must be a JSON object')
                settings.update(overrides)
            backends[name] = make_backend(name, settings, args.kiss_python)
        args.output.mkdir(parents=True, exist_ok=False)
        output_created = True
        comparison = {'dataset_sha256': dataset.sha256, 'dataset_metadata': dataset.metadata,
                      'scope': 'frozen_frontend_backend_comparison',
                      'requested_backends': list(names), 'complete': False,
                      'real_world_validation': False, 'results': {}}
        write_json(args.output/'comparison.json', comparison)
        for name in names:
            backend, settings = backends[name]
            result = replay(dataset, backend, resolutions, roi=args.roi,
                            same_place_confirmed=args.same_place_confirmed)
            comparison['results'][name] = write_result(args.output/name, dataset, result,
                                                        settings, args.export_bundles)
            write_json(args.output/'comparison.json', comparison)
        comparison['complete'] = True
        write_json(args.output/'comparison.json', comparison)
        print(json.dumps(json_value(comparison), ensure_ascii=False, indent=2, allow_nan=False))
    except (ValueError, TypeError, OSError, KeyError, ImportError, RuntimeError) as error:
        if output_created:
            write_json(args.output/'FAILED.json', {'complete': False, 'error': str(error)})
        parser.exit(2, f'Offline comparison failed: {error}\n')
    finally:
        for backend, _ in backends.values():
            if callable(getattr(backend, 'close', None)):
                backend.close()


if __name__ == '__main__':
    main()
