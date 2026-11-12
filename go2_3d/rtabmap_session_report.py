#!/usr/bin/env python3
"""Read-only trajectory/seam measurements from one persisted RTAB-Map session.

These are internal consistency measurements, not ground-truth localization
accuracy. No ICP/alignment is fitted by this tool. A closed-looking trajectory
without an accepted global loop edge is explicitly reported as NOT verified.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from geometry import transform_points
from map_start import validate_start_pose
from rtabmap_graph import correction_at


def _rows(path):
    if not path.exists():
        return []
    rows = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        if line.strip():
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise ValueError(f'{path.name}:{number}: incomplete/invalid JSON; stop recording before reporting') from error
    return rows


def load_session(directory):
    directory = Path(directory)
    graph = json.loads((directory/'graph_latest.json').read_text())
    raw = {}
    for row in _rows(directory/'gicp_poses.jsonl'):
        stamp = int(row['stamp_ns'])
        pose = validate_start_pose(row['pose'])
        if stamp in raw and not np.allclose(raw[stamp], pose, atol=1e-10, rtol=0):
            raise ValueError('conflicting source poses in journal')
        raw[stamp] = pose
    keyframes = []
    for path in (directory/'keyframes').glob('*.npz'):
        with np.load(path, allow_pickle=False) as frame:
            stamp = int(frame['stamp_ns'])
            pose = validate_start_pose(frame['pose'])
            points = np.array(frame['points'], dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError(f'invalid local point cloud: {path.name}')
        if stamp not in raw or not np.allclose(raw[stamp], pose, atol=1e-10, rtol=0):
            raise ValueError(f'keyframe lacks matching source pose: {path.name}')
        keyframes.append((stamp, pose, points))
    keyframes.sort(key=lambda frame: frame[0])
    if len(keyframes) < 2 or len({frame[0] for frame in keyframes}) != len(keyframes):
        raise ValueError('need at least two distinct saved local keyframes')
    return keyframes, raw, graph, _rows(directory/'events.jsonl')


def endpoint_error(first, last):
    relative = np.linalg.inv(first) @ last
    angle = np.arccos(np.clip((np.trace(relative[:3, :3])-1.)/2., -1., 1.))
    return {'translation_m': float(np.linalg.norm(relative[:3, 3])),
            'horizontal_translation_m': float(np.linalg.norm((last-first)[:2, 3])),
            'rotation_deg': float(np.degrees(angle))}


def seam_distances(first, second, roi, threshold=.20, max_points=20000):
    """Symmetric nearest-neighbor distances in a fixed ROI, without fitting."""
    roi = np.asarray(roi, dtype=float)
    if roi.shape != (6,) or not np.isfinite(roi).all() or not np.all(roi[:3] < roi[3:]):
        raise ValueError('ROI must contain finite increasing xmin ymin zmin xmax ymax zmax')
    if not np.isfinite(threshold) or threshold <= 0 or max_points < 20:
        raise ValueError('threshold must be positive and max_points >= 20')
    selected = []
    counts = []
    for points in (first, second):
        points = points[np.all((points >= roi[:3]) & (points <= roi[3:]), axis=1)]
        counts.append(len(points))
        if len(points) > max_points:
            points = points[np.linspace(0, len(points)-1, max_points, dtype=int)]
        selected.append(points)
    result = {'roi': roi.tolist(), 'points_in_roi': counts,
              'sampled_points': [len(points) for points in selected],
              'distance_threshold_m': threshold, 'registration_fitted': False}
    # An empty target would produce misleading zero distances in some APIs.
    if min(counts) < 20:
        result.update(status='insufficient_fixed_structure_points', median_m=None,
                      p95_m=None, coverage_within_threshold=None)
        return result
    import open3d as o3d
    clouds = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)) for points in selected]
    distances = [np.asarray(clouds[i].compute_point_cloud_distance(clouds[1-i])) for i in (0, 1)]
    both = np.concatenate(distances)
    result.update(status='measured', median_m=float(np.median(both)),
                  p95_m=float(np.percentile(both, 95)),
                  coverage_within_threshold=[float(np.mean(value <= threshold)) for value in distances])
    return result


def analyze(keyframes, raw, graph, events=(), first_index=0, last_index=-1,
            roi=None, threshold=.20, max_points=20000, same_place_confirmed=False):
    optimized = {int(stamp): validate_start_pose(pose)
                 for stamp, pose in graph['optimized_poses'].items()}
    if not optimized or not optimized.keys() <= raw.keys():
        raise ValueError('optimized graph lacks exact source pose associations')
    anchors = [(stamp, optimized[stamp] @ np.linalg.inv(raw[stamp])) for stamp in sorted(optimized)]
    first_index = first_index if first_index >= 0 else len(keyframes)+first_index
    last_index = last_index if last_index >= 0 else len(keyframes)+last_index
    if not 0 <= first_index < last_index < len(keyframes):
        raise ValueError('select two ordered, distinct keyframe indices')
    first, last = keyframes[first_index], keyframes[last_index]
    corrected = [correction_at(frame[0], anchors) @ frame[1] for frame in (first, last)]
    loops = {tuple(sorted((int(link['from_id']), int(link['to_id']))))
             for link in graph.get('links', []) if int(link['type']) == 1
             and int(link['from_id']) > 0 and int(link['to_id']) > 0
             and int(link['from_id']) != int(link['to_id'])}
    covered = anchors[0][0] <= first[0] and last[0] <= anchors[-1][0]
    result = {
        'interpretation': 'internal consistency only; no ground-truth accuracy or real-world loop success is established',
        'graph_revision': graph.get('graph_revision'),
        'keyframe_count': len(keyframes), 'source_pose_count': len(raw),
        'selected_indices': [first_index, last_index],
        'selected_stamps_ns': [first[0], last[0]],
        'same_physical_place_confirmed_by_user': bool(same_place_confirmed),
        'selected_frames_within_graph_anchors': covered,
        'omitted_retained_anchor_count': len(graph.get('omitted_retained_anchors', [])),
        'global_loop_edges': [list(pair) for pair in sorted(loops)],
        'global_loop_constraint_count': len(loops),
        'closure_evidence': ('accepted_global_constraint_present' if loops else 'NOT_VERIFIED_no_accepted_global_loop_constraint'),
        'real_world_loop_success_verified': False,
        'raw_gicp_start_end': endpoint_error(first[1], last[1]),
        'optimized_start_end': endpoint_error(*corrected),
        'events_recorded': len(events),
        'event_types': {},
        'latest_graph_apply_recorded': False,
        'map_rebuild_events': 0,
        'recent_loop_and_map_events': [],
    }
    for event in events:
        name = event.get('event', event.get('type', 'unspecified'))
        result['event_types'][name] = result['event_types'].get(name, 0)+1
        if name == 'graph_applied':
            result['latest_graph_apply_recorded'] |= event.get('graph_revision') == graph.get('graph_revision')
            result['map_rebuild_events'] += int(bool(event.get('map_rebuilt')))
        if ((name == 'loop_candidate' and (event.get('highest_hypothesis_id') or 0) > 0)
                or (name == 'graph_applied' and (event.get('map_rebuilt')
                    or event.get('global_loop_constraints_added') or event.get('global_loop_constraints_removed')))):
            keys = ('event', 'host_time_ns', 'ref_id', 'highest_hypothesis_id',
                    'highest_hypothesis_score', 'rejected_hypothesis', 'visual_inliers',
                    'graph_revision', 'global_loop_constraints_added',
                    'global_loop_constraints_removed', 'optimization_iterations', 'map_rebuilt')
            result['recent_loop_and_map_events'].append({key: event[key] for key in keys if key in event})
    result['recent_loop_and_map_events'] = result['recent_loop_and_map_events'][-20:]
    if roi is not None:
        result['fixed_roi_seam'] = {
            'raw_gicp': seam_distances(transform_points(first[2], first[1]),
                                       transform_points(last[2], last[1]), roi, threshold, max_points),
            'optimized': seam_distances(transform_points(first[2], corrected[0]),
                                        transform_points(last[2], corrected[1]), roi, threshold, max_points),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session_directory')
    parser.add_argument('--first-index', type=int, default=0)
    parser.add_argument('--last-index', type=int, default=-1)
    parser.add_argument('--same-place-confirmed', action='store_true',
                        help='Operator confirms selected scans came from the same physical place; not independent ground truth')
    parser.add_argument('--roi', type=float, nargs=6, metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'))
    parser.add_argument('--distance-threshold', type=float, default=.20)
    parser.add_argument('--max-points', type=int, default=20000)
    args = parser.parse_args()
    try:
        data = load_session(args.session_directory)
        report = analyze(*data, first_index=args.first_index, last_index=args.last_index,
                         roi=args.roi, threshold=args.distance_threshold,
                         max_points=args.max_points, same_place_confirmed=args.same_place_confirmed)
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    except (ValueError, OSError, KeyError) as error:
        parser.exit(2, f'Cannot report session: {error}\n')


if __name__ == '__main__':
    main()
