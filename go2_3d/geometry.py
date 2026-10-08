"""ROS-independent SE(3), cloud filtering and observed-space grid export."""
import math
from pathlib import Path
import numpy as np
import yaml


def pose_matrix(position, quaternion):
    q = np.asarray(quaternion, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("Invalid quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    matrix[:3, 3] = position
    if not np.isfinite(matrix).all():
        raise ValueError("Nonfinite pose")
    return matrix


def quaternion_from_matrix(matrix):
    # Stable eigendecomposition, including rotations near pi.
    r = matrix[:3, :3]
    k = np.array([
        [r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
        [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
        [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
        [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], r.trace()],
    ]) / 3.0
    _, vectors = np.linalg.eigh(k)
    q = vectors[:, -1]
    return q if q[3] >= 0 else -q


def transform_points(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def rotation_angle(matrix):
    return math.acos(float(np.clip((np.trace(matrix[:3, :3])-1)/2, -1, 1)))


def filter_points(points, min_range=0.35, max_range=20.0):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    distances = np.linalg.norm(points, axis=1)
    return points[np.isfinite(points).all(axis=1) &
                  (distances >= min_range) & (distances <= max_range)]


def cloud_xyz(message):
    """Respect PointCloud2 field offsets, datatype, endianness and row padding."""
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [endian+types[fields[k].datatype] for k in ('x','y','z')],
                      'offsets': [fields[k].offset for k in ('x','y','z')],
                      'itemsize': message.point_step})
    data = np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data, strides=(message.row_step, message.point_step))
    return np.column_stack([data[k].reshape(-1) for k in ('x','y','z')])


def line_cells(start, end):
    x, y = start
    ex, ey = end
    dx, dy = abs(ex-x), -abs(ey-y)
    sx, sy = (1 if x < ex else -1), (1 if y < ey else -1)
    error = dx + dy
    while True:
        yield x, y
        if x == ex and y == ey:
            break
        twice = 2*error
        if twice >= dy:
            error += dy
            x += sx
        if twice <= dx:
            error += dx
            y += sy


class ObservedGrid:
    """Unknown stays unknown; occupied voxels override any observed free rays."""
    def __init__(self, resolution=0.10, max_cells=4_000_000):
        if resolution <= 0:
            raise ValueError("resolution must be positive")
        self.resolution = resolution
        self.max_cells = max_cells
        self.free = set()

    def observe(self, points, origin):
        start = tuple(np.floor(np.asarray(origin)[:2]/self.resolution).astype(int))
        delta = points[:, :2] - np.asarray(origin)[:2]
        distances = np.linalg.norm(delta, axis=1)
        # Only the nearest return per angular bin clears space; never a ray
        # through a nearer obstacle to a farther ceiling or floor return.
        bins = np.floor((np.arctan2(delta[:,1], delta[:,0])+np.pi)/0.01).astype(int)
        selected = {}
        for index in np.argsort(distances):
            if distances[index] < 0.2 or distances[index] > 20:
                continue
            selected.setdefault(int(bins[index]), int(index))
        for index in selected.values():
            end = tuple(np.floor(points[index,:2]/self.resolution).astype(int))
            cells = list(line_cells(start, end))
            self.free.update(cells[:-1])
        if len(self.free) > self.max_cells:
            raise RuntimeError("Observed-space limit exceeded; save a smaller map")

    def export(self, points, directory, floor_z=-0.30, min_height=0.10, max_height=1.5):
        obstacle = points[(points[:,2] >= floor_z+min_height) &
                          (points[:,2] <= floor_z+max_height)]
        occupied = set(map(tuple, np.floor(obstacle[:,:2]/self.resolution).astype(int)))
        observed = self.free | occupied
        if not occupied or not self.free:
            raise ValueError("No occupied or observed-free cells; check height slice and sensor data")
        coordinates = np.asarray(list(observed))
        lower, upper = coordinates.min(axis=0)-2, coordinates.max(axis=0)+2
        width, height = upper-lower+1
        if width*height > self.max_cells:
            raise RuntimeError("Grid bounds exceed memory limit")
        image = np.full((height, width), 205, dtype=np.uint8)
        for cells, value in ((self.free, 254), (occupied, 0)):
            cells = np.asarray(list(cells))-lower
            image[height-1-cells[:,1], cells[:,0]] = value
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        (directory/'nav.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+image.tobytes())
        metadata = dict(image='nav.pgm', mode='trinary', resolution=self.resolution,
                        origin=[float(lower[0]*self.resolution), float(lower[1]*self.resolution), 0.0],
                        negate=0, occupied_thresh=0.65, free_thresh=0.196)
        (directory/'nav.yaml').write_text(yaml.safe_dump(metadata))
        return metadata
