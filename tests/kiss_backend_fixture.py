"""Dense synthetic submaps for testing real MapClosures, not a farm dataset.

The independent places deliberately contain many ORB density-map features.
The last place repeats the first with an injected odometry displacement. This
isolates the backend chain; its geometry/range/density are not Go2 L1 evidence.
"""
import numpy as np


def rich_scene(seed):
    rng = np.random.default_rng(seed)
    axis = np.arange(-35, 35, .35)
    x, y = np.meshgrid(axis, axis)
    clouds = [np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]]
    for _ in range(150):
        cx, cy = rng.uniform(-28, 28, 2)
        width, depth = rng.uniform(.5, 2.5, 2)
        height = rng.uniform(.6, 5)
        ax = np.arange(-width/2, width/2, .13)
        ay = np.arange(-depth/2, depth/2, .13)
        az = np.arange(0, height, .13)
        u, z = np.meshgrid(ax, az)
        v, zz = np.meshgrid(ay, az)
        clouds.extend(np.c_[u.ravel()+cx, np.full(u.size, cy+side*depth/2), z.ravel()]
                      for side in (-1, 1))
        clouds.extend(np.c_[np.full(v.size, cx+side*width/2), v.ravel()+cy, zz.ravel()]
                      for side in (-1, 1))
    return np.vstack(clouds)


def submap_revisit():
    for index, x in enumerate((0, 10, 20, 30, 40, 50, 3)):
        matrix = np.eye(4)
        matrix[0, 3] = x
        yield rich_scene(17 if index == 6 else 17+index), matrix
