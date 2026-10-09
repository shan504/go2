"""Navigation-grid component filtering; never modifies the localization PCD."""
from pathlib import Path
import numpy as np
import yaml


def settings():
    config = yaml.safe_load(Path(__file__).with_name('navigation_filter.yaml').read_text())
    if (type(config.get('minimal_group_size')) is not int or
            not 1 <= config['minimal_group_size'] <= 100 or
            config.get('group_connectivity_type') not in (4, 8)):
        raise ValueError('Invalid navigation_filter.yaml: group size 1..100, connectivity 4 or 8')
    return config


def small_components(mask, minimum=3, connectivity=8):
    """Return a mask of occupied groups smaller than minimum, with no dilation."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2 or type(minimum) is not int or minimum < 1 or connectivity not in (4, 8):
        raise ValueError('Invalid component-filter input')
    remove = np.zeros_like(mask)
    if minimum == 1:
        return remove, 0
    remaining = set(map(tuple, np.argwhere(mask)))
    offsets = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1)
               if (dy or dx) and (connectivity == 8 or abs(dy)+abs(dx) == 1)]
    groups = 0
    while remaining:
        first = remaining.pop()
        stack, group = [first], [first]
        while stack:
            row, col = stack.pop()
            for dy, dx in offsets:
                cell = row+dy, col+dx
                if cell in remaining:
                    remaining.remove(cell)
                    stack.append(cell)
                    group.append(cell)
        if len(group) < minimum:
            rows, cols = np.asarray(group).T
            remove[rows, cols] = True
            groups += 1
    return remove, groups


def clean_image(image, config=None):
    """Only selected occupied pixels become free; unknown pixels are untouched."""
    config = settings() if config is None else config
    removed, groups = small_components(np.asarray(image) <= 10,
                                      config['minimal_group_size'],
                                      config['group_connectivity_type'])
    result = np.asarray(image).copy()
    result[removed] = 254
    return result, dict(removed_cells=int(removed.sum()), removed_groups=groups,
                       retained_occupied_cells=int(np.count_nonzero(result <= 10)),
                       **config)


def clean_directory(directory, config=None):
    directory = Path(directory)
    nav = yaml.safe_load((directory/'nav.yaml').read_text())
    path = (directory/nav['image']).resolve()
    if path.parent != directory.resolve():
        raise ValueError('Navigation image must be inside its map directory')
    raw = path.read_bytes().split(b'\n', 3)
    width, height = map(int, raw[1].split())
    if raw[0] != b'P5' or raw[2] != b'255' or len(raw[3]) != width*height:
        raise ValueError('Unsupported navigation PGM; original map must be retained')
    if abs(nav['origin'][2]) > 1e-8:
        raise ValueError('Rotated grid origin unsupported')
    image = np.frombuffer(raw[3], dtype=np.uint8).reshape(height, width)
    clean, report = clean_image(image, config)
    path.write_bytes(b'\n'.join(raw[:3])+b'\n'+clean.tobytes())
    # Record the cleaned observed navigation area so a subsequent repair does
    # not throw away newly permitted cells. PCD remains the full 3D source.
    rows, cols = np.nonzero(clean >= 250)
    lower = np.rint(np.asarray(nav['origin'][:2])/nav['resolution']).astype(int)
    cells = np.column_stack((cols+lower[0], height-1-rows+lower[1])).astype(np.int32)
    np.savez_compressed(directory/'observed_free.npz', cells=cells)
    return report
