"""Conservative area resampling of known-free cells between ground-aligned maps."""
from pathlib import Path
import numpy as np
import yaml


def visible_free(source):
    source = Path(source)
    nav = yaml.safe_load((source/'nav.yaml').read_text())
    raw = (source/nav['image']).read_bytes().split(b'\n',3)
    width,height = map(int,raw[1].split())
    if raw[0]!=b'P5' or raw[2]!=b'255' or len(raw[3])!=width*height:
        raise ValueError('Unsupported source PGM; refusing to guess free areas')
    # PGM rows are top-down; navigation lattice rows are bottom-up. Use the
    # visible grid, not archive entries that were overridden by obstacles.
    known = np.frombuffer(raw[3],np.uint8).reshape(height,width)[::-1]>=250
    return nav,known


def resample_free(source, ground, transform, resolution, max_cells=4_000_000):
    """Keep a fine cell only when its old-grid bounding box is all known free.

    Old cells describe areas, not isolated samples. Inverse-project the full
    fine cell onto the old floor lattice; conservative bounding boxes also
    prevent rotation from clearing any previously unknown/occupied cell.
    """
    nav,known = visible_free(source)
    old_resolution = float(nav['resolution'])
    if not 0 < resolution <= old_resolution+1e-8:
        raise ValueError('Free-area transfer requires an equal or finer resolution')
    plane = np.asarray(ground['plane'],dtype=float)
    matrix = np.asarray(transform,dtype=float)
    if (plane.shape!=(4,) or matrix.shape!=(4,4) or not np.isfinite(plane).all() or
            not np.isfinite(matrix).all() or plane[2]<0.9):
        raise ValueError('Invalid ground/transform for free-area transfer')
    angle = float(nav['origin'][2])
    rotation = np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    floor_xy = matrix[:2,:2]-np.outer(matrix[:2,2],plane[:2]/plane[2])
    linear = floor_xy@rotation
    offset = (matrix[:2,3]-matrix[:2,2]*plane[3]/plane[2]+
              floor_xy@np.asarray(nav['origin'][:2]))
    inverse = np.linalg.inv(linear)
    rows,cols = np.nonzero(known)
    if not len(rows):
        return set()
    lower = np.array([cols.min(),rows.min()])*old_resolution
    upper = (np.array([cols.max(),rows.max()])+1)*old_resolution
    corners = np.array([[lower[0],lower[1]],[upper[0],lower[1]],
                        [lower[0],upper[1]],[upper[0],upper[1]]])@linear.T+offset
    lo = np.floor(corners.min(0)/resolution).astype(int)
    hi = np.floor(corners.max(0)/resolution).astype(int)
    width,height = hi-lo+1
    if width*height>max_cells:
        raise ValueError('Transferred grid bounds exceed memory limit')
    # Tiny inward offsets treat shared cell boundaries consistently despite
    # float32 PCD/ROS resolutions. They do not bridge gaps between cells.
    steps = np.array([[0.,0.],[1.,0.],[0.,1.],[1.,1.]])
    steps = (steps*(1-2e-7)+1e-7)*resolution
    result = set()
    for start in range(0,int(width*height),65536):
        indices = np.arange(start,min(start+65536,int(width*height)))
        cells = np.column_stack((indices%width+lo[0],indices//width+lo[1]))
        old = (cells[:,None,:]*resolution+steps-offset)@inverse.T/old_resolution
        a = np.floor(old.min(1)).astype(int)
        b = np.floor(old.max(1)).astype(int)
        good = ((a>=0).all(1)&(b[:,0]<known.shape[1])&(b[:,1]<known.shape[0]))
        span = (b-a).max(0)
        for dy in range(int(span[1])+1):
            for dx in range(int(span[0])+1):
                needed = (a[:,0]+dx<=b[:,0])&(a[:,1]+dy<=b[:,1])
                x,y = a[:,0]+dx,a[:,1]+dy
                inside = (x>=0)&(x<known.shape[1])&(y>=0)&(y<known.shape[0])
                free = np.zeros(len(cells),dtype=bool)
                free[inside] = known[y[inside],x[inside]]
                good &= ~needed|free
        result.update(map(tuple,cells[good]))
    return result


def restore_ancestor_free(root, metadata, grid):
    """Restore coarse known areas for maps made by the centre-only converter."""
    root = Path(root).resolve()
    current = metadata
    transform = np.eye(4)
    visited = set()
    restored = set()
    while current.get('ground_aligned') and current.get('leveled_from'):
        # Paths may name a legacy sibling or a version in library/NAME.
        # Remap the Docker /maps prefix when running an offline exported test.
        saved = Path(current['leveled_from'])
        if saved.is_absolute() and saved.parts[:2] == ('/','maps'):
            parent = root/Path(*saved.parts[2:])
        elif saved.is_absolute():
            parent = saved
        else:
            parent = root/saved
        parent = parent.resolve()
        if root not in parent.parents or parent in visited:
            raise ValueError('Invalid/cyclic retained source-map path')
        visited.add(parent)
        if not parent.is_dir():
            raise ValueError(f'Retained source map missing: {parent}; cannot restore old free areas')
        step = np.asarray(current['map_from_previous_map'],dtype=float)
        transform = transform@step
        previous = yaml.safe_load((parent/'metadata.yaml').read_text())
        ground = previous.get('ground_model',dict(plane=step[2].tolist()))
        restored.update(resample_free(parent,ground,transform,grid.resolution,grid.max_cells))
        current = previous
    grid.free.update(restored)
    return len(restored)
