"""Run unmodified native NavFn against old and corrected sparse-floor grids."""
import sys
import subprocess
import tempfile
from unittest.mock import patch
from pathlib import Path
import numpy as np
import yaml
from test_free_space import sparse_corridor,old_centres
from free_space import visible_free
from level_map import level
from repair_map import repair

with tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);sparse_corridor(root)
    with patch('level_map.resample_free',side_effect=old_centres):
        broken=level(root)
    meta=yaml.safe_load((broken/'metadata.yaml').read_text());meta.pop('free_area_version')
    (broken/'metadata.yaml').write_text(yaml.safe_dump(meta))
    fixed=repair(root)
    nav,known=visible_free(broken);rows,cols=np.nonzero(known)
    xy=(np.column_stack((cols,rows))+0.5)*nav['resolution']+nav['origin'][:2]
    start=xy[np.argmin(np.linalg.norm(xy-[0.475,0.475],axis=1))]
    goal=xy[np.argmin(np.linalg.norm(xy-[1.475,0.475],axis=1))]
    for directory,expected in ((broken,0),(fixed,1)):
        nav,_=visible_free(directory)
        cells=np.floor((np.array([start,goal])-nav['origin'][:2])/nav['resolution']).astype(int)
        subprocess.run([sys.argv[1],str(directory/'nav.pgm'),*map(str,cells.ravel()),str(expected)],check=True)
