#!/usr/bin/env python3
"""Create a local review bundle without changing maps or contacting the robot."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import zipfile


def export(project):
    project = Path(project).resolve()
    maps = project/'maps'
    source = (maps/'latest').resolve(strict=True)
    source.relative_to(maps.resolve())
    required = ['map.pcd','nav.pgm','nav.yaml','metadata.yaml']
    for name in required:
        if not (source/name).is_file():
            raise ValueError(f'Missing {source/name}')
    runtime = project/'runtime'
    runtime.mkdir(exist_ok=True)
    output = runtime/'go2-map-debug.zip'
    temporary = runtime/'go2-map-debug.tmp'
    entries = [(source/name,'map/'+name) for name in required]
    if (source/'observed_free.npz').is_file():
        entries.append((source/'observed_free.npz','map/observed_free.npz'))
    for name in ('nav2_3d.yaml','go2_edu.urdf'):
        path = runtime/'config'/name
        if path.is_file():
            entries.append((path,'config/'+name))
    log = runtime/'navigation-debug.log'
    if log.is_file():
        entries.append((log,'navigation-debug.log'))
    manifest = dict(created_utc=datetime.now(timezone.utc).isoformat(),map_directory=source.name,
                    files={name:hashlib.sha256(path.read_bytes()).hexdigest() for path,name in entries})
    with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for path,name in entries:
            archive.write(path,name)
        archive.writestr('manifest.json',json.dumps(manifest,indent=2))
    # Map files are often owned by the Docker root user. The bundle must be
    # downloadable by the SSH user after tools.sh reads them through sudo.
    temporary.chmod(0o644)
    temporary.replace(output)
    print(f'Map review bundle: {output} ({output.stat().st_size/1024/1024:.2f} MiB); original files unchanged')
    return output


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',default=str(Path(__file__).resolve().parents[1]))
    export(parser.parse_args().project)
