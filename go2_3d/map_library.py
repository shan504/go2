#!/usr/bin/env python3
"""Versioned named map bundles. Archive copies; originals are never removed."""
import argparse
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import re
import shutil
import uuid
import yaml


def map_name(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}',value):
        raise ValueError('Map name must be 1..64 ASCII letters/digits/underscore/hyphen, starting with a letter or digit')
    return value


def inside(root,path):
    root, path = Path(root).resolve(),Path(path).resolve()
    if root not in path.parents:
        raise ValueError(f'Map path must stay inside {root}: {path}')
    return path


def validate_bundle(directory):
    directory = Path(directory).resolve(strict=True)
    for filename in ('map.pcd','nav.yaml','metadata.yaml'):
        path = inside(directory,directory/filename)
        if not path.is_file() or not path.stat().st_size:
            raise ValueError(f'Incomplete map bundle: {path}')
    nav = yaml.safe_load((directory/'nav.yaml').read_text())
    metadata = yaml.safe_load((directory/'metadata.yaml').read_text())
    if not isinstance(nav,dict) or not isinstance(metadata,dict):
        raise ValueError('Map configuration and metadata must be mappings')
    image = Path(nav.get('image',''))
    if not image.name or image.is_absolute():
        raise ValueError('Map image must be relative to its bundle')
    if not inside(directory,directory/image).is_file():
        raise ValueError(f'Map image missing: {image}')
    return directory


def atomic_link(target,link):
    target, link = Path(target),Path(link)
    link.parent.mkdir(parents=True,exist_ok=True)
    if link.exists() and not link.is_symlink():
        raise ValueError(f'Refusing to replace a real directory/file: {link}')
    temporary = link.parent/('.latest-'+uuid.uuid4().hex)
    try:
        temporary.symlink_to(os.path.relpath(target,link.parent))
        os.replace(temporary,link)
    finally:
        temporary.unlink(missing_ok=True)


def new_directory(root,name='default',prefix=''):
    root = Path(root).resolve()
    parent = inside(root,root/'library'/map_name(name))
    return parent/(prefix+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))


def source_name(root,source):
    relative = inside(root,source).relative_to(Path(root).resolve())
    return map_name(relative.parts[1]) if len(relative.parts)==3 and relative.parts[0]=='library' else 'default'


def publish(root,directory):
    root = Path(root).resolve()
    directory = validate_bundle(inside(root,directory))
    if directory.parent.parent == root/'library':
        atomic_link(directory,directory.parent/'latest')
    atomic_link(directory,root/'latest')


def fingerprints(directory):
    directory = Path(directory).resolve()
    values = {}
    for path in sorted(directory.rglob('*')):
        inside(directory,path)
        if path.is_file() and path.name != 'archive.yaml':
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024),b''):
                    digest.update(block)
            values[str(path.relative_to(directory))] = digest.hexdigest()
    return values


def archive(root,name,all_legacy=False):
    root = Path(root).resolve()
    name = map_name(name)
    current = validate_bundle(inside(root,(root/'latest').resolve(strict=True)))
    sources = [current]
    if all_legacy:
        for candidate in sorted(root.iterdir()):
            if candidate.is_dir() and not candidate.is_symlink() and candidate.name != 'library' and not candidate.name.startswith('.'):
                try:
                    validate_bundle(candidate)
                except (ValueError,FileNotFoundError,yaml.YAMLError):
                    continue
                if candidate != current:
                    sources.append(candidate)
    parent = inside(root,root/'library'/name)
    parent.mkdir(parents=True,exist_ok=True)
    selected = None
    for source in sources:
        hashes = fingerprints(source)
        destination = parent/source.name
        if destination.exists():
            if fingerprints(destination) != hashes:
                raise ValueError(f'Archive version conflict; original preserved: {destination}')
        else:
            partial = parent/('.archive-'+uuid.uuid4().hex)
            try:
                shutil.copytree(source,partial)
                validate_bundle(partial)
                if fingerprints(partial) != hashes:
                    raise ValueError('Archive copy verification failed')
                (partial/'archive.yaml').write_text(yaml.safe_dump(dict(source=str(source),sha256=hashes)))
                os.rename(partial,destination)
            finally:
                if partial.exists():
                    shutil.rmtree(partial)
        if source == current:
            selected = destination
        print(f'ARCHIVED {name}: {destination}; original retained: {source}',flush=True)
    atomic_link(selected,parent/'latest')
    print(f'Current loaded map unchanged: {current}',flush=True)
    return selected


def select(root,name,version='latest'):
    root = Path(root).resolve()
    name = map_name(name)
    if version != 'latest':
        map_name(version.replace('.','_'))
    directory = validate_bundle(inside(root,root/'library'/name/version))
    if directory.parent != (root/'library'/name).resolve():
        raise ValueError('Selected version must belong to the named map')
    publish(root,directory)
    print(f'SELECTED {name}: {directory}; initialize localization in this map before navigation',flush=True)
    return directory


def list_maps(root):
    root = Path(root).resolve()
    current = (root/'latest').resolve()
    print(f'Selected map: {current}')
    library = root/'library'
    if not library.exists():
        print('No named maps yet; tools.sh archive-map indoor --all imports retained legacy versions')
        return
    for parent in sorted(library.iterdir()):
        if not parent.is_dir() or parent.is_symlink():
            continue
        for directory in sorted(parent.iterdir()):
            if directory.is_symlink() or not directory.is_dir() or directory.name.startswith('.'):
                continue
            try:
                validate_bundle(inside(root,directory))
            except (ValueError,FileNotFoundError,yaml.YAMLError):
                continue
            tags = []
            if directory == current:
                tags.append('SELECTED')
            if directory == (parent/'latest').resolve():
                tags.append('named latest')
            print(f'{parent.name}/{directory.name} {", ".join(tags)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps',default='/maps')
    actions = parser.add_subparsers(dest='action',required=True)
    item = actions.add_parser('archive')
    item.add_argument('name',type=map_name)
    item.add_argument('--all',action='store_true',dest='all_legacy')
    item = actions.add_parser('select')
    item.add_argument('name',type=map_name)
    item.add_argument('version',nargs='?',default='latest')
    actions.add_parser('list')
    args = parser.parse_args()
    try:
        if args.action == 'archive':
            archive(args.maps,args.name,args.all_legacy)
        elif args.action == 'select':
            select(args.maps,args.name,args.version)
        else:
            list_maps(args.maps)
    except (ValueError,OSError,yaml.YAMLError) as error:
        parser.exit(1,f'{error}\n')


if __name__ == '__main__':
    main()
