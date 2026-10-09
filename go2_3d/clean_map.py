#!/usr/bin/env python3
"""Copy the current map and remove tiny navigation obstacles; retain full PCD."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import shutil
import yaml
from denoise import clean_directory


def clean(root):
    root = Path(root).resolve()
    source = (root/'latest').resolve(strict=True)
    if source.parent != root:
        raise ValueError('latest must refer to a map directory inside the maps root')
    destination = root/('cleaned-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    link = root/'.latest-clean'
    try:
        shutil.copytree(source, destination, symlinks=False)
        report = clean_directory(destination)
        metadata = yaml.safe_load((destination/'metadata.yaml').read_text())
        metadata.update(navigation_denoise=report, cleaned_from=str(source))
        (destination/'metadata.yaml').write_text(yaml.safe_dump(metadata))
        if link.is_symlink():
            link.unlink()
        link.symlink_to(destination.name)
        os.replace(link, root/'latest')
    except Exception:
        if link.is_symlink():
            link.unlink()
        if destination.exists():
            shutil.rmtree(destination)
        raise
    print(f'CLEANED navigation grid: {destination}\nOriginal retained: {source}\n'
          f'Denoise: {report}\nPCD and map coordinates unchanged; select a new goal after restart.', flush=True)
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps', default='/maps')
    clean(parser.parse_args().maps)
