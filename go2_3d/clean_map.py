#!/usr/bin/env python3
"""Copy the current map and remove tiny navigation obstacles; retain full PCD."""
import argparse
from pathlib import Path
import shutil
import yaml
from denoise import clean_directory
from map_library import inside, new_directory, publish, source_name


def clean(root):
    root = Path(root).resolve()
    source = inside(root,(root/'latest').resolve(strict=True))
    destination = new_directory(root,source_name(root,source),'cleaned-')
    try:
        shutil.copytree(source, destination, symlinks=False)
        # This is a new derived version, not the byte-identical archive copy.
        (destination/'archive.yaml').unlink(missing_ok=True)
        report = clean_directory(destination)
        metadata = yaml.safe_load((destination/'metadata.yaml').read_text())
        metadata.update(navigation_denoise=report, cleaned_from=str(source))
        (destination/'metadata.yaml').write_text(yaml.safe_dump(metadata))
        publish(root,destination)
    except Exception:
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
