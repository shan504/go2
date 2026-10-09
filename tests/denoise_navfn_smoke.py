"""Run upstream denoise + native NavFn; reproduce and open a speckled corridor."""
import sys
import subprocess
import tempfile
from pathlib import Path
from test_denoise import corridor, write_map
from denoise import clean_directory


with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    source = write_map(root, corridor())
    args = [sys.argv[1], str(source/'nav.pgm'), '15', '8', '65', '8']
    subprocess.run(args+['0'], check=True)
    subprocess.run(args+['1', '8'], check=True)
    # Offline /map cleaning must reproduce the same successful route as the
    # runtime DenoiseLayer algorithm, still with full body clearance.
    clean_directory(source)
    subprocess.run(args+['1'], check=True)
    # A coherent larger object still seals this narrow route.
    image = corridor(); image[7:10, 31:34] = 0
    other = root/'larger'; other.mkdir()
    large = write_map(other, image)
    subprocess.run([sys.argv[1], str(large/'nav.pgm'), '15', '8', '65', '8', '0', '8'], check=True)
    print('PASS upstream DenoiseLayer algorithm + NavFn: speckles removed, route restored; wall/larger object retained')
