"""Exercise real launcher preflight/order for all modes; Docker is recorded only."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

project = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
    root = Path(tmp)
    checkout = root/'project with spaces'
    (checkout/'go2_3d').mkdir(parents=True)
    shutil.copy2(project/'go2_3d/run.sh', checkout/'go2_3d/run.sh')
    maps = checkout/'maps/latest'; maps.mkdir(parents=True)
    for name in ('map.pcd', 'nav.yaml', 'metadata.yaml'):
        (maps/name).write_text('fixture')
    dds = root/'cyclonedds.xml'; dds.write_text('<CycloneDDS/>')
    commands = root/'commands'; commands.mkdir()
    sudo = commands/'sudo'
    sudo.write_text('''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
with open(os.environ['GO2_LAUNCH_LOG'], 'a') as log:
    log.write(json.dumps(args)+'\\n')
if args[:3] == ['docker', 'container', 'inspect']:
    sys.exit(0 if args[-1] == 'go2-sdk' else 1)
if any('prepare.py' in arg for arg in args):
    sys.exit(int(os.environ['GO2_PREPARE_EXIT']))
sys.exit(0)
''')
    sudo.chmod(0o755)
    logfile = root/'commands.jsonl'
    environment = dict(os.environ, PATH=str(commands)+os.pathsep+os.environ['PATH'],
                       GO2_LAUNCH_LOG=str(logfile), GO2_PREPARE_EXIT='0', GO2_DDS_FILE=str(dds))
    for mode in ('mapping', 'localization', 'navigation'):
        for profile in ('standard', 'dense'):
            logfile.write_text('')
            result = subprocess.run(['bash', str(checkout/'go2_3d/run.sh'), mode, profile],
                                    env=environment, capture_output=True, text=True, timeout=5)
            assert result.returncode == 0, (mode, profile, result.stderr)
            calls = [json.loads(line) for line in logfile.read_text().splitlines()]
            preparation = next(i for i, call in enumerate(calls) if any('prepare.py' in arg for arg in call))
            prepare = calls[preparation]
            assert prepare[-2:] == ['bash', mode]
            assert '--mode "$1"' in prepare[-3]
            assert f'{checkout}/maps:/maps:ro' in prepare
            assert f'{checkout}/runtime:/runtime:rw' in prepare
            stop = next(i for i, call in enumerate(calls) if call[:2] == ['docker', 'stop'])
            assert preparation < stop, 'Map preparation must succeed before stopping sensors'
            launch_arguments = calls[-1][calls[-1].index('bash')+1:]
            assert launch_arguments[:2] == [mode, '/opt/go2_project/go2_3d/'+
                                           ('gicp_dense.yaml' if profile == 'dense' else 'gicp.yaml')]
            assert launch_arguments[2:] == ['true','0.0','0.0','0.0','0.0','default','true']
            assert 'initial_from_map_start:="$9"' in calls[-1][calls[-1].index('bash')-1]
            assert ('no /initialpose clicks required' in result.stdout) == (mode != 'mapping')
            if mode != 'mapping':
                for coordinates in (['1.25','-0.4','35'], ['1.25','-0.4','35','0.37']):
                    logfile.write_text('')
                    result = subprocess.run(['bash',str(checkout/'go2_3d/run.sh'),mode,profile,*coordinates],
                                            env=environment,capture_output=True,text=True,timeout=5)
                    assert result.returncode == 0,(mode,profile,result.stderr)
                    explicit = [json.loads(line) for line in logfile.read_text().splitlines()][-1]
                    explicit_arguments = explicit[explicit.index('bash')+1:]
                    assert explicit_arguments[2:] == ['true',*coordinates[:3],
                                                      coordinates[3] if len(coordinates)==4 else '0.0',
                                                      'default','false']
                    assert 'no /initialpose clicks required' not in result.stdout
    environment['GO2_PREPARE_EXIT'] = '1'
    logfile.write_text('')
    result = subprocess.run(['bash', str(checkout/'go2_3d/run.sh'), 'navigation', 'dense'],
                            env=environment, capture_output=True, text=True, timeout=5)
    calls = [json.loads(line) for line in logfile.read_text().splitlines()]
    assert result.returncode == 1
    assert not any(call[:2] == ['docker', 'stop'] or '--name' in call for call in calls)
print('PASS shared map startup: mapping/localization/navigation standard+dense; '
      'selected-map start automatic initialization and explicit XYZ/yaw override; '
      'current policy preflight precedes sensor stop; failed grid preparation prevents launch')
