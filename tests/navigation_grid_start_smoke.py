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
    for name in ('capture_navigation.py', 'fault_store.py'):
        shutil.copy2(project/'go2_3d'/name, checkout/'go2_3d'/name)
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
            assert launch_arguments[2:] == ['true','0.0','0.0','0.0','0.0','default','true','lidar','','']
            # Execute the actual shell command with harmless source/ros2 shims.
            # This catches empty ROS CLI values, unlike recording Docker alone.
            shell = calls[-1][calls[-1].index('-c')+1]
            capture = commands/'ros2'
            capture.write_text('#!/usr/bin/env python3\nimport json, sys\nprint(json.dumps(sys.argv[1:]))\n')
            capture.chmod(0o755)
            for calibration, session in (('', ''), ('', '/maps/session one'),
                                         ('/calibration/measured camera.yaml', ''),
                                         ('/calibration/measured camera.yaml', '/maps/session one')):
                args = launch_arguments[:-2] + [calibration, session]
                launched = subprocess.run(['bash', '-c', 'source() { :; }; '+shell, 'bash', *args],
                                          env=environment, capture_output=True, text=True, check=True)
                cli = json.loads(launched.stdout)
                assert all(not arg.endswith(':=') for arg in cli), cli
                assert ('camera_calibration:='+calibration in cli) == bool(calibration), cli
                assert ('session_directory:='+session in cli) == bool(session), cli
                assert 'loop_backend:=lidar' in cli
            failed_source = subprocess.run(['bash', '-c', 'source() { return 1; }; '+shell,
                                             'bash', *launch_arguments], env=environment,
                                            capture_output=True, text=True)
            assert failed_source.returncode != 0 and not failed_source.stdout
            assert 'initial_from_map_start:="$9"' in calls[-1][calls[-1].index('bash')-1]
            if mode == 'navigation':
                assert 'GO2_DIAGNOSTICS_SESSION=/diagnostics' in calls[-1]
                mounts = [calls[-1][i+1] for i, item in enumerate(calls[-1][:-1]) if item == '-v']
                assert any(value.endswith(':/diagnostics:rw') for value in mounts), mounts
                assert any(value.startswith('go2.diagnostics.session=') for value in calls[-1])
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
                                                      'default','false','lidar','','']
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
