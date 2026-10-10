"""Exercise actual shell map commands with a recording Docker stub, no robot."""
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
    shutil.copy2(project/'go2_3d/tools.sh',checkout/'go2_3d/tools.sh')
    commands = root/'commands'; commands.mkdir()
    sudo = commands/'sudo'
    sudo.write_text('''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
with open(os.environ['GO2_MAP_COMMAND_LOG'],'a') as log:
    log.write(json.dumps(args)+'\\n')
if args[:3] == ['docker','container','inspect']:
    sys.exit(0 if os.environ['GO2_TEST_CONTAINER_RUNNING']=='yes' else 1)
sys.exit(0)
''')
    sudo.chmod(0o755)
    logfile = root/'commands.jsonl'
    environment = dict(os.environ,PATH=str(commands)+os.pathsep+os.environ['PATH'],
        GO2_MAP_COMMAND_LOG=str(logfile),GO2_TEST_CONTAINER_RUNNING='yes')
    def run(*args):
        logfile.write_text('')
        result = subprocess.run(['bash',str(checkout/'go2_3d/tools.sh'),*args],
            env=environment,capture_output=True,text=True,timeout=5)
        calls = [json.loads(line) for line in logfile.read_text().splitlines()]
        return result,calls
    result,calls = run('archive-map','indoor','--all')
    assert result.returncode==0,(result.stdout,result.stderr)
    assert calls[-1][-3:]==['archive','indoor','--all']
    assert f'{checkout}/maps:/maps:rw' in calls[-1]
    assert '--network' in calls[-1] and 'none' in calls[-1]
    result,calls = run('use-map','indoor')
    assert result.returncode==1 and len(calls)==1
    assert 'Ctrl+C' in result.stderr,'Switching live localization must be refused'
    environment['GO2_TEST_CONTAINER_RUNNING']='no'
    result,calls = run('use-map','indoor','20261010T100000.000000Z')
    assert result.returncode==0 and calls[-1][-3:]==['select','indoor','20261010T100000.000000Z']
    result,calls = run('maps')
    assert result.returncode==0 and calls[-1][-1]=='list'
    result,calls = run('archive-map')
    assert result.returncode==2 and not calls
    for command in ('resume','cancel'):
        result,calls = run(command)
        assert result.returncode==0 and calls[-1][-1]==command
        assert '/navigation/$1' in calls[-1][-3]
print('PASS map shell commands: complete archive arguments, paths with spaces, live-map switch refusal, offline selection/listing and explicit route resume/cancel')
