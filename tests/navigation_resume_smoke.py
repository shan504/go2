"""Read-only pose capture against real ROS TF, and host restart argument wiring."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from unitree_api.msg import Request


class PoseSource(Node):
    def __init__(self):
        super().__init__('restart_pose_fixture')
        self.valid, self.stale = True, False
        self.writes = []
        self.validity = self.create_publisher(Bool, '/localization/valid', 10)
        self.transforms = TransformBroadcaster(self)
        self.create_subscription(Request, '/api/sport/request', self.writes.append, 10)
        self.create_subscription(Bool, '/control/enable', self.writes.append, 10)
        self.create_timer(0.02, self.publish)

    def publish(self):
        self.validity.publish(Bool(data=self.valid))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
        stamp = self.get_clock().now().to_msg()
        if self.stale:
            stamp.sec -= 3
        transform.header.stamp = stamp
        transform.transform.translation.x = 0.423
        transform.transform.translation.y = 0.025
        transform.transform.translation.z = 0.351
        transform.transform.rotation.z = math.sin(math.pi/12)
        transform.transform.rotation.w = math.cos(math.pi/12)
        self.transforms.sendTransform(transform)


project = Path(__file__).resolve().parents[1]
rclpy.init()
node = PoseSource()
executor = SingleThreadedExecutor()
executor.add_node(node)
thread = threading.Thread(target=executor.spin, daemon=True)
thread.start()
try:
    result = subprocess.run([sys.executable, str(project/'go2_3d/snapshot_pose.py')],
                            capture_output=True, text=True, timeout=9)
    assert result.returncode == 0, result.stderr
    pose = json.loads(result.stdout)
    assert abs(pose['x']-0.423) < 1e-6 and abs(pose['y']-0.025) < 1e-6
    assert abs(pose['yaw_degrees']-30) < 1e-6 and abs(pose['z']-0.351) < 1e-6
    for invalid, stale in ((True, False), (False, True)):
        node.valid, node.stale = not invalid, stale
        result = subprocess.run([sys.executable, str(project/'go2_3d/snapshot_pose.py')],
                                capture_output=True, text=True, timeout=9)
        assert result.returncode == 2 and not result.stdout.strip(), result
        assert 'Cannot retain pose' in result.stderr
    assert not node.writes, 'Pose capture wrote a robot/control request'
finally:
    executor.shutdown()
    thread.join(timeout=2)
    node.destroy_node()
    rclpy.shutdown()

# No real Docker, launcher or robot in this shell check. Intercept only the
# subprocess boundary and ensure finite captured pose reaches the real parser.
with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    fake_project = root/'project'
    (fake_project/'go2_3d').mkdir(parents=True)
    (fake_project/'go2_3d/tools.sh').write_text((project/'go2_3d/tools.sh').read_text())
    for command, content in {
        'sudo': '''#!/usr/bin/env python3
import json, os, sys, time
args = ' '.join(sys.argv[1:])
if 'inspect' in args: print('true')
elif 'snapshot_pose.py' in args:
    print(json.dumps(dict(x=0.423,y=0.025,z=0.351,yaw_degrees=30,captured_at=time.time())))
elif 'topic pub' in args: print('simulated motion disable')
else: raise SystemExit('Unexpected sudo invocation: '+args)
''',
        'bash': '''#!/usr/bin/env python3
import json, os, sys
if sys.argv[1].endswith('/go2_3d/run.sh'):
    open(os.environ['GO2_RESUME_ARGUMENTS'],'w').write(json.dumps(sys.argv[2:]))
else: os.execv('/bin/bash',['bash',*sys.argv[1:]])
''',
    }.items():
        script = root/command
        script.write_text(content)
        script.chmod(0o755)
    log = root/'arguments.json'
    environment = dict(os.environ, PATH=str(root)+os.pathsep+os.environ['PATH'],
                       GO2_RESUME_ARGUMENTS=str(log))
    result = subprocess.run(['/bin/bash', str(fake_project/'go2_3d/tools.sh'), 'restart-navigation'],
                            env=environment, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert json.loads(log.read_text()) == ['navigation','dense','0.423','0.025','30','0.351']
print('PASS navigation restart: valid current map XYZ/yaw, invalid/stale refusal without robot writes, retained numeric pose passed to launcher')
