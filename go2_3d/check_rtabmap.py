#!/usr/bin/env python3
"""Read calibration and probe the installed backend before stopping live sensors.

The probe uses a private ROS domain and temporary database, receives no robot
data and never broadcasts TF or enables motion. A package version alone is not
enough: query the actual node's declared ROS/core parameters too.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET


def check_versions():
    from ament_index_python.packages import get_package_share_directory
    versions = {}
    for package in ('rtabmap_slam', 'rtabmap_msgs'):
        manifest = Path(get_package_share_directory(package)) / 'package.xml'
        version = ET.parse(manifest).getroot().findtext('version', '')
        try:
            supported = tuple(int(part) for part in version.split('.')[:3]) >= (0, 22, 1)
        except ValueError:
            supported = False
        if not supported:
            raise RuntimeError(f'{package} {version}: integration requires >= 0.22.1')
        versions[package] = version
    return versions


def probe_backend():
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType

    expected = {'gen_depth': True, 'subscribe_rgb': True, 'subscribe_depth': False,
                'subscribe_scan_cloud': True, 'subscribe_odom': True,
                'publish_tf': False, 'approx_sync': False, 'Reg/Strategy': '2'}
    # Domain in 100..199 is below the usual Linux ephemeral-port range.
    os.environ['ROS_DOMAIN_ID'] = str(100 + uuid.uuid4().int % 100)
    with tempfile.TemporaryDirectory(prefix='go2-rtabmap-probe-') as directory:
        args = ['ros2', 'run', 'rtabmap_slam', 'rtabmap', '--ros-args',
                '-r', '__node:=go2_rtabmap_probe',
                '-p', f'database_path:={directory}/probe.db']
        for name, value in expected.items():
            # Core parameters must remain strings even when they look numeric.
            value = f'"{value}"' if isinstance(value, str) else str(value).lower()
            args += ['-p', f'{name}:={value}']
        log_path = Path(directory) / 'probe.log'
        with log_path.open('w+') as log:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            node = None
            try:
                rclpy.init()
                node = rclpy.create_node('go2_rtabmap_preflight')
                client = node.create_client(GetParameters, '/go2_rtabmap_probe/get_parameters')
                deadline = time.monotonic() + 25.0
                while not client.wait_for_service(timeout_sec=0.25):
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Backend parameter service failed to start')
                future = client.call_async(GetParameters.Request(names=list(expected)))
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if not future.done() or future.exception() is not None:
                    raise RuntimeError('Backend parameter probe timed out or failed')
                values = future.result().values
                if len(values) != len(expected):
                    raise RuntimeError('Backend parameter response is incomplete')
                for (name, wanted), actual in zip(expected.items(), values):
                    kind = (ParameterType.PARAMETER_STRING if isinstance(wanted, str)
                            else ParameterType.PARAMETER_BOOL)
                    observed = actual.string_value if isinstance(wanted, str) else actual.bool_value
                    if actual.type != kind or observed != wanted:
                        raise RuntimeError(f'Installed backend does not support configured {name}={wanted}')
            except Exception as error:
                log.flush()
                tail = log_path.read_text(errors='replace')[-6000:]
                raise RuntimeError(f'{error}\nBackend probe log:\n{tail}') from error
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--urdf')
    args = parser.parse_args()
    from camera_calibration import load_calibration, validate_input_topic
    calibration = load_calibration(args.calibration)
    validate_input_topic(calibration, '/camera/image/compressed')
    if args.urdf:
        names = {link.attrib.get('name') for link in ET.parse(args.urdf).getroot().findall('link')}
        if calibration.optical_frame in names:
            raise RuntimeError('Calibrated optical frame already exists in robot URDF; '
                               'choose a unique measured optical frame to avoid duplicate TF')
    versions = check_versions()
    probe_backend()
    print('RTAB-Map preflight passed: calibration, message packages and live parameter API; '
          + ', '.join(f'{name}={version}' for name, version in versions.items()))


if __name__ == '__main__':
    main()
