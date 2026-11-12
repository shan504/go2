"""Wait for an offline mapper and save its completed revision; no motion API."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait-ready', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('go2_offline_map_save')
    client = node.create_client(Trigger, '/save_3d_map')
    deadline = time.monotonic()+args.timeout
    report = dict(success=False, message='mapping service unavailable')
    try:
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=1.0):
            pass
        if not client.service_is_ready():
            raise RuntimeError(report['message'])
        if args.wait_ready:
            return
        while time.monotonic() < deadline:
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline-time.monotonic()))
            if not future.done():
                raise RuntimeError('map save did not finish before the replay deadline')
            response = future.result()
            report = dict(success=response.success, message=response.message)
            if response.success:
                break
            # Final KISS partial-submap check/rebuild runs asynchronously. A
            # failure never becomes an unoptimized successful export.
            if 'pending or failed' not in response.message:
                raise RuntimeError(response.message)
            rclpy.spin_once(node, timeout_sec=0.5)
        if not report['success']:
            raise RuntimeError(report['message'])
        print(json.dumps(report))
    finally:
        if args.output:
            args.output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
