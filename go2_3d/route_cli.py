#!/usr/bin/env python3
"""Save and reuse local patrol routes through the existing navigation queue."""
import argparse
import json
import sys
import time
import uuid

from route_store import RouteStore, route_name, validate_route


def exchange(request, timeout=15.0):
    """One request, one correlated receipt; never retry a motion request blindly."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node('go2_route_client_' + uuid.uuid4().hex[:8])
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    reply = None
    sent = False

    def receive(message):
        nonlocal reply
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if (isinstance(value, dict) and value.get('request_id') == request['request_id']
                and isinstance(value.get('success'), bool)):
            reply = value

    publisher = node.create_publisher(String, '/navigation/route/command', qos)
    subscription = node.create_subscription(String, '/navigation/route/result', receive, qos)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not sent:
                if not publisher.get_subscription_count() or not node.count_publishers('/navigation/route/result'):
                    continue
                publisher.publish(String(data=json.dumps(request, allow_nan=False)))
                sent = True
            if reply is not None:
                if not reply['success']:
                    raise ValueError(reply.get('message', 'Route request rejected'))
                return reply
        if sent:
            raise TimeoutError('No receipt for request ' + request['request_id'] +
                '; the command may have been accepted. Check /operator/status before repeating it.')
        raise TimeoutError('Route interface unavailable. Start navigation with the updated code; '
                           'restart it once after installing this feature. No route was sent.')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default='/maps/routes', help='Persistent local route directory')
    parser.add_argument('--timeout', type=float, default=15.0, help='ROS discovery/receipt timeout in seconds')
    commands = parser.add_subparsers(dest='command', required=True)
    save = commands.add_parser('save', help='Save all selected points, including already completed ones')
    save.add_argument('name')
    save.add_argument('--overwrite', action='store_true', help='Explicitly replace this saved route')
    for command in ('load', 'run', 'show'):
        commands.add_parser(command).add_argument('name')
    commands.add_parser('list')
    args = parser.parse_args(argv)
    if not 1.0 <= args.timeout <= 60.0:
        parser.error('--timeout must be between 1 and 60 seconds')
    store = RouteStore(args.directory)
    try:
        if args.command != 'list':
            route_name(args.name)
        if args.command == 'list':
            entries = store.list_routes()
            if not entries:
                print('No saved routes. Select points, then run tools.sh route-save NAME.')
            for entry in entries:
                print(f'{entry["name"]}: {entry["waypoints"]} points; map={entry["map"]}')
            return 0
        if args.command == 'show':
            print(json.dumps(store.load(args.name), ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        request = dict(request_id=str(uuid.uuid4()),
                       command='snapshot' if args.command == 'save' else args.command)
        if args.command in ('load', 'run'):
            request['route'] = store.load(args.name)
        result = exchange(request, args.timeout)
        if args.command == 'save':
            route = validate_route(result.get('route'))
            path = store.save(args.name, route, overwrite=args.overwrite)
            print(f'SAVED {args.name}: {len(route["waypoints"])} points; {path}; '
                  'local file survives navigation restarts and does not enable motion.')
        elif args.command == 'load':
            print(f'LOADED {args.name}: {result.get("message", "route queued")}. '
                  'Motion was not enabled; use tools.sh enable when ready.')
        else:
            print(f'START REQUESTED {args.name}: {result.get("message", "route loaded")}. '
                  'Existing localization, motion and obstacle checks decide dispatch; '
                  'follow /operator/status for execution results.')
        return 0
    except (ValueError, OSError, TimeoutError) as error:
        print('Route command failed: ' + str(error), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Route client interrupted. This does not cancel an accepted route; '
              'use tools.sh disable to stop.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
