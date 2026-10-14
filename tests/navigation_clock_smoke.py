"""Real ROS startup with bounded future TF skew; no robot or motion writes."""
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from nav2_ready import Nav2Ready


def check(label, position, offsets):
    rclpy.init()
    gate, source = Nav2Ready(), Node('clock_skew_sensors')
    validity = source.create_publisher(Bool, '/localization/valid', 10)
    broadcaster = TransformBroadcaster(source)
    calls = []
    state = dict(valid=False, publish_valid=True, offsets=offsets, samples=0)

    def startup(request, response):
        calls.append(request.command)
        response.success = True
        return response

    source.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', startup)

    def emit():
        if state['publish_valid']:
            validity.publish(Bool(data=state['valid']))
        values = state['offsets']
        offset = values[state['samples'] % len(values)]
        state['samples'] += 1
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.header.stamp = Time(nanoseconds=source.get_clock().now().nanoseconds+
                                     int(offset*1e9)).to_msg()
        transform.transform.translation.x, transform.transform.translation.y = position
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)

    source.create_timer(0.02, emit)
    executor = SingleThreadedExecutor()
    for node in (gate, source):
        executor.add_node(node)

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    def wait(predicate, seconds=3):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError(f'{label}: startup did not recover from bounded TF clock skew')

    try:
        wait(lambda: gate.received is not None and gate.client.service_is_ready() and
             gate.buffer.can_transform('map', 'base_footprint', Time()))
        spin(0.7)
        assert not calls, 'Fresh TF authorized startup without valid localization'
        state.update(valid=True, offsets=[-0.60])
        spin(0.8)
        assert not calls and gate.ready_since is None, 'Stale TF authorized startup'
        state['offsets'] = [0.20]
        spin(0.8)
        assert not calls and gate.ready_since is None, 'Excessively future TF authorized startup'
        state['publish_valid'] = False
        spin(0.6)
        state['offsets'] = offsets
        spin(0.8)
        assert not calls, 'Fresh TF authorized startup with expired localization validity'
        state['publish_valid'] = True
        beginning = time.monotonic()
        wait(lambda: gate.started)
        assert time.monotonic()-beginning >= 0.45, 'Continuous readiness settling was bypassed'
        assert calls == [ManageLifecycleNodes.Request.STARTUP]
        spin(0.4)
        assert len(calls) == 1, 'Startup repeated after successful activation'
        print(f'PASS {label}: future TF up to80ms and mixed clock jitter start once after0.5s; '
              'invalid/stale localization, stale TF and200ms future TF remain blocked', flush=True)
    finally:
        executor.shutdown()
        gate.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    check('origin-map', (0.0, 0.0), [0.05, 0.08, 0.06])
    check('translated-map', (12.0, -8.0), [-0.02, 0.021, 0.012, 0.07, 0.005])
