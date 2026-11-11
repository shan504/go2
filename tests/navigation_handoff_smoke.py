"""Real ROS action/parameter handoffs after SUCCEEDED, without robot motion.

Only fault injection uses mocks: unavailable/late bridge replies and an optional
diagnostic exception. Goals, terminal results, cancellation, TF, costmaps and
localization recovery use the real ROS endpoints from the existing smoke tests.
"""
import copy
import threading
import time
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty

from automatic_resume_smoke import Operator, RecoveryEndpoints, wait_for


def main():
    rclpy.init(args=['--ros-args', '-p', 'mode:=navigation'])
    operator, endpoints = Operator(), RecoveryEndpoints()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (operator, endpoints):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    template = PoseStamped()
    template.header.frame_id = 'map'
    template.pose.orientation.w = 1.

    def start(xs=(.4, .8, 1.2)):
        endpoints.recovery_event = None
        endpoints.respond_to_revalidation = False
        endpoints.pending_revalidation_response = None
        before = endpoints.goals
        for index, x in enumerate(xs, 1):
            message = copy.deepcopy(template)
            message.pose.position.x = x
            endpoints.goal.publish(message)
            wait_for(lambda: len(operator.waypoints) == index)
        time.sleep(.15)
        assert endpoints.goals == before, 'Collecting a new route started motion'
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals == before + 1 and operator.goal_handle is not None)
        return before + 1

    def finish(index, total):
        endpoints.outcomes[index] = 4
        wait_for(lambda: operator.completed_waypoints == total and
                 operator.goal_handle is None and not operator.goal_pending)
        assert not operator.waypoints
        assert not operator.route_started

    def tf_age():
        transform = operator.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        return (operator.get_clock().now() -
                rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9

    def seen_since(offset, fragment):
        return any(fragment in text for _, text in endpoints.operator_messages[offset:])

    def no_extra_goal(count, seconds=.7):
        time.sleep(seconds)
        assert endpoints.goals == count, (endpoints.goals, count, endpoints.positions)

    def enabled_reply():
        return GetParameters.Response(values=[
            Parameter('enable_control', value=True).get_parameter_value()])

    try:
        endpoints.valid = True
        wait_for(lambda: operator.localized() and len(operator.grids) == 3 and
                 operator.action.server_is_ready() and operator.motion_state.service_is_ready() and
                 all(client.service_is_ready() for client in operator.costmap_clear.values()))
        wait_for(lambda: operator.buffer.can_transform('map', 'base_footprint', rclpy.time.Time()))
        wait_for(lambda: 0 <= tf_age() < .2)

        # One Enable must carry three ordered goals through two independent
        # temporary handoff failures. Never replay the already successful goal.
        first = start()
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        endpoints.outcomes[first] = 4
        wait_for(lambda: operator.completed_waypoints == 1 and operator.goal_handle is None)
        wait_for(lambda: seen_since(offset, 'TF stale'))
        no_extra_goal(first)
        endpoints.tf_enabled = True
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        endpoints.grid_enabled['/global_costmap/costmap'] = False
        wait_for(lambda: time.monotonic() - operator.grids['/global_costmap/costmap'][1] > 3.1)
        offset = len(endpoints.operator_messages)
        endpoints.outcomes[first + 1] = 4
        wait_for(lambda: operator.completed_waypoints == 2 and operator.goal_handle is None)
        wait_for(lambda: seen_since(offset, '/global_costmap/costmap stale'))
        no_extra_goal(first + 1)
        endpoints.grid_enabled['/global_costmap/costmap'] = True
        wait_for(lambda: endpoints.goals == first + 2 and operator.goal_handle is not None)
        finish(first + 2, 3)
        assert endpoints.positions[first - 1:] == [.4, .8, 1.2]
        print('PASS one Enable: three ordered successes across stale TF and stale costmap recovery', flush=True)

        # Discovery loss at a handoff must retain the tail until the existing
        # bridge service is visible again; it must not discard the route.
        first = start((.4, .8))
        offset = len(endpoints.operator_messages)
        with patch.object(operator.motion_state, 'service_is_ready', return_value=False):
            endpoints.outcomes[first] = 4
            wait_for(lambda: operator.completed_waypoints == 1 and operator.goal_handle is None)
            wait_for(lambda: seen_since(offset, 'state service unavailable'))
            no_extra_goal(first)
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        finish(first + 1, 2)
        assert endpoints.positions[first - 1:] == [.4, .8]

        # A timed-out readback is superseded by a fresh query. Its late True
        # response must neither dispatch concurrently nor duplicate the tail.
        first = start((.4, .8))
        pending = Future(executor=executor)
        original_call = operator.motion_state.call_async
        calls = []

        def delay_first(request):
            calls.append(request)
            return pending if len(calls) == 1 else original_call(request)

        offset = len(endpoints.operator_messages)
        with patch.object(operator.motion_state, 'call_async', side_effect=delay_first):
            endpoints.outcomes[first] = 4
            wait_for(lambda: len(calls) == 1)
            wait_for(lambda: seen_since(offset, 'timed out'))
            wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
            pending.set_result(enabled_reply())
            no_extra_goal(first + 1)
            finish(first + 1, 2)
        assert endpoints.positions[first - 1:] == [.4, .8]
        print('PASS bridge discovery/query recovery preserves order; late timed-out reply is inert', flush=True)

        # Cancel while a readback is pending. A late enabled reply is not new
        # user authorization; no queued waypoint may escape cancellation.
        first = start((.4, .8))
        pending = Future(executor=executor)
        with patch.object(operator.motion_state, 'call_async', return_value=pending) as delayed:
            endpoints.outcomes[first] = 4
            wait_for(lambda: delayed.call_count == 1)
            endpoints.cancel.publish(Empty())
            wait_for(lambda: not operator.waypoints and not operator.route_started)
            pending.set_result(enabled_reply())
            no_extra_goal(first)

        # Cancel an actual watchdog retry as well as the pending-query case.
        first = start((.4, .8))
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        endpoints.outcomes[first] = 4
        wait_for(lambda: seen_since(offset, 'TF stale'))
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_started)
        endpoints.tf_enabled = True
        wait_for(lambda: 0 <= tf_age() < .2)
        no_extra_goal(first)
        print('PASS cancellation revokes pending replies and scheduled next-waypoint retries', flush=True)

        # Localization loss BETWEEN goals retains only the unvisited tail.
        # Fresh validity and TF alone are insufficient; require new multi-scan
        # proof, stable localization, bridge disable and both costmap clears.
        first = start()
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        endpoints.outcomes[first] = 4
        wait_for(lambda: seen_since(offset, 'TF stale'))
        endpoints.valid = False
        wait_for(lambda: operator.route_paused and operator.pause_disable_confirmed and
                 not endpoints.get_parameter('enable_control').value)
        assert [pose.pose.position.x for _, pose in operator.waypoints] == [.8, 1.2]
        assert operator.completed_waypoints == 1
        endpoints.valid = True
        endpoints.tf_enabled = True
        wait_for(operator.localized)
        no_extra_goal(first, 1.35)
        endpoints.event(endpoints.recovery_epoch + 1, confirmations=2)
        no_extra_goal(first, 1.35)
        endpoints.event(endpoints.recovery_epoch + 1, source_age=60.)
        no_extra_goal(first, 1.35)
        clears = dict(endpoints.clear_calls)
        endpoints.respond_to_revalidation = True
        endpoints.event(endpoints.recovery_epoch + 1)
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        assert all(endpoints.clear_calls[name] > clears[name] for name in clears)
        assert endpoints.get_parameter('enable_control').value
        endpoints.outcomes[first + 1] = 4
        wait_for(lambda: endpoints.goals == first + 2 and operator.goal_handle is not None)
        finish(first + 2, 3)
        assert endpoints.positions[first - 1:] == [.4, .8, 1.2]
        print('PASS handoff localization pause retains tail and requires verified recovery before automatic resume', flush=True)

        # Persistent missing readiness exhausts the bounded retry window.
        # Preserve the order, stop automatic attempts and require new intent.
        first = start()
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        endpoints.outcomes[first] = 4
        wait_for(lambda: seen_since(offset, 'Waypoint dispatch timed out'), seconds=20)
        assert operator.route_paused and not operator.route_started
        assert operator.completed_waypoints == 1
        assert [pose.pose.position.x for _, pose in operator.waypoints] == [.8, 1.2]
        endpoints.tf_enabled = True
        wait_for(lambda: 0 <= tf_age() < .2)
        no_extra_goal(first)
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_paused)
        print('PASS persistent unready handoff times out with remaining route intact and no automatic motion', flush=True)

        # Optional endpoint diagnostics cannot undo an acknowledged success.
        first = start((.4, .8))
        with patch.object(operator, 'goal_error_text', side_effect=RuntimeError('injected diagnostic failure')):
            endpoints.outcomes[first] = 4
            wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
            finish(first + 1, 2)
        assert endpoints.positions[first - 1:] == [.4, .8]
        assert endpoints.max_active == 1
        print('PASS diagnostic exception cannot replay success; all routes have one active action at most', flush=True)

        # An initial Enable waiting for bridge discovery has never executed a
        # goal. It does not inherit the recovery authority of a completed-leg
        # handoff: validity plus mapper proof cannot start this queued route.
        before = endpoints.goals
        endpoints.respond_to_revalidation = False
        endpoints.pending_revalidation_response = None
        endpoints.recovery_event = None
        message = copy.deepcopy(template)
        message.pose.position.x = .4
        endpoints.goal.publish(message)
        wait_for(lambda: len(operator.waypoints) == 1)
        offset = len(endpoints.operator_messages)
        with patch.object(operator.motion_state, 'service_is_ready', return_value=False):
            endpoints.enable.publish(Bool(data=True))
            wait_for(lambda: seen_since(offset, 'state service unavailable'))
            assert endpoints.goals == before
            endpoints.valid = False
            wait_for(lambda: operator.route_paused and operator.pause_disable_confirmed and
                     not endpoints.get_parameter('enable_control').value)
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.event(endpoints.recovery_epoch + 1)
        no_extra_goal(before, 1.35)
        assert not endpoints.get_parameter('enable_control').value
        assert len(operator.waypoints) == 1 and operator.completed_waypoints == 0
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_paused)
        print('PASS never-executed initial dispatch cannot acquire automatic localization recovery authority', flush=True)
    finally:
        endpoints.tf_enabled = True
        endpoints.grid_enabled['/global_costmap/costmap'] = True
        endpoints.accept_wait.set()
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
