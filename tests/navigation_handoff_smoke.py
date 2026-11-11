"""Real ROS waypoint handoffs, with controlled readiness/callback failures.

Actions, parameters, TF, costmaps, cancellation and recovery events use ROS
endpoints. Dependency discovery, delayed replies and diagnostic errors are
injected explicitly. No Nav2 planner, physical DDS or robot is driven here.
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

    def start(xs=(.4, .8)):
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
        assert endpoints.goals == before, 'Collection started an unauthorized new route'
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals == before + 1 and operator.goal_handle is not None)
        return before + 1

    def succeed(index, next_index=None):
        endpoints.outcomes[index] = 4
        if next_index is not None:
            wait_for(lambda: endpoints.goals == next_index and operator.goal_handle is not None)

    def finish(index, total):
        succeed(index)
        wait_for(lambda: operator.completed_waypoints == total and
                 operator.goal_handle is None and not operator.goal_pending)
        assert not operator.waypoints and not operator.route_started

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

        # Exactly one Enable must complete all four waypoints in order, even
        # when dependencies disappear at different handoff boundaries.
        first = start((.4, .8, 1.2, 1.6))
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        succeed(first)
        wait_for(lambda: operator.completed_waypoints == 1 and operator.goal_handle is None)
        wait_for(lambda: seen_since(offset, 'TF stale'))
        no_extra_goal(first)
        endpoints.tf_enabled = True
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        endpoints.grid_enabled['/global_costmap/costmap'] = False
        wait_for(lambda: time.monotonic() - operator.grids['/global_costmap/costmap'][1] > 3.1)
        offset = len(endpoints.operator_messages)
        succeed(first + 1)
        wait_for(lambda: operator.completed_waypoints == 2 and operator.goal_handle is None)
        wait_for(lambda: seen_since(offset, '/global_costmap/costmap stale'))
        no_extra_goal(first + 1)
        endpoints.grid_enabled['/global_costmap/costmap'] = True
        wait_for(lambda: endpoints.goals == first + 2 and operator.goal_handle is not None)
        with patch.object(operator, 'goal_error_text', side_effect=RuntimeError('injected diagnostic failure')):
            succeed(first + 2, first + 3)
            finish(first + 3, 4)
        assert endpoints.positions[first - 1:] == [.4, .8, 1.2, 1.6]
        print('PASS one Enable: four ordered successes despite handoff TF/grid gaps and diagnostic errors', flush=True)

        # A route between completed actions has no moving action to time out.
        # A long discovery outage must retain the authorized tail and resume
        # when it recovers, rather than silently becoming a manual-only pause.
        first = start()
        offset = len(endpoints.operator_messages)
        with patch.object(operator.motion_state, 'service_is_ready', return_value=False):
            succeed(first)
            wait_for(lambda: operator.completed_waypoints == 1 and operator.goal_handle is None)
            wait_for(lambda: seen_since(offset, 'state service unavailable'))
            no_extra_goal(first, 16.)
            assert operator.route_started and not operator.route_paused
            assert [pose.pose.position.x for _, pose in operator.waypoints] == [.8]
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        finish(first + 1, 2)
        assert endpoints.positions[first - 1:] == [.4, .8]
        print('PASS bridge unavailable beyond 15 seconds retains route and recovers without another Enable', flush=True)

        # Timeout only the client's first readback. Its late True response
        # must not dispatch a second action once a fresh query has succeeded.
        first = start()
        pending = Future(executor=executor)
        original_call = operator.motion_state.call_async
        calls = []

        def delay_first(request):
            calls.append(request)
            return pending if len(calls) == 1 else original_call(request)

        offset = len(endpoints.operator_messages)
        with patch.object(operator.motion_state, 'call_async', side_effect=delay_first):
            succeed(first)
            wait_for(lambda: len(calls) == 1)
            wait_for(lambda: seen_since(offset, 'timed out'))
            wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
            pending.set_result(enabled_reply())
            no_extra_goal(first + 1)
            finish(first + 1, 2)
        assert endpoints.positions[first - 1:] == [.4, .8]
        print('PASS query timeout preserves tail; late enabled reply cannot duplicate dispatch', flush=True)

        # Cancel in each handoff wait state; restoring readiness or receiving
        # an old success cannot revive the canceled route.
        first = start()
        pending = Future(executor=executor)
        with patch.object(operator.motion_state, 'call_async', return_value=pending) as delayed:
            succeed(first)
            wait_for(lambda: delayed.call_count == 1)
            endpoints.cancel.publish(Empty())
            wait_for(lambda: not operator.waypoints and not operator.route_started)
            pending.set_result(enabled_reply())
            no_extra_goal(first)
        first = start()
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        succeed(first)
        wait_for(lambda: seen_since(offset, 'TF stale'))
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_started)
        endpoints.tf_enabled = True
        wait_for(lambda: 0 <= tf_age() < .2)
        no_extra_goal(first)
        print('PASS cancellation revokes pending readbacks and scheduled handoff retries', flush=True)

        # Replaying a terminal callback must not clear the new action handle
        # or count the old success twice. Capture its real arguments so the
        # request token and generation are the ones actually used by ROS.
        first = start()
        result_calls = []
        original_result = operator.goal_result

        def record_result(*args, **kwargs):
            result_calls.append((args, kwargs))
            return original_result(*args, **kwargs)

        with patch.object(operator, 'goal_result', side_effect=record_result):
            succeed(first, first + 1)
            wait_for(lambda: len(result_calls) == 1)
            active_handle = operator.goal_handle
            args, kwargs = result_calls[0]
            original_result(*args, **kwargs)
            assert operator.goal_handle is active_handle
            assert operator.completed_waypoints == 1
            no_extra_goal(first + 1)
            finish(first + 1, 2)
        assert endpoints.positions[first - 1:] == [.4, .8]
        print('PASS stale terminal callback cannot clear the next goal or count completion twice', flush=True)

        # A localization pause BETWEEN goals retains only the unvisited tail.
        # Fresh Bool/TF alone cannot restore motion; the existing recovery
        # proof, stable pose and refreshed costmap rules continue to apply.
        first = start((.4, .8, 1.2))
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > .6)
        offset = len(endpoints.operator_messages)
        succeed(first)
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
        clears = dict(endpoints.clear_calls)
        endpoints.respond_to_revalidation = True
        endpoints.event(endpoints.recovery_epoch + 1)
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        assert all(endpoints.clear_calls[name] > clears[name] for name in clears)
        succeed(first + 1, first + 2)
        finish(first + 2, 3)
        assert endpoints.positions[first - 1:] == [.4, .8, 1.2]
        print('PASS localization pause preserves tail and resumes only after verified recovery', flush=True)

        # A true failed Nav2 action remains a deliberate manual-resume case;
        # enabling motion again alone must not turn it into automatic retry.
        first = start()
        endpoints.outcomes[first] = 6
        wait_for(lambda: operator.route_paused and operator.goal_handle is None)
        assert [pose.pose.position.x for _, pose in operator.waypoints] == [.4, .8]
        endpoints.enable.publish(Bool(data=True))
        no_extra_goal(first)
        endpoints.resume.publish(Empty())
        wait_for(lambda: endpoints.goals == first + 1 and operator.goal_handle is not None)
        succeed(first + 1, first + 2)
        finish(first + 2, 2)
        assert endpoints.positions[first - 1:] == [.4, .4, .8]
        assert endpoints.max_active == 1
        print('PASS ABORTED retains failed point and tail; explicit Resume retries in order with one active action', flush=True)
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
