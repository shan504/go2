"""Real Humble BT navigator + Wait behavior + Operator; synthetic action endpoints.

Run in an isolated ROS domain/container. No SDK, robot, planner or motor is used.
The real XML/plugins perform replanning, cancellation, waiting and bounded retry.
"""
import copy
import os
import signal
from collections import Counter
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from lifecycle_msgs.srv import ChangeState
from action_msgs.msg import GoalStatusArray
from rclpy.qos import qos_profile_action_status_default
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Bool, Empty
import yaml
from navigation_gateway_smoke import Operator, TestEndpoints, wait_for, PoseStamped


class Endpoints(TestEndpoints):
    def __init__(self):
        super().__init__(start_action=False)
        self.plans = Counter()
        self.events = []
        self.mode = 'three'
        self.active_follow = 0
        self.cancel_delay = 0.15  # exceeds Humble's default 20ms BT RPC timeout
        self.wait_samples = []
        self.create_subscription(GoalStatusArray, '/wait/_action/status',
            lambda msg: self.wait_samples.append(self.active_follow)
            if any(status.status == 2 for status in msg.status_list) else None,
            qos_profile_action_status_default)
        self.group = ReentrantCallbackGroup()
        self.planner = ActionServer(self, ComputePathToPose, '/compute_path_to_pose',
            execute_callback=self.plan, callback_group=self.group)
        self.controller = ActionServer(self, FollowPath, '/follow_path',
            execute_callback=self.follow, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.resume = self.create_publisher(Empty, '/navigation/resume', 1)

    def plan(self, handle):
        x = round(handle.request.goal.pose.position.x, 2)
        self.plans[x] += 1
        attempt = self.plans[x]
        self.events.append(('plan', x, attempt, time.monotonic()))
        fail = (self.mode == 'three' and x == 0.8 and attempt == 2 or
                self.mode in ('blocked', 'interrupt') and attempt >= 2)
        result = ComputePathToPose.Result()
        if fail:
            self.events.append(('plan_fail', x, attempt, time.monotonic()))
            handle.abort()
        else:
            result.path = NavPath()
            result.path.header.frame_id = 'map'
            start = PoseStamped()
            start.header.frame_id = 'map'
            start.pose.orientation.w = 1.0
            result.path.poses = [start, copy.deepcopy(handle.request.goal)]
            handle.succeed()
        return result

    def follow(self, handle):
        x = round(handle.request.path.poses[-1].pose.position.x, 2)
        self.active_follow += 1
        self.events.append(('follow_start', x, self.plans[x], time.monotonic()))
        started = time.monotonic()
        try:
            while time.monotonic()-started < 20:
                if handle.is_cancel_requested:
                    time.sleep(self.cancel_delay)
                    self.events.append(('follow_stop', x, self.plans[x], time.monotonic()))
                    handle.canceled()
                    return FollowPath.Result()
                # Represents a controller command; never published to a motor topic.
                self.events.append(('motion', x, self.plans[x], time.monotonic()))
                if self.mode == 'three' and (x != 0.8 or self.plans[x] >= 3):
                    if time.monotonic()-started > 0.3:
                        handle.succeed()
                        return FollowPath.Result()
                time.sleep(0.02)
            handle.abort()
            return FollowPath.Result()
        finally:
            self.active_follow -= 1


def main():
    rclpy.init(args=['--ros-args', '-p', 'mode:=navigation', '-p', 'auto_resume_localization:=false'])
    endpoints, operator = Endpoints(), Operator()
    # Production runs Operator independently of the planner/controller. Keep its
    # 0.5s heartbeat watchdog independent of synthetic action worker scheduling.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(endpoints)
    operator_executor = SingleThreadedExecutor()
    operator_executor.add_node(operator)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    operator_spinner = threading.Thread(target=operator_executor.spin, daemon=True)
    spinner.start()
    operator_spinner.start()
    processes = []
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        params = root/'params.yaml'
        tree = str(Path(__file__).resolve().parents[1]/'go2_3d/navigate.xml')
        params.write_text(yaml.safe_dump({
            'bt_navigator': {'ros__parameters': {
                'global_frame': 'map', 'robot_base_frame': 'base_footprint',
                # Synthetic Python action workers can acknowledge after 20ms.
                # Isolate transport scheduling from deliberate planning failures;
                # this fixture timeout does not change robot configuration.
                'default_server_timeout': 1000,
                'default_nav_to_pose_bt_xml': tree, 'default_nav_through_poses_bt_xml': tree,
                'plugin_lib_names': ['nav2_'+name+'_bt_node' for name in (
                    'compute_path_to_pose_action', 'follow_path_action', 'rate_controller',
                    'pipeline_sequence', 'recovery_node', 'wait_action')]}},
            'behavior_server': {'ros__parameters': {
                'global_frame': 'map', 'robot_base_frame': 'base_footprint',
                'behavior_plugins': ['wait'], 'wait': {'plugin': 'nav2_behaviors/Wait'}}}}))
        logs = []
        try:
            for package, executable in (('nav2_behaviors', 'behavior_server'),
                                        ('nav2_bt_navigator', 'bt_navigator')):
                log = (root/(executable+'.log')).open('w+')
                logs.append(log)
                processes.append(subprocess.Popen(['ros2', 'run', package, executable,
                    '--ros-args', '--params-file', str(params), '-r', 'cmd_vel:=/test_unused_cmd_vel',
                    '-r', 'goal_pose:=/navigation/nav2_direct_goal_unused'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
                client = endpoints.create_client(ChangeState, '/'+executable+'/change_state')
                assert client.wait_for_service(timeout_sec=30), executable
                for transition in (1, 3):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    future = client.call_async(request)
                    wait_for(future.done, 30)
                    assert future.result().success, (executable, transition)
            endpoints.valid = True
            wait_for(lambda: operator.localized() and len(operator.grids) == 3 and
                     operator.action.server_is_ready() and operator.motion_state.service_is_ready())
            wait_for(lambda: operator.buffer.can_transform('map', 'base_footprint', rclpy.time.Time()))

            def route(xs):
                for x in xs:
                    goal = PoseStamped()
                    goal.header.frame_id = 'map'
                    goal.pose.orientation.w = 1.0
                    goal.pose.position.x = x
                    endpoints.goal.publish(goal)
                wait_for(lambda: len(operator.waypoints) == len(xs))
                endpoints.enable.publish(Bool(data=True))

            def stationary_wait(x):
                wait_for(lambda: any(e[0] == 'follow_stop' and e[1] == x for e in endpoints.events))
                stopped = next(e[3] for e in reversed(endpoints.events) if e[0] == 'follow_stop' and e[1] == x)
                time.sleep(0.5)
                assert not endpoints.active_follow
                assert not any(e[0] == 'motion' and e[3] > stopped for e in endpoints.events)
                return stopped

            route((0.4, 0.8, 1.2))
            stationary_wait(0.8)
            wait_for(lambda: operator.completed_waypoints == 3, 20)
            assert not operator.waypoints and not operator.route_paused
            assert endpoints.plans[0.8] == 3, endpoints.plans
            stopped = next(e[3] for e in endpoints.events if e[0] == 'follow_stop' and e[1] == 0.8)
            replanned = next(e[3] for e in endpoints.events if e[:3] == ('plan', 0.8, 3))
            assert replanned-stopped >= 1.9
            print('PASS three goals in order; transient replan failure cancels old controller, waits stationary, retries SAME goal', flush=True)

            endpoints.mode = 'blocked'
            endpoints.events.clear()
            route((1.6, 2.0))
            stationary_wait(1.6)
            wait_for(lambda: operator.route_paused and operator.goal_handle is None, 20)
            assert [round(p.pose.position.x, 2) for _, p in operator.waypoints] == [1.6, 2.0]
            # One initial successful plan, one failed replan, then three retries.
            assert endpoints.plans[1.6] == 5 and endpoints.plans[2.0] == 0
            assert operator.completed_waypoints == 0
            count = sum(endpoints.plans.values())
            time.sleep(2.5)
            assert sum(endpoints.plans.values()) == count
            print('PASS persistent failure: initial pipeline + three retries only; failed goal and later goal retained, paused without skipping', flush=True)
            endpoints.mode = 'three'
            endpoints.resume.publish(Empty())
            wait_for(lambda: operator.completed_waypoints == 2, 15)
            assert not operator.waypoints and not operator.route_paused
            print('PASS explicit resume retries retained failed goal, then executes next goal', flush=True)

            for x, event in ((0.6, 'cancel'), (1.0, 'stop'), (1.4, 'localization')):
                endpoints.mode = 'interrupt'
                endpoints.events.clear()
                endpoints.plans.clear()
                route((x,))
                stationary_wait(x)
                count = endpoints.plans[x]
                if event == 'cancel':
                    endpoints.cancel.publish(Empty())
                elif event == 'stop':
                    endpoints.enable.publish(Bool(data=False))
                else:
                    endpoints.valid = False
                wait_for(lambda: operator.goal_handle is None and not operator.goal_pending)
                time.sleep(2.5)
                assert endpoints.plans[x] == count and not endpoints.active_follow
                assert not operator.route_started and operator.auto_resume is None
                if event != 'cancel':
                    assert not endpoints.get_parameter('enable_control').value
                if event == 'localization':
                    assert operator.route_paused and len(operator.waypoints) == 1
                    endpoints.valid = True
                    wait_for(operator.localized)
                    time.sleep(0.3)
                    assert endpoints.plans[x] == count
                    endpoints.mode = 'three'
                    endpoints.resume.publish(Empty())
                    # Localization pause disables the bridge; explicit enable
                    # acknowledges restored localization before dispatch.
                    endpoints.enable.publish(Bool(data=True))
                    wait_for(lambda: operator.completed_waypoints == 1, 15)
                    assert not operator.waypoints
                endpoints.cancel.publish(Empty())
                wait_for(lambda: not operator.waypoints and not operator.route_paused)
                print('PASS recovery wait interrupted by '+event+'; no retry or motion restart', flush=True)
            assert endpoints.wait_samples and not any(endpoints.wait_samples), endpoints.wait_samples
            print('PASS real Wait action status: no active FollowPath during any observed recovery wait', flush=True)
        except Exception:
            for log in logs:
                log.flush()
                print(Path(log.name).read_text()[-10000:])
            raise
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in reversed(processes):
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            operator_executor.shutdown()
            executor.shutdown()
            operator_spinner.join(5)
            spinner.join(5)
            operator.destroy_node()
            endpoints.destroy_node()
            rclpy.shutdown()
            for log in logs:
                log.close()


if __name__ == '__main__':
    main()
