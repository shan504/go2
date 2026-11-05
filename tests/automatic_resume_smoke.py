"""Actual ROS actions/parameters: resume only a previously running lost route.

Recovery events are controlled test endpoints; mapper registration/recovery is
validated separately. No Nav2 planner, physical DDS or robot is driven here.
"""
import copy
import json
import threading
import time
from unittest.mock import patch

import rclpy
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Empty, String

from navigation_queue_smoke import Operator, QueueEndpoints, wait_for, grid_cell


class RecoveryEndpoints(QueueEndpoints):
    def __init__(self):
        super().__init__()
        self.clear_calls = {'global': 0, 'local': 0}
        self.freeze_after_clear = False
        self.ghost_cells = {}
        self.recover_requests = 0
        self.recovery_event = None
        self.recovery_epoch = 0
        self.respond_to_revalidation = False
        self.pending_revalidation_response = None
        self.operator_messages = []
        self.create_subscription(String,'/operator/status',
            lambda message: self.operator_messages.append((time.monotonic(),message.data)),
            self.startup.qos_profile)
        self.recovery = self.create_publisher(String,'/localization/recovery',self.startup.qos_profile)
        self.initial = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
        self.create_subscription(Empty,'/localization/recover',self.requested,1)
        self.create_timer(.05,self.publish_recovery)
        self.clear_services = []
        for name in ('global','local'):
            service = self.create_service(ClearEntireCostmap,
                f'/{name}_costmap/clear_entirely_{name}_costmap',
                lambda request,response,name=name: self.clear(name,response),
                callback_group=ReentrantCallbackGroup())
            self.clear_services.append(service)

    def requested(self,message):
        self.recover_requests += 1
        if self.respond_to_revalidation and self.valid:
            self.pending_revalidation_response = dict(started=time.monotonic(),ticks=0)

    def publish_recovery(self):
        pending = self.pending_revalidation_response
        if pending is not None and self.respond_to_revalidation and self.valid and self.publish_valid:
            pending['ticks'] += 1
            if pending['ticks'] >= 3 and time.monotonic()-pending['started'] >= .15:
                self.pending_revalidation_response = None
                self.event(self.recovery_epoch+1)
                return
        if self.recovery_event is not None:
            self.recovery.publish(String(data=json.dumps(self.recovery_event)))

    def event(self,epoch,state='recovered',confirmations=3,source_age=0.):
        self.recovery_epoch = max(epoch,self.recovery_epoch+1)
        self.recovery_event = dict(state=state,epoch=self.recovery_epoch,confirmations=confirmations,
            source_stamp=self.get_clock().now().nanoseconds/1e9-source_age)
        self.publish_recovery()
        return self.recovery_epoch

    def clear(self,name,response):
        self.clear_calls[name] += 1
        if name in self.ghost_cells:
            _, grid = self.grids[f'/{name}_costmap/costmap']
            grid.data[self.ghost_cells.pop(name)] = 0
        if self.freeze_after_clear:
            self.grid_enabled['/global_costmap/costmap'] = False
            self.grid_enabled['/local_costmap/costmap'] = False
        return response


def main():
    rclpy.init(args=['--ros-args','-p','mode:=navigation'])
    operator, endpoints = Operator(),RecoveryEndpoints()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (operator,endpoints):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start()
    template = PoseStamped()
    template.header.frame_id = 'map'
    template.pose.orientation.w = 1.

    def click(x):
        message = copy.deepcopy(template)
        message.pose.position.x = x
        endpoints.goal.publish(message)

    def reset():
        endpoints.respond_to_revalidation = False
        endpoints.pending_revalidation_response = None
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and operator.goal_handle is None and not operator.goal_pending)
        endpoints.valid = True
        endpoints.recovery_event = None
        wait_for(operator.localized)

    def start():
        before = endpoints.goals
        click(.4)
        wait_for(lambda: len(operator.waypoints)==1)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals==before+1 and operator.goal_handle is not None)
        return before+1

    def pause():
        endpoints.valid = False
        wait_for(lambda: operator.auto_resume is not None and operator.route_paused)
        wait_for(lambda: operator.goal_handle is None and not operator.goal_pending and
                 operator.pause_disable_confirmed and not endpoints.get_parameter('enable_control').value)

    def assert_stopped(count,seconds=1.35):
        time.sleep(seconds)
        assert endpoints.goals == count, (endpoints.goals,count)
        assert not endpoints.get_parameter('enable_control').value

    try:
        endpoints.valid = True
        wait_for(lambda: operator.localized() and len(operator.grids)==3 and operator.action.server_is_ready()
                 and operator.motion_state.service_is_ready() and
                 all(client.service_is_ready() for client in operator.costmap_clear.values()))
        wait_for(lambda: operator.buffer.can_transform('map','base_footprint',rclpy.time.Time()))
        # Bootstrap and queued routes acquire no automatic movement authority.
        click(.4)
        wait_for(lambda: len(operator.waypoints)==1)
        endpoints.valid = False
        wait_for(lambda: operator.route_paused)
        assert operator.auto_resume is None
        endpoints.valid = True
        endpoints.event(1)
        assert_stopped(0)
        reset()

        current = start()
        click(.8)
        wait_for(lambda: len(operator.waypoints)==1)
        pause()
        assert [pose.pose.position.x for _,pose in operator.waypoints] == [.4,.8]
        wait_for(lambda: endpoints.recover_requests >= 1)
        # A fresh Bool, a stale recovery event, or only two confirmations
        # cannot restart an otherwise user-authorized retained route.
        endpoints.valid = True
        assert_stopped(current)
        requests = endpoints.recover_requests
        original_started = operator.auto_resume['started']
        endpoints.event(2,source_age=60.)
        wait_for(lambda: endpoints.recover_requests==requests+1)
        assert operator.auto_resume['started']==original_started
        assert_stopped(current)
        endpoints.event(3,confirmations=2)
        assert_stopped(current)
        # Fresh proof is still gated by the recovered body's static map cell.
        _, static = endpoints.grids['/map']
        col,row = grid_cell(operator.grids['/map'][0],(0.,0.))
        index = row*static.info.width+col
        static.data[index] = 100
        wait_for(lambda: operator.grids['/map'][0].data[index]==100)
        endpoints.event(4)
        assert_stopped(current)
        assert any('reason=validation:body:/map' in text and 'XY=' in text and
                   'cell=' in text and 'value=100' in text for _,text in endpoints.operator_messages), \
            'blocked robot cell was not exposed in recovery waiting status'
        endpoints.valid = False
        wait_for(lambda: operator.auto_resume is not None and operator.auto_resume['proof_epoch'] is None)
        endpoints.valid = True
        # Repeated delivery of the previous recovery event cannot authorize a
        # second interruption. A new epoch and new proof are required.
        assert_stopped(current)
        endpoints.event(5)
        _, global_grid = endpoints.grids['/global_costmap/costmap']
        col,row = grid_cell(operator.grids['/global_costmap/costmap'][0],(0.,0.))
        ghost = row*global_grid.info.width+col
        global_grid.data[ghost] = 100
        endpoints.ghost_cells['global'] = ghost
        static.data[index] = 0
        # Both clear services must finish and NEW grids must arrive afterward.
        endpoints.freeze_after_clear = True
        wait_for(lambda: operator.auto_resume is not None and operator.auto_resume['stage']=='refreshing')
        assert endpoints.clear_calls == {'global':1,'local':1}
        assert_stopped(current,.3)
        endpoints.freeze_after_clear = False
        _, local_grid = endpoints.grids['/local_costmap/costmap']
        col,row = grid_cell(operator.grids['/local_costmap/costmap'][0],(0.,0.))
        occupied = row*local_grid.info.width+col
        local_grid.data[occupied] = 100
        for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
            endpoints.grid_enabled[topic] = True
        assert_stopped(current)
        endpoints.respond_to_revalidation = True
        # The intentionally blocked-grid interval can outlive a test Bool
        # packet. Supply a NEW mapper proof here and answer any subsequent
        # real revalidation request while this positive case expects resume.
        endpoints.event(endpoints.recovery_epoch+1)
        local_grid.data[occupied] = 0
        wait_for(lambda: endpoints.goals==current+1 and operator.goal_handle is not None)
        assert endpoints.positions[-1] == .4
        assert operator.auto_resume is None and operator.route_started and not operator.route_paused
        endpoints.outcomes[current+1] = 4
        wait_for(lambda: endpoints.goals==current+2 and operator.goal_handle is not None)
        assert endpoints.positions[-1] == .8 and endpoints.max_active==1
        print('PASS verified recovery resumes retained route only after cancel/disable, stable TF and fresh cleared costmaps',flush=True)
        reset()

        # Loss of Bool delivery alone still requests three fresh mapper scans
        # rather than waiting forever for a mapper-side invalidity episode.
        current = start()
        requests = endpoints.recover_requests
        endpoints.publish_valid = False
        wait_for(lambda: operator.auto_resume is not None and operator.route_paused)
        wait_for(lambda: operator.goal_handle is None and operator.pause_disable_confirmed)
        assert 'localization_heartbeat_timeout' in operator.localization_pause_detail
        wait_for(lambda: endpoints.recover_requests > requests)
        endpoints.publish_valid = True
        wait_for(operator.localized)
        endpoints.respond_to_revalidation = True
        endpoints.event(6)
        wait_for(lambda: endpoints.goals==current+1 and operator.goal_handle is not None)
        reset()
        print('PASS heartbeat-only interruption can request verified mapper recovery and resume',flush=True)

        # Revocation after clearing has begun must invalidate every in-flight
        # callback; subsequent fresh grids cannot restore the old authority.
        current = start()
        pause()
        endpoints.freeze_after_clear = True
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.respond_to_revalidation = True
        endpoints.event(7)
        wait_for(lambda: operator.auto_resume is not None and operator.auto_resume['stage']=='refreshing')
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: operator.auto_resume is None and not operator.waypoints)
        endpoints.freeze_after_clear = False
        for topic in ('/global_costmap/costmap','/local_costmap/costmap'):
            endpoints.grid_enabled[topic] = True
        assert_stopped(current)
        reset()
        print('PASS manual stop during asynchronous costmap recovery revokes all late callbacks',flush=True)

        # Every explicit override must revoke the old recovery intent.
        for offset,override in enumerate(('disable','cancel','initialpose','newgoal'),10):
            current = start()
            pause()
            if override == 'disable':
                endpoints.enable.publish(Bool(data=False))
            elif override == 'cancel':
                endpoints.cancel.publish(Empty())
            elif override == 'initialpose':
                endpoints.initial.publish(PoseWithCovarianceStamped())
            else:
                click(.8)
            wait_for(lambda: operator.auto_resume is None)
            endpoints.valid = True
            endpoints.event(offset)
            assert_stopped(current)
            reset()
        print('PASS manual stop/cancel/initialpose/new waypoint revoke recovery authority',flush=True)

        # Manual initialization can arrive BEFORE validity falls. Both the
        # actual pose topic and the mapper event must immediately revoke the
        # running route, not merely clear an intent that does not yet exist.
        for override,manual_epoch,proof_epoch in (('initialpose',16,17),('manual_state',18,19)):
            current = start()
            assert operator.route_started and operator.auto_resume is None
            if override == 'initialpose':
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id = 'map'
                initial.pose.pose.orientation.w = 1.
                endpoints.initial.publish(initial)
            else:
                endpoints.event(manual_epoch,state='manual_initialization')
            wait_for(lambda: operator.route_paused and not operator.route_started and
                     operator.route_pause_reason=='manual_initialization' and
                     operator.goal_handle is None and not endpoints.get_parameter('enable_control').value)
            assert operator.auto_resume is None and operator.localized()
            endpoints.valid = False
            wait_for(lambda: not operator.localized())
            assert operator.auto_resume is None
            endpoints.valid = True
            endpoints.event(proof_epoch)
            assert_stopped(current)
            assert len(operator.waypoints)==1 and not operator.route_started
            # The veto lasts until a NEW explicit user enable; it does not
            # make the retained route permanently unusable.
            endpoints.enable.publish(Bool(data=True))
            wait_for(lambda: endpoints.goals==current+1 and operator.goal_handle is not None)
            reset()
        print('PASS active-route initialpose/manual event before Bool loss cannot create future automatic authority',flush=True)

        current = start()
        endpoints.outcomes[current] = 6
        wait_for(lambda: operator.route_paused and operator.route_pause_reason=='navigation' and operator.goal_handle is None)
        endpoints.valid = False
        time.sleep(.2)
        endpoints.valid = True
        endpoints.event(20)
        time.sleep(1.35)
        assert endpoints.goals==current and operator.auto_resume is None
        reset()
        print('PASS aborted Nav2 action is never automatically retried',flush=True)

        # An abort racing a localization cancellation is still a navigation
        # failure, even though its callback belongs to the paused generation.
        current = start()
        endpoints.server.register_cancel_callback(lambda request: CancelResponse.REJECT)
        endpoints.valid = False
        wait_for(lambda: operator.auto_resume is not None)
        endpoints.outcomes[current] = 6
        wait_for(lambda: operator.goal_handle is None and operator.auto_resume is None and
                 not endpoints.get_parameter('enable_control').value)
        endpoints.valid = True
        endpoints.event(21)
        assert_stopped(current)
        endpoints.server.register_cancel_callback(lambda request: CancelResponse.ACCEPT)
        reset()
        print('PASS abort during localization cancellation revokes automatic retry',flush=True)

        # A one-shot recovered event can be delayed beyond its scan age gate
        # even though tracking remains healthy. Request NEW proof rather than
        # silently waiting until the route authority expires.
        current = start()
        requests = endpoints.recover_requests
        pause()
        wait_for(lambda: endpoints.recover_requests > requests)
        requests = endpoints.recover_requests
        original_started = operator.auto_resume['started']
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.tf_enabled = False
        endpoints.respond_to_revalidation = True
        endpoints.event(30)
        wait_for(lambda: any('reason=validation:tf:' in text for _,text in endpoints.operator_messages))
        since = time.monotonic()
        time.sleep(1.2)
        # TF age changes at every watchdog tick. A stable category key must
        # throttle these numeric changes rather than publishing at 10 Hz.
        assert sum(when >= since and 'reason=validation:tf:' in text
                   for when,text in endpoints.operator_messages) <= 1
        endpoints.respond_to_revalidation = False
        requests = endpoints.recover_requests
        endpoints.event(31,source_age=60.)
        wait_for(lambda: endpoints.recover_requests==requests+1)
        assert operator.auto_resume['proof_epoch'] is None
        assert operator.auto_resume['started']==original_started
        assert_stopped(current,2.25)
        assert endpoints.recover_requests==requests+1, 'duplicate stale events repeatedly requested recovery'
        endpoints.tf_enabled = True
        endpoints.respond_to_revalidation = True
        endpoints.event(32)
        wait_for(lambda: endpoints.goals==current+1 and operator.goal_handle is not None)
        assert endpoints.positions[-1]==.4 and endpoints.max_active==1
        reset()
        print('PASS stale recovered event requests one fresh proof, preserves deadline and resumes after new proof/clears',flush=True)

        # A stale-proof retry queued behind backoff is still subordinate to
        # manual stop; neither its timer nor a later fresh proof may rearm it.
        current = start()
        requests = endpoints.recover_requests
        pause()
        wait_for(lambda: endpoints.recover_requests > requests)
        requests = endpoints.recover_requests
        endpoints.valid = True
        stale_epoch = endpoints.event(33,source_age=60.)
        wait_for(lambda: operator.auto_resume is not None and
                 operator.auto_resume['pending_revalidation_epoch']==stale_epoch)
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: operator.auto_resume is None and not operator.waypoints)
        assert_stopped(current,2.25)
        assert endpoints.recover_requests==requests, 'revoked retry still requested mapper revalidation'
        endpoints.event(34)
        assert_stopped(current)
        reset()
        print('PASS manual stop revokes queued stale-proof request and rejects later recovery authority',flush=True)

        # Delay only the client's acknowledgment of a REAL successful enable.
        # A newer stale proof must revoke that in-flight callback's token and
        # confirm a real disable before it can obtain fresh recovery authority.
        current = start()
        pause()
        endpoints.valid = True
        wait_for(operator.localized)
        original_call = operator.parameters.call_async
        held = {'once': False, 'ready': threading.Event()}
        def delay_first_auto_enable(request):
            actual = original_call(request)
            if (not held['once'] and operator.auto_resume is not None and
                    operator.auto_resume['stage']=='enabling' and
                    any(parameter.name=='enable_control' and parameter.value.bool_value
                        for parameter in request.parameters)):
                held['once'] = True
                held['proxy'] = Future(executor=executor)
                def finished(future):
                    held['response'] = future.result()
                    held['ready'].set()
                actual.add_done_callback(finished)
                return held['proxy']
            return actual
        with patch.object(operator.parameters,'call_async',side_effect=delay_first_auto_enable):
            endpoints.event(50)
            wait_for(lambda: held['ready'].is_set() and operator.auto_resume is not None and
                     operator.auto_resume['stage']=='enabling' and
                     endpoints.get_parameter('enable_control').value)
            previous = operator.auto_resume
            original_started = previous['started']
            requests = endpoints.recover_requests
            endpoints.event(51,source_age=60.)
            wait_for(lambda: operator.auto_resume is not None and operator.auto_resume is not previous and
                     operator.pause_disable_confirmed and not endpoints.get_parameter('enable_control').value)
            assert operator.auto_resume['started']==original_started
            wait_for(lambda: endpoints.recover_requests==requests+1)
            held['proxy'].set_result(held['response'])
            assert_stopped(current,.3)
            endpoints.respond_to_revalidation = True
            endpoints.event(52)
            wait_for(lambda: endpoints.goals==current+1 and operator.goal_handle is not None)
            assert endpoints.positions[-1]==.4 and endpoints.max_active==1
        reset()
        print('PASS stale proof during real enable revokes old acknowledgment until disable and new recovery proof',flush=True)

        # Missing Nav2 clearing capability and recovery timeouts fail closed.
        for service in endpoints.clear_services:
            endpoints.destroy_service(service)
        current = start()
        operator.auto_resume_timeout = 3.5
        pause()
        missing_clear_started = time.monotonic()
        endpoints.respond_to_revalidation = True
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.event(40)
        wait_for(lambda: any(when >= missing_clear_started and 'reason=clear_services' in text
                            for when,text in endpoints.operator_messages),seconds=4)
        wait_for(lambda: operator.auto_resume is None,seconds=5)
        assert_stopped(current,.2)
        assert operator.route_paused and len(operator.waypoints)==1
        print('PASS missing clear services/recovery timeout retain route without motion',flush=True)
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
