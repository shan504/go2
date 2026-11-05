"""Actual ROS actions/parameters: resume only a previously running lost route.

Recovery events are controlled test endpoints; mapper registration/recovery is
validated separately. No Nav2 planner, physical DDS or robot is driven here.
"""
import copy
import json
import threading
import time

import rclpy
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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

    def publish_recovery(self):
        if self.recovery_event is not None:
            self.recovery.publish(String(data=json.dumps(self.recovery_event)))

    def event(self,epoch,state='recovered',confirmations=3,source_age=0.):
        self.recovery_event = dict(state=state,epoch=epoch,confirmations=confirmations,
            source_stamp=self.get_clock().now().nanoseconds/1e9-source_age)
        self.publish_recovery()

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
        endpoints.event(2,source_age=60.)
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

        # Missing Nav2 clearing capability and recovery timeouts fail closed.
        for service in endpoints.clear_services:
            endpoints.destroy_service(service)
        current = start()
        operator.auto_resume_timeout = 2.5
        pause()
        endpoints.valid = True
        endpoints.event(22)
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
