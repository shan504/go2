#!/usr/bin/env python3
"""Foxglove publish-panel operations and /goal_pose -> Nav2 action gateway."""
import copy
import json
import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry import pose_matrix, grid_cell, quaternion_from_matrix, planar_pose
from motion_profile import ARRIVAL_RADIUS


class DispatchNotReady(ValueError):
    """A live dependency can recover without changing the queued target."""


class Operator(Node):
    def __init__(self):
        super().__init__('go2_operator')
        self.declare_parameter('mode','mapping')
        self.mode = self.get_parameter('mode').value
        self.declare_parameter('auto_resume_localization', True)
        self.declare_parameter('auto_resume_stable_seconds', 1.0)
        self.declare_parameter('auto_resume_timeout', 60.0)
        self.auto_resume_enabled = self.get_parameter('auto_resume_localization').value
        self.auto_resume_stable = float(self.get_parameter('auto_resume_stable_seconds').value)
        self.auto_resume_timeout = float(self.get_parameter('auto_resume_timeout').value)
        if (not math.isfinite(self.auto_resume_stable) or self.auto_resume_stable < 1.0 or
                not math.isfinite(self.auto_resume_timeout) or self.auto_resume_timeout <= 0):
            raise ValueError('automatic resume needs stable_seconds >= 1 and a positive timeout')
        self.auto_resume = None
        self.last_auto_resume_revocation = ''
        self.recovery_epoch = -1
        self.recovery_state = 'idle'
        self.recovery_detail = ''
        self.pause_disable_confirmed = False
        self.costmap_clear = {}
        self.valid = False
        self.valid_received = None
        self.localization_state = 'state not received'
        self.localization_state_received = None
        self.localization_pause_detail = ''
        self.goal_handle = None
        self.goal_pending = False
        self.cancel_requested = False
        self.cancelling = False
        self.last_goal_status = None
        self.waypoints = deque()
        self.inflight_waypoint = None
        self.route_paused = False
        self.route_pause_reason = None
        self.route_started = False
        self.waypoint_number = 0
        self.completed_waypoints = 0
        self.goal_generation = 0
        self.motion_generation = 0
        self.enable_query = None
        self.dispatch_wait_started = None
        self.dispatch_retry_at = 0.0
        self.dispatch_wait_reason = ''
        self.pause_disable_future = None
        self.startup_status = 'startup status not received'
        self.grids = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.action = None
        if self.mode == 'navigation':
            from nav2_msgs.action import NavigateToPose
            from nav2_msgs.srv import ClearEntireCostmap
            self.goal_type = NavigateToPose
            self.action = ActionClient(self,NavigateToPose,'/navigate_to_pose')
            self.clear_type = ClearEntireCostmap
            for name in ('global', 'local'):
                self.costmap_clear[name] = self.create_client(ClearEntireCostmap,
                    f'/{name}_costmap/clear_entirely_{name}_costmap')
        self.save = self.create_client(Trigger,'/save_3d_map')
        self.parameters = self.create_client(SetParameters,'/go2_edu_dds_bridge/set_parameters')
        self.motion_state = self.create_client(GetParameters,'/go2_edu_dds_bridge/get_parameters')
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String,'/operator/status',retained)
        self.recover_localization = self.create_publisher(Empty,'/localization/recover',1)
        self.waypoint_markers = self.create_publisher(MarkerArray,'/navigation/waypoints',retained)
        if self.mode == 'navigation':
            self.create_subscription(String,'/navigation/startup_status',
                lambda msg: setattr(self,'startup_status',msg.data),retained)
            for topic in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
                self.create_subscription(OccupancyGrid,topic,
                    lambda msg,topic=topic: self.grids.__setitem__(topic,(msg,time.monotonic())),retained)
        self.create_subscription(Bool,'/localization/valid',self.on_validity,1)
        self.create_subscription(String,'/localization/state',self.on_localization_state,1)
        self.create_subscription(String,'/localization/recovery',self.on_recovery,retained)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',
            self.on_initial_pose,1)
        self.create_subscription(Empty,'/mapping/save',self.on_save,1)
        self.create_subscription(Bool,'/control/enable',self.on_enable,1)
        self.create_subscription(Empty,'/navigation/cancel',self.on_cancel,1)
        self.create_subscription(Empty,'/navigation/resume',self.on_resume,1)
        self.create_subscription(PoseStamped,'/goal_pose',self.on_goal,100)
        # Retain the explicit single-click waypoint input used by older saved
        # layouts. The unrelated /clicked_point measuring tool stays separate.
        self.create_subscription(PointStamped,'/navigation/waypoint',self.on_waypoint,100)
        self.create_timer(0.1,self.watchdog)
        # Full snapshots also recover the numbered route after a panel
        # reconnects with a volatile subscription, without changing motion.
        if self.mode == 'navigation':
            self.create_timer(1.0,self.publish_waypoints)
        self.report(f'Operator ready: mode={self.mode}; collect /goal_pose waypoints then enable motion; execution requires live localization and free costmap cells')

    def report(self,text):
        if self.mode == 'navigation':
            current = self.inflight_waypoint[0] if self.inflight_waypoint else '-'
            text += f'; route current={current} queued={len(self.waypoints)} completed={self.completed_waypoints} paused={self.route_paused}'
            text += f' started={self.route_started}'
            if self.route_paused and self.auto_resume is None and self.last_auto_resume_revocation:
                text += f'; auto_resume_revoked={self.last_auto_resume_revocation}'
            self.publish_waypoints()
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    @property
    def waiting_goal(self):
        return self.waypoints[0][1] if self.waypoints else None

    def publish_waypoints(self):
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'map'
        markers = [clear]
        entries = ([self.inflight_waypoint] if self.inflight_waypoint else []) + list(self.waypoints)
        for number,pose in entries:
            active = self.inflight_waypoint is not None and number == self.inflight_waypoint[0]
            marker = Marker(id=number,ns='waypoints',type=Marker.SPHERE,action=Marker.ADD)
            marker.header.frame_id = 'map'
            marker.pose = copy.deepcopy(pose.pose)
            marker.pose.position.z = 0.12
            marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color.a = 1.0
            marker.color.r = 1.0 if active or self.route_paused else 0.1
            marker.color.g = 0.5 if active else 0.1 if self.route_paused else 0.9
            marker.color.b = 0.1
            markers.append(marker)
            label = copy.deepcopy(marker)
            label.ns, label.type, label.text = 'waypoint_numbers',Marker.TEXT_VIEW_FACING,str(number)
            label.pose.position.z = 0.32
            label.scale.z = 0.18
            markers.append(label)
        self.waypoint_markers.publish(MarkerArray(markers=markers))

    def localized(self):
        return self.valid and self.valid_received is not None and time.monotonic()-self.valid_received < 0.5

    def on_validity(self,message):
        self.valid, self.valid_received = message.data,time.monotonic()
        if not message.data and self.auto_resume is not None:
            self.auto_resume['stable_since'] = None

    def clear_auto_resume(self, reason=None):
        """Manual/new route intent supersedes the previous recovery authority."""
        previous = self.auto_resume
        self.auto_resume = None
        if previous is not None:
            self.last_auto_resume_revocation = reason or 'automatic recovery authorization cleared'
        if (previous is not None and previous['stage'] == 'enabling' and
                self.parameters.service_is_ready()):
            request = SetParameters.Request()
            request.parameters = [Parameter('enable_control',value=False).to_parameter_msg()]
            try:
                self.parameters.call_async(request)
            except Exception as error:
                self.get_logger().error(f'Automatic enable revoked; bridge disable request failed: {error}')
        if previous is not None and reason:
            self.report('Automatic resume authorization revoked: '+reason)

    def report_auto_wait(self, reason, detail=''):
        """Expose the actual gate without flooding status at watchdog rate."""
        intent = self.auto_resume
        if intent is None:
            return
        now = time.monotonic()
        key = (reason, intent['stage'])
        if intent.get('wait_report_key') == key and now-intent.get('wait_report_time', 0.) < 5.:
            return
        intent['wait_report_key'], intent['wait_report_time'] = key, now
        remaining = max(0., self.auto_resume_timeout-(now-intent['started']))
        self.report(f'Automatic resume waiting: reason={reason} stage={intent["stage"]} '
            f'baseline_epoch={intent["baseline_epoch"]} proof_epoch={intent["proof_epoch"]} '
            f'recovery_epoch={self.recovery_epoch} recovery_state={self.recovery_state} '
            f'remaining={remaining:.1f}s; {detail}')

    def request_queued_revalidation(self, intent):
        """Coalesce stale proof retries; repeated DDS samples send no flood."""
        epoch = intent.get('pending_revalidation_epoch')
        now = time.monotonic()
        if (self.auto_resume is not intent or epoch is None or
                now-intent.get('last_revalidation_at', float('-inf')) < 2.):
            return
        self.recover_localization.publish(Empty())
        intent['last_revalidation_at'] = now
        intent['last_stale_request_epoch'] = epoch
        intent['pending_revalidation_epoch'] = None

    def reject_stale_recovery(self, intent, epoch, age):
        # A recovered event ends the mapper's episode. Merely ignoring its
        # old scan timestamp would leave tracking=True with no future proof.
        # Keep the age gate and ask for NEW scans, within the original window.
        if intent['stage'] == 'enabling':
            # Invalidate the old parameter callback by identity before its
            # response can dispatch an action, and confirm a new disable.
            intent = dict(intent)
            self.auto_resume = intent
            self.disable_paused_bridge()
        intent.update(proof_epoch=None, stable_since=None, stage='waiting',
                      clears={}, cleared_at=None, baseline_epoch=epoch,
                      proof_rejection=f'recovered epoch={epoch} source_age={age:.3f}s rejected as stale')
        if epoch > intent.get('last_stale_request_epoch', -1):
            intent['pending_revalidation_epoch'] = epoch
        self.report_auto_wait('stale_recovery_proof', intent['proof_rejection']+'; requesting fresh mapper proof')

    def on_recovery(self, message):
        try:
            event = json.loads(message.data)
            epoch, state = event['epoch'], event['state']
            if not isinstance(epoch, int) or isinstance(epoch, bool) or epoch < self.recovery_epoch:
                return
            if state not in ('idle', 'searching', 'confirming', 'recovered', 'failed', 'manual_initialization'):
                return
        except (ValueError, KeyError, TypeError):
            return
        previous_epoch, previous_state = self.recovery_epoch, self.recovery_state
        self.recovery_epoch, self.recovery_state = epoch, state
        self.recovery_detail = str(event.get('reason', ''))
        if state == 'manual_initialization':
            # Pose and validity use independent DDS topics. Revoke authority
            # even if this event precedes Bool(false) and no pause intent yet
            # exists. Repeated delivery of the same event cannot veto a later
            # explicit user enable a second time.
            if epoch > previous_epoch or previous_state != state:
                self.on_initial_pose(None)
            return
        intent = self.auto_resume
        if intent is None:
            return
        if state == 'failed':
            self.clear_auto_resume(state)
            self.report('Automatic route recovery stopped: '+state+'; route retained; explicit enable required')
        elif state in ('searching', 'confirming'):
            if intent['stage'] == 'enabling':
                self.clear_auto_resume(state)
                return
            intent.update(proof_epoch=None, stable_since=None, stage='waiting', clears={}, cleared_at=None)
            pending = intent.get('pending_revalidation_epoch')
            if pending is not None and epoch > pending:
                intent['pending_revalidation_epoch'] = None
        elif (state == 'recovered' and epoch > intent['baseline_epoch'] and
              isinstance(event.get('confirmations'), int) and event['confirmations'] >= 3 and
              event.get('source_stamp') is not None and epoch != intent['proof_epoch']):
            try:
                stamp = float(event['source_stamp'])
                age = self.get_clock().now().nanoseconds/1e9-stamp
                if not math.isfinite(stamp) or not -.1 <= age <= 1.0:
                    if math.isfinite(stamp) and age > 1.0:
                        self.reject_stale_recovery(intent, epoch, age)
                    return
            except (ValueError, TypeError):
                return
            intent['proof_epoch'] = epoch
            intent['stable_since'] = None
            intent['pending_revalidation_epoch'] = None
            intent['proof_rejection'] = ''

    def on_initial_pose(self, message):
        if self.mode != 'navigation':
            return
        self.clear_auto_resume('manual initial pose')
        # Remove the prior user's executing-route authority BEFORE invoking
        # the shared pause path, so it cannot create automatic resume intent.
        self.route_started = False
        self.pause_for_localization()
        self.route_pause_reason = 'manual_initialization'
        self.report('Route paused: manual initial pose; waypoints retained; explicit enable required after localization recovers')

    def on_localization_state(self, message):
        self.localization_state, self.localization_state_received = message.data, time.monotonic()

    def localization_loss_detail(self):
        now = time.monotonic()
        age = now-self.valid_received if self.valid_received is not None else float('inf')
        reason = 'localization_heartbeat_timeout' if not 0 <= age < 0.5 else 'localization_reported_invalid'
        state_age = now-self.localization_state_received if self.localization_state_received is not None else float('inf')
        return (f'reason={reason} validity_age={age:.2f}s; latest_state_age={state_age:.2f}s '
                f'latest_mapper_{self.localization_state}')

    def watchdog(self):
        if not self.localized() and (self.goal_handle is not None or self.goal_pending or self.waiting_goal is not None):
            if not self.route_paused:
                self.pause_for_localization()
        self.try_automatic_resume()
        if self.enable_query is not None and time.monotonic()-self.enable_query[2] > 2.0:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
            self.defer_dispatch('motion bridge state query timed out')
        # A result callback is only one opportunity to advance the route.
        # Keep servicing an authorized tail after transient handoff failures.
        self.check_motion_state()

    def reset_dispatch_wait(self):
        self.dispatch_wait_started = None
        self.dispatch_retry_at = 0.0
        self.dispatch_wait_reason = ''

    def defer_dispatch(self, reason):
        self.dispatch_retry_at = time.monotonic()+0.5
        if reason != self.dispatch_wait_reason:
            self.dispatch_wait_reason = reason
            self.report(f'Waypoint dispatch waiting: {reason}; route retained; automatic retry')

    def pause_for_localization(self):
        # Only an already executing, explicitly enabled route grants recovery
        # authority. A collected route or bootstrap can never start itself.
        recover_running = (self.auto_resume_enabled and self.mode == 'navigation' and
                           self.route_started and not self.route_paused and
                           (self.goal_handle is not None or self.goal_pending or
                            self.inflight_waypoint is not None or self.enable_query is not None or
                            (self.dispatch_wait_started is not None and self.completed_waypoints > 0)))
        self.reset_dispatch_wait()
        self.goal_generation += 1
        self.route_started = False
        self.route_paused = True
        self.route_pause_reason = 'localization'
        self.pause_disable_confirmed = False
        self.auto_resume = (dict(generation=self.goal_generation, motion_generation=self.motion_generation,
            started=time.monotonic(), baseline_epoch=self.recovery_epoch, proof_epoch=None,
            stable_since=None, stage='waiting', clears={}, cleared_at=None,
            pending_revalidation_epoch=None, last_stale_request_epoch=-1,
            last_revalidation_at=float('-inf'), proof_rejection='')
            if recover_running else None)
        if self.auto_resume is not None:
            self.last_auto_resume_revocation = ''
        self.localization_pause_detail = self.localization_loss_detail()
        if self.inflight_waypoint is not None:
            self.waypoints.appendleft(self.inflight_waypoint)
            self.inflight_waypoint = None
        if self.enable_query is not None:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
        if self.goal_pending:
            self.cancel_requested = True
        self.disable_paused_bridge()
        self.cancel_active_goal()
        if self.auto_resume is not None:
            self.recover_localization.publish(Empty())
            self.auto_resume['last_revalidation_at'] = time.monotonic()
        self.report('Route paused: live localization lost; '+self.localization_pause_detail+
                    ('; waypoints retained; automatic resume awaits verified recovery, stable TF and cleared costmaps'
                     if self.auto_resume is not None else
                     '; waypoints retained; enable motion after localization recovers'))

    def disable_paused_bridge(self):
        # Canceling an action is asynchronous. Confirm that the real bridge
        # is disabled as well before any later recovery can authorize motion.
        self.pause_disable_confirmed = False
        if self.parameters.service_is_ready() and self.pause_disable_future is None:
            request = SetParameters.Request()
            request.parameters = [Parameter('enable_control',value=False).to_parameter_msg()]
            future = self.parameters.call_async(request)
            self.pause_disable_future = future
            generation = self.goal_generation
            def disabled(result):
                if self.pause_disable_future is result:
                    self.pause_disable_future = None
                if generation != self.goal_generation:
                    return
                try:
                    results = result.result().results
                    if len(results)!=1 or not results[0].successful:
                        self.report('Route paused; motion bridge disable rejected; action cancellation requested')
                    else:
                        self.pause_disable_confirmed = True
                except Exception as error:
                    self.report(f'Route paused; motion bridge disable failed: {error}; action cancellation requested')
            future.add_done_callback(disabled)

    def validate_recovered_route(self, require_costmaps=True):
        """Require both the retained target and the recovered body to be free."""
        if require_costmaps:
            self.validate_goal(self.waiting_goal)
        else:
            self.validate_map_goal(self.waiting_goal)
        topics = ('/map', '/global_costmap/costmap', '/local_costmap/costmap') if require_costmaps else ('/map',)
        for topic in topics:
            grid, _ = self.grids[topic]
            transform = self.buffer.lookup_transform(grid.header.frame_id,'base_footprint',Time())
            age = (self.get_clock().now()-Time.from_msg(transform.header.stamp)).nanoseconds/1e9
            if not -0.1 <= age <= .5:
                raise ValueError(f'{topic}: recovered robot TF stale age={age:.3f}s')
            xy = [transform.transform.translation.x, transform.transform.translation.y]
            cell = grid_cell(grid, xy)
            if cell is None:
                raise ValueError(f'{topic}: recovered robot XY=({xy[0]:.3f},{xy[1]:.3f}) outside grid')
            value = grid.data[cell[1]*grid.info.width+cell[0]]
            if value < 0 or value >= 99:
                raise ValueError(f'{topic}: recovered robot XY=({xy[0]:.3f},{xy[1]:.3f}) '
                                 f'cell={cell} value={value} is not free')

    def try_automatic_resume(self):
        intent = self.auto_resume
        if intent is None:
            return
        if (intent['generation'] != self.goal_generation or intent['motion_generation'] != self.motion_generation or
                not self.route_paused or self.route_pause_reason != 'localization' or not self.waypoints):
            self.clear_auto_resume()
            return
        now = time.monotonic()
        if now-intent['started'] > self.auto_resume_timeout:
            self.clear_auto_resume('recovery timeout')
            self.report('Automatic recovery wait expired; route retained; explicit enable required')
            return
        self.request_queued_revalidation(intent)
        if not self.localized():
            intent['stable_since'] = None
            if intent['stage'] == 'enabling':
                self.clear_auto_resume()
                self.report('Automatic resume stopped: localization lost during enable; explicit enable required')
            elif intent['proof_epoch'] is not None:
                # A second loss invalidates proof from the first recovery.
                # In particular a heartbeat gap needs a new mapper request
                # even when its own sensor validity never became false.
                intent.update(proof_epoch=None, baseline_epoch=self.recovery_epoch,
                              stage='waiting', clears={}, cleared_at=None)
                self.recover_localization.publish(Empty())
                intent['last_revalidation_at'] = now
            self.report_auto_wait('localization_unavailable', self.localization_loss_detail())
            return
        if intent['proof_epoch'] is None:
            self.report_auto_wait('recovery_proof',
                f'{intent.get("proof_rejection") or self.recovery_detail}; '
                f'queued_revalidation_epoch={intent.get("pending_revalidation_epoch")}')
            return
        if not self.pause_disable_confirmed or self.pause_disable_future is not None:
            self.report_auto_wait('bridge_disable', f'confirmed={self.pause_disable_confirmed} '
                f'pending={self.pause_disable_future is not None}')
            return
        if self.goal_handle is not None or self.goal_pending or self.cancelling:
            self.report_auto_wait('action_cancel', f'handle={self.goal_handle is not None} '
                f'goal_pending={self.goal_pending} cancelling={self.cancelling}')
            return
        if not self.action.server_is_ready():
            self.report_auto_wait('nav2_action_unavailable', self.startup_status)
            return
        try:
            # Old rolling grids can contain ghosts at the previous pose.
            # Clear them after the static map/TF gate, then validate all NEW
            # costmaps before restoring any movement authority.
            self.validate_recovered_route(require_costmaps=intent['stage'] in ('refreshing', 'enabling'))
        except (ValueError, TransformException, KeyError) as error:
            intent['stable_since'] = None
            text = str(error)
            topic = next((topic for topic in ('/global_costmap/costmap', '/local_costmap/costmap', '/map')
                          if topic in text), '')
            category = ('tf' if isinstance(error, TransformException) or 'TF' in text else
                        'body' if 'recovered robot' in text else 'goal_or_grid')
            self.report_auto_wait(f'validation:{category}:{topic}', text)
            return
        if intent['stable_since'] is None:
            intent['stable_since'] = now
            self.report_auto_wait('stabilizing', f'required={self.auto_resume_stable:.1f}s')
            return
        if now-intent['stable_since'] < self.auto_resume_stable:
            self.report_auto_wait('stabilizing', f'stable_age={now-intent["stable_since"]:.2f}s '
                                  f'required={self.auto_resume_stable:.1f}s')
            return
        if intent['stage'] == 'waiting':
            if not all(client.service_is_ready() for client in self.costmap_clear.values()):
                missing = [name for name,client in self.costmap_clear.items() if not client.service_is_ready()]
                self.report_auto_wait('clear_services', f'unavailable={missing}; explicit enable remains available')
                return
            intent['stage'] = 'clearing'
            try:
                for name, client in self.costmap_clear.items():
                    future = client.call_async(self.clear_type.Request())
                    intent['clears'][name] = future
            except Exception as error:
                self.clear_auto_resume()
                self.report(f'Automatic resume stopped: cannot clear costmaps: {error}; explicit enable required')
                return
            self.report('Verified localization recovery; clearing both Nav2 costmaps before route resume')
            return
        if intent['stage'] == 'clearing':
            if not all(future.done() for future in intent['clears'].values()):
                self.report_auto_wait('costmap_clear',
                    f'completed={[(name,future.done()) for name,future in intent["clears"].items()]}')
                return
            try:
                for future in intent['clears'].values():
                    if future.result() is None:
                        raise ValueError('missing costmap clear response')
            except Exception as error:
                self.clear_auto_resume()
                self.report(f'Automatic resume stopped: costmap clear failed: {error}; explicit enable required')
                return
            intent['stage'] = 'refreshing'
            intent['cleared_at'] = now
            intent['cleared_stamp'] = self.get_clock().now().nanoseconds
            return
        if intent['stage'] == 'refreshing':
            for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
                grid, received = self.grids[topic]
                if (received <= intent['cleared_at'] or
                        Time.from_msg(grid.header.stamp).nanoseconds < intent['cleared_stamp']):
                    grid_age = (self.get_clock().now()-Time.from_msg(grid.header.stamp)).nanoseconds/1e9
                    self.report_auto_wait('costmap_refresh:'+topic,
                        f'{topic} receipt_age={now-received:.3f}s header_age={grid_age:.3f}s; '
                        'waiting for a grid produced after both clear responses')
                    return
            intent['stage'] = 'enabling'
            self.on_enable(Bool(data=True), automatic=True)
        elif intent['stage'] == 'enabling':
            self.report_auto_wait('bridge_enable', 'waiting for parameter acknowledgment; route remains paused')

    def on_save(self,message):
        if self.mode != 'mapping':
            self.report('Save rejected: switch to mapping mode')
            return
        if not self.save.service_is_ready():
            self.report('3D map save service unavailable')
            return
        future = self.save.call_async(Trigger.Request())
        def done(result):
            try:
                response = result.result()
                self.report(f'Save success={response.success}: {response.message}')
            except Exception as error:
                self.report(f'Save failed: {error}')
        future.add_done_callback(done)

    def on_enable(self,message,automatic=False):
        intent = self.auto_resume if automatic else None
        if automatic and intent is None:
            return
        if not automatic:
            self.clear_auto_resume('manual enable' if message.data else 'manual disable')
            if message.data:
                self.last_auto_resume_revocation = ''
            self.motion_generation += 1
        generation = self.motion_generation
        route_generation = self.goal_generation
        enabled = bool(message.data)
        start_route = enabled and self.waiting_goal is not None
        if not enabled:
            # Cancellation must still work if the parameter service is down.
            self.on_cancel(Empty())
            route_generation = self.goal_generation
        if message.data and (self.mode != 'navigation' or not self.localized()):
            if automatic:
                self.clear_auto_resume()
            self.report('Enable rejected: navigation mode and live GICP localization required')
            return
        if enabled and self.pause_disable_future is not None:
            if automatic:
                self.clear_auto_resume()
            self.report('Enable deferred: waiting for paused motion to disable; enable again after it stops')
            return
        if enabled and not self.action.server_is_ready():
            if automatic:
                self.clear_auto_resume()
            self.report(f'Enable rejected: Nav2 NavigateToPose action unavailable; {self.startup_status}')
            return
        if not self.parameters.service_is_ready():
            if automatic:
                self.clear_auto_resume()
            self.report('Motion bridge parameter service unavailable')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter('enable_control',value=bool(message.data)).to_parameter_msg()]
        future = self.parameters.call_async(request)
        def done(result):
            if generation != self.motion_generation or route_generation != self.goal_generation:
                return
            if automatic and self.auto_resume is not intent:
                return
            try:
                results = result.result().results
                accepted = len(results)==1 and results[0].successful
                text = 'Motion enable=%s; parameter accepted=%s' % (enabled,accepted)
                if not accepted:
                    if automatic:
                        self.clear_auto_resume()
                    text += '; '+('; '.join(r.reason for r in results) or 'missing parameter result')
                elif start_route and self.waiting_goal is not None:
                    if not self.localized():
                        self.pause_for_localization()
                        return
                    if automatic:
                        try:
                            self.validate_recovered_route()
                        except (ValueError, TransformException, KeyError) as error:
                            self.clear_auto_resume()
                            self.report(f'Automatic resume stopped: {error}; route retained; explicit enable required')
                            return
                    if self.route_paused and self.route_pause_reason not in ('localization', 'manual_initialization'):
                        self.report(text+'; previous goal failed; fix route and send a new goal after cancelling, or tools.sh resume to retry; route paused')
                        return
                    self.route_paused = False
                    self.route_pause_reason = None
                    self.route_started = True
                    if automatic:
                        self.auto_resume = None
                        text += '; verified automatic localization recovery; retrying retained waypoint'
                    self.report(text)
                    self.check_motion_state()
                    return
                elif enabled and self.goal_handle is None and not self.goal_pending:
                    text += ('; previous goal failed; fix route and send a new goal' if self.last_goal_status==6
                             else '; no active navigation goal; select /goal_pose waypoints then enable motion to start')
                self.report(text)
            except Exception as error:
                if automatic:
                    self.clear_auto_resume()
                self.report(f'Motion parameter failed: {error}')
        future.add_done_callback(done)

    def on_cancel(self,message):
        self.clear_auto_resume('manual cancel')
        self.reset_dispatch_wait()
        self.goal_generation += 1
        self.route_paused = False
        self.route_pause_reason = None
        self.route_started = False
        if self.waiting_goal is not None:
            self.waypoints.clear()
            self.report('Queued navigation goal cancelled')
        if self.enable_query is not None:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
        if self.goal_pending:
            self.cancel_requested = True
        self.cancel_active_goal()
        self.publish_waypoints()

    def cancel_active_goal(self):
        if self.goal_handle is not None and not self.cancelling:
            handle = self.goal_handle
            self.cancelling = True
            future = handle.cancel_goal_async()
            self.report('Navigation cancellation requested')
            future.add_done_callback(lambda f: self.report('Navigation cancel response received'))

    def on_resume(self,message):
        self.clear_auto_resume('manual resume')
        if not self.route_paused or not self.waypoints:
            self.report('No paused waypoint route to resume')
            return
        if not self.localized():
            self.report('Resume rejected: live GICP localization required')
            return
        self.route_paused = False
        self.route_pause_reason = None
        self.route_started = True
        self.report('Retrying paused waypoint; remaining order preserved')
        self.check_motion_state()

    def on_waypoint(self,message):
        pose = PoseStamped()
        pose.header = copy.deepcopy(message.header)
        pose.pose.position = copy.deepcopy(message.point)
        pose.pose.orientation.w = 1.0
        self.on_goal(pose)

    def on_goal(self,message):
        if self.action is None:
            self.report('Goal rejected: navigation mode required')
            return
        if message.header.frame_id != 'map':
            self.report(f'Goal rejected: frame_id={message.header.frame_id!r}; set the Foxglove 3D display frame to map')
            return
        try:
            p,q = message.pose.position,message.pose.orientation
            pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        except ValueError as error:
            self.report(f'Goal rejected: {error}')
            return
        if self.cancelling or (self.goal_pending and self.cancel_requested):
            self.report('Goal rejected: wait for current cancellation to finish')
            return
        if len(self.waypoints) + int(self.inflight_waypoint is not None) >= 100:
            self.report('Goal rejected: waypoint queue limit is 100')
            return
        try:
            # Collection is map-relative and must survive Nav2 startup,
            # transient TF/costmap loss and rolling-window changes. Every
            # dispatch still rechecks all live conditions in send_waiting_goal.
            self.validate_map_goal(message)
        except (ValueError,TransformException) as error:
            self.report(f'Goal rejected: {error}')
            return
        self.clear_auto_resume('new waypoint')
        # New routes are collected until an explicit Enable, even if the
        # bridge remained enabled after the previous route completed.
        if not self.waypoints and self.inflight_waypoint is None:
            self.waypoint_number = self.completed_waypoints = 0
        self.waypoint_number += 1
        self.waypoints.append((self.waypoint_number,copy.deepcopy(message)))
        self.report(f'Waypoint #{self.waypoint_number} appended at ({message.pose.position.x:.2f}, {message.pose.position.y:.2f})')
        if not self.localized():
            if not self.route_paused:
                self.pause_for_localization()
        elif self.route_started:
            self.check_motion_state()
        else:
            self.report('Goal queued; collect remaining /goal_pose waypoints, then enable motion once to start the route')

    def check_motion_state(self):
        if (not self.route_started or self.waiting_goal is None or self.route_paused or
                self.goal_handle is not None or self.goal_pending or self.cancelling):
            return
        if not self.localized():
            self.pause_for_localization()
            return
        if self.enable_query is not None:
            return
        now = time.monotonic()
        if now < self.dispatch_retry_at:
            return
        if self.dispatch_wait_started is None:
            self.dispatch_wait_started = now
        elif now-self.dispatch_wait_started >= 15.0:
            reason = self.dispatch_wait_reason
            self.reset_dispatch_wait()
            self.route_paused, self.route_started = True, False
            self.route_pause_reason = 'validation'
            self.report(f'Waypoint dispatch timed out: {reason}; route retained; publish /navigation/resume after fixing readiness')
            return
        if not self.motion_state.service_is_ready():
            self.defer_dispatch('motion bridge state service unavailable')
            return
        generation = self.goal_generation
        request = GetParameters.Request(names=['enable_control'])
        try:
            future = self.motion_state.call_async(request)
        except Exception as error:
            self.defer_dispatch(f'motion state query failed: {error}')
            return
        self.enable_query = (future,generation,time.monotonic())
        def done(result):
            if self.enable_query is None or self.enable_query[0] is not result:
                return
            self.enable_query = None
            if (generation != self.goal_generation or self.waiting_goal is None or
                    self.route_paused or not self.route_started):
                return
            try:
                values = result.result().values
                if len(values)!=1 or values[0].type!=Parameter.Type.BOOL.value:
                    raise ValueError('enable_control is not a boolean parameter')
                if not values[0].bool_value:
                    self.reset_dispatch_wait()
                    self.route_started = False
                    p = self.waiting_goal.pose.position
                    self.report(f'Goal queued at ({p.x:.2f}, {p.y:.2f}); motion disabled; enable motion to start Nav2')
                    return
            except Exception as error:
                self.defer_dispatch(f'motion state query failed: {error}')
                return
            self.send_waiting_goal()
        future.add_done_callback(done)

    def send_waiting_goal(self):
        message = self.waiting_goal
        if (message is None or not self.route_started or self.route_paused or
                self.goal_handle is not None or self.goal_pending or self.cancelling):
            return
        if not self.localized():
            self.pause_for_localization()
            return
        # A held waypoint may outlive a map update or localization correction.
        try:
            if not self.action.server_is_ready():
                raise DispatchNotReady('Nav2 NavigateToPose action unavailable')
            self.validate_goal(message)
        except (DispatchNotReady,TransformException) as error:
            self.defer_dispatch(str(error))
            return
        except ValueError as error:
            self.reset_dispatch_wait()
            self.route_paused = True
            self.route_started = False
            self.route_pause_reason = 'validation'
            self.report(f'Queued goal rejected on start: {error}; route paused; fix and publish /navigation/resume or cancel route')
            return
        self.reset_dispatch_wait()
        waypoint = self.waypoints.popleft()
        self.inflight_waypoint = waypoint
        generation = self.goal_generation
        p,q = message.pose.position,message.pose.orientation
        goal = self.goal_type.Goal()
        goal.pose = copy.deepcopy(message)
        # This is a fixed map waypoint, not a body-relative historical pose.
        # Normalize its planar orientation and use latest-time semantics.
        goal.pose.header.stamp = Time().to_msg()
        pose = planar_pose(pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
        goal.pose.pose.position.z = 0.0
        yaw = quaternion_from_matrix(pose)
        goal.pose.pose.orientation.x,goal.pose.pose.orientation.y,goal.pose.pose.orientation.z,goal.pose.pose.orientation.w = map(float,yaw)
        goal.behavior_tree = '/opt/go2_project/go2_3d/navigate.xml'
        self.goal_pending,self.cancel_requested = True,False
        try:
            future = self.action.send_goal_async(goal)
            self.report('Map goal sent to Nav2 action')
            future.add_done_callback(lambda result: self.goal_response(result,goal.pose,waypoint,generation))
        except Exception as error:
            self.goal_pending = False
            self.pause_waypoint(waypoint,generation)
            self.report(f'Nav2 goal request failed: {error}')
            return

    def validate_map_goal(self,message):
        sample = self.grids.get('/map')
        if sample is None:
            raise ValueError('/map unavailable; wait for the saved map')
        grid,_ = sample
        if (grid.header.frame_id != 'map' or grid.info.resolution <= 0 or
                not grid.info.width or not grid.info.height or
                len(grid.data) != grid.info.width*grid.info.height):
            raise ValueError('/map invalid grid')
        index = grid_cell(grid,[message.pose.position.x,message.pose.position.y])
        if index is None:
            raise ValueError('/map: goal outside grid')
        value = grid.data[index[1]*grid.info.width+index[0]]
        if value < 0 or value >= 99:
            raise ValueError(f'/map: goal cell={index} value={value} is not free')

    def validate_goal(self,message):
        tf = self.buffer.lookup_transform('map','base_footprint',Time())
        age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        if not -0.1 <= age <= 0.5:
            raise DispatchNotReady(f'map -> base_footprint TF stale ({age:.2f}s)')
        xy = [message.pose.position.x,message.pose.position.y]
        for topic in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
            sample = self.grids.get(topic)
            if sample is None:
                raise DispatchNotReady(f'{topic} unavailable; wait for Nav2 active')
            grid,received = sample
            if topic != '/map' and time.monotonic()-received > 3.0:
                raise DispatchNotReady(f'{topic} stale; wait for live localization/costmaps')
            if (grid.info.resolution <= 0 or not grid.info.width or not grid.info.height or
                    len(grid.data) != grid.info.width*grid.info.height):
                raise ValueError(f'{topic} invalid grid')
            target = xy
            if grid.header.frame_id != 'map':
                tf = self.buffer.lookup_transform(grid.header.frame_id,'map',Time())
                t,q = tf.transform.translation,tf.transform.rotation
                target = (pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])@[xy[0],xy[1],0.0,1.0])[:2]
            index = grid_cell(grid,target)
            if index is None:
                if topic == '/local_costmap/costmap':
                    # A rolling local window need not contain a distant map
                    # waypoint; Nav2 checks local obstacles along the path.
                    continue
                raise ValueError(f'{topic}: goal outside grid')
            value = grid.data[index[1]*grid.info.width+index[0]]
            if value < 0 or value >= 99:
                reason = 'unknown' if value < 0 else 'occupied' if value == 100 else 'inside obstacle clearance'
                raise ValueError(f'{topic}: goal ({xy[0]:.2f}, {xy[1]:.2f}) {reason}, cell={index} value={value}')

    def pause_waypoint(self,waypoint,generation):
        self.inflight_waypoint = None
        if generation == self.goal_generation:
            self.reset_dispatch_wait()
            self.clear_auto_resume('navigation failure')
            self.waypoints.appendleft(waypoint)
            self.route_paused = True
            self.route_started = False
            self.route_pause_reason = 'navigation'

    def goal_response(self,future,goal_pose,waypoint,generation):
        self.goal_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.clear_auto_resume('navigation rejected')
                self.pause_waypoint(waypoint,generation)
                self.report('Nav2 rejected goal; route paused; publish /navigation/resume to retry')
                if generation != self.goal_generation and self.route_started:
                    self.check_motion_state()
                return
            self.goal_handle = handle
            self.last_goal_status = None
            self.report('Nav2 accepted goal')
            handle.get_result_async().add_done_callback(lambda result: self.goal_result(result,goal_pose,waypoint,generation))
            if generation != self.goal_generation or self.cancel_requested:
                self.cancel_active_goal()
            elif not self.localized():
                self.pause_for_localization()
        except Exception as error:
            self.clear_auto_resume('navigation goal response failed')
            self.pause_waypoint(waypoint,generation)
            self.report(f'Nav2 goal request failed: {error}')
            if generation != self.goal_generation and self.route_started:
                self.check_motion_state()

    def goal_error_text(self,goal_pose):
        try:
            tf = self.buffer.lookup_transform('map','base_footprint',Time())
            age = (self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
            if not -0.1 <= age <= 0.5:
                return '; final map pose unavailable (stale TF)'
            p = tf.transform.translation
            error = math.hypot(p.x-goal_pose.pose.position.x,p.y-goal_pose.pose.position.y)
            return f'; final XY error={error:.3f}m; arrival radius={ARRIVAL_RADIUS:.2f}m; final heading unrestricted'
        except TransformException:
            return '; final map pose unavailable (TF lookup failed)'

    def goal_result(self,future,goal_pose,waypoint,generation):
        self.goal_handle = None
        self.cancelling = False
        self.inflight_waypoint = None
        try:
            status = future.result().status
        except Exception as error:
            self.clear_auto_resume('navigation result failed')
            self.pause_waypoint(waypoint,generation)
            self.report(f'Nav2 result failed: {error}')
            return
        self.last_goal_status = status
        if status not in (4,5):
            self.clear_auto_resume('navigation failed during localization pause')
        current = generation == self.goal_generation
        if current and status == 4:
            self.completed_waypoints += 1
            if self.waypoints and self.route_started and not self.route_paused:
                # Preserve executing-route authority even if localization
                # disappears in the gap before the next bridge query.
                self.dispatch_wait_started = time.monotonic()
        elif current:
            self.pause_waypoint(waypoint,generation)
        text = f'Nav2 goal completed: action status={status}'
        if status==4:
            text += ' (SUCCEEDED); reached waypoint'
        elif status==6:
            text += ' (ABORTED); no active goal; enabling motion does not retry the failed goal'
        elif status==5:
            text += (' (CANCELED); localization pause; waypoints retained'
                     if self.route_pause_reason == 'localization' else
                     ' (CANCELED); manual initial pose; waypoints retained'
                     if self.route_pause_reason == 'manual_initialization' else ' (CANCELED); route cancelled')
            if self.route_pause_reason == 'localization':
                text += '; '+self.localization_pause_detail
        if current and status==4 and not self.waypoints:
            self.route_started = False
            text += '; all queued waypoints completed'
        if current and status != 4:
            text += '; previous goal failed; fix route and send a new goal after cancelling, or publish /navigation/resume to retry; route paused'
        # Optional arrival diagnostics must never put a SUCCEEDED waypoint
        # back into the queue or turn a completed action into a route failure.
        try:
            text += self.goal_error_text(goal_pose)
        except Exception as error:
            text += f'; final map pose diagnostic unavailable: {error}'
        self.report(text)
        if self.route_started and not self.route_paused:
            self.check_motion_state()


def main():
    rclpy.init()
    node = Operator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
