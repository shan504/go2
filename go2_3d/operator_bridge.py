#!/usr/bin/env python3
"""Foxglove publish-panel operations and /goal_pose -> Nav2 action gateway."""
import copy
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
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry import pose_matrix, grid_cell, quaternion_from_matrix, planar_pose
from motion_profile import ARRIVAL_RADIUS


class Operator(Node):
    def __init__(self):
        super().__init__('go2_operator')
        self.declare_parameter('mode','mapping')
        self.mode = self.get_parameter('mode').value
        self.valid = False
        self.valid_received = None
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
        self.pause_disable_future = None
        self.startup_status = 'startup status not received'
        self.grids = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.action = None
        if self.mode == 'navigation':
            from nav2_msgs.action import NavigateToPose
            self.goal_type = NavigateToPose
            self.action = ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.save = self.create_client(Trigger,'/save_3d_map')
        self.parameters = self.create_client(SetParameters,'/go2_edu_dds_bridge/set_parameters')
        self.motion_state = self.create_client(GetParameters,'/go2_edu_dds_bridge/get_parameters')
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String,'/operator/status',retained)
        self.waypoint_markers = self.create_publisher(MarkerArray,'/navigation/waypoints',retained)
        if self.mode == 'navigation':
            self.create_subscription(String,'/navigation/startup_status',
                lambda msg: setattr(self,'startup_status',msg.data),retained)
            for topic in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
                self.create_subscription(OccupancyGrid,topic,
                    lambda msg,topic=topic: self.grids.__setitem__(topic,(msg,time.monotonic())),retained)
        self.create_subscription(Bool,'/localization/valid',self.on_validity,1)
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

    def watchdog(self):
        if not self.localized() and (self.goal_handle is not None or self.goal_pending or self.waiting_goal is not None):
            if not self.route_paused:
                self.pause_for_localization()
        if self.enable_query is not None and time.monotonic()-self.enable_query[2] > 2.0:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
            self.on_cancel(Empty())
            self.report('Goal rejected: motion bridge state query timed out')

    def pause_for_localization(self):
        # Retain map waypoints, but stop/cancel the action. Recovery alone
        # never resumes motion: the user must explicitly enable the route.
        self.goal_generation += 1
        self.route_started = False
        self.route_paused = True
        self.route_pause_reason = 'localization'
        if self.inflight_waypoint is not None:
            self.waypoints.appendleft(self.inflight_waypoint)
            self.inflight_waypoint = None
        if self.enable_query is not None:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
        if self.goal_pending:
            self.cancel_requested = True
        # Canceling an action is asynchronous. Disable the real bridge too,
        # so newly arriving controller commands cannot move it on recovery.
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
                except Exception as error:
                    self.report(f'Route paused; motion bridge disable failed: {error}; action cancellation requested')
            future.add_done_callback(disabled)
        self.cancel_active_goal()
        self.report('Route paused: live localization lost; waypoints retained; enable motion after localization recovers')

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

    def on_enable(self,message):
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
            self.report('Enable rejected: navigation mode and live GICP localization required')
            return
        if enabled and self.pause_disable_future is not None:
            self.report('Enable deferred: waiting for paused motion to disable; enable again after it stops')
            return
        if enabled and not self.action.server_is_ready():
            self.report(f'Enable rejected: Nav2 NavigateToPose action unavailable; {self.startup_status}')
            return
        if not self.parameters.service_is_ready():
            self.report('Motion bridge parameter service unavailable')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter('enable_control',value=bool(message.data)).to_parameter_msg()]
        future = self.parameters.call_async(request)
        def done(result):
            if generation != self.motion_generation or route_generation != self.goal_generation:
                return
            try:
                results = result.result().results
                accepted = len(results)==1 and results[0].successful
                text = 'Motion enable=%s; parameter accepted=%s' % (enabled,accepted)
                if not accepted:
                    text += '; '+('; '.join(r.reason for r in results) or 'missing parameter result')
                elif start_route and self.waiting_goal is not None:
                    if not self.localized():
                        if not self.route_paused:
                            self.pause_for_localization()
                        return
                    if self.route_paused and self.route_pause_reason != 'localization':
                        self.report(text+'; previous goal failed; fix route and send a new goal after cancelling, or tools.sh resume to retry; route paused')
                        return
                    self.route_paused = False
                    self.route_pause_reason = None
                    self.route_started = True
                    self.report(text)
                    self.check_motion_state()
                    return
                elif enabled and self.goal_handle is None and not self.goal_pending:
                    text += ('; previous goal failed; fix route and send a new goal' if self.last_goal_status==6
                             else '; no active navigation goal; select /goal_pose waypoints then enable motion to start')
                self.report(text)
            except Exception as error:
                self.report(f'Motion parameter failed: {error}')
        future.add_done_callback(done)

    def on_cancel(self,message):
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
                self.goal_handle is not None or self.goal_pending):
            return
        if self.enable_query is not None:
            return
        if not self.motion_state.service_is_ready():
            self.on_cancel(Empty())
            self.report('Goal rejected: motion bridge state service unavailable')
            return
        generation = self.goal_generation
        request = GetParameters.Request(names=['enable_control'])
        future = self.motion_state.call_async(request)
        self.enable_query = (future,generation,time.monotonic())
        def done(result):
            if self.enable_query is None or self.enable_query[0] is not result:
                return
            self.enable_query = None
            if generation != self.goal_generation or self.waiting_goal is None or self.route_paused:
                return
            try:
                values = result.result().values
                if len(values)!=1 or values[0].type!=Parameter.Type.BOOL.value:
                    raise ValueError('enable_control is not a boolean parameter')
                if not values[0].bool_value:
                    self.route_started = False
                    p = self.waiting_goal.pose.position
                    self.report(f'Goal queued at ({p.x:.2f}, {p.y:.2f}); motion disabled; enable motion to start Nav2')
                    return
                self.send_waiting_goal()
            except Exception as error:
                self.on_cancel(Empty())
                self.report(f'Goal rejected: motion state query failed: {error}')
        future.add_done_callback(done)

    def send_waiting_goal(self):
        message = self.waiting_goal
        if message is None:
            return
        # A held waypoint may outlive a map update or localization correction.
        try:
            if not self.localized():
                raise ValueError('live GICP localization required')
            if not self.action.server_is_ready():
                raise ValueError('Nav2 NavigateToPose action unavailable')
            self.validate_goal(message)
        except (ValueError,TransformException) as error:
            self.route_paused = True
            self.route_started = False
            self.route_pause_reason = 'validation'
            self.report(f'Queued goal rejected on start: {error}; route paused; fix and publish /navigation/resume or cancel route')
            return
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
            raise ValueError(f'map -> base_footprint TF stale ({age:.2f}s)')
        xy = [message.pose.position.x,message.pose.position.y]
        for topic in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
            sample = self.grids.get(topic)
            if sample is None:
                raise ValueError(f'{topic} unavailable; wait for Nav2 active')
            grid,received = sample
            if topic != '/map' and time.monotonic()-received > 3.0:
                raise ValueError(f'{topic} stale; wait for live localization/costmaps')
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
            self.waypoints.appendleft(waypoint)
            self.route_paused = True
            self.route_started = False
            self.route_pause_reason = 'navigation'

    def goal_response(self,future,goal_pose,waypoint,generation):
        self.goal_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
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
            self.last_goal_status = status
            current = generation == self.goal_generation
            if current and status == 4:
                self.completed_waypoints += 1
            elif current:
                self.pause_waypoint(waypoint,generation)
            text = f'Nav2 goal completed: action status={status}'
            if status==4:
                text += ' (SUCCEEDED); reached waypoint'
            elif status==6:
                text += ' (ABORTED); no active goal; enabling motion does not retry the failed goal'
            elif status==5:
                text += (' (CANCELED); localization pause; waypoints retained'
                         if self.route_pause_reason == 'localization' else ' (CANCELED); route cancelled')
            if current and status==4 and not self.waypoints:
                self.route_started = False
                text += '; all queued waypoints completed'
            if current and status != 4:
                text += '; previous goal failed; fix route and send a new goal after cancelling, or publish /navigation/resume to retry; route paused'
            text += self.goal_error_text(goal_pose)
            self.report(text)
            if self.route_started and not self.route_paused:
                self.check_motion_state()
        except Exception as error:
            self.pause_waypoint(waypoint,generation)
            self.report(f'Nav2 result failed: {error}')


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
