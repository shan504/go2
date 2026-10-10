#!/usr/bin/env python3
"""Foxglove publish-panel operations and /goal_pose -> Nav2 action gateway."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
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
        self.waiting_goal = None
        self.goal_generation = 0
        self.motion_generation = 0
        self.enable_query = None
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
        self.create_subscription(PoseStamped,'/goal_pose',self.on_goal,1)
        self.create_timer(0.1,self.watchdog)
        self.report(f'Operator ready: mode={self.mode}; goals require live localization and free costmap cells')

    def report(self,text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def localized(self):
        return self.valid and self.valid_received is not None and time.monotonic()-self.valid_received < 0.5

    def on_validity(self,message):
        self.valid, self.valid_received = message.data,time.monotonic()

    def watchdog(self):
        if not self.localized() and (self.goal_handle is not None or self.goal_pending or self.waiting_goal is not None):
            if self.waiting_goal is not None:
                self.report('Queued goal discarded: live localization lost; select a new goal after relocalizing')
            self.on_cancel(Empty())
        if self.enable_query is not None and time.monotonic()-self.enable_query[2] > 2.0:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
            self.on_cancel(Empty())
            self.report('Goal rejected: motion bridge state query timed out')

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
        enabled = bool(message.data)
        if not enabled:
            # Cancellation must still work if the parameter service is down.
            self.on_cancel(Empty())
        if message.data and (self.mode != 'navigation' or not self.localized()):
            self.report('Enable rejected: navigation mode and live GICP localization required')
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
            if generation != self.motion_generation:
                return
            try:
                results = result.result().results
                accepted = len(results)==1 and results[0].successful
                text = 'Motion enable=%s; parameter accepted=%s' % (enabled,accepted)
                if not accepted:
                    text += '; '+('; '.join(r.reason for r in results) or 'missing parameter result')
                elif enabled and self.waiting_goal is not None:
                    self.report(text)
                    self.check_motion_state()
                    return
                elif enabled and self.goal_handle is None and not self.goal_pending:
                    text += ('; previous goal failed; fix route and send a new goal' if self.last_goal_status==6
                             else '; no active navigation goal; publish /goal_pose to start')
                self.report(text)
            except Exception as error:
                self.report(f'Motion parameter failed: {error}')
        future.add_done_callback(done)

    def on_cancel(self,message):
        self.goal_generation += 1
        if self.waiting_goal is not None:
            self.waiting_goal = None
            self.report('Queued navigation goal cancelled')
        if self.enable_query is not None:
            future,_,_ = self.enable_query
            self.enable_query = None
            self.motion_state.remove_pending_request(future)
        if self.goal_pending:
            self.cancel_requested = True
        if self.goal_handle is not None and not self.cancelling:
            handle = self.goal_handle
            self.cancelling = True
            future = handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.report('Navigation cancel response received'))
            self.report('Navigation cancellation requested')

    def on_goal(self,message):
        if self.action is None or not self.localized():
            self.report('Goal rejected: navigation mode and live GICP localization required')
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
        if self.goal_handle is not None or self.goal_pending:
            self.report('Goal rejected: cancel the current goal before selecting another')
            return
        if not self.action.server_is_ready():
            self.report(f'Goal rejected: Nav2 NavigateToPose action unavailable; {self.startup_status}')
            return
        try:
            self.validate_goal(message)
        except (ValueError,TransformException) as error:
            self.report(f'Goal rejected: {error}')
            return
        # Selecting a waypoint must not start Nav2's progress timer while the
        # motion bridge is disabled. Read the real bridge state, not a UI cache.
        self.goal_generation += 1
        self.waiting_goal = copy.deepcopy(message)
        self.check_motion_state()

    def check_motion_state(self):
        if self.waiting_goal is None:
            return
        if self.enable_query is not None:
            old,_,_ = self.enable_query
            self.motion_state.remove_pending_request(old)
            self.enable_query = None
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
            if generation != self.goal_generation or self.waiting_goal is None:
                return
            try:
                values = result.result().values
                if len(values)!=1 or values[0].type!=Parameter.Type.BOOL.value:
                    raise ValueError('enable_control is not a boolean parameter')
                if not values[0].bool_value:
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
        self.waiting_goal = None
        # A held waypoint may outlive a map update or localization correction.
        try:
            if not self.localized():
                raise ValueError('live GICP localization required')
            if not self.action.server_is_ready():
                raise ValueError('Nav2 NavigateToPose action unavailable')
            self.validate_goal(message)
        except (ValueError,TransformException) as error:
            self.report(f'Queued goal rejected on start: {error}')
            return
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
        future = self.action.send_goal_async(goal)
        future.add_done_callback(lambda result: self.goal_response(result,goal.pose))
        self.report('Map goal sent to Nav2 action')

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

    def goal_response(self,future,goal_pose):
        self.goal_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.report('Nav2 rejected goal')
                return
            self.goal_handle = handle
            self.last_goal_status = None
            handle.get_result_async().add_done_callback(lambda result: self.goal_result(result,goal_pose))
            self.report('Nav2 accepted goal')
            if self.cancel_requested or not self.localized():
                self.on_cancel(Empty())
        except Exception as error:
            self.report(f'Nav2 goal request failed: {error}')

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

    def goal_result(self,future,goal_pose):
        try:
            status = future.result().status
            self.last_goal_status = status
            text = f'Nav2 goal completed: action status={status}'
            if status==4:
                text += ' (SUCCEEDED); reached waypoint'
            elif status==6:
                text += ' (ABORTED); no active goal; enabling motion does not retry the failed goal'
            text += self.goal_error_text(goal_pose)
            self.report(text)
        except Exception as error:
            self.report(f'Nav2 result failed: {error}')
        self.goal_handle = None
        self.cancelling = False


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
