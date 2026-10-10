"""Real Nav2 action protocol against a test server; no planner or robot motion."""
import sys
import time
import threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from std_msgs.msg import Bool,String
from std_msgs.msg import Empty
from rcl_interfaces.msg import SetParametersResult
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from operator_bridge import Operator
from geometry import grid_cell


def wait_for(predicate,seconds=10):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('Navigation gateway condition timed out')


class TestEndpoints(Node):
    def __init__(self,start_action=True):
        super().__init__('go2_edu_dds_bridge')
        self.declare_parameter('enable_control',False)
        self.valid = False
        self.tf_enabled = True
        self.goals, self.cancels = 0,0
        self.last_goal = None
        self.abort_goal = False
        self.succeed_goal = False
        self.reject_enable = False
        self.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
            successful=not (self.reject_enable and any(p.name=='enable_control' and p.value for p in parameters)),
            reason='test bridge refuses enable' if self.reject_enable else ''))
        self.operator_status = ''
        self.pub = self.create_publisher(Bool,'/localization/valid',1)
        self.enable = self.create_publisher(Bool,'/control/enable',1)
        self.goal = self.create_publisher(PoseStamped,'/goal_pose',1)
        self.cancel = self.create_publisher(Empty,'/navigation/cancel',1)
        self.tf = TransformBroadcaster(self)
        self.startup = self.create_publisher(String,'/navigation/startup_status',
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.startup.publish(String(data='Nav2 deferred awaiting TF'))
        self.grids = {}
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/operator/status',lambda m: setattr(self,'operator_status',m.data),retained)
        for topic in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
            publisher = self.create_publisher(OccupancyGrid,topic,retained)
            grid = OccupancyGrid()
            grid.header.frame_id = 'odom' if topic.startswith('/local_') else 'map'
            grid.info.width = grid.info.height = 50
            grid.info.resolution = 0.1
            grid.info.origin.position.x = grid.info.origin.position.y = -2.0
            grid.info.origin.orientation.w = 1.0
            grid.data = [0]*2500
            self.grids[topic] = (publisher,grid)
        self.create_timer(0.05,lambda: self.pub.publish(Bool(data=self.valid)))
        self.create_timer(0.05,self.sensors)
        if start_action:
            self.start_action()

    def start_action(self):
        self.server = ActionServer(self,NavigateToPose,'/navigate_to_pose',
            execute_callback=self.execute,cancel_callback=lambda request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def sensors(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for child in ('base_footprint','odom'):
            tf = TransformStamped(header=Header(stamp=stamp,frame_id='map'),child_frame_id=child)
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        if self.tf_enabled:
            self.tf.sendTransform(transforms)
        for publisher,grid in self.grids.values():
            grid.header.stamp = stamp
            publisher.publish(grid)

    def execute(self,handle):
        self.goals += 1
        self.last_goal = handle.request.pose
        if self.abort_goal:
            handle.abort()
            return NavigateToPose.Result()
        if self.succeed_goal:
            handle.succeed()
            return NavigateToPose.Result()
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.cancels += 1
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        handle.abort()
        return NavigateToPose.Result()


if __name__ == '__main__':
    rclpy.init(args=['--ros-args','-p','mode:=navigation'])
    operator, endpoints = Operator(),TestEndpoints(start_action=False)
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(operator)
    executor.add_node(endpoints)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start()
    try:
        wait_for(lambda: operator.valid_received is not None and
                 operator.motion_state.service_is_ready() and len(operator.grids)==3)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.orientation.w = 1.0
        endpoints.goal.publish(pose)
        endpoints.enable.publish(Bool(data=True))
        time.sleep(0.3)
        assert endpoints.goals == 0
        assert endpoints.get_parameter('enable_control').value is False
        endpoints.valid = True
        wait_for(operator.localized)
        wait_for(lambda: operator.buffer.can_transform('map','base_footprint',rclpy.time.Time()))
        # A healthy localization does not mean Nav2 itself has started.
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'Enable rejected: Nav2 NavigateToPose action unavailable' in endpoints.operator_status)
        assert 'Nav2 deferred awaiting TF' in endpoints.operator_status
        assert endpoints.get_parameter('enable_control').value is False
        endpoints.goal.publish(pose)
        wait_for(lambda: 'Goal rejected: Nav2 NavigateToPose action unavailable' in endpoints.operator_status)
        assert endpoints.goals==0
        endpoints.start_action()
        wait_for(operator.action.server_is_ready)
        # An old body-relative pose must not bypass the gateway into Nav2.
        pose.header.frame_id = 'base_link'
        pose.header.stamp.sec = 1
        endpoints.goal.publish(pose)
        time.sleep(0.2)
        assert endpoints.goals == 0
        pose.header.frame_id = 'map'
        # Reject actual map obstacles and inflation clearance before recovery.
        for topic,value in (('/map',100),('/local_costmap/costmap',99)):
            _,grid = endpoints.grids[topic]
            # ROS wire resolution is float32; use the received geometry so
            # a boundary cell is identical to the actual Nav2 message.
            col,row = grid_cell(operator.grids[topic][0],(1.0,0.0))
            index = row*grid.info.width+col
            grid.data[index] = value
            wait_for(lambda: operator.grids[topic][0].data[index]==value)
            endpoints.goal.publish(pose)
            time.sleep(0.2)
            assert endpoints.goals == 0,f'{topic} blocked goal reached Nav2'
            grid.data[index] = 0
            wait_for(lambda: operator.grids[topic][0].data[index]==0)
        endpoints.tf_enabled = False
        time.sleep(0.7)
        endpoints.goal.publish(pose)
        time.sleep(0.2)
        assert endpoints.goals == 0,'A true validity flag with stale TF allowed a goal'
        endpoints.tf_enabled = True
        wait_for(lambda: (operator.get_clock().now()-rclpy.time.Time.from_msg(
            operator.buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).header.stamp)).nanoseconds < 100_000_000)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.get_parameter('enable_control').value is True)
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals == 1 and operator.goal_handle is not None)
        assert endpoints.last_goal.header.frame_id == 'map'
        assert endpoints.last_goal.pose.position.x == 1.0
        assert endpoints.last_goal.header.stamp.sec == 0 and endpoints.last_goal.header.stamp.nanosec == 0
        assert pose.header.stamp.sec == 1,'Gateway changed the original message timestamp'
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: endpoints.cancels == 1 and operator.goal_handle is None)
        assert endpoints.get_parameter('enable_control').value is False
        # Selecting first and enabling later must not consume progress time.
        endpoints.goal.publish(pose)
        wait_for(lambda: 'Goal queued' in endpoints.operator_status)
        time.sleep(10.5)  # Longer than the real controller's 10-second allowance.
        assert endpoints.goals==1,'Disabled motion still started the Nav2 action'
        endpoints.reject_enable = True
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'parameter accepted=False' in endpoints.operator_status)
        assert endpoints.goals==1 and endpoints.get_parameter('enable_control').value is False
        endpoints.reject_enable = False
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals==2 and operator.goal_handle is not None)
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: endpoints.cancels==2 and operator.goal_handle is None)
        # Cancelling a held waypoint must prevent a subsequent enable dispatch.
        endpoints.goal.publish(pose)
        wait_for(lambda: 'Goal queued' in endpoints.operator_status)
        endpoints.cancel.publish(Empty())
        wait_for(lambda: 'Queued navigation goal cancelled' in endpoints.operator_status)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'no active navigation goal' in endpoints.operator_status)
        assert endpoints.goals==2
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: endpoints.get_parameter('enable_control').value is False)
        # Localization loss discards a held waypoint; it cannot silently resume.
        endpoints.goal.publish(pose)
        wait_for(lambda: 'Goal queued' in endpoints.operator_status)
        endpoints.valid = False
        wait_for(lambda: operator.waiting_goal is None and not operator.localized())
        endpoints.valid = True
        wait_for(operator.localized)
        # Revalidate costmaps at dispatch, not only when the user first clicks.
        endpoints.goal.publish(pose)
        wait_for(lambda: 'Goal queued' in endpoints.operator_status)
        _,grid = endpoints.grids['/global_costmap/costmap']
        col,row = grid_cell(operator.grids['/global_costmap/costmap'][0],(1.0,0.0))
        index = row*grid.info.width+col
        grid.data[index] = 100
        wait_for(lambda: operator.grids['/global_costmap/costmap'][0].data[index]==100)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'Queued goal rejected on start' in endpoints.operator_status)
        assert endpoints.goals==2
        grid.data[index] = 0
        wait_for(lambda: operator.grids['/global_costmap/costmap'][0].data[index]==0)
        # A distant map goal may lie beyond the rolling local window.
        _,local = endpoints.grids['/local_costmap/costmap']
        local.info.width = local.info.height = 10
        local.info.origin.position.x = local.info.origin.position.y = -0.5
        local.data = [0]*100
        wait_for(lambda: operator.grids['/local_costmap/costmap'][0].info.width==10)
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals == 3 and operator.goal_handle is not None)
        endpoints.valid = False
        wait_for(lambda: endpoints.cancels == 3 and operator.goal_handle is None)
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.abort_goal = True
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals==4 and operator.last_goal_status==6 and operator.goal_handle is None)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'previous goal failed; fix route and send a new goal' in endpoints.operator_status)
        assert endpoints.goals==4,'Enabling motion retried an aborted goal'
        endpoints.abort_goal, endpoints.succeed_goal = False,True
        pose.pose.position.x = 0.20
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals==5 and operator.last_goal_status==4 and operator.goal_handle is None)
        wait_for(lambda: '(SUCCEEDED); reached waypoint' in endpoints.operator_status)
        assert 'final XY error=0.200m' in endpoints.operator_status
        assert 'final heading unrestricted' in endpoints.operator_status
        # Nearness is diagnostic only: never relabel a genuine Nav2 abort as a
        # success just because the last TF lies inside the arrival radius.
        endpoints.abort_goal, endpoints.succeed_goal = True,False
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals==6 and operator.last_goal_status==6 and operator.goal_handle is None)
        wait_for(lambda: '(ABORTED)' in endpoints.operator_status)
        assert 'final XY error=0.200m' in endpoints.operator_status
        assert '(SUCCEEDED)' not in endpoints.operator_status
        print('PASS Nav2 gateway: disabled hold/enable/cancel/revalidation, true SUCCEEDED and ABORTED labels with current XY error, no fabricated success or failed-goal retry')
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()
