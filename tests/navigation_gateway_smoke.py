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
from std_msgs.msg import Bool
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
    def __init__(self):
        super().__init__('go2_edu_dds_bridge')
        self.declare_parameter('enable_control',False)
        self.valid = False
        self.tf_enabled = True
        self.goals, self.cancels = 0,0
        self.last_goal = None
        self.pub = self.create_publisher(Bool,'/localization/valid',1)
        self.enable = self.create_publisher(Bool,'/control/enable',1)
        self.goal = self.create_publisher(PoseStamped,'/goal_pose',1)
        self.tf = TransformBroadcaster(self)
        self.grids = {}
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
    operator, endpoints = Operator(),TestEndpoints()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(operator)
    executor.add_node(endpoints)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start()
    try:
        wait_for(lambda: operator.valid_received is not None and operator.action.server_is_ready() and len(operator.grids)==3)
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
        # A distant map goal may lie beyond the rolling local window.
        _,local = endpoints.grids['/local_costmap/costmap']
        local.info.width = local.info.height = 10
        local.info.origin.position.x = local.info.origin.position.y = -0.5
        local.data = [0]*100
        wait_for(lambda: operator.grids['/local_costmap/costmap'][0].info.width==10)
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals == 2 and operator.goal_handle is not None)
        endpoints.valid = False
        wait_for(lambda: endpoints.cancels == 2 and operator.goal_handle is None)
        print('PASS Nav2 gateway: old body-frame goal rejected, map/clearance blocked goals rejected, fixed-map action, enable/disable and cancel on localization loss')
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()
