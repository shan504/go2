"""Isolated ROS Sport endpoints; no robot, mode switch or hardware connection."""
import copy
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from unitree_api.msg import Request, Response

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from control_bridge import LocalizedBridge


class SimulatedRobot(Node):
    def __init__(self):
        super().__init__('simulated_sport_endpoints')
        self.valid = self.cloud_enabled = True
        self.command = Twist()
        self.command.linear.x = 0.08
        self.requests = []
        self.status = ''
        self.odom = self.create_publisher(Odometry,'/utlidar/robot_odom',qos_profile_sensor_data)
        self.cloud = self.create_publisher(PointCloud2,'/utlidar/cloud_base',qos_profile_sensor_data)
        self.localization = self.create_publisher(Bool,'/localization/valid',10)
        self.velocity = self.create_publisher(Twist,'/cmd_vel',10)
        self.response = self.create_publisher(Response,'/api/sport/response',10)
        self.create_subscription(Request,'/api/sport/request',self.on_request,10)
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/control/status',lambda m: setattr(self,'status',m.data),retained)
        self.create_timer(0.02,self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id,odom.child_frame_id = 'odom','base_link'
        odom.header.stamp = stamp
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)
        if self.cloud_enabled:
            cloud = PointCloud2()
            cloud.header.frame_id,cloud.header.stamp = 'base_link',stamp
            self.cloud.publish(cloud)
        self.localization.publish(Bool(data=self.valid))
        if self.command is not None:
            self.velocity.publish(self.command)

    def on_request(self,message):
        self.requests.append(message)
        response = Response()
        response.header.identity = copy.deepcopy(message.header.identity)
        # Exercise an actual nonzero API response code, not a fabricated success.
        response.header.status.code = 501
        self.response.publish(response)


rclpy.init(args=['--ros-args','-p','allow_motion:=true'])
bridge,robot = LocalizedBridge(),SimulatedRobot()
executor = SingleThreadedExecutor()
executor.add_node(bridge)
executor.add_node(robot)


def spin(seconds):
    deadline = time.monotonic()+seconds
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=0.02)


def wait_for(predicate,seconds=5):
    deadline = time.monotonic()+seconds
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError('Motion bridge condition timed out')


try:
    wait_for(lambda: bridge.localization_received is not None and bridge.cloud_received is not None)
    spin(0.25)
    assert bridge.move_requests==0,'Disabled gate forwarded a command'
    robot.command = None
    spin(0.2)  # Drain in-flight source messages before checking enable reset.
    bridge.set_parameters([Parameter('enable_control',value=True)])
    spin(0.6)
    assert bridge.move_requests==0,'Enable reused a command received while disabled'
    wait_for(lambda: 'gate=waiting for fresh /cmd_vel' in robot.status)
    robot.command = Twist()
    robot.command.linear.x = 0.8
    wait_for(lambda: bridge.nonzero_move_requests>=3 and bridge.last_response is not None)
    assert bridge.last_response[1:]==(1008,501)
    wait_for(lambda: 'gate=forwarding Move requests' in robot.status and 'api=1008 code=501' in robot.status)
    moves = [r for r in robot.requests if r.header.identity.api_id==1008]
    assert moves and abs(json.loads(moves[-1].parameter)['x']-0.15)<1e-6
    assert json.loads(moves[-1].parameter)['y']==0.0
    # A response from another SDK client must not replace our diagnostic state.
    unrelated = Response()
    unrelated.header.identity.id,unrelated.header.identity.api_id = 1,1008
    unrelated.header.status.code = 777
    robot.response.publish(unrelated)
    spin(0.1)
    assert bridge.last_response[2]==501
    robot.valid = False
    wait_for(lambda: bridge.stop_requests>=1)
    count = bridge.move_requests
    spin(0.3)
    assert bridge.move_requests==count,'Invalid localization kept sending Move'
    wait_for(lambda: 'gate=localization invalid or stale' in robot.status)
    robot.valid,robot.command = True,None
    spin(0.6)
    assert bridge.move_requests==count,'Localization recovery replayed an old command'
    robot.command = Twist()
    wait_for(lambda: bridge.move_requests>count)
    assert bridge.nonzero_move_requests==count
    wait_for(lambda: 'gate=received zero /cmd_vel' in robot.status)
    robot.command.linear.x = 0.08
    wait_for(lambda: bridge.nonzero_move_requests>count)
    robot.cloud_enabled = False
    stops = bridge.stop_requests
    wait_for(lambda: bridge.stop_requests>stops)
    count = bridge.move_requests
    spin(0.2)
    assert bridge.move_requests==count,'Stale cloud kept sending Move'
    wait_for(lambda: 'gate=odometry or cloud stale' in robot.status)
    assert bridge.cloud_received is not None
    print('PASS localized motion bridge: disabled hold, fresh-command requirement, Move 1008 bounded JSON, correlated response codes, localization/cloud loss stop without replay')
finally:
    executor.shutdown()
    bridge.destroy_node()
    robot.destroy_node()
    rclpy.shutdown()
