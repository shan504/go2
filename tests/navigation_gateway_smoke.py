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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
from operator_bridge import Operator


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
        self.goals, self.cancels = 0,0
        self.last_goal = None
        self.pub = self.create_publisher(Bool,'/localization/valid',1)
        self.enable = self.create_publisher(Bool,'/control/enable',1)
        self.goal = self.create_publisher(PoseStamped,'/goal_pose',1)
        self.create_timer(0.05,lambda: self.pub.publish(Bool(data=self.valid)))
        self.server = ActionServer(self,NavigateToPose,'/navigate_to_pose',
            execute_callback=self.execute,cancel_callback=lambda request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

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
        wait_for(lambda: operator.valid_received is not None and operator.action.server_is_ready())
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
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.get_parameter('enable_control').value is True)
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals == 1 and operator.goal_handle is not None)
        assert endpoints.last_goal.header.frame_id == 'map'
        assert endpoints.last_goal.pose.position.x == 1.0
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: endpoints.cancels == 1 and operator.goal_handle is None)
        assert endpoints.get_parameter('enable_control').value is False
        endpoints.goal.publish(pose)
        wait_for(lambda: endpoints.goals == 2 and operator.goal_handle is not None)
        endpoints.valid = False
        wait_for(lambda: endpoints.cancels == 2 and operator.goal_handle is None)
        print('PASS Nav2 gateway: invalid-localization rejection, goal action conversion, parameter enable/disable, cancel on localization loss')
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()
