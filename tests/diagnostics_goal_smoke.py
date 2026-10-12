"""Real ROS goal-input observation; observer never sends goals or motion."""
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PointStamped,PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String
from diagnose import Diagnostics


def main():
    rclpy.init()
    source = Node('synthetic_ui_goal_source')
    retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status = source.create_publisher(String,'/operator/status',retained)
    status.publish(String(data='no active navigation goal; route queued=0; goal_inputs_received=0'))
    messages = []
    for topic,kind,x in (('/navigation/waypoint',PointStamped,0.4),('/goal_pose',PoseStamped,0.8),
                         ('/initialpose',PoseWithCovarianceStamped,1.2)):
        publisher = source.create_publisher(kind,topic,10)
        msg = kind()
        msg.header.frame_id = 'map'
        p = msg.point if kind is PointStamped else msg.pose.position if kind is PoseStamped else msg.pose.pose.position
        p.x = x
        messages.append((publisher,msg))
    node = Diagnostics()
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    executor.add_node(node)
    try:
        deadline = time.monotonic()+3
        while time.monotonic()<deadline:
            for publisher,msg in messages:
                msg.header.stamp = source.get_clock().now().to_msg()
                publisher.publish(msg)
            executor.spin_once(timeout_sec=0.05)
        summary = node.summary()
        assert 'goal_inputs_received=0' in summary,'Observer failed retained operator-status delivery'
        for topic in ('/navigation/waypoint','/goal_pose','/initialpose'):
            assert node.counts.get(topic,0)>1,topic
            assert f'{topic}: messages=' in summary,summary
            assert len(node.get_publishers_info_by_topic(topic))==1,'Observer advertised a goal publisher'
        for value in (0.4,0.8,1.2):
            assert f'xyz=[{value}, 0.0, 0.0]' in summary
        for topic in ('/control/enable','/navigation/cancel','/cmd_vel','/api/sport/request'):
            assert node.count_publishers(topic)==0,f'Observer wrote {topic}'
        print('PASS read-only diagnostics: single-click/legacy/initialpose input counters, retained operator status, no goal/control/Sport publishers')
    finally:
        executor.shutdown()
        node.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
