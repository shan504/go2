"""Exercise read-only diagnostics with real ROS retained grids and TF, no motion."""
import sys
import time
import math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool,Header
from sensor_msgs.msg import LaserScan,PointCloud2
from tf2_ros import StaticTransformBroadcaster
from navcheck import NavCheck,cell,connected
from ros_cloud import xyz_message
import numpy as np

rclpy.init()
source=Node('synthetic_readonly_nav_source')
retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
map_pub=source.create_publisher(OccupancyGrid,'/map',retained)
local_pub=source.create_publisher(OccupancyGrid,'/local_costmap/costmap',retained)
valid_pub=source.create_publisher(Bool,'/localization/valid',10)
scan_pub=source.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
cloud_pub=source.create_publisher(PointCloud2,'/point_cloud2',qos_profile_sensor_data)
broadcaster=StaticTransformBroadcaster(source)
frames=[]
for parent,child,x in [('map','odom',0.5),('odom','base_link',0.2)]:
    tf=TransformStamped()
    tf.header.frame_id=parent
    tf.header.stamp=source.get_clock().now().to_msg()
    tf.child_frame_id=child
    tf.transform.rotation.w=1.0
    tf.transform.translation.x=x
    tf.transform.translation.y=0.25 if child=='base_link' else 0.0
    frames.append(tf)
broadcaster.sendTransform(frames)
grid=OccupancyGrid()
grid.header.frame_id='map'
grid.info.width=6
grid.info.height=4
grid.info.resolution=0.5
grid.info.origin.orientation.w=1.0
data=np.zeros((4,6),dtype=int)
data[:,2]=100
grid.data=data.ravel().tolist()
map_pub.publish(grid)
grid.header.frame_id='odom'
local_pub.publish(grid)
# Publish first, then subscribe: this exercises retained map QoS.
node=NavCheck((2.25,0.75))
deadline=time.monotonic()+3
while time.monotonic()<deadline:
    valid_pub.publish(Bool(data=True))
    header=Header(stamp=source.get_clock().now().to_msg(),frame_id='base_link')
    scan=LaserScan(header=header,range_min=0.35,range_max=20.0,ranges=[0.5,1.0,float('inf')])
    scan_pub.publish(scan)
    cloud_pub.publish(xyz_message(header,np.array([[1.55,0.5,-0.18],[1.55,0.5,0.4],[8,8,0.3]])))
    rclpy.spin_once(node,timeout_sec=0.1)
summary=node.summary()
print(summary)
assert node.latest['/map'].header.frame_id=='map'
assert node.latest['/local_costmap/costmap'].header.frame_id=='odom'
assert node.latest['/localization/valid'].data is True
assert 'Robot in map: xyz=[0.7, 0.25, 0.0]' in summary
assert 'robot cell=(1, 0) value=0: free' in summary
assert 'robot cell=(0, 0) value=0: free' in summary
assert 'goal cell=(4, 1) value=0: free' in summary
assert 'goal cell=(3, 1) value=0: free' in summary
assert summary.count('NO: disconnected at cell-center level')==2
assert 'closest lethal cell to goal:' in summary
assert 'at scan stamp+0.05s=True' in summary
assert 'finite_returns=2' in summary
assert 'near goal XY (radius 0.25m): points=2' in summary
assert 'Legacy map lacks a ground model' in summary
data[1,4]=-1
assert connected(data,(1,0),(4,1))=='NO: goal cell blocked'
data[:,2]=98
data[1,4]=0
assert connected(data,(1,0),(4,1)).startswith('YES')
# A rotated/translated origin must not change the physical target indexing.
grid.info.origin.position.x=2.0
grid.info.origin.position.y=-1.0
grid.info.origin.orientation.z=math.sin(math.pi/4)
grid.info.origin.orientation.w=math.cos(math.pi/4)
assert cell(grid,(1.25,1.25))==(4,1)
assert cell(grid,(9.0,9.0)) is None
node.destroy_node()
source.destroy_node()
rclpy.shutdown()
print('PASS real ROS retained grids, live status, map/odom target transform, blocked/connectivity checks and rotated origin')
