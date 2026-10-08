"""Real ROS startup gate and ground-filter integration; no motion commands."""
import sys
import tempfile
import threading
import time
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool,Header
from sensor_msgs.msg import PointCloud2
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import TransformBroadcaster
from nav2_ready import Nav2Ready
from obstacle_cloud import ObstacleCloud
from ros_cloud import xyz_message
from geometry import cloud_xyz,pose_matrix,transform_points,quaternion_from_matrix


def wait(predicate,seconds=8):
    deadline = time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('ROS readiness/ground-filter condition not reached')


with tempfile.TemporaryDirectory() as tmp:
    (Path(tmp)/'metadata.yaml').write_text(yaml.safe_dump(dict(
        ground_model=dict(plane=[0,0,1,0.46]),
        parameters=dict(obstacle_min_height=0.10,obstacle_max_height=1.5))))
    rclpy.init(args=['--ros-args','-p',f'map_directory:={tmp}'])
    gate,obstacles,source = Nav2Ready(),ObstacleCloud(),Node('test_nav2_sensors')
    calls,packets = [],[]
    def startup(request,response):
        calls.append(request.command)
        response.success = True
        return response
    service = source.create_service(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes',startup)
    valid = source.create_publisher(Bool,'/localization/valid',1)
    cloud = source.create_publisher(PointCloud2,'/point_cloud2',qos_profile_sensor_data)
    source.create_subscription(PointCloud2,'/navigation/obstacle_cloud',packets.append,qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(source)
    state = dict(valid=False,tf=False)
    # A rolled standing body makes a fixed body-z slice disagree with ground.
    matrix = pose_matrix([0,0,0],[np.sin(0.2/2),0,0,np.cos(0.2/2)])
    world_points = np.array([[1,1,-0.46],[1,1,-0.36],[1,1,0.30],[1,1,1.5]])
    body_points = transform_points(world_points,np.linalg.inv(matrix))
    def emit():
        stamp = source.get_clock().now().to_msg()
        valid.publish(Bool(data=state['valid']))
        if state['tf']:
            tf = TransformStamped(header=Header(stamp=stamp,frame_id='map'),child_frame_id='base_link')
            q = quaternion_from_matrix(matrix)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = map(float,q)
            broadcaster.sendTransform(tf)
        cloud.publish(xyz_message(Header(stamp=stamp,frame_id='base_link'),body_points))
    source.create_timer(0.05,emit)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (gate,obstacles,source):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        time.sleep(0.8)
        assert not calls and not packets,'Unlocalized startup/filter was allowed'
        state['valid'] = True
        time.sleep(0.8)
        assert not calls and not packets,'A true flag without map TF started navigation'
        state['tf'] = True
        wait(lambda: len(calls)==1 and gate.started and len(packets)>0)
        assert calls == [ManageLifecycleNodes.Request.STARTUP]
        decoded = cloud_xyz(packets[-1])
        np.testing.assert_allclose(decoded,body_points[[1,2]],atol=1e-6)
        assert packets[-1].header.frame_id=='base_link'
        assert packets[-1].header.stamp.sec>0
        time.sleep(0.4)
        assert len(calls)==1,'Nav2 lifecycle startup was repeated'
        state['valid'] = False
        time.sleep(0.3)
        count = len(packets)
        time.sleep(0.3)
        assert len(packets)==count,'Invalid localization kept publishing obstacle packets'
        print('PASS ROS Nav2 startup: no startup without GICP+TF; exactly one startup when ready')
        print('PASS ROS ground obstacles: body roll + floor removal + true obstacle retention + invalidity stops')
    finally:
        executor.shutdown();thread.join(timeout=3)
        for node in (gate,obstacles,source):
            node.destroy_node()
        rclpy.shutdown()
