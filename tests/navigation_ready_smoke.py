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
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool,Header,String
from sensor_msgs.msg import PointCloud2
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import TransformBroadcaster
from nav2_ready import Nav2Ready
from obstacle_cloud import ObstacleCloud
from ros_cloud import xyz_message
from geometry import cloud_xyz,pose_matrix,transform_points,quaternion_from_matrix,planar_pose


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
    calls,packets,startup_statuses = [],[],[]
    def startup(request,response):
        calls.append(request.command)
        response.success = True
        return response
    service = source.create_service(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes',startup)
    valid = source.create_publisher(Bool,'/localization/valid',1)
    localization_status = source.create_publisher(String,'/localization/status',qos_profile_sensor_data)
    source.create_subscription(String,'/navigation/startup_status',
        lambda message: startup_statuses.append(message.data),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    cloud = source.create_publisher(PointCloud2,'/point_cloud2',qos_profile_sensor_data)
    source.create_subscription(PointCloud2,'/navigation/obstacle_cloud',packets.append,qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(source)
    state = dict(valid=False,publish_valid=False,tf=False,status=None,status_samples=0)
    # A rolled standing body makes a fixed body-z slice disagree with ground.
    matrix = pose_matrix([0,0,0],[np.sin(0.2/2),0,0,np.cos(0.2/2)])
    world_points = np.array([[1,1,-0.46],[1,1,-0.36],[1,1,0.30],[1,1,1.5]])
    body_points = transform_points(world_points,np.linalg.inv(matrix))
    def emit():
        stamp = source.get_clock().now().to_msg()
        if state['publish_valid']:
            valid.publish(Bool(data=state['valid']))
        if state['status'] is not None:
            state['status_samples'] += 1
            localization_status.publish(String(data=state['status']+f'; cloud_samples={state["status_samples"]}'))
        if state['tf']:
            tf = TransformStamped(header=Header(stamp=stamp,frame_id='map'),child_frame_id='base_link')
            q = quaternion_from_matrix(matrix)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = map(float,q)
            projected = planar_pose(matrix)
            footprint = TransformStamped(header=Header(stamp=stamp,frame_id='base_link'),child_frame_id='base_footprint')
            relative = np.linalg.inv(matrix)@projected
            t,q = relative[:3,3],quaternion_from_matrix(relative)
            footprint.transform.translation.x,footprint.transform.translation.y,footprint.transform.translation.z = map(float,t)
            footprint.transform.rotation.x,footprint.transform.rotation.y,footprint.transform.rotation.z,footprint.transform.rotation.w = map(float,q)
            broadcaster.sendTransform([tf,footprint])
        cloud.publish(xyz_message(Header(stamp=stamp,frame_id='base_link'),body_points))
    source.create_timer(0.05,emit)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (gate,obstacles,source):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        wait(lambda: any('/localization/valid: no messages' in text and
                         '/localization/status: no messages' in text for text in startup_statuses))
        assert not calls and not packets,'Unlocalized startup/filter was allowed'
        state['publish_valid'] = True
        for detail in ('Waiting for actual initial map pose; Nav2 startup deferred',
                       'Waiting for fresh cloud and timestamped odometry',
                       'Localization invalid: overlap/residual rejected: fitness=0.100 rmse=0.240'):
            state['status'] = detail
            wait(lambda: any(detail in text and '/localization/valid=False receipt_age=' in text and
                             '/localization/status receipt_age=' in text for text in startup_statuses))
            assert not calls and not packets,'Mapper wait/rejection status bypassed localization'
        # Changing mapper counters must not turn every sensor packet into a log.
        reports = len(startup_statuses)
        time.sleep(0.4)
        assert len(startup_statuses)-reports <= 1,'Mapper counter noise flooded startup diagnostics'
        state['valid'] = True
        time.sleep(0.8)
        assert not calls and not packets,'A true flag without map TF started navigation'
        # Fresh TF and a successful status string cannot authorize an old Bool.
        state['publish_valid'] = False
        time.sleep(0.6)
        state['tf'] = True
        state['status'] = 'GICP fitness=0.999 rmse=0.001'
        wait(lambda: any('/localization/valid: stale; last=True receipt_age=' in text and
                         'GICP fitness=0.999' in text for text in startup_statuses))
        assert not calls,'Stale validity or informational status started Nav2'
        state['status'] = None
        wait(lambda: any('/localization/status receipt_age=' in text and
                         'stale; last reason: GICP fitness=0.999' in text for text in startup_statuses),seconds=12)
        assert not calls,'Stale mapper status started Nav2'
        state['publish_valid'] = True
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
        print('PASS ROS startup diagnostics: initialpose/cloud/GICP reasons forwarded; missing/stale status and validity distinguished; counter noise throttled')
        print('PASS ROS ground obstacles: body roll + floor removal + true obstacle retention + invalidity stops')
    finally:
        executor.shutdown();thread.join(timeout=3)
        for node in (gate,obstacles,source):
            node.destroy_node()
        rclpy.shutdown()
