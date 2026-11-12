"""Real Humble GICP export/TF test, synthetic sensors, no RTAB or hardware."""
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from ros_smoke import Mapper, Sensors, wait_for, transform_matrix
from geometry import cloud_xyz, pose_matrix


with tempfile.TemporaryDirectory() as temporary:
    rclpy.init(args=['--ros-args', '-p', 'mode:=mapping', '-p', 'mapping_backend:=rtabmap',
                    '-p', 'map_frame:=gicp_odom', '-p', 'loop_closure_enabled:=false',
                    '-p', 'registration_period:=0.15', '-p', 'output_directory:='+temporary])
    mapper, sensors = Mapper(), Sensors()
    received = {'odoms': {}, 'clouds': {}, 'keys': {}, 'map': None}
    def stamp(message):
        return Time.from_msg(message.header.stamp).nanoseconds
    sensors.create_subscription(Odometry, '/gicp/odom',
        lambda m: received['odoms'].update({stamp(m): m}), 100)
    sensors.create_subscription(PointCloud2, '/gicp/scan_cloud',
        lambda m: received['clouds'].update({stamp(m): m}), 100)
    sensors.create_subscription(PointCloud2, '/gicp/keyframe/cloud',
        lambda m: received['keys'].update({stamp(m): m}), 100)
    sensors.create_subscription(Odometry, '/gicp/keyframe/odom', lambda m: None, 100)
    backend_ready = sensors.create_publisher(Bool, '/mapping/backend_ready',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sensors.create_subscription(PointCloud2, '/map_cloud',
        lambda m: received.update(map=m), 1)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mapper)
    executor.add_node(sensors)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    sensors.emitter.start()
    try:
        assert mapper.loop_worker is None
        time.sleep(1.0)
        assert not mapper.seeded and mapper.map_point_count == 0, 'First keyframe committed before backend readiness'
        backend_ready.publish(Bool(data=True))
        wait_for(lambda: len(received['keys']) >= 1 and len(received['clouds']) >= 2)
        sensors.truth = pose_matrix([.35, 0., 0.], [0., 0., 0., 1.])
        sensors.odom = pose_matrix([.45, 0., 0.], [0., 0., 0., 1.])
        wait_for(lambda: len(received['keys']) >= 2 and mapper.last_keyframe[0, 3] > .25)
        wait_for(lambda: received['map'] is not None)
        # Header/TF and local cloud all refer to the same accepted measurement.
        common = sorted(received['odoms'].keys() & received['clouds'].keys())
        latest = common[-1]
        odom, cloud = received['odoms'][latest], received['clouds'][latest]
        assert odom.header.frame_id == 'gicp_odom' and odom.child_frame_id == 'base_link'
        assert cloud.header.frame_id == 'base_link'
        assert received['map'].header.frame_id == 'gicp_odom'
        assert abs(odom.pose.pose.position.x - .35) < .04
        assert abs(odom.pose.pose.position.x - .45) > .05, 'Native odometry was exported as GICP'
        wait_for(lambda: sensors.buffer.can_transform('gicp_odom', 'base_link', Time(nanoseconds=latest)))
        tf_pose = transform_matrix(sensors.buffer.lookup_transform('gicp_odom', 'base_link',
                                   Time(nanoseconds=latest)).transform)
        np.testing.assert_allclose(tf_pose[:3, 3],
            [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z], atol=1e-6)
        assert not sensors.buffer.can_transform('map', 'odom', Time()), 'Frontend published backend TF'
        points = cloud_xyz(cloud)
        assert len(points) > 100 and np.isfinite(points).all()
        result = mapper.save_map(Trigger.Request(), Trigger.Response())
        assert not result.success and 'unoptimized' in result.message
        print('PASS GICP external backend: accepted local scan/pose timestamps, genuine GICP vs native odom, '
              'single TF ownership, old loop disabled and unoptimized save blocked')
    finally:
        sensors.stop.set()
        sensors.emitter.join(timeout=3)
        executor.shutdown()
        spinner.join(timeout=3)
        mapper.destroy_node()
        sensors.destroy_node()
        rclpy.shutdown()
