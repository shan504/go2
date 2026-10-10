"""Real ROS2 smoke test with synthetic 3D sensors, no robot or motion publisher."""
import sys
import time
import threading
import tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool, Empty, String
from tf2_ros import TransformBroadcaster,Buffer,TransformListener,TransformException
from mapper import Mapper,transform_matrix
from geometry import pose_matrix,quaternion_from_matrix,transform_points,cloud_xyz,planar_pose
from test_3d import room
# stdlib has an operator module; load our node without shadowing it.
import importlib.util
spec = importlib.util.spec_from_file_location('go2_operator',Path(__file__).resolve().parents[1]/'go2_3d/operator_bridge.py')
operator_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(operator_module)


def wait_for(predicate,seconds=15):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError('ROS smoke condition did not become true')


class Sensors(Node):
    def __init__(self):
        super().__init__('synthetic_sensors')
        self.pub = self.create_publisher(PointCloud2,'/point_cloud2',qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.initial = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
        self.save = self.create_publisher(Empty,'/mapping/save',1)
        self.valid = None
        self.operator_status = ''
        self.localization_status = ''
        self.registered = None
        self.create_subscription(Bool,'/localization/valid',lambda m: setattr(self,'valid',m.data),1)
        self.create_subscription(String,'/operator/status',lambda m: setattr(self,'operator_status',m.data),1)
        self.create_subscription(String,'/localization/status',lambda m: setattr(self,'localization_status',m.data),1)
        self.create_subscription(PointCloud2,'/registered_cloud',lambda m: setattr(self,'registered',m),qos_profile_sensor_data)
        self.points = room()
        self.truth, self.odom = np.eye(4),np.eye(4)
        self.stop = threading.Event()
        self.emitter = threading.Thread(target=self.emit,daemon=True)

    def emit(self):
        while not self.stop.is_set():
            header = Header(stamp=self.get_clock().now().to_msg(),frame_id='odom')
            tf = TransformStamped(header=header,child_frame_id='base_link')
            q,t = quaternion_from_matrix(self.odom),self.odom[:3,3]
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = map(float,t)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w = map(float,q)
            self.tf.sendTransform(tf)
            time.sleep(0.03)
            points = transform_points(self.points,np.linalg.inv(self.truth))
            self.pub.publish(point_cloud2.create_cloud_xyz32(
                Header(stamp=header.stamp,frame_id='base_link'),points.astype(np.float32).tolist()))
            self.stop.wait(0.17)

    def correction_matches(self,expected):
        try:
            tf = self.buffer.lookup_transform('map','odom',Time())
            return np.allclose(transform_matrix(tf.transform),expected,atol=0.04)
        except TransformException:
            return False

    def correction_stamp(self):
        return Time.from_msg(self.buffer.lookup_transform('map','odom',Time()).header.stamp).nanoseconds


def phase(directory,mode,dense=False,auto=False):
    arguments = ['--ros-args','-p',f'mode:={mode}','-p',f'output_directory:={directory}',
                 '-p',f'map_directory:={directory}/latest','-p','map_name:=indoor']
    if dense:
        arguments += ['--params-file',str(Path(__file__).resolve().parents[1]/'go2_3d/gicp_dense.yaml')]
    if auto:
        arguments += ['-p','auto_initialize:=true','-p','initial_x:=0.0','-p','initial_y:=0.0','-p','initial_yaw_degrees:=0.0']
    rclpy.init(args=arguments)
    mapper,operator,sensors = Mapper(),operator_module.Operator(),Sensors()
    if mode == 'mapping':
        sensors.points = room()[::2]
        if dense:
            sensors.truth = sensors.odom = pose_matrix([0,0,0],[np.sin(0.12/2),0,0,np.cos(0.12/2)])
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (mapper,operator,sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start()
    sensors.emitter.start()
    try:
        if mode == 'mapping':
            wait_for(lambda: len(mapper.points)>100)
            initial_count = len(mapper.points)
            # A stationary nonrepeating scan must enrich the actual saved map,
            # rather than relying on Foxglove's display decay/accumulation.
            sensors.points = room()
            wait_for(lambda: len(mapper.points)>initial_count*1.3)
            wait_for(lambda: sensors.registered is not None)
            assert sensors.registered.header.frame_id == 'map'
            assert np.ptp(cloud_xyz(sensors.registered)[:,2])>2.0,'Registered scan lost 3D height'
            assert np.ptp(mapper.points[:,2])>2.0,'Map lost 3D height'
            assert len(mapper.registration_points)<len(mapper.points),'Registration target uses full dense map'
            # Publish/deserialize real ROS cloud payload, including fine XYZ.
            packed = sensors.registered
            assert packed.point_step == 12 and len(packed.data) == packed.width*12
            if dense:
                assert mapper.p['map_voxel_size'] == 0.02
                assert mapper.p['max_map_points'] == 1000000
                assert len(mapper.history)>1
            truth = pose_matrix([0.40,0.0,0.0],[0,0,0,1])
            odom = pose_matrix([0.48,0.03,0.0],[0,0,0,1])
            sensors.truth,sensors.odom = truth,odom
            wait_for(lambda: sensors.correction_matches(truth@np.linalg.inv(odom)))
            wait_for(lambda: mapper.last_keyframe is not None and mapper.last_keyframe[0,3]>0.3)
            sensors.save.publish(Empty())
            wait_for(lambda: 'Save success=True' in sensors.operator_status)
            for name in ('map.pcd','nav.yaml','nav.pgm','metadata.yaml'):
                assert (Path(directory)/'latest'/name).is_file(),name
            import open3d as o3d
            saved = o3d.io.read_point_cloud(str(Path(directory)/'latest/map.pcd'))
            assert (Path(directory)/'latest').resolve().parent == Path(directory)/'library/indoor'
            assert (Path(directory)/'library/indoor/latest').resolve() == (Path(directory)/'latest').resolve()
            assert len(saved.points)>initial_count*1.3,'PCD was coarsened back to registration resolution'
            assert sensors.valid is False,'Mapping must never enable navigation motion'
            if dense:
                import yaml
                metadata = yaml.safe_load((Path(directory)/'latest/metadata.yaml').read_text())
                np.testing.assert_allclose(metadata['ground_model']['plane'][:3],[0,0,1],atol=0.02,
                                           err_msg='Initial body roll tilted the saved floor')
            print('PASS ROS mapping: stationary 3D enrichment + map-frame registered scan + GICP TF + dense PCD and Nav2 grid')
        else:
            if not auto:
                wait_for(lambda: sensors.valid is False and mapper.latest is not None)
                assert not mapper.initialized,'Localization must wait for initialpose'
                pose = PoseWithCovarianceStamped()
                pose.pose.pose.orientation.w = 1.0
                sensors.initial.publish(pose)
                time.sleep(0.3)
                assert not mapper.initialized,'An empty frame_id must not initialize localization'
                pose.header.frame_id = 'map'
                sensors.initial.publish(pose)
            wait_for(lambda: sensors.valid is True)
            if auto:
                assert mapper.initial_hint_used,'Numerical startup pose was not used'
            wait_for(lambda: 'fine=True' in sensors.localization_status)
            wait_for(lambda: sensors.correction_matches(np.eye(4)))
            wait_for(lambda: sensors.buffer.can_transform('map','base_footprint',Time()))
            footprint = transform_matrix(sensors.buffer.lookup_transform('map','base_footprint',Time()).transform)
            np.testing.assert_allclose(footprint,planar_pose(sensors.odom),atol=0.04)
            # Roll/pitch and standing height belong to the 3D body. A Nav2
            # footprint must stay on the grid without erasing measured yaw.
            tilted = pose_matrix([0.0,0.0,0.04],[0.10,-0.04,0.055,0.99])
            sensors.truth,sensors.odom = tilted,tilted
            def footprint_matches():
                try:
                    actual = transform_matrix(sensors.buffer.lookup_transform('map','base_footprint',Time()).transform)
                    body = transform_matrix(sensors.buffer.lookup_transform('map','base_link',Time()).transform)
                    return (np.allclose(actual,planar_pose(tilted),atol=0.04) and
                            np.allclose(body,tilted,atol=0.04))
                except TransformException:
                    return False
            wait_for(footprint_matches)
            sensors.truth,sensors.odom = np.eye(4),np.eye(4)
            wait_for(lambda: sensors.correction_matches(np.eye(4)) and
                     np.allclose(mapper.accepted_odom,np.eye(4),atol=0.01))
            # Nav2 must get timestamped TF between GICP updates. Keep live
            # odometry while briefly pausing registration, then check that
            # the held accepted correction advances without future stamps.
            mapper.registration_timer.cancel()
            first_stamp = sensors.correction_stamp()
            wait_for(lambda: sensors.correction_stamp()-first_stamp > 100_000_000,seconds=0.6)
            assert sensors.correction_stamp() <= sensors.get_clock().now().nanoseconds
            mapper.registration_timer.reset()
            # A contradictory fresh scan with unchanged odometry must fail,
            # not be smoothed into a valid localization or published TF.
            sensors.truth = pose_matrix([0.12,0,0],[0,0,0,1])
            wait_for(lambda: 'Rejected GICP correction jump' in sensors.localization_status)
            wait_for(lambda: sensors.valid is False)
            with mapper.lock:
                rejected_correction = mapper.correction.copy()
            time.sleep(1.0)
            np.testing.assert_allclose(mapper.correction,rejected_correction,atol=1e-8)
            assert sensors.valid is False,'Rejected scan was falsely reported valid'
            invalid_stamp = sensors.correction_stamp()
            time.sleep(0.4)
            assert sensors.correction_stamp() == invalid_stamp,'Rejected localization kept refreshing TF'
            sensors.truth = np.eye(4)
            wait_for(lambda: sensors.valid is True)
            sensors.stop.set()
            sensors.emitter.join()
            wait_for(lambda: sensors.valid is False,seconds=5)
            stale_stamp = sensors.correction_stamp()
            time.sleep(0.3)
            assert sensors.correction_stamp() == stale_stamp,'Stale odometry kept refreshing TF'
            print('PASS ROS localization: PCD reload + fine GICP + live TF between matches + rejected/stale TF stops')
    finally:
        sensors.stop.set()
        sensors.emitter.join(timeout=3)
        executor.shutdown()
        spinner.join(timeout=3)
        for node in (mapper,operator,sensors):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    dense = '--dense' in sys.argv[1:]
    if dense:
        from ros_cloud import xyz_message
        from rclpy.serialization import serialize_message,deserialize_message
        million = np.random.default_rng(42).uniform([-5,-5,-0.3],[5,5,2.0],(1000000,3))
        message = xyz_message(Header(frame_id='map'),million)
        decoded = deserialize_message(serialize_message(message),PointCloud2)
        assert decoded.width == 1000000 and decoded.row_step == 12000000
        np.testing.assert_allclose(cloud_xyz(decoded)[::10000],million[::10000],atol=1e-6)
        print('PASS dense payload: 1,000,000 XYZ points survived ROS CDR serialization')
    with tempfile.TemporaryDirectory() as directory:
        phase(directory,'mapping',dense)
        phase(directory,'localization',dense)
        phase(directory,'localization',dense,auto=True)
