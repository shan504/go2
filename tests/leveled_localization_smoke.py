"""Exercise z=0 2D seeding and true 3D body/planar footprint in real ROS2.

An optional --maps directory runs the same contract against an actual leveled
PCD using synthetic scans sampled from it. It never publishes motion commands.
"""
import argparse
import tempfile
import threading
import time
from pathlib import Path
import numpy as np
import open3d as o3d
import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from ros_smoke import Sensors,wait_for
from mapper import Mapper,transform_matrix
from geometry import pose_matrix,transform_points,planar_pose
from test_3d import room


def check(directory):
    directory = Path(directory)
    metadata = yaml.safe_load((directory/'metadata.yaml').read_text())
    points = np.asarray(o3d.io.read_point_cloud(str(directory/'map.pcd')).points)
    truth = np.array(metadata.get('map_from_previous_map',np.eye(4)))
    truth[2,3] = metadata['body_height_above_ground']
    # Keep the scan local and bounded, with real 3D structure. This validates
    # the frame/height contract; it is not a replay of live robot sensor data.
    local = points[np.linalg.norm(points[:,:2]-truth[:2,3],axis=1)<3.5]
    if len(local)>7000:
        local = local[np.random.default_rng(31).choice(len(local),7000,replace=False)]
    local = local+np.random.default_rng(33).normal(0,0.003,local.shape)
    rclpy.init(args=['--ros-args','-p','mode:=localization','-p',f'map_directory:={directory}',
                    '-p','auto_initialize:=true','-p','scan_window:=0.5',
                    '-p','initial_x:=0.0','-p','initial_y:=0.0','-p','initial_z:=0.0',
                    '-p','initial_yaw_degrees:=0.0'])
    mapper,sensors = Mapper(),Sensors()
    sensors.points,sensors.truth,sensors.odom = local,truth,np.eye(4)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (mapper,sensors):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start();sensors.emitter.start()
    try:
        wait_for(lambda: sensors.valid is True,seconds=30)
        wait_for(lambda: sensors.buffer.can_transform('map','base_footprint',Time()))
        wait_for(lambda: 'fine=True' in sensors.localization_status)
        time.sleep(1.2)
        assert sensors.valid is True,'Lifted seed did not sustain accepted GICP'
        body = transform_matrix(sensors.buffer.lookup_transform('map','base_link',Time()).transform)
        footprint = transform_matrix(sensors.buffer.lookup_transform('map','base_footprint',Time()).transform)
        np.testing.assert_allclose(body,truth,atol=0.015)
        np.testing.assert_allclose(footprint,planar_pose(truth),atol=0.015)
        assert abs(body[2,3]-metadata['body_height_above_ground'])<0.015
        assert abs(footprint[2,3])<0.003
        assert abs(mapper.map_metadata['ground_model']['plane'][3])<1e-6
        assert sensors.registered is not None and sensors.registered.header.frame_id=='map'
        print(f'PASS leveled ROS localization: z=0 seed -> body z={body[2,3]:.4f}m; '
              f'footprint z={footprint[2,3]:.4f}m; fine GICP; PCD points={len(points)}',flush=True)
    finally:
        sensors.stop.set();sensors.emitter.join(timeout=3)
        executor.shutdown();spinner.join(timeout=3)
        for node in (mapper,sensors):
            node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps')
    args = parser.parse_args()
    if args.maps:
        check(args.maps)
    else:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            matrix = pose_matrix([0,0,0.3],[0,0,0,1])
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(transform_points(room(),matrix)))
            o3d.io.write_point_cloud(str(root/'map.pcd'),pc,compressed=True)
            (root/'metadata.yaml').write_text(yaml.safe_dump(dict(ground_model=dict(plane=[0.,0.,1.,0.]),
                body_height_above_ground=0.3,map_from_previous_map=matrix.tolist())))
            check(root)
