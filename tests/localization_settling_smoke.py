"""Real GICP: accepted moving correction may settle after odometry stops."""
import tempfile
import threading
from pathlib import Path
import numpy as np
import open3d as o3d
import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ros_smoke import Mapper, Sensors, wait_for
from geometry import pose_matrix
from test_3d import room


def main():
    with tempfile.TemporaryDirectory() as tmp:
        directory = Path(tmp)
        # Repeating floor samples make horizontal translation weakly
        # observable in a sparse floor-only scan, while the full room can
        # establish a reliable correction before a waypoint transition.
        x,y = np.meshgrid(np.arange(-3.,4.01,0.1),np.arange(-2.,3.01,0.1))
        floor = np.column_stack((x.ravel(),y.ravel(),np.full(x.size,-0.3)))
        structure = room()
        points = np.vstack((structure[structure[:,2]>-0.299],floor))
        o3d.io.write_point_cloud(str(directory/'map.pcd'),
            o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)),compressed=True)
        (directory/'metadata.yaml').write_text(yaml.safe_dump(
            dict(ground_model=dict(plane=[0.,0.,1.,0.3]))))
        rclpy.init(args=['--ros-args','-p','mode:=localization','-p',f'map_directory:={tmp}',
                        '-p','auto_initialize:=true','-p','stationary_translation_limit:=0.05',
                        '-p','stationary_rotation_limit_degrees:=2.0',
                        '-p','localization_voxel_size:=0.05','-p','localization_correspondence:=0.20',
                        '-p','localization_min_fitness:=0.55','-p','localization_max_rmse:=0.10'])
        mapper,sensors = Mapper(),Sensors()
        sensors.points = points
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (mapper,sensors):
            executor.add_node(node)
        spinner = threading.Thread(target=executor.spin,daemon=True)
        spinner.start()
        sensors.emitter.start()
        last_stamp = None
        def match_next():
            nonlocal last_stamp
            def fresh():
                with mapper.lock:
                    return mapper.latest is not None and mapper.latest[0].header.stamp != last_stamp
            wait_for(fresh)
            with mapper.lock:
                last_stamp = mapper.latest[0].header.stamp
            mapper.process()
        try:
            wait_for(lambda: mapper.seeded and sensors.valid is True)
            mapper.registration_timer.cancel()
            truth = pose_matrix([0.4,0.,0.],[0.,0.,0.,1.])
            odom = pose_matrix([0.5,0.,0.],[0.,0.,0.,1.])
            sensors.truth,sensors.odom = truth,odom
            wait_for(lambda: mapper.latest is not None and np.allclose(mapper.latest[1],odom))
            match_next()
            assert mapper.last_accepted is not None,'Moving correction did not pass GICP'
            np.testing.assert_allclose(mapper.accepted_odom,odom,atol=1e-6)
            assert -0.04 < mapper.correction[0,3] < -0.01,'Correction was not smoothed'
            # At the first waypoint, the output filter still lags the
            # accepted correction by about 7.5cm. A nonrepeating LiDAR packet
            # can mainly contain floor: it must not use that lag as an ICP
            # prediction and jump to another repeated floor correspondence.
            sparse = floor[(floor[:,0]>=-1.6)&(floor[:,0]<=1.6)&
                           (floor[:,1]>=0.1)&(floor[:,1]<=1.5)]
            sensors.points = sparse
            wait_for(lambda: mapper.latest is not None and mapper.latest[0].width == len(sparse))
            match_next()
            assert mapper.last_accepted is not None, (
                'Sparse waypoint-transition scan chased the filtered pose: '+mapper.last_failure)
            np.testing.assert_allclose(mapper.accepted_match_correction[0,3],-0.1,atol=0.01)
            print('PASS real sparse GICP uses the accepted prediction across a waypoint stop, without chasing output-filter lag',flush=True)
            sensors.points = points
            wait_for(lambda: mapper.latest[0].width == len(points))
            for _ in range(9):
                match_next()
                assert mapper.last_accepted is not None,(
                    'Identical accepted correction rejected when odometry stopped: '+mapper.last_failure)
            np.testing.assert_allclose(mapper.correction@odom,truth,atol=0.012)
            print('PASS real GICP moving correction settles while stationary without false localization loss',flush=True)

            # The reported 5.5--7.4cm jumps must still be rejected when they
            # are real contradictions in a fully observable room scan.
            previous = mapper.accepted_match_correction.copy()
            for displacement in (0.055,0.074):
                sensors.truth = pose_matrix([0.4+displacement,0.,0.],[0.,0.,0.,1.])
                match_next()
                match_next()
                assert mapper.last_accepted is None
                assert 'Rejected GICP correction jump' in mapper.last_failure,mapper.last_failure
                np.testing.assert_allclose(mapper.accepted_match_correction,previous,atol=1e-6)
                sensors.truth = truth
                match_next()
                match_next()
                assert mapper.last_accepted is not None
            print('PASS genuine 5.5cm/7.4cm stationary jumps remain rejected and recover without poisoning the prediction',flush=True)

            # A genuinely new stationary jump is still rejected, and a rejected
            # measurement must not replace the accepted comparison reference.
            previous = mapper.accepted_match_correction.copy()
            sensors.truth = pose_matrix([0.55,0.,0.],[0.,0.,0.,1.])
            # Allow a full emitter packet to use the new scan pose.
            match_next()
            match_next()
            assert mapper.last_accepted is None
            assert 'Rejected GICP correction jump' in mapper.last_failure,mapper.last_failure
            np.testing.assert_allclose(mapper.accepted_match_correction,previous,atol=1e-6)
            sensors.truth = truth
            match_next()
            match_next()
            assert mapper.last_accepted is not None,'Rejected match poisoned later recovery'
            print('PASS new 15cm stationary jump remains rejected; stable reference recovers without reseeding',flush=True)
        finally:
            sensors.stop.set()
            sensors.emitter.join(timeout=3)
            executor.shutdown()
            spinner.join(timeout=3)
            mapper.destroy_node()
            sensors.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
