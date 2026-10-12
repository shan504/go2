"""Actual ROS/GICP patrol tolerance and bounded scan dropout, no robot."""
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
import yaml

from ros_smoke import Mapper, Sensors, wait_for
from geometry import pose_matrix
from test_3d import room


def main():
    with tempfile.TemporaryDirectory() as tmp:
        directory = Path(tmp)
        points = room()
        o3d.io.write_point_cloud(str(directory/'map.pcd'),
            o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)), compressed=True)
        (directory/'metadata.yaml').write_text(yaml.safe_dump(
            dict(ground_model=dict(plane=[0.,0.,1.,0.3]))))
        original_map = (directory/'map.pcd').read_bytes()
        rclpy.init(args=['--ros-args','-p','mode:=localization',
                        '-p',f'map_directory:={tmp}','-p','auto_initialize:=true',
                        '-p','scan_window:=0.5'])
        mapper,sensors = Mapper(),Sensors()
        sensors.packet_wait = 0.03
        sensors.packet_point_limit = 1500
        valid_events = []
        sensors.create_subscription(Bool,'/localization/valid',
                                    lambda msg: valid_events.append(msg.data),10)
        executor = MultiThreadedExecutor(num_threads=4)
        for node in (mapper,sensors):
            executor.add_node(node)
        spinner = threading.Thread(target=executor.spin,daemon=True)
        spinner.start()
        sensors.emitter.start()
        last_stamp = None

        def next_match():
            nonlocal last_stamp
            wait_for(lambda: mapper.latest is not None and mapper.latest[0].header.stamp != last_stamp)
            with mapper.lock:
                last_stamp = mapper.latest[0].header.stamp
            mapper.process()

        def change_pose(pose):
            sensors.truth = pose
            deadline = time.monotonic()+0.65
            while time.monotonic()<deadline:
                next_match()

        try:
            wait_for(lambda: mapper.seeded and sensors.valid is True)
            mapper.registration_timer.cancel()
            assert mapper.p['stationary_translation_limit'] == 0.25
            assert mapper.p['stationary_rotation_limit_degrees'] == 8.0
            assert mapper.p['scan_window'] == 0.5 and mapper.p['localization_voxel_size'] == 0.10
            wait_for(lambda: 'scan_frames=8 ' in sensors.localization_status or
                            'scan_frames=9 ' in sensors.localization_status)
            beginning = len(valid_events)
            for displacement in (0.076,0.,0.091,0.,0.101,0.):
                angle = np.deg2rad(1.89 if displacement else 0.)
                pose = pose_matrix([displacement,0.,0.],
                                   [0.,0.,np.sin(angle/2),np.cos(angle/2)])
                change_pose(pose)
                assert mapper.last_accepted is not None, mapper.last_failure
                np.testing.assert_allclose(mapper.accepted_match_correction,pose,atol=0.035)
            wait_for(lambda: len(valid_events)>beginning)
            assert all(valid_events[beginning:]), 'Small patrol corrections interrupted live localization'
            print('PASS accumulated patrol GICP: 0.5s/~8 sampled packets, 10cm registration voxel, 7.6/9.1/10.1cm and 1.89deg corrections keep localization continuously valid',flush=True)

            # One missing scan holds ONLY the original accepted estimate, with
            # no correction change and no refresh of its receipt/deadline.
            accepted = mapper.last_accepted
            reference = mapper.accepted_match_correction.copy()
            correction = mapper.correction.copy()
            beginning = len(valid_events)
            sensors.points = np.zeros((3,3))
            wait_for(lambda: mapper.latest is not None and mapper.latest[0].width == 3)
            with mapper.lock:
                last_good = max(receipt for packet,_,receipt in mapper.history if packet.width>3)
            # Window geometry may still be usable. Empty packets must never
            # refresh the deadline as if their own geometry was accepted.
            while time.monotonic()-last_good<0.58:
                next_match()
                assert mapper.last_accepted is None or mapper.last_accepted<=last_good
            assert mapper.last_accepted is not None
            accepted = mapper.last_accepted
            reference = mapper.accepted_match_correction.copy()
            correction = mapper.correction.copy()
            next_match()
            assert mapper.last_accepted == accepted, 'A failed scan refreshed/erased the accepted deadline'
            np.testing.assert_array_equal(mapper.accepted_match_correction,reference)
            np.testing.assert_array_equal(mapper.correction,correction)
            mapper.health()
            wait_for(lambda: sensors.valid is True)
            assert all(valid_events[beginning:]), 'One short missing scan destroyed a fresh accepted pose'
            sensors.points = points
            wait_for(lambda: mapper.latest[0].width == 1500)
            next_match()
            wait_for(lambda: sensors.valid is True)
            assert mapper.last_accepted>accepted
            print('PASS one missing scan: unchanged prior/reference and original 1s deadline; a genuine fresh match recovers',flush=True)

            # Even with fresh DDS clouds and odometry, repeated bad scans must
            # expire the old acceptance. They cannot perpetually renew it.
            accepted = mapper.last_accepted
            sensors.points = np.zeros((3,3))
            wait_for(lambda: mapper.latest[0].width == 3)
            while time.monotonic()-accepted <= mapper.p['validity_timeout']+0.25:
                next_match()
            wait_for(lambda: sensors.valid is False)
            stopped_stamp = sensors.correction_stamp()
            time.sleep(0.3)
            assert sensors.correction_stamp() == stopped_stamp, 'Persistent bad scans kept refreshing TF'
            sensors.points = points
            wait_for(lambda: mapper.latest[0].width == 1500)
            next_match()
            next_match()
            wait_for(lambda: sensors.valid is True)
            print('PASS persistent missing scans: old 1s deadline expires, localization invalid and map TF stops despite fresh DDS input',flush=True)

            for pose in (pose_matrix([0.28,0.,0.],[0.,0.,0.,1.]),
                         pose_matrix([0.,0.,0.],[0.,0.,np.sin(np.deg2rad(10.)/2),np.cos(np.deg2rad(10.)/2)])):
                reference = mapper.accepted_match_correction.copy()
                # Let a full fresh window carry the new pose before its first
                # match, so this checks one over-limit correction directly.
                sensors.truth = pose
                time.sleep(0.65)
                next_match()
                assert mapper.last_accepted is None, 'Large stationary jump was accepted/held'
                assert 'Rejected GICP correction jump' in mapper.last_failure, mapper.last_failure
                np.testing.assert_array_equal(mapper.accepted_match_correction,reference)
                mapper.health()
                wait_for(lambda: sensors.valid is False)
                sensors.truth = np.eye(4)
                time.sleep(0.65)
                next_match()
                wait_for(lambda: sensors.valid is True)
            assert (directory/'map.pcd').read_bytes() == original_map
            print('PASS over-limit 28cm/10deg corrections invalidate immediately; rejected results do not poison recovery; saved PCD unchanged',flush=True)
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
