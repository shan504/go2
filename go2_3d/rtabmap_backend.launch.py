"""Optional visual place recognition + lidar verification, external GICP odometry.

The adapter supplies an explicitly bounded, exact-stamp input quartet. RTAB-Map
never owns TF or the frontend map, and its optimized graph rebuilds the map
through the existing mapping/export helpers in go2_rtabmap_backend.
"""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def rtabmap_parameters(database, use_sim_time=False):
    """One production parameter set shared by launch and actual backend tests."""
    # String values below are RTAB-Map core parameters, not ROS bool/numbers.
    return {
        'frame_id': 'base_link', 'odom_frame_id': '', 'map_frame_id': 'map',
        'publish_tf': False, 'database_path': str(database),
        'use_sim_time': use_sim_time,
        'subscribe_rgb': True, 'subscribe_depth': False,
        'subscribe_stereo': False, 'subscribe_rgbd': False,
        'subscribe_scan_cloud': True, 'subscribe_odom': True,
        'approx_sync': False, 'topic_queue_size': 20, 'sync_queue_size': 20,
        'qos_image': 1, 'qos_camera_info': 1, 'qos_scan': 1, 'qos_odom': 1,
        'gen_depth': True, 'gen_depth_decimation': 1,
        'gen_depth_fill_holes_size': 0, 'gen_depth_fill_iterations': 0,
        'Reg/Strategy': '2', 'Vis/EstimationType': '1',
        'RGBD/LoopClosureIdentityGuess': 'false',
        'RGBD/ProximityBySpace': 'false', 'RGBD/ProximityByTime': 'false',
        'RGBD/NeighborLinkRefining': 'false',
        'RGBD/CreateOccupancyGrid': 'false',
        'Mem/DepthAsMask': 'false', 'Vis/DepthAsMask': 'false',
        'Mem/RehearsalSimilarity': '1.0', 'Mem/ReduceGraph': 'false',
        'Mem/BadSignaturesIgnored': 'false', 'Rtabmap/PublishStats': 'true',
        'RGBD/Enabled': 'true', 'RGBD/OptimizeMaxError': '3',
        'Rtabmap/DetectionRate': '0', 'RGBD/LinearUpdate': '0',
        'RGBD/AngularUpdate': '0', 'Rtabmap/TimeThr': '0', 'Rtabmap/MemoryThr': '0',
        'Rtabmap/ImagesAlreadyRectified': 'true', 'Mem/IncrementalMemory': 'true',
        'Optimizer/Iterations': '20', 'Optimizer/Robust': 'false',
    }


def backend_nodes(context):
    root = Path('/opt/go2_project')
    sys.path.insert(0, str(root / 'go2_3d'))
    from camera_calibration import load_calibration, validate_input_topic
    from check_rtabmap import check_versions

    calibration = LaunchConfiguration('camera_calibration').perform(context)
    camera = load_calibration(calibration)
    validate_input_topic(camera, '/camera/image/compressed')
    urdf = Path('/runtime/config/go2_edu.urdf')
    if urdf.is_file() and camera.optical_frame in {
            link.attrib.get('name') for link in ET.parse(urdf).getroot().findall('link')}:
        raise RuntimeError('Calibrated optical frame already exists in robot URDF')
    # gen_depth + RGB-only/scan_cloud is audited in 0.22.1 and newer Humble
    # releases. Fail before creating the backend on an older installed package.
    check_versions()
    session = Path(LaunchConfiguration('session_directory').perform(context))
    if not session.is_absolute():
        raise RuntimeError('session_directory must be an absolute writable directory')
    session.mkdir(parents=True, exist_ok=True)
    database = session / 'rtabmap.db'
    if database.exists():
        raise RuntimeError(f'Refusing to reuse an existing RTAB-Map database: {database}')

    # This default is declared by Mapper rather than written in every profile.
    frontend = {'map_publish_period': 2.0}
    for path in (root / 'go2_3d/gicp.yaml',
                 Path(LaunchConfiguration('gicp_params').perform(context))):
        frontend.update(yaml.safe_load(path.read_text())['go2_gicp']['ros__parameters'])
    map_parameters = {key: frontend[key] for key in (
        'map_voxel_size', 'voxel_size', 'grid_resolution', 'floor_z',
        'obstacle_min_height', 'obstacle_max_height', 'map_publish_period')}
    map_parameters.update(
        map_name=ParameterValue(LaunchConfiguration('map_name'), value_type=str),
        output_directory=LaunchConfiguration('output_directory').perform(context),
        session_directory=str(session),
        calibration_file=calibration, max_pair_skew=0.10)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    map_parameters['use_sim_time'] = use_sim_time

    rtab_parameters = rtabmap_parameters(database, use_sim_time)
    # launch_ros otherwise YAML-parses strings such as '2'/'false' and changes
    # their types; the RTAB-Map core declares these parameters as strings.
    rtab_parameters = {key: ParameterValue(value, value_type=str) if '/' in key else value
                       for key, value in rtab_parameters.items()}
    nodes = [
        Node(executable='/usr/bin/python3', name='go2_rtabmap_camera', output='screen',
             arguments=[str(root / 'go2_3d/rtabmap_camera.py')],
             parameters=[{'calibration_file': calibration, 'use_sim_time': use_sim_time}]),
        Node(executable='/usr/bin/python3', name='go2_rtabmap_backend', output='screen',
             arguments=[str(root / 'go2_3d/rtabmap_backend.py')],
             parameters=[map_parameters]),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             namespace='rtabmap', output='screen', parameters=[rtab_parameters],
             remappings=[('rgb/image', '/rtabmap/input/image'),
                         ('rgb/camera_info', '/rtabmap/input/camera_info'),
                         ('scan_cloud', '/rtabmap/input/scan_cloud'),
                         ('odom', '/rtabmap/input/odom')]),
    ]
    # Never continue a session after losing its calibration, graph or map owner.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='RTAB-Map backend component exited'))]))
        for node in nodes]
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_calibration'),
        DeclareLaunchArgument('session_directory'),
        DeclareLaunchArgument('map_name', default_value='default'),
        DeclareLaunchArgument('output_directory', default_value='/maps'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gicp_params', default_value='/opt/go2_project/go2_3d/gicp.yaml'),
        OpaqueFunction(function=backend_nodes),
    ])
