"""Replay the same recorded raw inputs with the existing or RTAB-Map backend.

No live DDS/camera, motion control, Nav2 or recorded global TF participates.
Play /point_cloud2, /odom and /camera/image/compressed with --clock at <=1x.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def replay_nodes(context):
    root = Path('/opt/go2_project')
    backend = LaunchConfiguration('loop_backend').perform(context)
    parameters = [str(root/'go2_3d/gicp.yaml'),
                  LaunchConfiguration('gicp_params').perform(context), {
                      'mode': 'mapping', 'use_sim_time': True,
                      'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
                      'output_directory': LaunchConfiguration('output_directory').perform(context),
                  }]
    remappings = []
    if backend == 'rtabmap':
        parameters.append({'mapping_backend': 'rtabmap', 'map_frame': 'gicp_odom',
                           'loop_closure_enabled': False})
        remappings = [('/map_cloud', '/gicp/map_cloud'),
                      ('/registered_cloud', '/gicp/registered_cloud'),
                      ('/save_3d_map', '/gicp/save_unoptimized_map')]
    nodes = [
        Node(executable='/usr/bin/python3', name='go2_replay_odom_tf', output='screen',
             arguments=[str(root/'go2_3d/replay_odom_tf.py')], parameters=[{'use_sim_time': True}]),
        Node(executable='/usr/bin/python3', name='go2_gicp', output='screen',
             arguments=[str(root/'go2_3d/mapper.py')], parameters=parameters, remappings=remappings),
    ]
    if backend == 'rtabmap':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(root/'go2_3d/rtabmap_backend.launch.py')),
            launch_arguments=({key: LaunchConfiguration(key) for key in (
                'gicp_params', 'map_name', 'camera_calibration',
                'session_directory', 'output_directory')}
            | {'use_sim_time': 'true'}).items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('loop_backend', default_value='lidar', choices=['lidar', 'rtabmap']),
        DeclareLaunchArgument('gicp_params', default_value='/opt/go2_project/go2_3d/gicp_dense.yaml'),
        DeclareLaunchArgument('map_name', default_value='replay'),
        DeclareLaunchArgument('output_directory', default_value='/maps'),
        DeclareLaunchArgument('camera_calibration', default_value=''),
        DeclareLaunchArgument('session_directory', default_value=''),
        OpaqueFunction(function=replay_nodes),
    ])
