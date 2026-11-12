"""Isolated raw-bag A/B: original GICP with either lidar or KISS backend.

Start only the mapper and timestamped odometry TF adapter. No SDK, camera,
operator, Nav2, command replay, or motion controller is part of this launch.
Use replay_kiss.sh for a network-isolated container and allowlisted bag topics.
"""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def replay_nodes(context):
    root = Path(__file__).resolve().parents[1]
    backend = LaunchConfiguration('loop_backend').perform(context)
    output = Path(LaunchConfiguration('output_directory').perform(context)).resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError('Replay output must be a new/empty directory; existing maps are never overwritten')
    output.mkdir(parents=True, exist_ok=True)
    profile = LaunchConfiguration('profile').perform(context)
    python = LaunchConfiguration('mapper_python').perform(context)
    parameters = [str(root/'go2_3d/gicp.yaml')]
    if profile == 'dense':
        parameters.append(str(root/'go2_3d/gicp_dense.yaml'))
    parameters.append({
        'mode': 'mapping', 'use_sim_time': True, 'map_frame': 'map',
        'mapping_backend': backend, 'loop_closure_enabled': True,
        'kiss_python': LaunchConfiguration('kiss_python').perform(context),
        'map_name': 'replay', 'output_directory': str(output/'maps'),
        'mapping_journal_directory': str(output/'journal'),
        'mapping_journal_provenance': LaunchConfiguration('provenance').perform(context),
    })
    return [
        Node(executable=python, name='go2_replay_odom_tf', output='screen',
             arguments=[str(root/'go2_3d/replay_odom_tf.py')], parameters=[{'use_sim_time': True}]),
        Node(executable=python, name='go2_gicp', output='screen',
             arguments=[str(root/'go2_3d/mapper.py')], parameters=parameters),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('loop_backend', default_value='kiss', choices=['lidar', 'kiss']),
        DeclareLaunchArgument('profile', default_value='dense', choices=['standard', 'dense']),
        DeclareLaunchArgument('output_directory'),
        DeclareLaunchArgument('mapper_python', default_value=sys.executable),
        DeclareLaunchArgument('kiss_python', default_value='/opt/kiss-venv/bin/python3'),
        DeclareLaunchArgument('provenance', default_value='unknown',
                             choices=['unknown', 'robot_recording', 'synthetic']),
        OpaqueFunction(function=replay_nodes),
    ])
