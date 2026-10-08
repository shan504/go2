"""Evaluate our include and actual Humble Nav2 conditions without starting processes."""
import importlib.util
import os
from pathlib import Path
from unittest.mock import patch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource


nav2_share = os.environ.get('GO2_NAV2_SHARE_DIR') or get_package_share_directory('nav2_bringup')
project = Path(__file__).resolve().parents[1]


def shares(package):
    # Only paths are supplied; substitutions and Nav2 conditions use real ROS launch.
    return nav2_share if package == 'nav2_bringup' else '/unused'


with patch('ament_index_python.packages.get_package_share_directory', side_effect=shares):
    spec = importlib.util.spec_from_file_location('go2_stack', project/'go2_3d/stack.launch.py')
    stack = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(stack)
    with patch.object(Path, 'read_text', return_value='<robot name="launch_test"/>'):
        description = stack.generate_launch_description()
    for mode in ('mapping', 'localization', 'navigation'):
        context = LaunchContext()
        context.launch_configurations['mode'] = mode
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        group = next(action for action in description.entities if isinstance(action,GroupAction))
        assert group.condition.evaluate(context) == (mode == 'navigation')
        include = next(action for action in group.get_sub_entities()
                       if isinstance(action, IncludeLaunchDescription)
                       and isinstance(action.launch_description_source, PythonLaunchDescriptionSource))
        if mode == 'navigation':
            remap = next(action for action in group.get_sub_entities() if isinstance(action,SetRemap))
            remap.execute(context)
            assert ('goal_pose','/navigation/nav2_direct_goal_unused') in context.launch_configurations['ros_remaps']
            included = include.execute(context)
            for action in included:
                if not isinstance(action, LaunchDescription):
                    action.execute(context)
            nav2 = included[-1]
            for action in nav2.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            conditions = [action.condition.evaluate(context) for action in nav2.entities
                          if getattr(action, 'condition', None) is not None]
            assert conditions == [True, False], conditions
            assert context.launch_configurations['use_composition'] == 'False'
            assert context.launch_configurations['autostart'] == 'false'
        print(f'PASS launch conditions: {mode}')
