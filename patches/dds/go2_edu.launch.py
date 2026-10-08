"""Use the installed SDK assets and Humble SLAM/Nav2 with native EDU DDS."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    here = Path(__file__).resolve().parent
    sdk = Path(get_package_share_directory("go2_robot_sdk"))
    config = here / "config"
    nav_launch = Path(get_package_share_directory("nav2_bringup")) / "launch"
    mode = LaunchConfiguration("mode")
    map_file = LaunchConfiguration("map")
    real_time = {"use_sim_time": "false", "params_file": str(config / "nav2_edu.yaml"),
                 "autostart": "true", "use_composition": "false"}
    whitelist = "[\"^/(tf|tf_static|scan|point_cloud2|odom|map|map_metadata|robot_description|plan|local_plan|goal_pose|initialpose|cmd_vel.*)$\",\"^/(local_costmap|global_costmap)/(costmap|costmap_updates|published_footprint)$\"]"
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mapping", choices=["mapping", "localization"]),
        DeclareLaunchArgument("map", default_value=str(here / "maps/go2_map.yaml")),
        DeclareLaunchArgument("enable_control", default_value="false"),
        DeclareLaunchArgument("rviz2", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("topic_whitelist", default_value=whitelist),
        DeclareLaunchArgument("capabilities", default_value="[clientPublish,services,parameters,parametersSubscribe,connectionGraph,assets]"),
        DeclareLaunchArgument("port", default_value="8765"),
        Node(executable="/usr/bin/python3", name="go2_edu_dds_bridge", output="screen",
             arguments=[str(here / "go2_edu_dds_bridge.py")],
             parameters=[{"enable_control": ParameterValue(LaunchConfiguration("enable_control"), value_type=bool)}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="go2_robot_state_publisher", output="screen",
             parameters=[{"use_sim_time": False, "robot_description": (config / "go2_edu.urdf").read_text()}]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="go2_pointcloud_to_laserscan", output="screen",
             remappings=[("cloud_in", "/point_cloud2"), ("scan", "/scan")],
             parameters=[{"use_sim_time": False, "target_frame": "base_link",
                          "min_height": -0.2, "max_height": 0.5,
                          "angle_min": -3.141592653589793, "angle_max": 3.141592653589793,
                          "angle_increment": 0.008726646259971648,
                          "scan_time": 0.1, "range_min": 0.2, "range_max": 20.0,
                          "transform_tolerance": 0.2, "use_inf": True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("slam_toolbox")) / "launch/online_async_launch.py")),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
            launch_arguments={"use_sim_time": "false", "slam_params_file": str(config / "slam_edu.yaml")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_launch / "localization_launch.py")),
            condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
            launch_arguments=dict(real_time, map=map_file).items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_launch / "navigation_launch.py")),
            launch_arguments=real_time.items()),
        IncludeLaunchDescription(FrontendLaunchDescriptionSource(str(
            Path(get_package_share_directory("foxglove_bridge")) / "launch/foxglove_bridge_launch.xml")),
            condition=IfCondition(LaunchConfiguration("foxglove"))),
        Node(package="rviz2", executable="rviz2", name="go2_rviz2", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz2")),
             arguments=["-d", str(sdk / "config/single_robot_conf.rviz")],
             parameters=[{"use_sim_time": False}]),
    ])
