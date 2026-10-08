"""Live 3D sensor preview: deliberately has no SLAM/AMCL/map TF publisher."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    here = Path(__file__).resolve().parent
    dds_bridge = "/opt/go2_edu/go2_edu_dds_bridge.py"
    urdf = Path("/opt/go2_edu/config/go2_edu.urdf")
    whitelist = '["^/(tf|tf_static|odom|point_cloud2|robot_description)$","^/camera/image/compressed$"]'
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("topic_whitelist", default_value=whitelist),
        Node(executable="/usr/bin/python3", name="go2_edu_dds_bridge", output="screen",
             arguments=[dds_bridge], parameters=[{"enable_control": False}]),
        Node(executable="/usr/bin/python3", name="go2_camera_sample", output="screen",
             arguments=[str(here / "go2_camera_sample.py")]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="go2_robot_state_publisher", output="screen",
             parameters=[{"use_sim_time": False, "robot_description": urdf.read_text()}]),
        IncludeLaunchDescription(FrontendLaunchDescriptionSource(str(
            Path(get_package_share_directory("foxglove_bridge")) / "launch/foxglove_bridge_launch.xml"))),
    ])
