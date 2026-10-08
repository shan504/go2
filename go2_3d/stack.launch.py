"""Native Go2 DDS, camera, 3D GICP; optional Nav2, with no AMCL/2D SLAM."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    navigation = IfCondition(PythonExpression(["'",mode,"' == 'navigation'"]))
    mapper_mode = PythonExpression(["'mapping' if '",mode,"' == 'mapping' else 'localization'"])
    root = Path('/opt/go2_project')
    config = Path('/runtime/config')
    params = str(config/'nav2_3d.yaml')
    whitelist = '["^/(tf|tf_static|odom|point_cloud2|registered_cloud|map_cloud|map|map_metadata|scan|robot_description|initialpose|goal_pose|plan|local_plan|cmd_vel.*)$","^/camera/image/compressed$","^/(localization|operator|mapping|control|navigation)/.*$","^/(local_costmap|global_costmap)/.*$"]'
    return LaunchDescription([
        DeclareLaunchArgument('mode',default_value='mapping',choices=['mapping','localization','navigation']),
        DeclareLaunchArgument('camera',default_value='true'),
        DeclareLaunchArgument('gicp_params',default_value=str(root/'go2_3d/gicp.yaml')),
        DeclareLaunchArgument('port',default_value='8765'),
        DeclareLaunchArgument('auto_initialize',default_value='false'),
        DeclareLaunchArgument('initial_x',default_value='0.0'),
        DeclareLaunchArgument('initial_y',default_value='0.0'),
        DeclareLaunchArgument('initial_z',default_value='0.0'),
        DeclareLaunchArgument('initial_yaw_degrees',default_value='0.0'),
        DeclareLaunchArgument('topic_whitelist',default_value=whitelist),
        DeclareLaunchArgument('capabilities',default_value='[clientPublish,services,parameters,parametersSubscribe,connectionGraph,assets]'),
        DeclareLaunchArgument('send_buffer_limit',default_value='33554432'),
        Node(executable='/usr/bin/python3',name='go2_edu_dds_bridge',output='screen',
             arguments=[str(root/'go2_3d/control_bridge.py')],parameters=[{'enable_control':False,
                 'allow_motion':ParameterValue(PythonExpression(["'",mode,"' == 'navigation'"]),value_type=bool)}]),
        Node(executable='/usr/bin/python3',name='go2_camera_sample',output='screen',
             condition=IfCondition(LaunchConfiguration('camera')),
             arguments=[str(root/'patches/sensors/go2_camera_sample.py')]),
        Node(package='robot_state_publisher',executable='robot_state_publisher',output='screen',
             parameters=[{'robot_description':(config/'go2_edu.urdf').read_text(),'use_sim_time':False}]),
        Node(executable='/usr/bin/python3',name='go2_gicp',output='screen',
             arguments=[str(root/'go2_3d/mapper.py')],
             parameters=[str(root/'go2_3d/gicp.yaml'),LaunchConfiguration('gicp_params'),{'mode':mapper_mode,
                 'auto_initialize':ParameterValue(LaunchConfiguration('auto_initialize'),value_type=bool),
                 **{key:ParameterValue(LaunchConfiguration(key),value_type=float)
                    for key in ('initial_x','initial_y','initial_z','initial_yaw_degrees')}}]),
        Node(executable='/usr/bin/python3',name='go2_operator',output='screen',
             arguments=[str(root/'go2_3d/operator_bridge.py')],parameters=[{'mode':mode}]),
        Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',output='screen',
             condition=navigation,remappings=[('cloud_in','/navigation/obstacle_cloud'),('scan','/scan')],
             parameters=[{'target_frame':'base_link','min_height':-10.0,'max_height':10.0,
                          'range_min':0.35,'range_max':20.0,'scan_time':0.1,
                          'angle_min':-3.14159,'angle_max':3.14159,'angle_increment':0.01,
                          'transform_tolerance':0.2,'use_inf':False,'use_sim_time':False}]),
        Node(executable='/usr/bin/python3',name='go2_ground_obstacles',output='screen',
             condition=navigation,arguments=[str(root/'go2_3d/obstacle_cloud.py')]),
        Node(executable='/usr/bin/python3',name='go2_nav2_ready',output='screen',
             condition=navigation,arguments=[str(root/'go2_3d/nav2_ready.py')]),
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
             condition=navigation,parameters=[params,{'yaml_filename':'/maps/latest/nav.yaml','use_sim_time':False}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_map',
             condition=navigation,parameters=[{'autostart':True,'node_names':['map_server'],'use_sim_time':False}]),
        # Humble's bt_navigator subscribes to goal_pose and forwards it directly
        # to its action. Isolate this shortcut so only Operator handles clicks.
        GroupAction(condition=navigation,actions=[
          SetRemap(src='goal_pose',dst='/navigation/nav2_direct_goal_unused'),
          IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup'))/'launch/navigation_launch.py')),
             launch_arguments={'params_file':params,'use_sim_time':'false',
                                                   # Humble evaluates this inside PythonExpression(['not ', ...]).
                                                   'autostart':'false','use_composition':'False'}.items()),
        ]),
        IncludeLaunchDescription(FrontendLaunchDescriptionSource(str(Path(get_package_share_directory('foxglove_bridge'))/'launch/foxglove_bridge_launch.xml'))),
    ])
