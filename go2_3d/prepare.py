#!/usr/bin/env python3
"""Prepare independent SDK configuration copies for the 3D route."""
import sys
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0, '/opt/go2_project/patches/dds')
from prepare import prepare as legacy_prepare


def main():
    output = Path('/runtime/config')
    legacy_prepare(Path(get_package_share_directory('go2_robot_sdk')), output)
    nav = yaml.safe_load((output/'nav2_edu.yaml').read_text())
    nav.pop('amcl', None)
    for key in ('local_costmap','global_costmap'):
        params = nav[key][key]['ros__parameters']
        params.pop('robot_radius',None)
        params['footprint'] = '[[-0.40,-0.22],[-0.40,0.22],[0.40,0.22],[0.40,-0.22]]'
        params['robot_base_frame'] = 'base_link'
        params['global_frame'] = 'map' if key == 'global_costmap' else 'odom'
        params['transform_tolerance'] = 0.6
        params['resolution'] = 0.1
        params['update_frequency'] = 5.0 if key == 'local_costmap' else 1.0
        params['publish_frequency'] = 2.0 if key == 'local_costmap' else 1.0
        # Set explicit plugins rather than depending on a particular SDK preset.
        params['plugins'] = ['static_layer','obstacle_layer','inflation_layer'] if key == 'global_costmap' else ['obstacle_layer','inflation_layer']
        params['static_layer'] = {'plugin':'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local':True}
        params['obstacle_layer'] = {'plugin':'nav2_costmap_2d::ObstacleLayer',
            'observation_sources':'scan', 'scan':{'topic':'/scan','data_type':'LaserScan',
            'clearing':True,'marking':True,'inf_is_valid':True,'max_obstacle_height':2.0,
            'raytrace_min_range':0.35,'raytrace_max_range':15.0,
            'obstacle_min_range':0.35,'obstacle_max_range':12.0}}
        params['inflation_layer'] = {'plugin':'nav2_costmap_2d::InflationLayer',
                                    'inflation_radius':0.55,'cost_scaling_factor':3.0}
    nav['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] = True
    nav['planner_server']['ros__parameters']['GridBased'] = {
        'plugin':'nav2_navfn_planner/NavfnPlanner','tolerance':0.25,
        'use_astar':True,'allow_unknown':False}
    controller = nav['controller_server']['ros__parameters']
    controller.update(controller_frequency=10.0,min_x_velocity_threshold=0.001,
                      min_y_velocity_threshold=0.001,min_theta_velocity_threshold=0.001)
    follow = controller['FollowPath']
    follow['critics'] = ['ObstacleFootprint' if item == 'BaseObstacle' else item for item in follow['critics']]
    follow['ObstacleFootprint.scale'] = 1.0
    nav['bt_navigator']['ros__parameters'].update(global_frame='map',robot_base_frame='base_link',odom_topic='/odom')
    (output/'nav2_3d.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))


if __name__ == '__main__':
    main()
