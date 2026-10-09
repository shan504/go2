#!/usr/bin/env python3
"""Prepare independent SDK configuration copies for the 3D route."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0, str(Path(__file__).resolve().parent))
from denoise import settings
sys.path.insert(0, '/opt/go2_project/patches/dds')
from prepare import prepare as legacy_prepare


def main():
    output = Path('/runtime/config')
    legacy_prepare(Path(get_package_share_directory('go2_robot_sdk')), output)
    # The SDK attaches base_footprint rigidly to the tilting body. Replace only
    # that generated joint with Mapper's timestamped map-plane projection.
    tree = ET.parse(output/'go2_edu.urdf')
    for joint in list(tree.getroot().findall('joint')):
        if joint.find('child').get('link') == 'base_footprint':
            if joint.get('type') != 'fixed' or joint.find('parent').get('link') != 'base_link':
                raise RuntimeError('Unexpected base_footprint joint; refusing replacement')
            tree.getroot().remove(joint)
    if any(joint.find(part).get('link') == 'base_footprint'
           for joint in tree.getroot().findall('joint') for part in ('parent','child')):
        raise RuntimeError('Other base_footprint joints exist; refusing an ambiguous URDF edit')
    for link in list(tree.getroot().findall('link')):
        if link.get('name') == 'base_footprint':
            tree.getroot().remove(link)
    tree.write(output/'go2_edu.urdf', encoding='utf-8', xml_declaration=True)
    nav = yaml.safe_load((output/'nav2_edu.yaml').read_text())
    nav.pop('amcl', None)
    for key in ('local_costmap','global_costmap'):
        params = nav[key][key]['ros__parameters']
        params.pop('robot_radius',None)
        params['footprint'] = '[[-0.40,-0.22],[-0.40,0.22],[0.40,0.22],[0.40,-0.22]]'
        params['robot_base_frame'] = 'base_footprint'
        params['global_frame'] = 'map' if key == 'global_costmap' else 'odom'
        params['transform_tolerance'] = 0.6
        params['resolution'] = 0.05 if key == 'local_costmap' else 0.1
        params['footprint_padding'] = 0.01
        params['always_send_full_costmap'] = True
        if key == 'local_costmap':
            params.update(width=4,height=4,rolling_window=True)
        params['update_frequency'] = 5.0 if key == 'local_costmap' else 1.0
        params['publish_frequency'] = 2.0 if key == 'local_costmap' else 1.0
        # Set explicit plugins rather than depending on a particular SDK preset.
        params['plugins'] = (['static_layer'] if key == 'global_costmap' else []) + ['obstacle_layer','denoise_layer','inflation_layer']
        params['denoise_layer'] = {'plugin':'nav2_costmap_2d::DenoiseLayer',
                                  'enabled':True, **settings()}
        params['static_layer'] = {'plugin':'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local':True}
        params['obstacle_layer'] = {'plugin':'nav2_costmap_2d::ObstacleLayer',
            'observation_sources':'scan', 'scan':{'topic':'/scan','data_type':'LaserScan',
            'clearing':True,'marking':True,'inf_is_valid':False,'max_obstacle_height':2.0,
            'raytrace_min_range':0.35,'raytrace_max_range':15.0,
            'obstacle_min_range':0.35,'obstacle_max_range':12.0}}
        params['inflation_layer'] = {'plugin':'nav2_costmap_2d::InflationLayer',
                                    # Padded half-width 0.23m plus 0.02m soft
                                    # buffer. 0.02m total would omit body clearance.
                                    'inflation_radius':0.25,'cost_scaling_factor':12.0}
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
    nav['bt_navigator']['ros__parameters'].update(global_frame='map',robot_base_frame='base_footprint',odom_topic='/odom',
        default_nav_to_pose_bt_xml='/opt/go2_project/go2_3d/navigate.xml')
    nav['behavior_server']['ros__parameters']['robot_base_frame'] = 'base_footprint'
    (output/'nav2_3d.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))


if __name__ == '__main__':
    main()
