#!/usr/bin/env python3
"""Read-only Nav2 grid/TF diagnostics; never sends a goal or motion request."""
import argparse
from collections import deque
import math
from pathlib import Path
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
from geometry import pose_matrix, cloud_xyz, transform_points, grid_cell
from ground import heights


def cell(grid, xy):
    """World coordinate to cell, including a rotated grid origin."""
    return grid_cell(grid,xy)


def label(value):
    if value == -1:
        return 'unknown (blocked by allow_unknown=false)'
    if value >= 99:
        return 'inscribed/lethal (blocked)'
    return 'free' if value == 0 else 'graded inflation cost'


def connected(data, start, goal):
    """Cell-center connectivity only, not a footprint-aware Nav2 plan."""
    if start is None or goal is None:
        return 'outside grid'
    if data.size > 500_000:
        return 'not checked: grid exceeds 500000 cells'
    if not 0 <= data[goal[1],goal[0]] < 99:
        return 'NO: goal cell blocked'
    visited = np.zeros(data.shape,dtype=bool)
    visited[start[1],start[0]] = True
    pending = deque([start])
    while pending:
        x,y = pending.popleft()
        if (x,y) == goal:
            return 'YES (cell centers only; not a Nav2 plan)'
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if (0 <= nx < data.shape[1] and 0 <= ny < data.shape[0]
                    and not visited[ny,nx] and 0 <= data[ny,nx] < 99):
                visited[ny,nx] = True
                pending.append((nx,ny))
    return 'NO: disconnected at cell-center level'


class NavCheck(Node):
    def __init__(self, goal):
        super().__init__('go2_nav_readonly_check')
        self.goal, self.latest, self.received = goal, {}, {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        retained = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        kinds = {'/map':OccupancyGrid,'/global_costmap/costmap':OccupancyGrid,
                 '/local_costmap/costmap':OccupancyGrid,'/scan':LaserScan,
                 '/point_cloud2':PointCloud2,
                 '/localization/valid':Bool,'/localization/status':String,
                 '/operator/status':String}
        for topic,kind in kinds.items():
            def receive(msg,topic=topic):
                self.latest[topic], self.received[topic] = msg,time.monotonic()
            self.create_subscription(kind,topic,receive,
                                     retained if kind is OccupancyGrid or topic == '/operator/status' else qos_profile_sensor_data)

    def summary(self):
        lines = ['READ-ONLY Nav2 check; no goal, parameter or motion writes',
                 f'Goal in map: {self.goal}']
        config_path = Path('/runtime/config/nav2_3d.yaml')
        if config_path.exists():
            config = yaml.safe_load(config_path.read_text())
            for name in ('global_costmap','local_costmap'):
                p = config[name][name]['ros__parameters']
                lines.append(f'{name} generated config: frame={p["global_frame"]} '
                             f'initial_size={p.get("width")}x{p.get("height")}m resolution={p["resolution"]} '
                             f'inflation={p["inflation_layer"]} footprint={p["footprint"]} '
                             f'padding={p.get("footprint_padding")}')
            lines.append('Global StaticLayer resizes to the saved map; actual grid dimensions follow below.')
        for topic in ('/localization/valid','/localization/status','/operator/status'):
            msg = self.latest.get(topic)
            age = time.monotonic()-self.received[topic] if msg else math.inf
            lines.append(f'{topic}: {msg.data if msg else "NO MESSAGES"} receipt_age={age:.2f}s')
        try:
            tf = self.buffer.lookup_transform('map','base_link',Time())
            t,q = tf.transform.translation,tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            lines.append(f'Robot in map: xyz={[round(t.x,3),round(t.y,3),round(t.z,3)]} yaw={math.degrees(yaw):.1f}deg')
        except TransformException as error:
            lines.append(f'TF map -> base_link: MISSING; initialize GICP first. {error}')
        for topic in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
            grid = self.latest.get(topic)
            if grid is None:
                lines.append(f'{topic}: NO MESSAGES; publishers={self.count_publishers(topic)}')
                continue
            width,height = grid.info.width,grid.info.height
            if not width or not height or len(grid.data) != width*height:
                lines.append(f'{topic}: invalid or empty grid')
                continue
            data = np.asarray(grid.data).reshape(height,width)
            lines.append(f'{topic}: frame={grid.header.frame_id!r} cells={width}x{height} '
                         f'resolution={grid.info.resolution:.3f}m '
                         f'receipt_age={time.monotonic()-self.received[topic]:.2f}s '
                         f'free={np.count_nonzero(data==0)} unknown={np.count_nonzero(data==-1)} '
                         f'graded={np.count_nonzero((data>0)&(data<99))} '
                         f'inscribed/lethal={np.count_nonzero(data>=99)}')
            try:
                tf = self.buffer.lookup_transform(grid.header.frame_id,'base_link',Time())
                t = tf.transform.translation
                start = cell(grid,(t.x,t.y))
                if grid.header.frame_id == 'map':
                    target = cell(grid,self.goal)
                else:
                    tf = self.buffer.lookup_transform(grid.header.frame_id,'map',Time())
                    t,q = tf.transform.translation,tf.transform.rotation
                    matrix = pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
                    target = cell(grid,(matrix @ [*self.goal,0.0,1.0])[:2])
                for name,index in (('robot',start),('goal',target)):
                    value = int(data[index[1],index[0]]) if index else None
                    lines.append(f'  {name} cell={index} value={value}: {label(value) if value is not None else "outside grid"}')
                if target:
                    lethal = np.argwhere(data == 100)
                    if len(lethal):
                        squared = np.sum((lethal-np.array([target[1],target[0]]))**2,axis=1)
                        row,col = lethal[np.argmin(squared)]
                        distance = float(np.sqrt(squared.min())*grid.info.resolution)
                        lines.append(f'  closest lethal cell to goal: cell=({col}, {row}) '
                                     f'center_distance={distance:.3f}m (quantized grid distance)')
                lines.append('  connectivity: '+connected(data,start,target))
            except (TransformException,ValueError) as error:
                lines.append(f'  cell checks unavailable: {error}')
        scan = self.latest.get('/scan')
        if scan:
            ranges = np.asarray(scan.ranges)
            ranges = ranges[np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)]
            lines.append(f'/scan: frame={scan.header.frame_id!r} finite_returns={len(ranges)} '
                         f'min={float(ranges.min()) if len(ranges) else "none"}m '
                         f'returns_under_0.6m={np.count_nonzero(ranges<0.6)}')
            stamp = Time.from_msg(scan.header.stamp)
            lines.append(f'/scan host_age={(self.get_clock().now()-stamp).nanoseconds/1e9:.3f}s '
                         f'TF map <- {scan.header.frame_id} at scan stamp='
                         f'{bool(self.buffer.can_transform("map",scan.header.frame_id,stamp))} '
                         f'at scan stamp+0.05s={bool(self.buffer.can_transform("map",scan.header.frame_id,Time(nanoseconds=stamp.nanoseconds+50_000_000)))}')
        packet = self.latest.get('/point_cloud2')
        if packet:
            try:
                tf = self.buffer.lookup_transform('base_link','map',Time())
                t,q = tf.transform.translation,tf.transform.rotation
                target = (pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]) @ [*self.goal,0.0,1.0])[:2]
                points = cloud_xyz(packet)
                points = points[np.isfinite(points).all(axis=1)]
                nearby = points[np.linalg.norm(points[:,:2]-target,axis=1)<0.25]
                lines.append(f'Latest body-cloud packet near goal XY (radius 0.25m): points={len(nearby)}')
                if len(nearby):
                    lines.append(f'  body-frame z min/median/max={np.round(np.quantile(nearby[:,2],[0,0.5,1]),3).tolist()}m')
                    metadata_path = Path('/maps/latest/metadata.yaml')
                    metadata = yaml.safe_load(metadata_path.read_text()) if metadata_path.exists() else {}
                    if 'ground_model' in metadata:
                        body_from_map = pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
                        h = heights(transform_points(nearby,np.linalg.inv(body_from_map)),metadata['ground_model'])
                        p = metadata['parameters']
                        selected = (h>=p['obstacle_min_height'])&(h<=p['obstacle_max_height'])
                        lines.append(f'  ground-relative height min/median/max={np.round(np.quantile(h,[0,0.5,1]),3).tolist()}m; obstacle points={np.count_nonzero(selected)}')
                    else:
                        lines.append('  Legacy map lacks a ground model; run tools.sh repair-map before navigation')
            except (TransformException,ValueError) as error:
                lines.append(f'  body-cloud goal height check unavailable: {error}')
        return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('x',type=float)
    parser.add_argument('y',type=float)
    args = parser.parse_args()
    if not math.isfinite(args.x) or not math.isfinite(args.y):
        parser.error('goal coordinates must be finite')
    rclpy.init(args=[])
    node = NavCheck((args.x,args.y))
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=0.2)
        print(node.summary(),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
