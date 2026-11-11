"""Real ROS route collection independent of live Nav2 execution readiness."""
import copy
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker, MarkerArray
from navigation_gateway_smoke import Operator, TestEndpoints, wait_for, grid_cell


class CollectionEndpoints(TestEndpoints):
    def __init__(self):
        self.positions = []
        super().__init__(start_action=False)
        self.tf_enabled = False
        self.succeed_goal = True
        self.grid_enabled['/global_costmap/costmap'] = False
        self.grid_enabled['/local_costmap/costmap'] = False
        self.point = self.create_publisher(PointStamped, '/navigation/waypoint', 100)
        self.markers = None
        self.create_subscription(MarkerArray, '/navigation/waypoints',
            lambda message: setattr(self, 'markers', message),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def execute(self, handle):
        self.positions.append(handle.request.pose.pose.position.x)
        return super().execute(handle)


def main():
    rclpy.init(args=['--ros-args', '-p', 'mode:=navigation'])
    operator, endpoints = Operator(), CollectionEndpoints()
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (operator, endpoints):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    xs = [0.4, 0.8, 1.2]
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 1
    pose.pose.orientation.w = 1.0

    def queued():
        return [message.pose.position.x for _, message in operator.waypoints]

    def collect(first_point=False):
        for index, x in enumerate(xs):
            if index == (0 if first_point else 1):
                wait_for(lambda: endpoints.point.get_subscription_count() > 0)
                message = PointStamped()
                message.header = copy.deepcopy(pose.header)
                message.point.x = x
                endpoints.point.publish(message)
            else:
                message = copy.deepcopy(pose)
                message.pose.position.x = x
                endpoints.goal.publish(message)
            wait_for(lambda: len(operator.waypoints) == index + 1)
        assert queued() == xs
        assert [number for number, _ in operator.waypoints] == [1, 2, 3]
        wait_for(lambda: endpoints.markers is not None and
                 [marker.text for marker in endpoints.markers.markers
                  if marker.type == Marker.TEXT_VIEW_FACING] == ['1', '2', '3'])
        assert [marker.pose.position.x for marker in endpoints.markers.markers
                if marker.type == Marker.SPHERE] == xs

    def clear_route():
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: not operator.waypoints and not operator.route_paused and
                 not endpoints.get_parameter('enable_control').value and
                 operator.pause_disable_future is None)

    def blocked(reason):
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: operator.route_pause_reason == 'validation')
        wait_for(lambda: reason in endpoints.operator_status)
        assert endpoints.goals == 0 and endpoints.positions == []
        assert queued() == xs, 'Execution readiness discarded the collected route'
        assert [number for number, _ in operator.waypoints] == [1, 2, 3]
        clear_route()

    def tf_age():
        transform = operator.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        return (operator.get_clock().now() -
                rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9

    def costmaps_fresh():
        return all(topic in operator.grids and time.monotonic() - operator.grids[topic][1] < 0.2
                   for topic in ('/global_costmap/costmap', '/local_costmap/costmap'))

    try:
        wait_for(lambda: operator.valid_received is not None and '/map' in operator.grids and
                 operator.parameters.service_is_ready() and operator.motion_state.service_is_ready() and
                 endpoints.goal.get_subscription_count())
        assert not operator.action.server_is_ready() and len(operator.grids) == 1
        # Both current Pose and retained Point layouts can collect the whole
        # route before localization, TF, costmaps or the action server exist.
        collect()
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'Enable rejected: navigation mode and live GICP localization required'
                 in endpoints.operator_status)
        assert not endpoints.get_parameter('enable_control').value and endpoints.goals == 0
        assert queued() == xs
        endpoints.valid = True
        wait_for(lambda: operator.localized() and operator.pause_disable_future is None)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'Enable rejected: Nav2 NavigateToPose action unavailable' in endpoints.operator_status)
        assert queued() == xs and endpoints.goals == 0
        assert not endpoints.get_parameter('enable_control').value
        endpoints.start_action()
        wait_for(operator.action.server_is_ready)
        blocked('map')  # The live map -> base_footprint transform is absent.

        endpoints.tf_enabled = True
        wait_for(lambda: operator.buffer.can_transform('map', 'base_footprint', rclpy.time.Time()))
        wait_for(lambda: 0 <= tf_age() < 0.2)
        collect()
        blocked('/global_costmap/costmap unavailable')

        for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
            endpoints.grid_enabled[topic] = True
        wait_for(costmaps_fresh)
        endpoints.tf_enabled = False
        wait_for(lambda: tf_age() > 0.6)
        collect()
        blocked('TF stale')

        endpoints.tf_enabled = True
        wait_for(lambda: 0 <= tf_age() < 0.2)
        for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
            endpoints.grid_enabled[topic] = False
        wait_for(lambda: all(time.monotonic() - operator.grids[topic][1] > 3.1
                            for topic in ('/global_costmap/costmap', '/local_costmap/costmap')))
        collect()
        blocked('/global_costmap/costmap stale')

        for topic in ('/global_costmap/costmap', '/local_costmap/costmap'):
            endpoints.grid_enabled[topic] = True
        wait_for(costmaps_fresh)
        # Legacy Point input still uses the same static-map rejection.
        _, static = endpoints.grids['/map']
        col, row = grid_cell(operator.grids['/map'][0], (0.4, 0.0))
        index = row * static.info.width + col
        static.data[index] = 100
        wait_for(lambda: operator.grids['/map'][0].data[index] == 100)
        point = PointStamped()
        point.header.frame_id = 'map'
        point.point.x = 0.4
        endpoints.point.publish(point)
        wait_for(lambda: 'Goal rejected: /map:' in endpoints.operator_status)
        assert not operator.waypoints and endpoints.goals == 0
        static.data[index] = 0
        wait_for(lambda: operator.grids['/map'][0].data[index] == 0)

        _, local = endpoints.grids['/local_costmap/costmap']
        col, row = grid_cell(operator.grids['/local_costmap/costmap'][0], (0.4, 0.0))
        index = row * local.info.width + col
        local.data[index] = 99
        wait_for(lambda: operator.grids['/local_costmap/costmap'][0].data[index] == 99)
        collect(first_point=True)
        blocked('inside obstacle clearance')
        local.data[index] = 0
        wait_for(lambda: operator.grids['/local_costmap/costmap'][0].data[index] == 0)

        # A newly selected map can have a different extent, origin and cell
        # size. Collection and static rejection must follow its live geometry.
        endpoints.grid_enabled['/map'] = False
        static.info.width, static.info.height = 30, 20
        static.info.resolution = 0.2
        static.info.origin.position.x, static.info.origin.position.y = -1.5, -1.0
        static.data = [0] * 600
        endpoints.grid_enabled['/map'] = True
        wait_for(lambda: operator.grids['/map'][0].info.width == 30 and
                 operator.grids['/map'][0].info.origin.position.x == -1.5)
        col, row = grid_cell(operator.grids['/map'][0], (0.8, 0.0))
        index = row * static.info.width + col
        static.data[index] = -1
        wait_for(lambda: operator.grids['/map'][0].data[index] == -1)
        point.point.x = 0.8
        endpoints.point.publish(point)
        wait_for(lambda: 'value=-1 is not free' in endpoints.operator_status)
        assert not operator.waypoints and endpoints.goals == 0
        static.data[index] = 0
        wait_for(lambda: operator.grids['/map'][0].data[index] == 0)
        collect()
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: operator.completed_waypoints == 3 and not operator.waypoints and
                 operator.goal_handle is None and not operator.goal_pending)
        assert endpoints.goals == 3 and endpoints.positions == xs
        assert not operator.route_started and pose.header.stamp.sec == 1
        assert endpoints.last_goal.header.stamp.sec == endpoints.last_goal.header.stamp.nanosec == 0
        print('PASS mixed Pose/retained Point collection: three numbered goals retained without action, localization, TF or costmaps')
        print('PASS execution gates: missing/stale TF/costmaps and local clearance block motion; healthy Enable completes all three in order')
    finally:
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
