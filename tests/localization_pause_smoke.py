"""Real ROS: distinguish mapper invalidity and missing validity heartbeat."""
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from navigation_gateway_smoke import Operator, TestEndpoints, wait_for


def main():
    rclpy.init(args=['--ros-args', '-p', 'mode:=navigation'])
    operator, endpoints = Operator(), TestEndpoints()
    state = endpoints.create_publisher(String, '/localization/state', 1)
    timer = endpoints.create_timer(0.05, lambda: state.publish(
        String(data='state=registration_timeout valid=False; accepted_age=3.10s')))
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (operator, endpoints):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        endpoints.valid = True
        wait_for(lambda: operator.localized() and '/map' in operator.grids and
                 endpoints.goal.get_subscription_count() > 0)

        def collect():
            for i in range(4):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.orientation.w = 1.0
                pose.pose.position.x = 0.4*(i+1)
                endpoints.goal.publish(pose)
                wait_for(lambda: len(operator.waypoints) == i+1)

        collect()
        endpoints.valid = False
        wait_for(lambda: operator.route_paused)
        assert 'reason=localization_reported_invalid' in operator.localization_pause_detail
        assert 'state=registration_timeout' in operator.localization_pause_detail
        assert len(operator.waypoints) == 4 and endpoints.goals == 0
        endpoints.valid = True
        wait_for(operator.localized)
        time.sleep(0.2)
        assert operator.route_paused and not operator.route_started and endpoints.goals == 0
        print('PASS mapper invalidity: registration reason recorded; all four points retained; recovery does not auto-start', flush=True)

        # Cancel explicitly, then collect again to exercise heartbeat loss.
        operator.on_cancel(None)
        collect()
        endpoints.publish_valid = False
        wait_for(lambda: operator.route_paused)
        assert 'reason=localization_heartbeat_timeout' in operator.localization_pause_detail
        assert len(operator.waypoints) == 4 and endpoints.goals == 0
        endpoints.publish_valid = True
        wait_for(operator.localized)
        assert operator.route_paused and not operator.route_started
        print('PASS heartbeat loss: independent 0.5s watchdog, distinct reason, retained queue and explicit restart required', flush=True)
    finally:
        timer.cancel()
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
