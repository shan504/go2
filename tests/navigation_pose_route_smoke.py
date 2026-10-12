"""Pose-only route collection/start/pause with actual ROS actions and services."""
import json
from pathlib import Path
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import MarkerArray, Marker
from navigation_queue_smoke import Operator, QueueEndpoints, PoseStamped, wait_for


def main():
    preset = json.loads((Path(__file__).resolve().parents[1]/'docs/foxglove_go2_navigation.json').read_text())
    for panel in ('3D!g2initial','3D!g2goal'):
        assert preset['configById'][panel]['topics']['/navigation/waypoints']['visible']
    publish = preset['configById']['3D!g2goal']['publish']
    assert publish['type'] == 'pose' and publish['poseTopic'] == '/goal_pose'
    rclpy.init(args=['--ros-args','-p','mode:=navigation'])
    operator, endpoints = Operator(), QueueEndpoints()
    endpoints.destroy_publisher(endpoints.goal)
    endpoints.goal = endpoints.create_publisher(PoseStamped,publish['poseTopic'],100)
    executor = MultiThreadedExecutor(num_threads=5)
    for node in (operator,endpoints):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start()
    def clicks(xs):
        for x in xs:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            endpoints.goal.publish(pose)
    def queued():
        return [p.pose.position.x for _,p in operator.waypoints]
    def complete(index):
        endpoints.outcomes[index] = 4
    try:
        endpoints.valid = True
        wait_for(lambda: operator.localized() and len(operator.grids)==3 and
                 operator.action.server_is_ready() and operator.parameters.service_is_ready())
        wait_for(lambda: operator.buffer.can_transform('map','base_footprint',rclpy.time.Time()))
        wait_for(lambda: endpoints.goal.get_subscription_count()>0)
        clicks((0.4,0.8,1.2))
        wait_for(lambda: queued()==[0.4,0.8,1.2])
        assert not operator.route_started and endpoints.goals==0
        late = []
        endpoints.create_subscription(MarkerArray,'/navigation/waypoints',late.append,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE))
        wait_for(lambda: late and len(late[-1].markers)==7)
        assert [m.text for m in late[-1].markers if m.type==Marker.TEXT_VIEW_FACING]==['1','2','3']
        assert [m.pose.position.x for m in late[-1].markers if m.type==Marker.SPHERE]==[0.4,0.8,1.2]
        assert len({(m.ns,m.id) for m in late[-1].markers})==7
        endpoints.enable.publish(Bool(data=True))
        for index in (1,2,3):
            wait_for(lambda: endpoints.goals==index and operator.goal_handle is not None)
            complete(index)
        wait_for(lambda: operator.completed_waypoints==3 and operator.goal_handle is None)
        assert not operator.route_started and not operator.waypoints
        assert endpoints.get_parameter('enable_control').value is True
        print('PASS three Pose-only inputs, one Enable, ordered real SUCCEEDED results, late volatile numbered-route display')

        # No Point publisher existed for that complete three-waypoint route.
        # A later use of the unrelated browser tool must not start navigation.
        point = endpoints.create_publisher(PointStamped,'/clicked_point',10)
        point.publish(PointStamped())
        time.sleep(0.15)
        assert not operator.waypoints and endpoints.goals==3

        # An enabled bridge left over from the previous route is not permission
        # to start a newly collected route before the user's next Enable.
        enable_entered, enable_release = threading.Event(),threading.Event()
        def delayed_enable(parameters):
            if any(p.name=='enable_control' and p.value for p in parameters):
                enable_entered.set()
                assert enable_release.wait(2),'Enable test latch timed out'
            return SetParametersResult(successful=True)
        endpoints.add_on_set_parameters_callback(delayed_enable)
        endpoints.enable.publish(Bool(data=True))
        wait_for(enable_entered.is_set)
        clicks((0.4,0.8))
        wait_for(lambda: queued()==[0.4,0.8])
        enable_release.set()
        wait_for(lambda: 'Motion enable=True' in endpoints.operator_status)
        endpoints.remove_on_set_parameters_callback(delayed_enable)
        time.sleep(0.2)
        assert endpoints.goals==3 and not operator.route_started
        endpoints.valid = False
        wait_for(lambda: operator.route_pause_reason=='localization')
        wait_for(lambda: endpoints.get_parameter('enable_control').value is False and operator.pause_disable_future is None)
        clicks((1.2,))
        wait_for(lambda: queued()==[0.4,0.8,1.2])
        endpoints.valid = True
        wait_for(operator.localized)
        time.sleep(0.2)
        assert endpoints.goals==3
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals==4 and operator.goal_handle is not None)
        endpoints.valid = False
        wait_for(lambda: endpoints.cancels==1 and operator.goal_handle is None)
        wait_for(lambda: endpoints.get_parameter('enable_control').value is False and operator.pause_disable_future is None)
        assert queued()==[0.4,0.8,1.2] and not operator.route_started
        endpoints.valid = True
        wait_for(operator.localized)
        time.sleep(0.2)
        assert endpoints.goals==4,'Recovery silently restarted the canceled action'
        endpoints.enable.publish(Bool(data=True))
        for index in (5,6,7):
            wait_for(lambda: endpoints.goals==index and operator.goal_handle is not None)
            complete(index)
        wait_for(lambda: operator.completed_waypoints==3 and operator.goal_handle is None)
        assert endpoints.positions==[0.4,0.8,1.2,0.4,0.4,0.8,1.2]
        assert endpoints.max_active==1
        print('PASS new route stays collected with bridge enabled; localization pauses retain order; explicit Enable retries retained head')

        clicks((0.4,0.8))
        wait_for(lambda: queued()==[0.4,0.8])
        endpoints.accept_wait.clear()
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: operator.goal_pending)
        endpoints.valid = False
        wait_for(lambda: operator.cancel_requested and queued()==[0.4,0.8])
        endpoints.accept_wait.set()
        wait_for(lambda: endpoints.cancels==2 and not operator.goal_pending and operator.goal_handle is None)
        assert queued()==[0.4,0.8]
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_paused)
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.enable.publish(Bool(data=True))
        time.sleep(0.2)
        assert endpoints.goals==8 and not operator.route_started
        clicks((0.4,0.8))
        wait_for(lambda: queued()==[0.4,0.8])
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals==9 and operator.goal_handle is not None)
        endpoints.outcomes[9] = 6
        wait_for(lambda: operator.route_pause_reason=='navigation' and operator.goal_handle is None)
        endpoints.valid = False
        time.sleep(0.2)
        assert operator.route_pause_reason=='navigation'
        endpoints.valid = True
        wait_for(operator.localized)
        endpoints.enable.publish(Bool(data=True))
        time.sleep(0.2)
        assert endpoints.goals==9 and queued()==[0.4,0.8]
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: not operator.waypoints and not operator.route_paused)
        print('PASS late acceptance on localization loss retains route; Cancel/Stop clear; genuine navigation failure does not auto-retry')
    finally:
        if 'enable_release' in locals():
            enable_release.set()
        endpoints.accept_wait.set()
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
