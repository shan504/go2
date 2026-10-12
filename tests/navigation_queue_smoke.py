"""Actual ROS action/service queue protocol; no planner, DDS or robot motion."""
import copy
import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import MarkerArray, Marker
from navigation_gateway_smoke import Operator, TestEndpoints, wait_for, PoseStamped, grid_cell


class QueueEndpoints(TestEndpoints):
    def __init__(self):
        self.positions = []
        self.outcomes = {}
        self.active = self.max_active = 0
        self.accept_wait = threading.Event()
        self.accept_wait.set()
        self.accept_calls = 0
        self.reject_next = False
        super().__init__(start_action=False)
        self.server = ActionServer(self,NavigateToPose,'/navigate_to_pose',
            execute_callback=self.execute,goal_callback=self.accept,
            cancel_callback=lambda request: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.resume = self.create_publisher(Empty,'/navigation/resume',1)
        self.markers = None
        self.create_subscription(MarkerArray,'/navigation/waypoints',
            lambda m: setattr(self,'markers',m),self.startup.qos_profile)

    def accept(self,request):
        self.accept_calls += 1
        assert self.accept_wait.wait(10),'Test acceptance latch timed out'
        if self.reject_next:
            self.reject_next = False
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self,handle):
        self.goals += 1
        index = self.goals
        self.positions.append(handle.request.pose.pose.position.x)
        self.active += 1
        self.max_active = max(self.active,self.max_active)
        try:
            deadline = time.monotonic()+20
            while time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    self.cancels += 1
                    handle.canceled()
                    return NavigateToPose.Result()
                if index in self.outcomes:
                    if self.outcomes[index] == 4:
                        handle.succeed()
                    else:
                        handle.abort()
                    return NavigateToPose.Result()
                time.sleep(0.01)
            handle.abort()
            return NavigateToPose.Result()
        finally:
            self.active -= 1


def main():
    rclpy.init(args=['--ros-args','-p','mode:=navigation'])
    operator, endpoints = Operator(),QueueEndpoints()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(operator)
    executor.add_node(endpoints)
    spinner = threading.Thread(target=executor.spin,daemon=True)
    spinner.start()
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.orientation.w = 1.0
    def click(x):
        goal = copy.deepcopy(pose)
        goal.pose.position.x = x
        endpoints.goal.publish(goal)
    try:
        endpoints.valid = True
        wait_for(lambda: operator.localized() and len(operator.grids)==3 and operator.action.server_is_ready()
                 and operator.motion_state.service_is_ready())
        wait_for(lambda: operator.buffer.can_transform('map','base_footprint',rclpy.time.Time()))
        for number,x in enumerate((0.4,0.8,1.2),1):
            click(x)
            wait_for(lambda: len(operator.waypoints)==number)
        time.sleep(0.3)
        assert endpoints.goals == 0,'Disabled route started Nav2'
        wait_for(lambda: endpoints.markers is not None and len(endpoints.markers.markers)==7)
        assert [m.text for m in endpoints.markers.markers if m.type==Marker.TEXT_VIEW_FACING] == ['1','2','3']
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals==1 and operator.goal_handle is not None)
        click(1.6)
        wait_for(lambda: len(operator.waypoints)==3)
        endpoints.outcomes[1] = 4
        wait_for(lambda: endpoints.goals==2 and operator.goal_handle is not None)
        # Re-read the real bridge between waypoints, rather than caching enable.
        endpoints.set_parameters([Parameter('enable_control',value=False)])
        endpoints.outcomes[2] = 4
        wait_for(lambda: operator.completed_waypoints==2 and operator.goal_handle is None)
        time.sleep(0.3)
        assert endpoints.goals==2 and operator.waiting_goal.pose.position.x==1.2
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.goals==3 and operator.goal_handle is not None)
        endpoints.outcomes[3] = 6
        wait_for(lambda: operator.route_paused and operator.goal_handle is None)
        assert [p.pose.position.x for _,p in operator.waypoints]==[1.2,1.6]
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: 'route paused' in endpoints.operator_status)
        time.sleep(0.2)
        assert endpoints.goals==3,'Enable silently retried failed waypoint'
        endpoints.resume.publish(Empty())
        wait_for(lambda: endpoints.goals==4 and operator.goal_handle is not None)
        endpoints.outcomes[4] = 4
        wait_for(lambda: endpoints.goals==5 and operator.goal_handle is not None)
        endpoints.outcomes[5] = 4
        wait_for(lambda: operator.completed_waypoints==4 and operator.goal_handle is None)
        assert endpoints.positions==[0.4,0.8,1.2,1.2,1.6],endpoints.positions
        assert not operator.waypoints and endpoints.max_active==1
        print('PASS ordered queue, append during execution, real enable readback, failure pause and explicit retry')

        click(0.4)
        wait_for(lambda: endpoints.goals==6 and operator.goal_handle is not None)
        click(0.8)
        wait_for(lambda: len(operator.waypoints)==1)
        publisher,grid = endpoints.grids['/global_costmap/costmap']
        col,row = grid_cell(operator.grids['/global_costmap/costmap'][0],(0.8,0.0))
        index = row*grid.info.width+col
        grid.data[index] = 100
        wait_for(lambda: operator.grids['/global_costmap/costmap'][0].data[index]==100)
        endpoints.outcomes[6] = 4
        wait_for(lambda: operator.route_paused and operator.goal_handle is None)
        assert endpoints.goals==6 and operator.waiting_goal.pose.position.x==0.8
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_paused)
        grid.data[index] = 0
        wait_for(lambda: operator.grids['/global_costmap/costmap'][0].data[index]==0)

        # Stop during acceptance must cancel that late handle and clear its tail.
        endpoints.accept_wait.clear()
        click(0.4)
        wait_for(lambda: operator.goal_pending)
        click(0.8)
        wait_for(lambda: len(operator.waypoints)==1)
        endpoints.enable.publish(Bool(data=False))
        wait_for(lambda: operator.cancel_requested and not operator.waypoints)
        endpoints.accept_wait.set()
        wait_for(lambda: endpoints.cancels==1 and operator.goal_handle is None and not operator.goal_pending)
        endpoints.enable.publish(Bool(data=True))
        wait_for(lambda: endpoints.get_parameter('enable_control').value is True)
        time.sleep(0.2)
        assert endpoints.goals==7 and not operator.waypoints
        endpoints.reject_next = True
        click(1.0)
        wait_for(lambda: operator.route_paused and not operator.goal_pending)
        assert endpoints.goals==7 and operator.waiting_goal.pose.position.x==1.0
        endpoints.cancel.publish(Empty())
        wait_for(lambda: not operator.waypoints and not operator.route_paused)
        click(0.4)
        wait_for(lambda: endpoints.goals==8 and operator.goal_handle is not None)
        click(0.8)
        wait_for(lambda: len(operator.waypoints)==1)
        endpoints.valid = False
        wait_for(lambda: endpoints.cancels==2 and operator.goal_handle is None and not operator.waypoints)
        endpoints.valid = True
        wait_for(operator.localized)
        time.sleep(0.2)
        assert endpoints.goals==8
        print('PASS next-goal costmap revalidation, pending-acceptance stop, rejection pause and localization-loss cancellation')
    finally:
        endpoints.accept_wait.set()
        executor.shutdown()
        spinner.join(timeout=3)
        operator.destroy_node()
        endpoints.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
