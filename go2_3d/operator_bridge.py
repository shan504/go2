#!/usr/bin/env python3
"""Foxglove publish-panel operations and /goal_pose -> Nav2 action gateway."""
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from geometry import pose_matrix


class Operator(Node):
    def __init__(self):
        super().__init__('go2_operator')
        self.declare_parameter('mode','mapping')
        self.mode = self.get_parameter('mode').value
        self.valid = False
        self.valid_received = None
        self.goal_handle = None
        self.goal_pending = False
        self.cancel_requested = False
        self.cancelling = False
        self.action = None
        if self.mode == 'navigation':
            from nav2_msgs.action import NavigateToPose
            self.goal_type = NavigateToPose
            self.action = ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.save = self.create_client(Trigger,'/save_3d_map')
        self.parameters = self.create_client(SetParameters,'/go2_edu_dds_bridge/set_parameters')
        self.status = self.create_publisher(String,'/operator/status',10)
        self.create_subscription(Bool,'/localization/valid',self.on_validity,1)
        self.create_subscription(Empty,'/mapping/save',self.on_save,1)
        self.create_subscription(Bool,'/control/enable',self.on_enable,1)
        self.create_subscription(Empty,'/navigation/cancel',self.on_cancel,1)
        self.create_subscription(PoseStamped,'/goal_pose',self.on_goal,1)
        self.create_timer(0.1,self.watchdog)

    def report(self,text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def localized(self):
        return self.valid and self.valid_received is not None and time.monotonic()-self.valid_received < 0.5

    def on_validity(self,message):
        self.valid, self.valid_received = message.data,time.monotonic()

    def watchdog(self):
        if not self.localized() and (self.goal_handle is not None or self.goal_pending):
            self.on_cancel(Empty())

    def on_save(self,message):
        if self.mode != 'mapping':
            self.report('Save rejected: switch to mapping mode')
            return
        if not self.save.service_is_ready():
            self.report('3D map save service unavailable')
            return
        future = self.save.call_async(Trigger.Request())
        def done(result):
            try:
                response = result.result()
                self.report(f'Save success={response.success}: {response.message}')
            except Exception as error:
                self.report(f'Save failed: {error}')
        future.add_done_callback(done)

    def on_enable(self,message):
        if message.data and (self.mode != 'navigation' or not self.localized()):
            self.report('Enable rejected: navigation mode and live GICP localization required')
            return
        if not self.parameters.service_is_ready():
            self.report('Motion bridge parameter service unavailable')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter('enable_control',value=bool(message.data)).to_parameter_msg()]
        future = self.parameters.call_async(request)
        def done(result):
            try:
                results = result.result().results
                self.report('Motion enable=%s; parameter accepted=%s' % (message.data,all(r.successful for r in results)))
            except Exception as error:
                self.report(f'Motion parameter failed: {error}')
        future.add_done_callback(done)
        if not message.data:
            self.on_cancel(Empty())

    def on_cancel(self,message):
        if self.goal_pending:
            self.cancel_requested = True
        if self.goal_handle is not None and not self.cancelling:
            handle = self.goal_handle
            self.cancelling = True
            future = handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.report('Navigation cancel response received'))
            self.report('Navigation cancellation requested')

    def on_goal(self,message):
        if self.action is None or not self.localized():
            self.report('Goal rejected: navigation mode and live GICP localization required')
            return
        if message.header.frame_id != 'map':
            self.report('Goal rejected: frame_id must be map')
            return
        try:
            p,q = message.pose.position,message.pose.orientation
            pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
        except ValueError as error:
            self.report(f'Goal rejected: {error}')
            return
        if self.goal_handle is not None or self.goal_pending:
            self.report('Goal rejected: cancel the current goal before selecting another')
            return
        if not self.action.server_is_ready():
            self.report('Nav2 NavigateToPose action unavailable')
            return
        goal = self.goal_type.Goal()
        goal.pose = message
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pending,self.cancel_requested = True,False
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.goal_response)
        self.report('Map goal sent to Nav2 action')

    def goal_response(self,future):
        self.goal_pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.report('Nav2 rejected goal')
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.goal_result)
            self.report('Nav2 accepted goal')
            if self.cancel_requested or not self.localized():
                self.on_cancel(Empty())
        except Exception as error:
            self.report(f'Nav2 goal request failed: {error}')

    def goal_result(self,future):
        try:
            self.report(f'Nav2 goal completed: action status={future.result().status}')
        except Exception as error:
            self.report(f'Nav2 result failed: {error}')
        self.goal_handle = None
        self.cancelling = False


def main():
    rclpy.init()
    node = Operator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
