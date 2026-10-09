#!/usr/bin/env python3
"""Start Nav2 lifecycle nodes only after live GICP and timestamped TF are ready."""
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import Bool,String
from nav2_msgs.srv import ManageLifecycleNodes
from tf2_ros import Buffer,TransformListener,TransformException


class Nav2Ready(Node):
    def __init__(self):
        super().__init__('go2_nav2_ready')
        self.valid,self.received,self.ready_since = False,None,None
        self.attempted,self.started = False,False
        self.wait_reason,self.wait_reported = None,0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.client = self.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
        self.status = self.create_publisher(String,'/navigation/startup_status',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool,'/localization/valid',self.on_valid,1)
        self.create_timer(0.1,self.tick)
        self.report('Waiting for accepted GICP and map TF; Nav2 lifecycle startup deferred')

    def report(self,text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def on_valid(self,msg):
        self.valid,self.received = msg.data,time.monotonic()

    def waiting(self,reason,text):
        now = time.monotonic()
        if reason != self.wait_reason or now-self.wait_reported >= 5.0:
            self.wait_reason,self.wait_reported = reason,now
            self.report(text)

    def tick(self):
        if self.attempted:
            return
        now = time.monotonic()
        ready = self.valid and self.received is not None and now-self.received<0.5
        reason,text = 'localization','Nav2 waiting: live accepted GICP localization required'
        if ready:
            try:
                pose = self.buffer.lookup_transform('map','base_footprint',Time())
                stamp = Time.from_msg(pose.header.stamp)
                age = (self.get_clock().now()-stamp).nanoseconds/1e9
                ready = 0 <= age<0.3
                reason,text = 'tf_age',f'Nav2 waiting: map -> base_footprint TF age={age:.3f}s; requires 0..0.3s'
            except TransformException as error:
                ready = False
                reason,text = 'tf_missing',f'Nav2 waiting: map -> base_footprint TF unavailable: {error}'
        if not ready:
            self.ready_since = None
            self.waiting(reason,text)
            return
        if self.ready_since is None:
            self.ready_since = now
        if now-self.ready_since<0.5:
            self.waiting('settling','Nav2 waiting: GICP/TF must remain ready for 0.5s')
            return
        if not self.client.service_is_ready():
            self.waiting('manager','Nav2 waiting: /lifecycle_manager_navigation/manage_nodes service unavailable')
            return
        self.attempted = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.report('GICP/TF ready; starting Nav2 lifecycle nodes')
        self.client.call_async(request).add_done_callback(self.completed)

    def completed(self,future):
        try:
            self.started = future.result().success
            self.report('Nav2 active; motion remains disabled' if self.started else 'Nav2 lifecycle startup failed; inspect node errors')
        except Exception as error:
            self.report(f'Nav2 lifecycle startup failed: {error}')


if __name__ == '__main__':
    rclpy.init()
    node = Nav2Ready()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
