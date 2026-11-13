#!/usr/bin/env python3
"""Relay the robot's native DDS sensors and optionally Nav2 velocity requests."""
import copy
import json
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from unitree_api.msg import Request

from bridge_logic import CommandGate, SensorClock


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(value, 1_000_000_000)


class Go2EduBridge(Node):
    def __init__(self):
        super().__init__("go2_edu_dds_bridge")
        self.declare_parameter("enable_control", False)
        self.clock_map = SensorClock()
        self.gate = CommandGate()
        self.gate.set_enabled(self.get_parameter("enable_control").value)
        self.odom_received = None
        self.cloud_received = None
        self.first_odom_stamp_ns = None
        self.last_cloud_source_ns = None
        self.sensor_timing = {}
        self._reported_clock_state = None
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/point_cloud2", qos_profile_sensor_data)
        self.request_pub = self.create_publisher(Request, "/api/sport/request", 10)
        self.create_subscription(Odometry, "/utlidar/robot_odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/utlidar/cloud_base", self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(0.05, self.control_tick)
        self.get_logger().info("EDU DDS bridge ready: cloud_base -> point_cloud2; robot_odom -> odom + TF")
        self.get_logger().info("enable_control=%s" % self.gate.enabled)

    def record_sensor_timing(self, source, raw_ns, mapped_ns, now_ns, accepted, reason):
        self.sensor_timing[source] = dict(raw_stamp_ns=raw_ns, host_receipt_ns=now_ns,
            mapped_stamp_ns=mapped_ns,
            mapped_age_s=None if mapped_ns is None else (now_ns-mapped_ns)/1e9,
            accepted=accepted, reason=reason)
        status = self.clock_map.status()
        state = status['state']
        if state != self._reported_clock_state:
            self._reported_clock_state = state
            text = 'Sensor clock state: '+json.dumps(status, sort_keys=True)
            if state == 'fault':
                self.get_logger().error(text+'; sensor relay stopped; restart after host clock is stable')
            else:
                self.get_logger().info(text)
        if reason in ('sensor_time_in_future', 'ros_clock_jump', 'monotonic_clock_regressed'):
            self.get_logger().error('Sensor clock rejected sample: '+json.dumps(
                dict(source=source, clock=status, sample=self.sensor_timing[source]), sort_keys=True),
                throttle_duration_sec=2.0)

    def on_parameters(self, params):
        for param in params:
            if param.name == "enable_control":
                if type(param.value) is not bool:
                    return SetParametersResult(successful=False, reason="enable_control must be bool")
                if self.gate.set_enabled(param.value):
                    self.send_stop()
                self.get_logger().info("enable_control=%s; waiting for a new cmd_vel" % param.value)
        return SetParametersResult(successful=True)

    def on_odom(self, msg):
        if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
            self.get_logger().error("Ignoring unexpected odometry frames; expected odom -> base_link",
                                    throttle_duration_sec=5.0)
            return
        sensor_ns = stamp_ns(msg.header.stamp)
        clock_before_ns = time.monotonic_ns()
        now_ns = self.get_clock().now().nanoseconds
        clock_after_ns = time.monotonic_ns()
        accepted = self.clock_map.observe_odom_sample(
            sensor_ns, now_ns, clock_before_ns, clock_after_ns)
        shifted_ns = self.clock_map.convert(sensor_ns)
        self.record_sensor_timing('odom', sensor_ns, shifted_ns, now_ns, accepted,
                                  self.clock_map.status()['last_rejection_reason'])
        if not accepted:
            if self.clock_map.status()['state'] != 'ready':
                self.odom_received = self.cloud_received = None
            return
        out = copy.deepcopy(msg)
        set_stamp(out.header.stamp, shifted_ns)
        self.odom_pub.publish(out)
        tf = TransformStamped()
        tf.header = copy.deepcopy(out.header)
        tf.child_frame_id = out.child_frame_id
        tf.transform.translation.x = out.pose.pose.position.x
        tf.transform.translation.y = out.pose.pose.position.y
        tf.transform.translation.z = out.pose.pose.position.z
        tf.transform.rotation = copy.deepcopy(out.pose.pose.orientation)
        self.tf.sendTransform(tf)
        if self.first_odom_stamp_ns is None:
            self.first_odom_stamp_ns = shifted_ns
        self.odom_received = time.monotonic()

    def on_cloud(self, msg):
        if msg.header.frame_id != "base_link":
            self.get_logger().error("Ignoring unexpected cloud frame; expected base_link",
                                    throttle_duration_sec=5.0)
            return
        raw_ns = stamp_ns(msg.header.stamp)
        shifted_ns = self.clock_map.convert(raw_ns)
        now_ns = self.get_clock().now().nanoseconds
        reason = ('clock_not_ready' if shifted_ns is None else
                  'cloud_duplicate_or_reordered' if self.last_cloud_source_ns is not None and
                      raw_ns <= self.last_cloud_source_ns else
                  'sensor_time_in_future' if shifted_ns-now_ns > 100_000_000 else
                  'sensor_time_stale' if now_ns-shifted_ns > 1_000_000_000 else
                  'before_odom_history' if self.first_odom_stamp_ns is None or
                      shifted_ns < self.first_odom_stamp_ns else None)
        self.record_sensor_timing('cloud', raw_ns, shifted_ns, now_ns, reason is None, reason)
        if reason is not None:
            return
        # Coordinates are already in base_link: preserve them and their relative timestamps.
        msg.header = copy.deepcopy(msg.header)
        set_stamp(msg.header.stamp, shifted_ns)
        self.cloud_pub.publish(msg)
        self.last_cloud_source_ns = raw_ns
        self.cloud_received = time.monotonic()

    def on_cmd(self, msg):
        self.gate.receive(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def control_tick(self):
        now = time.monotonic()
        fresh = all(t is not None and 0 <= now - t < 1.0
                    for t in (self.odom_received, self.cloud_received))
        action, command = self.gate.poll(now, fresh)
        if action == "move":
            self.send_move(command)
        elif action == "stop":
            self.send_stop()

    def send_move(self, command):
        req = Request()
        req.header.identity.id = time.time_ns()
        req.header.identity.api_id = 1008
        req.parameter = json.dumps(dict(zip(("x", "y", "z"), command)))
        self.request_pub.publish(req)
        return req.header.identity.id

    def send_stop(self):
        req = Request()
        req.header.identity.id = time.time_ns()
        req.header.identity.api_id = 1003
        self.request_pub.publish(req)
        return req.header.identity.id


def main():
    # Keep the DDS context alive long enough to send StopMove on shutdown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    node = Go2EduBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.gate.set_enabled(False) and rclpy.ok():
            node.send_stop()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
