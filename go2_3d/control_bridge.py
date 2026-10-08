#!/usr/bin/env python3
"""Reuse DDS adapter, adding a mandatory live-localization gate for motion."""
import sys
import time
sys.path.insert(0, '/opt/go2_project/patches/dds')
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from rcl_interfaces.msg import ParameterDescriptor
from go2_edu_dds_bridge import Go2EduBridge
import signal


class LocalizedBridge(Go2EduBridge):
    def __init__(self):
        self.localization_valid = False
        self.localization_received = None
        self.motion_allowed = False
        super().__init__()
        self.declare_parameter('allow_motion',False,ParameterDescriptor(read_only=True))
        self.motion_allowed = self.get_parameter('allow_motion').value
        self.create_subscription(Bool, '/localization/valid', self.on_validity, 1)

    def on_validity(self, message):
        self.localization_valid = message.data
        self.localization_received = time.monotonic()

    def control_tick(self):
        now = time.monotonic()
        valid = (self.motion_allowed and self.localization_valid and self.localization_received is not None and
                 now-self.localization_received < 0.5)
        if not valid:
            action, _ = self.gate.poll(now, False)
            if action == 'stop':
                self.send_stop()
            return
        super().control_tick()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    node = LocalizedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gate.set_enabled(False) and rclpy.ok():
            node.send_stop()
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
