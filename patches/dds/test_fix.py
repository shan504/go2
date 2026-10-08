"""Local tests: no robot, network or ROS installation needed."""
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml
from bridge_logic import CommandGate, SensorClock
from prepare import prepare


class ClockTest(unittest.TestCase):
    def test_shared_offset_preserves_sensor_timing(self):
        clock = SensorClock()
        self.assertIsNone(clock.convert(1))
        self.assertTrue(clock.observe_odom(100_000_000_000, 1_000_000_000_000))
        self.assertEqual(clock.convert(100_100_000_000) - clock.convert(100_000_000_000), 100_000_000)
        # Transport delay on a later odometry sample must not change the offset.
        self.assertTrue(clock.observe_odom(101_000_000_000, 1_001_100_000_000))
        self.assertEqual(clock.convert(101_000_000_000), 1_001_000_000_000)
        self.assertFalse(clock.observe_odom(99_000_000_000, 1_002_000_000_000))
        self.assertEqual(clock.offset_ns, 900_000_000_000)


class GateTest(unittest.TestCase):
    def test_disabled_and_enable_require_fresh_command(self):
        gate = CommandGate()
        gate.receive(1.0, 0.0, 1.0, 1.0)
        self.assertEqual(gate.poll(1.1, True), (None, None))
        gate.set_enabled(True)
        self.assertEqual(gate.poll(1.2, True), (None, None))
        gate.receive(1.0, 1.0, 2.0, 1.3)
        action, command = gate.poll(1.4, True)
        self.assertEqual(action, "move")
        self.assertAlmostEqual(math.hypot(*command[:2]), 0.15)
        self.assertEqual(command[2], 0.3)
        self.assertTrue(gate.set_enabled(False))
        self.assertEqual(gate.poll(1.5, True), (None, None))
        gate.set_enabled(True)
        self.assertEqual(gate.poll(1.6, True), (None, None))

    def test_watchdog_sensor_loss_and_invalid_command(self):
        for failure in ("timeout", "sensor", "nan"):
            gate = CommandGate()
            gate.set_enabled(True)
            gate.receive(0.1, 0.0, 0.0, 10.0)
            self.assertEqual(gate.poll(10.1, True)[0], "move")
            if failure == "nan":
                gate.receive(float("nan"), 0.0, 0.0, 10.2)
            action, command = gate.poll(10.6 if failure == "timeout" else 10.2, failure != "sensor")
            self.assertEqual((action, command), ("stop", None))
            self.assertEqual(gate.poll(10.7, True), (None, None))


class ConfigTest(unittest.TestCase):
    def test_prepare_on_sdk_sources(self):
        # Source directory supplied by the local validation command.
        import os
        sdk = Path(os.environ["GO2_TEST_SDK_SOURCE"])
        before = {p: p.read_bytes() for p in sdk.rglob("*") if p.is_file()}
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            prepare(sdk, output)
            urdf = ET.parse(output / "go2_edu.urdf").getroot()
            child_links = {j.find("child").get("link") for j in urdf.findall("joint")}
            all_links = {j.get("name") for j in urdf.findall("link")}
            self.assertNotIn("base_link", child_links)
            self.assertNotIn("odom", all_links)
            self.assertNotIn("map", all_links)
            self.assertIn("base_link", all_links)
            nav = yaml.safe_load((output / "nav2_edu.yaml").read_text())
            def check_clock(obj):
                if isinstance(obj, dict):
                    for k, v in obj.items():
                        if k == "use_sim_time":
                            self.assertIs(v, False)
                        else:
                            check_clock(v)
                elif isinstance(obj, list):
                    for v in obj:
                        check_clock(v)
            check_clock(nav)
            costmap = nav["global_costmap"]["global_costmap"]["ros__parameters"]
            self.assertLessEqual(costmap["width"] * costmap["height"], 3600)
            self.assertTrue(costmap["track_unknown_space"])
            follow = nav["controller_server"]["ros__parameters"]["FollowPath"]
            smoother = nav["velocity_smoother"]["ros__parameters"]
            self.assertEqual(follow["max_vel_x"], smoother["max_velocity"][0])
            self.assertEqual(follow["max_vel_theta"], smoother["max_velocity"][2])
            self.assertLess(follow["trans_stopped_velocity"], follow["max_vel_x"])
        for p, data in before.items():
            self.assertEqual(p.read_bytes(), data)


if __name__ == "__main__":
    unittest.main()
