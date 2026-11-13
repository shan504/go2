"""Native sensor clock calibration without ROS or any actuator interface."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'patches/dds'))
from bridge_logic import SensorClock


SENSOR = 100_000_000_000
HOST = 1_000_000_000_000
MONOTONIC = 10_000_000_000


def observe(clock, sensor_elapsed, receipt_elapsed, ros_adjustment=0):
    return clock.observe_odom(SENSOR+sensor_elapsed, HOST+receipt_elapsed+ros_adjustment,
                              MONOTONIC+receipt_elapsed)


def ready_clock():
    clock = SensorClock()
    for step in range(6):
        ready = observe(clock, step*100_000_000, step*100_000_000)
        assert ready == (step == 5)
    return clock


class SensorClockTest(unittest.TestCase):
    def test_delayed_first_packet_does_not_permanently_future_date_both_streams(self):
        clock = SensorClock()
        first_delay = 170_000_000
        first_offset = HOST+first_delay-SENSOR
        self.assertFalse(observe(clock, 0, first_delay))
        self.assertIsNone(clock.offset_ns)
        self.assertIsNone(clock.convert(SENSOR))
        for step in range(1, 6):
            elapsed = first_delay+step*100_000_000
            self.assertEqual(observe(clock, elapsed, elapsed), step == 5)
            if step < 5:
                self.assertIsNone(clock.convert(SENSOR+elapsed))
        sensor = SENSOR+elapsed
        receipt = HOST+elapsed
        # Reproduce the old first-packet mapping, then check the replacement.
        self.assertEqual(receipt-(sensor+first_offset), -170_000_000)
        self.assertEqual(receipt-(sensor-60_000_000+first_offset), -110_000_000)
        self.assertEqual(clock.offset_ns, HOST-SENSOR)
        self.assertEqual(clock.convert(sensor), receipt)
        self.assertEqual(clock.convert(sensor)-clock.convert(sensor-60_000_000), 60_000_000)
        self.assertEqual(receipt-clock.convert(sensor-60_000_000), 60_000_000)
        self.assertEqual(clock.status()['state'], 'ready')

    def test_calibration_needs_both_elapsed_time_and_advancing_samples(self):
        clock = SensorClock()
        for elapsed in (0, 200_000_000, 400_000_000, 600_000_000):
            self.assertFalse(observe(clock, elapsed, elapsed))
            self.assertIsNone(clock.convert(SENSOR+elapsed))
        self.assertTrue(observe(clock, 800_000_000, 800_000_000))
        self.assertEqual(clock.status()['calibration_samples'], 5)
        clock = SensorClock()
        for step in range(30):
            self.assertFalse(observe(clock, step*10_000_000, step*10_000_000))
        self.assertEqual(clock.status()['state'], 'calibrating')
        self.assertTrue(observe(clock, 500_000_000, 500_000_000))

    def test_increased_transport_delay_never_reanchors_or_refreshes_old_sample(self):
        clock = ready_clock()
        offset = clock.offset_ns
        self.assertTrue(observe(clock, 600_000_000, 800_000_000))
        self.assertEqual(clock.convert(SENSOR+600_000_000), HOST+600_000_000)
        self.assertFalse(observe(clock, 700_000_000, 1_900_000_000))
        self.assertEqual(clock.status()['last_rejection_reason'], 'stale_odom')
        self.assertEqual(clock.offset_ns, offset)
        self.assertTrue(observe(clock, 2_000_000_000, 2_000_000_000))
        self.assertEqual(clock.offset_ns, offset)

    def test_reordered_and_duplicate_samples_do_not_calibrate_or_refresh(self):
        clock = SensorClock()
        self.assertFalse(observe(clock, 0, 0))
        for elapsed in (100_000_000, 700_000_000, 1_000_000_000):
            self.assertFalse(observe(clock, 0, elapsed))
        self.assertFalse(observe(clock, -1, 1_100_000_000))
        self.assertEqual(clock.status()['calibration_samples'], 1)
        self.assertIsNone(clock.offset_ns)
        clock = ready_clock()
        offset = clock.offset_ns
        self.assertFalse(observe(clock, 500_000_000, 600_000_000))
        self.assertEqual(clock.last_rejection_reason, 'duplicate_odom')
        self.assertFalse(observe(clock, 400_000_000, 700_000_000))
        self.assertEqual(clock.last_rejection_reason, 'reordered_odom')
        self.assertEqual(clock.offset_ns, offset)
        self.assertTrue(observe(clock, 800_000_000, 800_000_000))

    def test_future_sample_drops_without_changing_offset_and_can_recover(self):
        clock = ready_clock()
        offset = clock.offset_ns
        self.assertFalse(observe(clock, 710_000_000, 600_000_000))
        self.assertEqual(clock.last_rejection_reason, 'sensor_time_in_future')
        self.assertAlmostEqual(clock.status()['sample_age_s'], -.11)
        self.assertEqual(clock.status()['state'], 'ready')
        self.assertIsNone(clock.fault_reason)
        self.assertEqual(clock.offset_ns, offset)
        self.assertTrue(observe(clock, 720_000_000, 720_000_000))
        self.assertIsNone(clock.last_rejection_reason)
        self.assertEqual(clock.offset_ns, offset)
        # Preserve the existing inclusive 100ms future envelope.
        self.assertTrue(observe(clock, 900_000_000, 800_000_000))

    def test_rejected_future_timestamp_cannot_poison_next_good_odometry(self):
        clock = ready_clock()
        accepted = clock.last_odom_ns
        offset = clock.offset_ns
        self.assertFalse(observe(clock, 850_000_000, 600_000_000))
        self.assertEqual(clock.last_rejection_reason, 'sensor_time_in_future')
        self.assertEqual(clock.last_odom_ns, accepted)
        # This good packet is newer than the last published one but older
        # than the rejected outlier. It must not be mistaken for a replay.
        self.assertTrue(observe(clock, 680_000_000, 680_000_000))
        self.assertEqual(clock.convert(SENSOR+680_000_000), HOST+680_000_000)
        self.assertEqual(clock.offset_ns, offset)

    def test_gap_does_not_hide_stale_replay_or_rebase_live_time(self):
        clock = ready_clock()
        offset = clock.offset_ns
        self.assertFalse(observe(clock, 600_000_000, 5_600_000_000))
        self.assertEqual(clock.last_rejection_reason, 'stale_odom')
        self.assertEqual(clock.offset_ns, offset)
        self.assertTrue(observe(clock, 5_700_000_000, 5_700_000_000))
        self.assertEqual(clock.convert(SENSOR+5_700_000_000), HOST+5_700_000_000)
        # A reset source clock is a replay in the existing epoch, not a
        # reason to turn old sensor data into current observations.
        self.assertFalse(observe(clock, -99_000_000_000, 5_800_000_000))
        self.assertEqual(clock.last_rejection_reason, 'reordered_odom')
        self.assertEqual(clock.offset_ns, offset)

    def test_host_clock_steps_are_sticky_and_never_reanchor_tf_history(self):
        for step in (-200_000_000, 200_000_000):
            with self.subTest(step=step):
                clock = ready_clock()
                offset = clock.offset_ns
                self.assertFalse(observe(clock, 600_000_000, 600_000_000, step))
                self.assertEqual(clock.fault_reason, 'ros_clock_jump')
                self.assertEqual(clock.status()['fault_detail']['difference_ns'], step)
                self.assertEqual(clock.status()['state'], 'fault')
                self.assertIsNone(clock.convert(SENSOR+600_000_000))
                self.assertFalse(observe(clock, 700_000_000, 700_000_000))
                self.assertEqual(clock.offset_ns, offset)

    def test_delayed_host_clock_read_is_not_a_permanent_clock_fault(self):
        clock = ready_clock()
        offset, accepted, pair = clock.offset_ns, clock.last_odom_ns, clock._last_clock_pair
        for before, after in ((600_000_000, 800_000_000), (800_000_000, 600_000_000)):
            self.assertFalse(clock.observe_odom_sample(
                SENSOR+600_000_000, HOST+600_000_000, MONOTONIC+before, MONOTONIC+after))
            self.assertEqual(clock.last_rejection_reason, 'host_clock_sample_delayed')
            self.assertEqual(clock.status()['state'], 'ready')
            self.assertIsNone(clock.fault_reason)
            self.assertEqual((clock.offset_ns, clock.last_odom_ns, clock._last_clock_pair),
                             (offset, accepted, pair))
        self.assertTrue(clock.observe_odom_sample(SENSOR+810_000_000, HOST+810_000_000,
                                                  MONOTONIC+809_000_000, MONOTONIC+811_000_000))
        self.assertEqual(clock.offset_ns, offset)
        self.assertIsNone(clock.last_rejection_reason)

    def test_precise_clock_pair_still_detects_real_wall_clock_step(self):
        for step in (-200_000_000, 200_000_000):
            with self.subTest(step=step):
                clock = ready_clock()
                self.assertFalse(clock.observe_odom_sample(
                    SENSOR+600_000_000, HOST+600_000_000+step,
                    MONOTONIC+599_000_000, MONOTONIC+601_000_000))
                self.assertEqual(clock.fault_reason, 'ros_clock_jump')
                self.assertIsNone(clock.convert(SENSOR+600_000_000))

    def test_invalid_inputs_and_monotonic_clock_regression(self):
        clock = SensorClock()
        for sensor, host, monotonic in ((0, HOST, MONOTONIC),
                                       (SENSOR, 0, MONOTONIC),
                                       (SENSOR, HOST, -1),
                                       (float('nan'), HOST, MONOTONIC)):
            self.assertFalse(clock.observe_odom(sensor, host, monotonic))
            self.assertEqual(clock.last_rejection_reason, 'invalid_timestamp')
        clock = ready_clock()
        self.assertFalse(clock.observe_odom(SENSOR+600_000_000, HOST+600_000_000, MONOTONIC))
        self.assertEqual(clock.fault_reason, 'monotonic_clock_regressed')
        self.assertIsNone(clock.convert(SENSOR+600_000_000))


if __name__ == '__main__':
    unittest.main()
