import base64
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'go2_3d'))
from fault_recorder import (AsyncEvidenceWriter, FlightRecorder, classify_log, json_safe,
                            MAX_EVENT_SIZE)
from navigation_diagnostics import ControlHistory, no_motion_observation


class MemoryWriter:
    def __init__(self):
        self.calls = []
    def submit(self, operation, token=None, **payload):
        self.calls.append((operation, token, deepcopy(payload)))
        return True
    def status(self):
        return dict(enabled=True)
    def close(self):
        return True


class FlightRecorderTests(unittest.TestCase):
    def test_pre_snapshot_is_immutable_and_abort_updates_same_recovery_episode(self):
        writer = MemoryWriter()
        recorder = FlightRecorder(writer)
        snapshot = dict(route={'current': '5'}, commands={'cmd_vel': [0.3, 0, 0]})
        recorder.observe('cmd_vel', dict(velocity=[0.3, 0, 0]), 0)
        recorder.trigger('planner_failed', 'failed to create plan', 'planner_server', 1,
                         snapshot, dict(local_grid={'raw_int8': b'\x00\xff\x64'}))
        snapshot['commands']['cmd_vel'][0] = 99
        recorder.observe('rosout', dict(text='replanning same goal'), 3)
        recorder.trigger('planner_failed', 'retry failed', 'planner_server', 3, snapshot)
        recorder.tick(9, dict(route={'current': '5'}))
        recorder.observe('action_status', dict(status=6), 12)
        recorder.trigger('navigation_aborted', 'ABORTED', 'operator', 12,
                         dict(route={'paused': 'True'}), dict(last_action={'status': 6}))
        begins = [call for call in writer.calls if call[0] == 'begin']
        self.assertEqual(len(begins), 1)
        self.assertEqual(begins[0][2]['snapshot']['commands']['cmd_vel'][0], 0.3)
        updates = [call for call in writer.calls if call[0] == 'update']
        self.assertTrue(all(call[1] == begins[0][1] for call in updates))
        latest = updates[-1][2]
        self.assertEqual(latest['evidence']['trigger_timeline'][0]['code'], 'planner_failed')
        self.assertEqual(latest['evidence']['trigger_timeline'][-1]['code'], 'navigation_aborted')
        self.assertEqual(latest['evidence']['final_action']['status'], 6)
        self.assertIn('later_trigger_context', latest['evidence'])
        self.assertNotIn('initial_snapshot', latest['evidence'])
        self.assertNotIn('events_before', latest['evidence'])
        recorder.reset_episode(13, 'manual_resume')
        recorder.trigger('progress_timeout', 'new explicit attempt', 'controller', 14, {})
        self.assertEqual(len([call for call in writer.calls if call[0] == 'begin']), 2)

    def test_manual_stop_without_fault_is_session_event_not_a_new_fault(self):
        writer = MemoryWriter()
        recorder = FlightRecorder(writer)
        recorder.trigger('manual_stop', 'user stop', 'manual', 1, {}, severity='info', manual=True)
        self.assertEqual([call[0] for call in writer.calls], ['summary'])
        recorder.trigger('progress_timeout', 'Failed to make progress', 'controller', 2, {})
        recorder.trigger('manual_cancel', 'user cancel', 'manual', 3, {}, severity='info', manual=True)
        begins = [call for call in writer.calls if call[0] == 'begin']
        self.assertEqual(len(begins), 1)
        self.assertEqual(begins[0][2]['trigger']['code'], 'progress_timeout')
        self.assertEqual(writer.calls[-1][2]['evidence']['trigger_timeline'][-1]['code'], 'manual_cancel')

    def test_event_count_byte_and_time_limits_are_visible(self):
        recorder = FlightRecorder(max_events=3, max_event_bytes=1200)
        for now in range(10):
            recorder.observe('cmd_vel', dict(text='x'*100), now)
        self.assertLessEqual(len(recorder.events), 3)
        self.assertLessEqual(recorder.event_bytes, 1200)
        self.assertGreater(recorder.coverage_snapshot(9)['dropped_events'], 0)
        recorder.observe('rosout', dict(text='z'*(MAX_EVENT_SIZE*2)), 10)
        self.assertEqual(recorder.oversize_events, 1)
        recorder.tick(40)
        self.assertEqual(len(recorder.events), 0)
        coverage = recorder.coverage_snapshot(40)
        self.assertFalse(coverage['sources']['tf']['available'])
        self.assertGreater(coverage['sources']['cmd_vel']['receipt_age_s'], 20)

    def test_error_burst_keeps_first_trigger_and_final_abort_in_bounded_timeline(self):
        writer = MemoryWriter()
        recorder = FlightRecorder(writer)
        recorder.trigger('planner_failed', 'first', 'planner', 0, {})
        for index in range(100):
            recorder.trigger('tf_error', 'repeat', 'controller', index/100, {})
        recorder.trigger('navigation_aborted', 'final', 'action', 2, {})
        evidence = writer.calls[-1][2]['evidence']
        self.assertEqual(len(evidence['trigger_timeline']), 64)
        self.assertEqual(evidence['trigger_timeline'][0]['code'], 'planner_failed')
        self.assertEqual(evidence['trigger_timeline'][-1]['code'], 'navigation_aborted')
        self.assertTrue(evidence['timeline_truncated'])

    def test_shutdown_marks_short_post_window_interrupted(self):
        writer = MemoryWriter()
        recorder = FlightRecorder(writer)
        recorder.trigger('progress_timeout', 'stalled', 'controller', 1, {})
        recorder.close(2, dict(reason='shutdown'))
        latest = writer.calls[-1][2]['evidence']['post_capture']
        self.assertEqual(latest['state'], 'interrupted')
        self.assertEqual(latest['close_reason'], 'recorder_shutdown')
        self.assertEqual(latest['captured_seconds'], 1)

    def test_log_classification_keeps_benign_mapper_wait_context_only(self):
        self.assertIsNone(classify_log('go2_gicp', 30,
            'Waiting for odom -> base_link; retrying fresh packet: extrapolation into the future'))
        self.assertEqual(classify_log('planner_server', 30, 'failed to create plan'), 'planner_failed')
        self.assertEqual(classify_log('controller_server', 40, 'Failed to make progress'), 'progress_timeout')
        self.assertEqual(classify_log('DWBLocalPlanner', 40, 'No valid trajectories'), 'controller_no_valid_trajectory')
        self.assertEqual(classify_log('transformPoseInTargetFrame', 40, 'Extrapolation error'), 'tf_error')
        self.assertIsNone(classify_log('unrelated_server', 40, 'Failed'))
        self.assertIsNone(classify_log('go2_edu_dds_bridge', 20,
                                      'Sensor clock state: {"state":"calibrating"}'))
        self.assertEqual(classify_log('go2_edu_dds_bridge', 40,
            'Sensor clock rejected sample: sensor_time_in_future'), 'sensor_clock_error')
        self.assertEqual(classify_log('go2_edu_dds_bridge', 40,
            'Sensor clock state: {"state":"fault"}'), 'sensor_clock_error')

    def test_binary_grid_has_explicit_roundtrip_encoding_and_nonfinite_is_marked(self):
        result = json_safe(dict(raw=b'\x00\xff\x64', value=float('nan')))
        self.assertEqual(zlib.decompress(base64.b64decode(result['raw']['data'])), b'\x00\xff\x64')
        self.assertEqual(result['raw']['uncompressed_bytes'], 3)
        self.assertIsNone(result['value'])
        json.dumps(result, allow_nan=False)


class AsyncWriterTests(unittest.TestCase):
    def test_disk_writes_happen_on_worker_and_saved_input_is_frozen(self):
        calls = []
        owner = threading.get_ident()
        class Store:
            def record_fault(self, trigger, snapshot, evidence):
                calls.append(('begin', threading.get_ident(), snapshot, evidence))
                return 'session--fault'
            def update_fault(self, identity, snapshot, evidence):
                calls.append(('update', threading.get_ident(), identity, snapshot, evidence))
            def append_summary(self, snapshot):
                calls.append(('summary', threading.get_ident(), snapshot))
        writer = AsyncEvidenceWriter('unused', store_factory=lambda path: Store())
        data = {'velocity': [0.3]}
        writer.submit('begin', 'token', trigger={}, snapshot=data, evidence={'raw': b'abc'})
        data['velocity'][0] = 99
        writer.submit('update', 'token', snapshot={}, evidence={'post_capture': {'state': 'complete'}})
        writer.submit('summary', snapshot={'reason': 'periodic'})
        self.assertTrue(writer.close())
        self.assertEqual([call[0] for call in calls], ['begin', 'update', 'summary'])
        self.assertTrue(all(call[1] != owner for call in calls))
        self.assertEqual(calls[0][2]['velocity'], [0.3])
        self.assertEqual(calls[1][2], 'session--fault')
        self.assertEqual(writer.status()['saved_faults'], 1)

    def test_queue_overload_and_permission_failure_are_visible_without_raising(self):
        opened, release = threading.Event(), threading.Event()
        def blocked_factory(path):
            opened.set()
            release.wait(2)
            raise PermissionError('diagnostic directory is read-only')
        writer = AsyncEvidenceWriter('unused', store_factory=blocked_factory, queue_size=1)
        writer.submit('summary', snapshot={})
        self.assertTrue(opened.wait(1))
        self.assertTrue(writer.submit('summary', snapshot={}))
        self.assertFalse(writer.submit('summary', snapshot={}))
        release.set()
        self.assertFalse(writer.close())
        self.assertEqual(writer.status()['dropped_jobs'], 1)
        self.assertGreater(writer.status()['failed_writes'], 0)
        self.assertIsNotNone(writer.status()['last_error'])

    def test_failed_initial_record_can_recover_on_later_linked_update(self):
        calls = []
        class Store:
            def record_fault(self, trigger, snapshot, evidence):
                calls.append((trigger, snapshot))
                if len(calls) == 1:
                    raise OSError('temporary storage reserve shortage')
                return 'same-session--recovered'
            def update_fault(self, identity, snapshot, evidence):
                calls.append(identity)
        with patch('fault_recorder.time.monotonic', return_value=1000.0):
            writer = AsyncEvidenceWriter('unused', store_factory=lambda path: Store())
            writer.submit('begin', 'token', trigger={'code': 'progress_timeout'},
                          snapshot={'original': True}, evidence={'events_before': [1]})
            writer.jobs.join()
            self.assertEqual(writer.status()['unsaved_incidents'], 1)
        with patch('fault_recorder.time.monotonic', return_value=1003.0):
            writer.submit('update', 'token', snapshot={'final': True}, evidence={'events_after': [2]})
            writer.jobs.join()
            self.assertTrue(writer.close())
        self.assertEqual(calls[0], calls[1])
        self.assertEqual(calls[0][0]['recorder_token'], 'token')
        self.assertEqual(calls[2], 'same-session--recovered')
        self.assertEqual(writer.status()['saved_faults'], 1)
        self.assertEqual(writer.status()['unsaved_incidents'], 0)

    def test_completed_incident_id_cache_is_bounded(self):
        class Store:
            def record_fault(self, trigger, snapshot, evidence):
                return trigger['recorder_token']
        writer = AsyncEvidenceWriter('unused', store_factory=lambda path: Store())
        for index in range(270):
            writer.submit('begin', str(index), trigger={}, snapshot={}, evidence={})
            writer.jobs.join()
        self.assertTrue(writer.close())
        self.assertEqual(len(writer.ids), 256)
        self.assertEqual(writer.status()['saved_faults'], 270)


class NoMotionObservationTests(unittest.TestCase):
    def prepared(self):
        history = ControlHistory()
        history.status('operator', 'route current=1 queued=0 paused=False started=True', 0)
        for i in range(101):
            now = i/10
            history.command('cmd_vel', (0.3, 0, 0), now)
            history.odom('odom', 0, 0, 0, (0, 0, 0), now, stamp=100+now)
        history.status('localization_valid', 'True', 10)
        history.status('control', 'gate=forwarding Move requests; enable_control=True', 10)
        return history

    def test_only_fresh_forwarding_data_can_mark_no_motion_observed(self):
        history = self.prepared()
        self.assertIsNotNone(no_motion_observation(history, 10, 110, 0))
        history.status('operator', 'route current=- queued=1 paused=True started=False', 10)
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))
        history = self.prepared()
        history.status('control', 'gate=motion disabled; enable_control=False', 10)
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))

    def test_stale_commands_and_stale_or_future_odom_are_not_false_stalls(self):
        history = self.prepared()
        history.latest_commands['cmd_vel'] = (2.0, (0.3, 0, 0))
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))
        history = self.prepared()
        history.latest_odom['received'] = 8.0
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))
        history = self.prepared()
        history.latest_odom['stamp'] = 100
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))
        history.latest_odom['stamp'] = 111
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))
        history = self.prepared()
        history.latest_commands['cmd_vel'] = (10.0, (0, 0, 0))
        self.assertIsNone(no_motion_observation(history, 10, 110, 0))


if __name__ == '__main__':
    unittest.main()
