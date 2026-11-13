"""Bounded diagnostic event windows and asynchronous persistence, without ROS.

Only evidence is recorded. This module never enables motion or changes a route.
"""
import base64
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import queue
import threading
import time
import uuid
import zlib

PRE_SECONDS = 20.0
POST_SECONDS = 5.0
MAX_POST_SECONDS = 20.0
MAX_EVENTS = 4096
MAX_EVENT_BYTES = 2*1024*1024
MAX_EVENT_SIZE = 4096
MAX_TRIGGERS = 64
EXPECTED_SOURCES = ('cmd_vel_nav', 'cmd_vel', 'odom', 'plan', 'local_costmap',
    'localization_valid', 'localization_state', 'control', 'operator', 'startup',
    'rosout', 'action_status', 'tf', 'waypoints')


def utc_text(wall_time=None):
    return datetime.fromtimestamp(time.time() if wall_time is None else wall_time,
                                  timezone.utc).isoformat(timespec='milliseconds')


def json_safe(value, depth=0):
    if depth > 16:
        return '<depth limit>'
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bytes, bytearray)):
        return dict(encoding='zlib+base64', uncompressed_bytes=len(value),
                    data=base64.b64encode(zlib.compress(bytes(value))).decode('ascii'))
    if isinstance(value, dict):
        return {str(key): json_safe(item, depth+1) for key, item in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [json_safe(item, depth+1) for item in value]
    if value is None or isinstance(value, (str, bool, int)):
        return value
    return str(value)


def classify_log(name, level, message):
    """Return an observed failure category, never a claim of physical cause."""
    relevant = ('controller_server', 'planner_server', 'bt_navigator', 'go2_gicp',
                'go2_operator', 'go2_edu_dds_bridge', 'velocity_smoother',
                'behavior_server', 'go2_nav2_ready', 'transformPoseInTargetFrame',
                'DWB', 'Critic', 'costmap', 'lifecycle_manager', 'PathAlignmentController')
    if not any(part in name for part in relevant):
        return None
    lower = message.lower()
    if 'failed to make progress' in lower:
        return 'progress_timeout'
    if ('failed to create plan' in lower or 'failed to generate a valid path' in lower or
            'failed to plan' in lower):
        return 'planner_failed'
    if 'no valid trajectories' in lower or 'no legal trajectories' in lower:
        return 'controller_no_valid_trajectory'
    if 'path alignment stopped' in lower or 'immediate swept footprint' in lower:
        return 'controller_alignment_error'
    if ('extrapolation' in lower or 'transform' in lower) and level >= 40:
        return 'tf_error'
    if level >= 40 and any(word in lower for word in
            ('failed to load', 'could not load', 'cannot load', 'failed to configure',
             'failed to activate', 'transition failed')):
        return 'startup_failed'
    if level >= 40:
        return 'component_error'
    return None


class _StorageBackoff(Exception):
    pass


class AsyncEvidenceWriter:
    """All file access runs on one daemon worker, never on a ROS callback."""
    def __init__(self, session_dir, store_factory=None, queue_size=8):
        self.session_dir = str(session_dir)
        self.store_factory = store_factory
        self.jobs = queue.Queue(maxsize=queue_size)
        self.stopping = threading.Event()
        self.ids = {}
        self.pending_initial = {}
        self.pending_retry_at = {}
        self.pending_initial_dropped = 0
        self.dropped_jobs = 0
        self.failed_writes = 0
        self.saved_faults = 0
        self.updated_faults = 0
        self.last_fault_id = None
        self.last_error = None
        self.thread = threading.Thread(target=self._run, name='navigation-evidence-writer', daemon=True)
        self.thread.start()

    def submit(self, operation, token=None, **payload):
        # The caller already creates immutable evidence snapshots. deepcopy
        # keeps later ROS state mutation from changing queued trigger evidence.
        try:
            self.jobs.put_nowait((operation, token, deepcopy(payload)))
            return True
        except queue.Full:
            self.dropped_jobs += 1
            return False

    def _run(self):
        store = None
        next_open = 0.0
        while not self.stopping.is_set() or not self.jobs.empty():
            try:
                operation, token, payload = self.jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if operation == 'begin':
                    payload['trigger']['recorder_token'] = token
                    self.pending_initial[token] = payload
                    if len(self.pending_initial) > 8:
                        oldest = next(iter(self.pending_initial))
                        self.pending_initial.pop(oldest)
                        self.pending_retry_at.pop(oldest, None)
                        self.pending_initial_dropped += 1
                if store is None:
                    if time.monotonic() < next_open:
                        raise _StorageBackoff('evidence storage unavailable; waiting before reopening')
                    if self.store_factory is None:
                        from fault_store import EventStore
                        store = EventStore(self.session_dir)
                    else:
                        store = self.store_factory(self.session_dir)
                payload = json_safe(payload)
                if operation in ('begin', 'update') and token not in self.ids:
                    initial = self.pending_initial.get(token)
                    if initial is None:
                        raise OSError('initial incident was not retained; update cannot be linked')
                    if time.monotonic() < self.pending_retry_at.get(token, 0):
                        raise _StorageBackoff('initial incident unavailable; bounded retry waits for next update')
                    initial = json_safe(initial)
                    fault_id = store.record_fault(initial['trigger'], initial['snapshot'],
                                                  initial.get('evidence'))
                    self.ids[token] = fault_id
                    self.pending_initial.pop(token, None)
                    self.pending_retry_at.pop(token, None)
                    if len(self.ids) > 256:
                        self.ids.pop(next(iter(self.ids)))
                    self.last_fault_id = fault_id
                    self.saved_faults += 1
                if operation == 'update':
                    store.update_fault(self.ids[token], snapshot=payload.get('snapshot'),
                                       evidence=payload.get('evidence'))
                    self.updated_faults += 1
                elif operation == 'summary':
                    store.append_summary(payload['snapshot'])
                elif operation == 'heartbeat':
                    if hasattr(store, 'heartbeat'):
                        store.heartbeat(active=True)
                elif operation != 'begin':
                    raise ValueError('unknown evidence operation')
                if not self.pending_initial:
                    self.last_error = None
            except Exception as error:
                self.failed_writes += 1
                if not isinstance(error, _StorageBackoff):
                    self.last_error = str(error)[:512]
                    if token in self.pending_initial:
                        self.pending_retry_at[token] = time.monotonic()+2.0
                    if store is None:
                        next_open = time.monotonic()+5.0
            finally:
                self.jobs.task_done()
        if store is not None and hasattr(store, 'heartbeat'):
            try:
                store.heartbeat(active=False)
            except Exception as error:
                self.failed_writes += 1
                self.last_error = str(error)[:512]

    def status(self):
        return dict(enabled=True, queue_pending=self.jobs.qsize(),
                    dropped_jobs=self.dropped_jobs, failed_writes=self.failed_writes,
                    saved_faults=self.saved_faults, updated_faults=self.updated_faults,
                    unsaved_incidents=len(self.pending_initial),
                    pending_initial_dropped=self.pending_initial_dropped,
                    last_fault_id=self.last_fault_id, last_error=self.last_error,
                    writer_alive=self.thread.is_alive())

    def close(self, timeout=10.0):
        self.stopping.set()
        self.thread.join(timeout)
        return (not self.thread.is_alive() and not self.pending_initial and
                self.last_error is None and self.dropped_jobs == 0 and self.pending_initial_dropped == 0)


class FlightRecorder:
    """One incident keeps its first trigger across BT retries and final abort."""
    def __init__(self, writer=None, pre_seconds=PRE_SECONDS, post_seconds=POST_SECONDS,
                 max_events=MAX_EVENTS, max_event_bytes=MAX_EVENT_BYTES):
        self.writer = writer
        self.pre_seconds = pre_seconds
        self.post_seconds = post_seconds
        self.max_events = max_events
        self.max_event_bytes = max_event_bytes
        self.events = deque()
        self.event_bytes = 0
        self.dropped_events = 0
        self.oversize_events = 0
        self.coverage = {}
        self.episode = None
        self.last_completed = None

    def observe(self, source, data, now, ros_stamp=None, wall_time=None):
        event = dict(source=source, received_monotonic=now, time_utc=utc_text(wall_time),
                     ros_stamp=ros_stamp, data=json_safe(data))
        encoded = json.dumps(event, ensure_ascii=False, allow_nan=False).encode('utf-8')
        if len(encoded) > MAX_EVENT_SIZE:
            event['data'] = dict(truncated=True,
                preview=json.dumps(event['data'], ensure_ascii=False)[:1024])
            encoded = json.dumps(event, ensure_ascii=False).encode('utf-8')
            self.oversize_events += 1
        self.events.append((event, len(encoded)))
        self.event_bytes += len(encoded)
        while self.events and (len(self.events) > self.max_events or self.event_bytes > self.max_event_bytes):
            _, size = self.events.popleft()
            self.event_bytes -= size
            self.dropped_events += 1
        self._prune(now)
        state = self.coverage.setdefault(source, dict(messages=0))
        state.update(last_received_monotonic=now, last_ros_stamp=ros_stamp)
        state['messages'] += 1
        return event

    def _prune(self, now):
        while self.events and self.events[0][0]['received_monotonic'] < now-self.pre_seconds:
            _, size = self.events.popleft()
            self.event_bytes -= size

    def coverage_snapshot(self, now):
        return dict(pre_window_seconds=self.pre_seconds,
                    captured_pre_seconds=(max(0.0, now-self.events[0][0]['received_monotonic'])
                                          if self.events else 0.0),
                    raw_events=len(self.events),
                    raw_bytes=self.event_bytes, dropped_events=self.dropped_events,
                    oversize_events=self.oversize_events,
                    sources={name: dict(available=name in self.coverage,
                        receipt_age_s=now-self.coverage[name]['last_received_monotonic']
                            if name in self.coverage else None,
                        messages=self.coverage[name]['messages'] if name in self.coverage else 0,
                        last_ros_stamp=self.coverage[name].get('last_ros_stamp')
                            if name in self.coverage else None) for name in EXPECTED_SOURCES},
                    omissions=['no full point cloud or global/static occupancy grids',
                               'cross-topic messages are not an atomic snapshot',
                               'bounded in-memory windows may omit older or excess samples'])

    def _events_after(self, episode, now):
        # Preserve the beginning of the post window as well as the latest
        # trigger context; a long-running episode explicitly reports gaps.
        events = [deepcopy(event) for event, _ in self.events
                  if episode['started'] <= event['received_monotonic'] <= min(now, episode['deadline'])]
        return events

    def trigger(self, code, message, source, now, snapshot, context=None,
                severity='error', manual=False, ros_stamp=None, wall_time=None):
        trigger = dict(code=code, message=str(message)[:2048], source=source,
                       severity=severity, manual=manual, observed_at=utc_text(wall_time),
                       received_monotonic=now, ros_stamp=ros_stamp)
        self.observe('trigger', trigger, now, ros_stamp, wall_time)
        if manual and self.episode is None:
            self.summary(dict(reason=code, trigger=trigger, snapshot=snapshot))
            return False
        if self.episode is None:
            token = uuid.uuid4().hex
            initial = deepcopy(snapshot)
            evidence = dict(initial_snapshot=initial,
                events_before=[deepcopy(event) for event, _ in self.events],
                trigger_timeline=[trigger], context=deepcopy(context or {}),
                coverage=self.coverage_snapshot(now),
                post_capture=dict(state='pending', requested_seconds=self.post_seconds))
            self.episode = dict(token=token, started=now, deadline=now+self.post_seconds,
                timeline=[trigger], counts={code: 1}, initial=initial,
                latest_snapshot=deepcopy(snapshot), latest_context=deepcopy(context or {}),
                post_complete=False, events_after=[], last_update=now,
                last_trigger=trigger, timeline_truncated=False)
            if self.writer:
                self.writer.submit('begin', token, trigger=trigger, snapshot=initial, evidence=evidence)
            return True
        episode = self.episode
        episode['counts'][code] = episode['counts'].get(code, 0)+1
        episode['last_trigger'] = trigger
        if len(episode['timeline']) < MAX_TRIGGERS:
            episode['timeline'].append(trigger)
        else:
            # Keep the initial trigger and newest final state: an error burst
            # must not hide the eventual ABORT just because the list filled.
            episode['timeline'].pop(1)
            episode['timeline'].append(trigger)
            episode['timeline_truncated'] = True
        episode['latest_snapshot'] = deepcopy(snapshot)
        episode['latest_context'] = deepcopy(context or {})
        if not episode['post_complete']:
            episode['deadline'] = min(episode['started']+MAX_POST_SECONDS, now+self.post_seconds)
        if manual or code in ('navigation_aborted', 'goal_rejected', 'action_failure') or now-episode['last_update'] >= 1.0:
            self._update(now, state='complete' if episode['post_complete'] else 'pending')
        return False

    def _update(self, now, state, close_reason=None):
        episode = self.episode
        if episode is None:
            return
        if not episode['post_complete']:
            episode['events_after'] = self._events_after(episode, now)
        evidence = dict(trigger_timeline=deepcopy(episode['timeline']),
            trigger_counts=episode['counts'].copy(), timeline_truncated=episode['timeline_truncated'],
            events_after=deepcopy(episode['events_after']),
            latest_context=deepcopy(episode['latest_context']),
            coverage=self.coverage_snapshot(now),
            post_capture=dict(state=state, requested_seconds=self.post_seconds,
                captured_seconds=max(0.0, min(now, episode['deadline'])-episode['started']),
                close_reason=close_reason))
        if episode['post_complete']:
            evidence['later_trigger_context'] = [deepcopy(event) for event, _ in self.events]
            evidence['post_capture']['later_context_is_separate_window'] = True
        if episode['latest_context'].get('last_action') is not None:
            evidence['final_action'] = deepcopy(episode['latest_context']['last_action'])
        if self.writer:
            self.writer.submit('update', episode['token'], snapshot=episode['latest_snapshot'],
                               evidence=evidence)
        episode['last_update'] = now

    def tick(self, now, snapshot=None, context=None):
        self._prune(now)
        if self.episode and not self.episode['post_complete'] and now >= self.episode['deadline']:
            if snapshot is not None:
                self.episode['latest_snapshot'] = deepcopy(snapshot)
            if context is not None:
                self.episode['latest_context'] = deepcopy(context)
            self._update(now, 'complete')
            self.episode['post_complete'] = True

    def outcome(self, code, message, now, snapshot, context=None):
        if self.episode is not None:
            self.trigger(code, message, 'navigation_outcome', now, snapshot, context,
                         severity='info', manual=code.startswith('manual_'))

    def reset_episode(self, now, reason='new_goal'):
        if self.episode:
            self._update(now, 'complete' if self.episode['post_complete'] else 'interrupted', reason)
            self.last_completed = self.episode['token']
            self.episode = None

    def summary(self, snapshot):
        if self.writer:
            self.writer.submit('summary', snapshot=snapshot)

    def status(self):
        return dict(self.writer.status() if self.writer else dict(enabled=False,
            reason='GO2_DIAGNOSTICS_SESSION unset; console-only diagnostics'),
            pre_window_seconds=self.pre_seconds, post_window_seconds=self.post_seconds,
            raw_events=len(self.events), dropped_events=self.dropped_events,
            oversize_events=self.oversize_events, incident_active=self.episode is not None)

    def close(self, now, snapshot=None, context=None):
        if self.episode:
            if snapshot is not None:
                self.episode['latest_snapshot'] = deepcopy(snapshot)
            if context is not None:
                self.episode['latest_context'] = deepcopy(context)
            self._update(now, 'complete' if self.episode['post_complete'] else 'interrupted', 'recorder_shutdown')
        return self.writer.close() if self.writer else True
