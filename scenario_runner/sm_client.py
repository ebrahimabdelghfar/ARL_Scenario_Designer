"""Minimal system-manager client for the CSV scenario manager.

Reference implementation from micropilot_manager/docs/scenario_manager_integration.md (Appendix A).
Needs the manager overlay sourced last (micropilot_manager_msgs; the sim's thirdparty copy is stale).
Spins its own node on a background executor, so it can be called from the blocking scenario loop.
"""
import json
import math
import queue
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from micropilot_manager_msgs.msg import (LeafState, MissionStatus, SystemState,
                                         TransitionEvent, Waypoint)
from micropilot_manager_msgs.srv import FsmTransition, MissionCommand, MissionHistory, SetMission
from std_msgs.msg import String

SM = '/micropilot_system_manager_node'
TERMINAL = {MissionStatus.COMPLETED: 'COMPLETED', MissionStatus.ABORTED: 'ABORTED',
            MissionStatus.FAILED: 'FAILED'}


class SystemManagerClient:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('scenario_manager_sm_client')
        self.leaf = self.top = None          # latest ~/state/leaf, ~/state/top names
        self.mission_result = None           # last terminal MissionStatus name
        self.mission_id = None               # current scenario's mission (history fallback)
        self.processes = []                  # latest ~/process/status "processes"
        self._events = queue.Queue()         # TransitionEvent, in order
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        n = self.node
        n.create_subscription(LeafState, SM + '/state/leaf', lambda m: setattr(self, 'leaf', m.name), latched)
        n.create_subscription(SystemState, SM + '/state/top', lambda m: setattr(self, 'top', m.name), latched)
        n.create_subscription(TransitionEvent, SM + '/transitions', self._events.put,
                              QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        n.create_subscription(MissionStatus, SM + '/mission/status', self._on_mission, best_effort)
        n.create_subscription(String, SM + '/process/status', self._on_process, best_effort)
        self._fsm = n.create_client(FsmTransition, SM + '/fsm/trigger_event')
        self._set_mission = n.create_client(SetMission, SM + '/mission/set')
        self._mission_cmd = n.create_client(MissionCommand, SM + '/mission/command')
        self._history = n.create_client(MissionHistory, SM + '/mission/history')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def close(self):
        """Call once at the end of the batch (a still-spinning executor aborts the interpreter at exit)."""
        self._executor.shutdown()
        self._spin_thread.join(timeout=5.0)   # must leave rclpy before the node goes
        self.node.destroy_node()

    # ── callbacks ────────────────────────────────────────────────────────────
    def _on_mission(self, m):
        if m.status in TERMINAL:             # published exactly once, then IDLE
            self.mission_result = TERMINAL[m.status]

    def _on_process(self, m):
        self.processes = json.loads(m.data).get('processes', [])

    # ── plumbing ─────────────────────────────────────────────────────────────
    def _call(self, client, req, timeout=40.0):
        if not client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(client.srv_name + ' not available')
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                raise TimeoutError(client.srv_name + ' timed out')
            time.sleep(0.02)
        return fut.result()

    def _wait_until(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while not cond():
            if time.monotonic() > end:
                raise TimeoutError(f'{what} (top={self.top}, leaf={self.leaf})')
            time.sleep(0.1)

    def _wait_transition(self, done, timeout, what):
        """Consume ~/transitions until done(ev) is true; return that event.
        Rejected requests (from any client) arrive too, with success=False."""
        end = time.monotonic() + timeout
        while True:
            try:
                ev = self._events.get(timeout=max(0.0, end - time.monotonic()))
            except queue.Empty:
                raise TimeoutError(f'{what} (top={self.top}, leaf={self.leaf})')
            print(f'[sm] {ev.from_name} -> {ev.to_name}: {ev.message}')
            if done(ev):
                return ev

    def _drain(self):
        while not self._events.empty():
            self._events.get_nowait()

    def _group_status(self, group):
        return [p['status'] for p in self.processes if group in p['groups']]

    def trigger(self, event):
        self._drain()
        r = self._call(self._fsm, FsmTransition.Request(event_name=event))
        print(f'[sm] {event}: success={r.success} {r.from_name}->{r.to_name} "{r.message}"')
        return r

    # ── scenario lifecycle ───────────────────────────────────────────────────
    def wait_connected(self, timeout=60.0):
        self._wait_until(lambda: self.leaf is not None, timeout, 'no ~/state/leaf from the system manager')

    def power_on(self, timeout=180.0):
        """OFF.POWERED_OFF → STANDBY.IDLE with LOCALIZATION up and nav groups stopped."""
        self.mission_id = self.mission_result = None
        r = self.trigger('power_on')
        if not (r.success and r.to_state == SystemState.INIT):   # 0→0 = still SHUTTING_DOWN
            raise RuntimeError(f'power_on rejected in {self.leaf}: {r.message}')
        ev = self._wait_transition(lambda e: e.success and e.from_state == SystemState.INIT
                                   and e.to_state != SystemState.INIT,
                                   timeout, 'INIT did not finish')
        if ev.to_state != SystemState.STANDBY:
            raise RuntimeError(f'INIT failed: {ev.to_name} ({ev.message})')
        # STANDBY entry starts LOCALIZATION and stops PERCEPTION/NAVIGATION in the background.
        # all() (not == {'RUNNING'}): a profile without LOCALIZATION nodes has an empty group.
        self._wait_until(lambda: self.leaf == 'STANDBY.IDLE'
                         and all(s == 'RUNNING' for s in self._group_status('LOCALIZATION'))
                         and not {'RUNNING', 'CONFIGURED', 'ERROR'} & set(
                             self._group_status('PERCEPTION') + self._group_status('NAVIGATION')),
                         60.0, 'STANDBY not settled')

    def start_mission(self, waypoints, mission_id, timeout=90.0, **cfg):
        """SetMission, then wait for AUTONOMOUS. Fire NO engage event — arbitration is automatic."""
        self.mission_result, self.mission_id = None, mission_id
        self._drain()
        req = SetMission.Request(waypoints=waypoints, mission_id=mission_id, **cfg)
        r = self._call(self._set_mission, req)
        print(f'[sm] mission/set: success={r.success} "{r.message}"')
        if not r.success:
            raise RuntimeError('SetMission rejected: ' + r.message)
        ev = self._wait_transition(lambda e: e.success and (e.to_state in (SystemState.AUTONOMOUS, SystemState.FAULT)
                                                            or e.message == 'disengage_autonomy'),
                                   timeout, 'arbitration did not finish')
        if ev.to_state != SystemState.AUTONOMOUS:
            raise RuntimeError(f'not engaged: {ev.to_name} ({ev.message}) — reason on {SM}/log_stream')

    def shutdown(self, timeout=120.0):
        """Any state → OFF.POWERED_OFF (abort mission / reset fault first). Returns the mission result."""
        if self.top == 'AUTONOMOUS':
            r = self._call(self._mission_cmd, MissionCommand.Request(command='abort'))
            print(f'[sm] mission abort: success={r.success} "{r.message}"')
            self._wait_until(lambda: self.top != 'AUTONOMOUS', 30.0, 'still AUTONOMOUS after abort')
        if self.top == 'FAULT':
            self.trigger('system_reset')
        elif self.top == 'STANDBY':
            self._wait_until(lambda: self.leaf == 'STANDBY.IDLE', 60.0, 'STANDBY not idle')
            if not self.trigger('power_off').success:
                raise RuntimeError(f'power_off rejected in {self.leaf}')
        elif self.top != 'OFF':              # REMOTE_CONTROL / FREE_DRIVING / CHARGING
            raise RuntimeError(f'cannot power off from {self.leaf}')
        self._wait_until(lambda: self.leaf == 'OFF.POWERED_OFF', timeout, 'not powered off')
        if self.mission_result is None and self.mission_id:
            # the terminal status is published once, best effort — easy to miss right after abort
            r = self._call(self._history, MissionHistory.Request(mission_id=self.mission_id))
            self.mission_result = next((TERMINAL.get(s) for i, s in zip(r.mission_ids, r.statuses)
                                        if i == self.mission_id), None) if r.success else None
        return self.mission_result


# ── CARLA location → the WGS-84 the sim publishes on /sim/feedback/gps ──────
# Same pipeline as carla_telemetry_cpp CarlaGPS::update(): CARLA geolocation →
# ENU about the map geo-reference → re-projected about global_coordinates.
_A, _E2 = 6378137.0, 6.69437999014e-3


def _ecef(lat, lon, alt):
    la, lo = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1 - _E2 * math.sin(la) ** 2)
    return ((n + alt) * math.cos(la) * math.cos(lo), (n + alt) * math.cos(la) * math.sin(lo),
            (n * (1 - _E2) + alt) * math.sin(la))


def _enu_axes(lat, lon):
    la, lo = math.radians(lat), math.radians(lon)
    return ((-math.sin(lo), math.cos(lo), 0.0),
            (-math.sin(la) * math.cos(lo), -math.sin(la) * math.sin(lo), math.cos(la)),
            (math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)))


def carla_to_llh(carla_map, location, origin):
    """origin = (lat, lon, alt) from carla_interface_config.yaml `global_coordinates`."""
    import carla
    g = carla_map.transform_to_geolocation(location)
    r = carla_map.transform_to_geolocation(carla.Location(0.0, 0.0, 0.0))  # map geo-reference
    p, p0 = _ecef(g.latitude, g.longitude, g.altitude), _ecef(r.latitude, r.longitude, r.altitude)
    d = [a - b for a, b in zip(p, p0)]
    enu = [sum(ax[i] * d[i] for i in range(3)) for ax in _enu_axes(r.latitude, r.longitude)]
    axes, o = _enu_axes(origin[0], origin[1]), _ecef(*origin)
    x, y, z = (o[k] + sum(axes[j][k] * enu[j] for j in range(3)) for k in range(3))
    lon, rho = math.atan2(y, x), math.hypot(x, y)
    lat = math.atan2(z, rho * (1 - _E2))
    for _ in range(5):
        n = _A / math.sqrt(1 - _E2 * math.sin(lat) ** 2)
        alt = rho / math.cos(lat) - n
        lat = math.atan2(z, rho * (1 - _E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def goal_waypoint(carla_map, location, origin, label='goal'):
    lat, lon, alt = carla_to_llh(carla_map, location, origin)
    return Waypoint(latitude=lat, longitude=lon, altitude=alt, speed=0.0, label=label)
