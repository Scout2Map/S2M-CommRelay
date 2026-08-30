#!/usr/bin/env python3
# Bridges /events to Web-Monitoring over a small custom WebSocket protocol.
# Unlike rosbridge, this node buffers events while no client is connected and
# replays them in order once a client (re)connects, so a hazard raised during
# a network outage is never silently dropped.
import asyncio
import json
import math
import subprocess
import threading
import time
import zlib
from collections import deque

import rclpy
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from scout2map_msgs.msg import DriveStatus, SensorStatus
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger

import tf2_ros

try:
    import websockets
    # Import the submodule explicitly - some websockets versions (17+) do not
    # expose it as websockets.exceptions unless imported on its own.
    from websockets.exceptions import ConnectionClosed
except ImportError as exc:
    raise ImportError(
        "the 'websockets' package is required, install it with "
        "'pip install websockets --break-system-packages' or via rosdep"
    ) from exc


# Parameters exposed to the web settings panel, keyed by an internal node
# key (comm_relay resolves the actual ROS node name via the *_node_name
# parameters below, same pattern as every other topic/service name in this
# file). Deliberately excludes anything safety/timing critical - heartbeat,
# drive-link and TF timeouts, comm-loss gating, rough-terrain/rotation
# calibration constants, the enable_*_events toggles - because a live web
# control that can loosen a fail-safe mid-mission is a hazard the launch
# file audit trail does not have (2026-08-30).
SETTABLE_PARAMS = {
    'scout_vision': {
        'confidence_threshold': {'type': 'double', 'min': 0.05, 'max': 0.95},
        'nms_threshold': {'type': 'double', 'min': 0.05, 'max': 0.95},
        'max_fps': {'type': 'double', 'min': 0.5, 'max': 15.0},
    },
    'event_engine': {
        'event_repeat_interval_s': {'type': 'double', 'min': 1.0, 'max': 120.0},
        'vision_confidence_threshold': {'type': 'double', 'min': 0.05, 'max': 0.95},
        'vision_min_consecutive_frames': {'type': 'integer', 'min': 1, 'max': 10},
        'vision_event_cooldown_s': {'type': 'double', 'min': 0.5, 'max': 60.0},
        'prediction_event_cooldown_s': {'type': 'double', 'min': 1.0, 'max': 300.0},
        'publish_clear_events': {'type': 'bool'},
    },
}


def _param_value_to_py(pv: ParameterValue):
    if pv.type == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    if pv.type == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    if pv.type == ParameterType.PARAMETER_STRING:
        return pv.string_value
    return None


class CommRelayNode(Node):

    def __init__(self, **kwargs):
        super().__init__('comm_relay', **kwargs)

        self.declare_parameter('events_topic', '/events')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 9091)
        self.declare_parameter('buffer_max_len', 500)
        self.declare_parameter('buffer_max_age_s', 1800.0)
        self.declare_parameter('link_status_topic', '/relay/link_status')
        self.declare_parameter('link_status_period_s', 2.0)

        # /control/heartbeat is documented across the fleet (event_engine,
        # return_home) as coming from "the control server", but nothing on
        # real hardware ever actually published it - return_home could
        # never leave its pre-armed state and cmd_vel_safety_gate blocked
        # all motion whenever use_return_home:=true (2026-08-29). comm_relay
        # IS that control server's bridge to the robot, so it is the
        # correct publisher: heartbeat means "an operator's web client is
        # actually connected", not just "the process is alive".
        self.declare_parameter('heartbeat_topic', '/control/heartbeat')
        self.declare_parameter('heartbeat_period_s', 0.5)
        self.declare_parameter('ws_ping_interval_s', 10.0)
        self.declare_parameter('ws_ping_timeout_s', 10.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_relay_period_s', 2.0)
        self.declare_parameter('map_zlib_level', 6)

        # Robot Pose parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_relay_period_s', 0.1)

        # Drive Status & Battery topics
        self.declare_parameter('battery_topic', '/drive/battery')
        self.declare_parameter('drive_status_topic', '/drive/status')
        self.declare_parameter('telemetry_relay_period_s', 10.0)

        # Pico sensor-fusion MCU link/presence, published by sensor_bridge.
        # Surfaced the same way as drive_status so an operator sees a dead
        # or never-initialized sensor (e.g. BH1750 not answering on I2C)
        # without SSHing in and running ros2 topic echo.
        self.declare_parameter('sensor_status_topic', '/sensors/status')

        # scout_vision publishes its own inference health on the shared
        # /diagnostics bus (diagnostic_msgs/DiagnosticArray, alongside other
        # nodes like ekf_filter_node) - only the 'scout_vision/inference'
        # entry is relayed. This is what would have shown a camera that
        # never delivered a frame (2026-08-29) without ros2 topic echo.
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter(
            'vision_diagnostic_name', 'scout_vision/inference')

        # Return-home goal, so the operator can see where the robot is
        # headed instead of just the state text. Both topics are published
        # by return_home_node with TRANSIENT_LOCAL/RELIABLE QoS (latched) -
        # match it below so a relay started after the last publish still
        # gets the current value instead of waiting for the next change.
        self.declare_parameter('return_home_pose_topic', '/return_home/start_pose')
        self.declare_parameter('return_home_status_topic', '/return_home/status')

        # explore_lite's own cmd_vel output, so _stop_explore_mission can
        # publish a zero-velocity message on top of killing the subprocess -
        # terminate() alone can leave residual momentum if the process was
        # mid-command when it died.
        self.declare_parameter('explore_cmd_vel_topic', '/cmd_vel')

        # The NavigateToPose action explore_lite's own action client sends
        # goals to (bt_navigator) - shared with return_home_node's own
        # 'navigate_action' param and not affected by the /cmd_vel remap
        # use_return_home applies, so one client here covers both missions.
        # stop_mission(explore) needs this: killing the explore_lite
        # process does NOT cancel whatever goal it already sent - action
        # goal lifecycle is server-side, so bt_navigator just keeps
        # executing/replanning it forever, which is why "stop" looked like
        # it did nothing while controller_server kept logging "Passing new
        # path to controller" (2026-08-29).
        self.declare_parameter('navigate_action', '/navigate_to_pose')

        # Settings panel: threshold read/write plus the SETTABLE_PARAMS
        # allowlist above, resolved into per-node get/set_parameters
        # service names below (every rclpy node exposes these
        # automatically for its declare_parameter() calls, so no changes
        # were needed on the scout_vision/event_engine side for this part).
        self.declare_parameter('threshold_topic', '/threshold/set')
        self.declare_parameter('threshold_get_all_service', '/threshold/get_all')
        self.declare_parameter('vision_node_name', 'scout_vision')
        self.declare_parameter('event_engine_node_name', 'event_engine')

        self._events_topic = self.get_parameter('events_topic').value
        self._ws_host = self.get_parameter('ws_host').value
        self._ws_port = int(self.get_parameter('ws_port').value)
        self._buffer_max_len = int(self.get_parameter('buffer_max_len').value)
        self._buffer_max_age_s = float(self.get_parameter('buffer_max_age_s').value)
        self._link_status_topic = self.get_parameter('link_status_topic').value
        self._link_status_period_s = float(self.get_parameter('link_status_period_s').value)
        self._heartbeat_topic = self.get_parameter('heartbeat_topic').value
        self._heartbeat_period_s = float(self.get_parameter('heartbeat_period_s').value)
        self._ping_interval_s = float(self.get_parameter('ws_ping_interval_s').value)
        self._ping_timeout_s = float(self.get_parameter('ws_ping_timeout_s').value)
        self._map_topic = self.get_parameter('map_topic').value
        self._map_relay_period_s = float(self.get_parameter('map_relay_period_s').value)
        self._map_zlib_level = int(self.get_parameter('map_zlib_level').value)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._pose_relay_period_s = float(self.get_parameter('pose_relay_period_s').value)

        self._battery_topic = self.get_parameter('battery_topic').value
        self._drive_status_topic = self.get_parameter('drive_status_topic').value
        self._telemetry_relay_period_s = float(self.get_parameter('telemetry_relay_period_s').value)
        self._sensor_status_topic = self.get_parameter('sensor_status_topic').value
        self._diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self._vision_diagnostic_name = self.get_parameter(
            'vision_diagnostic_name').value

        self._return_home_pose_topic = self.get_parameter('return_home_pose_topic').value
        self._return_home_status_topic = self.get_parameter('return_home_status_topic').value

        self._explore_cmd_vel_topic = self.get_parameter('explore_cmd_vel_topic').value
        self._navigate_action = self.get_parameter('navigate_action').value

        self._threshold_topic = self.get_parameter('threshold_topic').value
        self._threshold_get_all_service = self.get_parameter(
            'threshold_get_all_service').value
        self._node_names = {
            'scout_vision': self.get_parameter('vision_node_name').value,
            'event_engine': self.get_parameter('event_engine_node_name').value,
        }

        self._telemetry_lock = threading.Lock()
        self._latest_battery_payload = None
        self._latest_drive_status_payload = None
        self._latest_sensor_status_payload = None
        self._latest_vision_status_payload = None

        # Cached so a client that connects after the last publish (both
        # topics are latched, but that only guarantees comm_relay's own
        # subscription gets the backlog - not any browser that joins later)
        # still gets the current goal/state right away, same idea as the
        # map catch-up below.
        self._return_home_lock = threading.Lock()
        self._latest_return_home_goal = None
        self._latest_return_home_status = None

        # Subprocess handler for exploration mission
        self._explore_process = None

        # Buffer entries: (monotonic_ts, wall_ts, seq, raw_json_str)
        # Held only while no web client is connected; see README for the
        # single-logical-consumer assumption behind this design.
        self._buffer = deque()
        self._buffer_lock = threading.Lock()
        self._dropped_count = 0
        self._seq = 0

        self._ws_clients = set()
        # One asyncio.Lock per connected client, guarded by self._buffer_lock
        # alongside self._ws_clients. Every send to a client goes through
        # _send_frames() so a multi-frame send (map meta + binary payload)
        # can never be interleaved with an unrelated event broadcast on the
        # same socket.
        self._client_locks = {}
        self._last_client_seen_mono = None
        self._link_state = 'lost'

        # /map is best-effort, latest-wins - no buffering, unlike /events.
        # Guarded by its own lock since it's updated from a different
        # subscription than the event buffer above.
        self._map_lock = threading.Lock()
        self._latest_map_meta = None
        self._latest_map_bytes = None
        self._map_dirty = False
        self._map_seq = 0

        # TF Listener setup for Robot Pose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, self._link_status_topic, 10)
        self._heartbeat_pub = self.create_publisher(Empty, self._heartbeat_topic, 10)
        self._sub = self.create_subscription(
            String, self._events_topic, self._on_event, 50)

        # Drive Telemetry Subscriptions (/drive/battery, /drive/status)
        self._battery_sub = self.create_subscription(
            BatteryState, self._battery_topic, self._on_battery, 10)
        self._drive_status_sub = self.create_subscription(
            DriveStatus, self._drive_status_topic, self._on_drive_status, 10)
        self._sensor_status_sub = self.create_subscription(
            SensorStatus, self._sensor_status_topic, self._on_sensor_status, 10)
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray, self._diagnostics_topic, self._on_diagnostics, 10)

        # ROS 2 Service Clients for Drive Controls
        self._cli_estop = self.create_client(Trigger, '/drive/estop')
        self._cli_clear_fault = self.create_client(Trigger, '/drive/clear_fault')
        self._cli_reset_odom = self.create_client(Trigger, '/drive/reset_odom')

        # ROS 2 Service Clients for Return-Home Controls
        self._cli_return_trigger = self.create_client(Trigger, '/return_home/trigger')
        self._cli_return_arm = self.create_client(SetBool, '/return_home/arm')

        # Settings panel: same fire-and-forget publish path an operator's
        # own /threshold/set publisher would use (event_engine's
        # threshold_callback does not care who published it), plus one
        # get/set_parameters client pair per SETTABLE_PARAMS entry -
        # standard services rclpy exposes automatically for every declared
        # parameter, so nothing new was needed on those nodes for this part.
        self._threshold_pub = self.create_publisher(String, self._threshold_topic, 10)
        self._cli_threshold_get_all = self.create_client(
            Trigger, self._threshold_get_all_service)
        self._param_clients = {}
        for node_key in SETTABLE_PARAMS:
            ros_node_name = self._node_names[node_key]
            self._param_clients[node_key] = {
                'get': self.create_client(
                    GetParameters, f'/{ros_node_name}/get_parameters'),
                'set': self.create_client(
                    SetParameters, f'/{ros_node_name}/set_parameters'),
            }

        # zero-velocity publisher used by _stop_explore_mission
        self._stop_pub = self.create_publisher(Twist, self._explore_cmd_vel_topic, 10)

        # Cancels explore_lite's active NavigateToPose goal on stop_mission.
        # A plain CancelGoal service client (the standard
        # <action>/_action/cancel_goal service every action server exposes)
        # rather than a full rclpy ActionClient, since comm_relay never sent
        # the goal itself and so has no local goal handle to reference - an
        # ActionClient can normally only cancel goals it sent. A request
        # with a zero-filled goal_id and zero timestamp cancels every goal
        # currently active on the server (action_msgs/srv/CancelGoal
        # semantics), which is what terminate()-ing explore_lite's process
        # does NOT do by itself, see the navigate_action parameter comment
        # above.
        self._navigate_cancel_client = self.create_client(
            CancelGoal, f'{self._navigate_action}/_action/cancel_goal')

        # Map publishers (slam_toolbox, nav2 map_server) typically latch the
        # last map with TRANSIENT_LOCAL durability - match it, otherwise a
        # relay started after the last publish would wait for the next one.
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos)

        # return_home_node publishes both with the same latched QoS as /map.
        self._return_home_pose_sub = self.create_subscription(
            PoseStamped, self._return_home_pose_topic,
            self._on_return_home_pose, map_qos)
        self._return_home_status_sub = self.create_subscription(
            String, self._return_home_status_topic,
            self._on_return_home_status, map_qos)

        self._status_timer = self.create_timer(
            self._link_status_period_s, self._publish_status)
        # Well under both consumers' timeouts (event_engine COMM_DEGRADED
        # 1.5s, return_home heartbeat_timeout_sec 3.0s default) with real
        # margin, not equal to either - an exact-equal period/timeout pair
        # is what caused the cmd_vel_safety_gate timing race fixed earlier
        # today (fcd1725), so this is deliberately several times faster
        # than the tightest threshold rather than matched to it.
        self._heartbeat_timer = self.create_timer(
            self._heartbeat_period_s, self._publish_heartbeat)
        self._map_timer = self.create_timer(
            self._map_relay_period_s, self._maybe_broadcast_map)
        self._pose_timer = self.create_timer(
            self._pose_relay_period_s, self._broadcast_robot_pose)
        self._telemetry_timer = self.create_timer(
            self._telemetry_relay_period_s, self._broadcast_telemetry)

        # The websocket server runs its own asyncio loop on a background
        # thread so it never blocks rclpy.spin() on the main thread.
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        self.get_logger().info(
            f'comm_relay listening on ws://{self._ws_host}:{self._ws_port} '
            f'(relaying {self._events_topic})')

    # --- asyncio side (background thread) ---

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as exc:  # noqa: BLE001 - keep the node alive, just log
            self.get_logger().error(f'websocket server stopped: {exc}')

    async def _serve(self):
        async with websockets.serve(
            self._on_client,
            self._ws_host,
            self._ws_port,
            ping_interval=self._ping_interval_s,
            ping_timeout=self._ping_timeout_s,
        ):
            await asyncio.Future()  # run until the loop is stopped

    async def _on_client(self, websocket):
        now_mono = time.monotonic()
        offline_s = None

        with self._buffer_lock:
            if self._last_client_seen_mono is not None:
                offline_s = round(now_mono - self._last_client_seen_mono, 1)
            backlog = list(self._buffer)
            # Cleared up front, before the send below completes. A client
            # that drops mid-replay loses the remainder rather than getting
            # it re-delivered on its next reconnect - acceptable for a
            # single-operator test tool, see README known limitations.
            self._buffer.clear()
            dropped = self._dropped_count
            self._dropped_count = 0
            self._ws_clients.add(websocket)
            self._client_locks[websocket] = asyncio.Lock()
            self._link_state = 'ok'

        self.get_logger().info(
            f'web client connected ({len(backlog)} buffered, '
            f'{dropped} dropped, offline {offline_s}s)')

        try:
            await self._send_frames(websocket, [json.dumps({
                'kind': 'status',
                'state': 'ok',
                'reconnected': offline_s is not None,
                'offline_s': offline_s,
                'buffered_count': len(backlog),
                'dropped_count': dropped,
            })])

            for _, wall_ts, seq, raw in backlog:
                await self._send_frames(
                    websocket, [self._wrap(raw, replay=True, wall_ts=wall_ts, seq=seq)])

            # New joiners get the latest known map right away rather than
            # waiting for the next periodic broadcast.
            with self._map_lock:
                latest_meta = self._latest_map_meta
                latest_bytes = self._latest_map_bytes
            if latest_meta is not None:
                await self._send_frames(websocket, [json.dumps(latest_meta), latest_bytes])

            # Same idea for the return-home goal/state - both topics are
            # latched at the ROS level so comm_relay already has the
            # current value even if it arrived before this client connected.
            with self._return_home_lock:
                latest_goal = self._latest_return_home_goal
                latest_status = self._latest_return_home_status
            if latest_goal is not None:
                await self._send_frames(websocket, [latest_goal])
            if latest_status is not None:
                await self._send_frames(websocket, [latest_status])

            # Same "catch up a new joiner immediately" idea as the map and
            # return-home pushes above - an operator opening the settings
            # panel should see current thresholds/params right away rather
            # than waiting on a get_settings round trip. Sent to just this
            # client (not broadcast) so it doesn't spam others already
            # connected.
            self._fetch_and_broadcast_settings(target=websocket)

            # Process Inbound Websocket Messages (Commands from Web Monitoring)
            async for raw_inbound in websocket:
                await self._handle_inbound_message(raw_inbound, websocket)

        except (ConnectionClosed, OSError):
            pass
        except Exception as exc:  # noqa: BLE001 - one bad client must not kill the server
            self.get_logger().warning(f'client session ended with error: {exc}')
        finally:
            with self._buffer_lock:
                self._ws_clients.discard(websocket)
                self._client_locks.pop(websocket, None)
                if not self._ws_clients:
                    self._last_client_seen_mono = time.monotonic()
                    self._link_state = 'lost'

    async def _handle_inbound_message(self, raw_msg, websocket=None):
        """Web UI로부터 들어온 command 수신 처리"""
        try:
            msg = json.loads(raw_msg)
            if msg.get('kind') != 'command':
                return

            cmd = msg.get('command')
            self.get_logger().info(f'Received Web Command: {cmd}')

            # drive control commands (/drive/estop, /drive/clear_fault, /drive/reset_odom)
            if cmd == 'estop':
                self._call_trigger_service(self._cli_estop, 'E-Stop (/drive/estop)')
            elif cmd == 'clear_fault':
                self._call_trigger_service(self._cli_clear_fault, 'Clear Fault (/drive/clear_fault)')
            elif cmd == 'reset_odom':
                self._call_trigger_service(self._cli_reset_odom, 'Reset Odom (/drive/reset_odom)')

            # start mission control commands (launch_mission, stop_mission)
            elif cmd == 'launch_mission':
                mission = msg.get('mission')
                if mission == 'explore':
                    self._start_explore_mission()
                elif mission == 'return_home':
                    self._start_return_mission()

            # end mission control commands (launch_mission, stop_mission)
            elif cmd == 'stop_mission':
                mission = msg.get('mission')
                if mission == 'explore':
                    self._stop_explore_mission()
                elif mission == 'return_home':
                    self._stop_return_mission()

            # settings panel commands (thresholds + the SETTABLE_PARAMS
            # allowlist) - get_settings re-sends to just the asking client,
            # the two setters broadcast the refreshed state to everyone
            # once the change is confirmed, so every open panel stays in
            # sync rather than only the one that made the edit.
            elif cmd == 'get_settings':
                self._fetch_and_broadcast_settings(target=websocket)
            elif cmd == 'set_threshold':
                event_type = msg.get('type')
                level = msg.get('level', 'warning')
                value = msg.get('value')
                if event_type is not None and value is not None:
                    self._set_threshold(event_type, level, value)
                else:
                    self.get_logger().warn(f'set_threshold rejected: missing type/value in {msg}')
            elif cmd == 'set_param':
                node_key = msg.get('node')
                param_name = msg.get('param')
                value = msg.get('value')
                if node_key is not None and param_name is not None and value is not None:
                    self._set_node_param(node_key, param_name, value)
                else:
                    self.get_logger().warn(f'set_param rejected: missing node/param/value in {msg}')

        except Exception as exc:
            self.get_logger().error(f'Error processing inbound websocket message: {exc}')

    def _call_trigger_service(self, client, name):
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'Service {name} is not available!')
            return

        req = Trigger.Request()
        client.call_async(req)
        self.get_logger().info(f'Successfully triggered service: {name}')

    def _start_explore_mission(self):
        if self._explore_process and self._explore_process.poll() is None:
            self.get_logger().warn('Explore mission is already running!')
            return
        self._explore_process = subprocess.Popen(
            ['ros2', 'launch', 'explore_lite', 'explore.launch.py']
        )
        self.get_logger().info('Launched explore_lite mission')
        self._broadcast_mission_status('EXPLORE')

    def _stop_explore_mission(self):
        if self._explore_process and self._explore_process.poll() is None:
            self._explore_process.terminate()
            self._explore_process = None
            self.get_logger().info('Terminated explore_lite mission')

        # comm_relay only tracks ONE subprocess handle for the lifetime of
        # this node instance (documented limitation, see README known
        # limitations). If comm_relay itself was restarted after
        # launch_mission - or the handle is stale for any other reason -
        # the branch above silently does nothing and explore_lite keeps
        # running: it will just pick a new frontier and send a fresh
        # NavigateToPose goal the moment the one _cancel_active_navigation
        # cancels below is cleared, making stop_mission look like it did
        # nothing (2026-08-29: cancel succeeded and the robot briefly
        # stopped, but a new goal appeared ~5s later - no "Terminated
        # explore_lite mission" log at all, meaning the handle branch above
        # never ran that time). Kill it by process name as a fallback so
        # this is never a silent no-op regardless of handle state.
        self._kill_stray_explore_processes()

        # send zero velocity to cancel active momentum and stop immediately
        stop_cmd = Twist()
        self._stop_pub.publish(stop_cmd)

        # Killing the explore_lite process above does not cancel the
        # NavigateToPose goal it already sent - bt_navigator keeps
        # executing/replanning that goal indefinitely regardless, which is
        # why the robot kept moving (controller_server logging "Passing new
        # path to controller" on repeat) after stop_mission looked like it
        # did nothing (2026-08-29). Cancel it explicitly.
        self._cancel_active_navigation()

        self._broadcast_mission_status('IDLE')

    def _kill_stray_explore_processes(self):
        try:
            result = subprocess.run(
                ['pkill', '-f', 'explore_lite'],
                capture_output=True, timeout=2.0)
        except Exception as exc:
            self.get_logger().error(f'failed to pkill stray explore_lite process(es): {exc}')
            return

        # pkill returns 1 when nothing matched - the common, expected case
        # once the handle-based terminate() above already did its job.
        # Only 0 (something was actually killed here, meaning the handle
        # was stale) is worth a log line; anything else is a real error.
        if result.returncode == 0:
            self.get_logger().warn(
                'killed stray explore_lite process(es) by name - comm_relay '
                'had no live subprocess handle for them')
        elif result.returncode not in (0, 1):
            stderr = result.stderr.decode(errors='replace').strip()
            self.get_logger().warn(f'pkill -f explore_lite exited {result.returncode}: {stderr}')

    def _cancel_active_navigation(self):
        if not self._navigate_cancel_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(
                f'{self._navigate_action} cancel service is not available - '
                'any in-flight navigation goal was NOT cancelled')
            return

        req = CancelGoal.Request()  # zero goal_id + zero timestamp = cancel all
        future = self._navigate_cancel_client.call_async(req)

        def _on_cancel_done(f):
            try:
                res = f.result()
                n = len(res.goals_canceling)
                self.get_logger().info(f'cancelled {n} active navigation goal(s)')
            except Exception as exc:
                self.get_logger().error(f'navigation cancel call failed: {exc}')

        future.add_done_callback(_on_cancel_done)

    def _start_return_mission(self):
        # call trigger service on the active return_home node
        if not self._cli_return_trigger.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Return home trigger service (/return_home/trigger) is not available!')
            return

        req = Trigger.Request()
        future = self._cli_return_trigger.call_async(req)
        
        # handle service response callback
        def _on_trigger_done(f):
            try:
                res = f.result()
                if res.success:
                    self.get_logger().info(f'Return home service triggered: {res.message}')
                    self._broadcast_mission_status('RETURN')
                else:
                    self.get_logger().warn(f'Return home service rejected: {res.message}')
            except Exception as exc:
                self.get_logger().error(f'Return home service call failed: {exc}')

        future.add_done_callback(_on_trigger_done)

    def _stop_return_mission(self):
        # disarm return_home node to cancel active return goal and trigger safe stop
        if not self._cli_return_arm.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Return home arm service (/return_home/arm) is not available!')
            return

        req = SetBool.Request()
        req.data = False
        future = self._cli_return_arm.call_async(req)

        # handle disarm response callback
        def _on_arm_done(f):
            try:
                res = f.result()
                self.get_logger().info(f'Return home disarmed: {res.message}')
            except Exception as exc:
                self.get_logger().error(f'Return home disarm call failed: {exc}')
            finally:
                self._broadcast_mission_status('IDLE')

        future.add_done_callback(_on_arm_done)

    def _broadcast_mission_status(self, status):
        msg = json.dumps({'kind': 'mission_status', 'mission': status})
        asyncio.run_coroutine_threadsafe(self._broadcast(msg), self._loop)

    def _set_threshold(self, event_type, level, value):
        # Reuses the same /threshold/set fire-and-forget path an operator's
        # own publisher would use - event_engine's threshold_callback does
        # the actual type/level/value validation and does not care who
        # published it, so comm_relay does no validation of its own here.
        try:
            value = float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(f'set_threshold rejected: {value!r} is not a number')
            return

        self._threshold_pub.publish(String(data=json.dumps({
            'type': event_type,
            'level': level,
            'value': value,
        })))
        self.get_logger().info(f'threshold set requested: {event_type}/{level} = {value}')

        # The publish above is fire-and-forget with no reply, so give
        # event_engine's subscription callback a moment to actually apply
        # it before reading it back - threading.Timer rather than an
        # asyncio call, since this runs on the asyncio loop thread and
        # scheduling loop callbacks from here would need call_soon (not
        # call_later's cross-thread-safe cousin, which is call_soon_threadsafe;
        # a plain stdlib timer sidesteps the distinction entirely).
        threading.Timer(0.3, self._fetch_and_broadcast_settings).start()

    def _set_node_param(self, node_key, param_name, value):
        allowed = SETTABLE_PARAMS.get(node_key)
        if allowed is None or param_name not in allowed:
            self.get_logger().warn(
                f'set_param rejected: {node_key}.{param_name} is not in the allowlist')
            return

        bounds = allowed[param_name]
        pv = ParameterValue()
        try:
            if bounds['type'] == 'bool':
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = bool(value)
            elif bounds['type'] == 'integer':
                value = int(value)
                if not (bounds['min'] <= value <= bounds['max']):
                    raise ValueError(f'out of range [{bounds["min"]}, {bounds["max"]}]')
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = value
            elif bounds['type'] == 'double':
                value = float(value)
                if not (bounds['min'] <= value <= bounds['max']):
                    raise ValueError(f'out of range [{bounds["min"]}, {bounds["max"]}]')
                pv.type = ParameterType.PARAMETER_DOUBLE
                pv.double_value = value
            else:
                raise ValueError(f'unknown allowlist type: {bounds["type"]}')
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f'set_param rejected: {node_key}.{param_name} = {value!r} ({exc})')
            return

        client = self._param_clients[node_key]['set']
        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn(f'{node_key} set_parameters service is not available!')
            return

        req = SetParameters.Request()
        req.parameters = [Parameter(name=param_name, value=pv)]
        future = client.call_async(req)

        def _on_set_done(f):
            try:
                res = f.result()
                ok = bool(res.results) and res.results[0].successful
                if ok:
                    self.get_logger().info(f'{node_key}.{param_name} set to {value}')
                else:
                    reason = res.results[0].reason if res.results else 'no result returned'
                    self.get_logger().warn(
                        f'{node_key}.{param_name} rejected by node: {reason}')
            except Exception as exc:
                self.get_logger().error(f'{node_key} set_parameters call failed: {exc}')
            finally:
                # SetParameters only replies after the node applies it, so
                # unlike _set_threshold's fire-and-forget publish, a
                # re-fetch right here (still on the executor thread that
                # just processed the reply) already sees the new value -
                # no race, no delay needed.
                self._fetch_and_broadcast_settings()

        future.add_done_callback(_on_set_done)

    def _fetch_and_broadcast_settings(self, target=None):
        """Fetches current thresholds + every SETTABLE_PARAMS value, then
        sends one 'settings' envelope - to just `target` when a single
        client is catching up (new connection, get_settings), or broadcast
        to everyone once a set_threshold/set_param command changes
        something, so every open panel stays in sync."""
        result = {'thresholds': None, 'params': {}}
        pending = {'count': 1 + len(SETTABLE_PARAMS)}
        pending_lock = threading.Lock()

        def _maybe_finish():
            with pending_lock:
                pending['count'] -= 1
                done = pending['count'] <= 0
            if not done:
                return
            envelope = json.dumps({'kind': 'settings', 'data': result})
            if target is not None:
                asyncio.run_coroutine_threadsafe(
                    self._send_frames(target, [envelope]), self._loop)
            else:
                asyncio.run_coroutine_threadsafe(self._broadcast(envelope), self._loop)

        if self._cli_threshold_get_all.wait_for_service(timeout_sec=0.5):
            future = self._cli_threshold_get_all.call_async(Trigger.Request())

            def _on_threshold_done(f):
                try:
                    res = f.result()
                    if res.success:
                        result['thresholds'] = json.loads(res.message)
                    else:
                        self.get_logger().warn(f'/threshold/get_all failed: {res.message}')
                except Exception as exc:
                    self.get_logger().error(f'/threshold/get_all call failed: {exc}')
                finally:
                    _maybe_finish()

            future.add_done_callback(_on_threshold_done)
        else:
            self.get_logger().warn('/threshold/get_all service is not available!')
            _maybe_finish()

        for node_key, params in SETTABLE_PARAMS.items():
            client = self._param_clients[node_key]['get']
            param_names = list(params.keys())

            if not client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(f'{node_key} get_parameters service is not available!')
                _maybe_finish()
                continue

            req = GetParameters.Request()
            req.names = param_names
            future = client.call_async(req)

            def _on_params_done(f, node_key=node_key, param_names=param_names):
                try:
                    res = f.result()
                    result['params'][node_key] = {
                        name: _param_value_to_py(val)
                        for name, val in zip(param_names, res.values)
                    }
                except Exception as exc:
                    self.get_logger().error(f'{node_key} get_parameters call failed: {exc}')
                finally:
                    _maybe_finish()

            future.add_done_callback(_on_params_done)

    async def _send_frames(self, client, frames):
        # Sends every frame in the list under that client's own lock, so a
        # multi-frame message (map meta + binary grid) can't be split up by
        # a concurrent broadcast to the same client.
        with self._buffer_lock:
            lock = self._client_locks.get(client)
        if lock is None:
            return
        async with lock:
            for frame in frames:
                await client.send(frame)

    async def _broadcast(self, envelope):
        with self._buffer_lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        results = await asyncio.gather(
            *(self._send_frames(c, [envelope]) for c in clients), return_exceptions=True)
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                with self._buffer_lock:
                    self._ws_clients.discard(client)
                    self._client_locks.pop(client, None)

    async def _broadcast_map(self):
        with self._map_lock:
            if self._latest_map_meta is None:
                return
            meta_json = json.dumps(self._latest_map_meta)
            payload = self._latest_map_bytes

        with self._buffer_lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        results = await asyncio.gather(
            *(self._send_frames(c, [meta_json, payload]) for c in clients),
            return_exceptions=True)
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                with self._buffer_lock:
                    self._ws_clients.discard(client)
                    self._client_locks.pop(client, None)

    # --- ROS side (executor thread) ---

    def _on_battery(self, msg: BatteryState):
        """sensor_msgs/msg/BatteryState 토픽 콜백 -> 최신 데이터 저장"""
        try:
            percentage = round(msg.percentage * 100.0, 1) if not math.isnan(msg.percentage) else 0.0
            voltage = round(float(msg.voltage), 2) if not math.isnan(msg.voltage) else 0.0

            warning_level = 'Normal'
            if percentage <= 10.0:
                warning_level = 'Dead'
            elif percentage <= 20.0:
                warning_level = 'Warning'

            payload = json.dumps({
                'kind': 'drive_battery',
                'data': {
                    'percentage': percentage,
                    'voltage': voltage,
                    'warning_level': warning_level,
                }
            })
            with self._telemetry_lock:
                self._latest_battery_payload = payload
        except Exception as exc:
            self.get_logger().error(f'Error parsing /drive/battery msg: {exc}')

    def _on_drive_status(self, msg: DriveStatus):
        """scout2map_msgs/msg/DriveStatus 토픽 콜백 -> 최신 데이터 저장"""
        try:
            payload = json.dumps({
                'kind': 'drive_status',
                'data': {
                    'link_ok': msg.link_ok,
                    'mcu_reboot_count': msg.mcu_reboot_count,
                    'cmd_timeout': msg.cmd_timeout,
                    'frames_ok': msg.frames_ok,
                    'crc_errors': msg.crc_errors,
                    'batt_warn': msg.batt_warn,
                    'batt_critical': msg.batt_critical,
                    'batt_dead': msg.batt_dead,
                }
            })
            with self._telemetry_lock:
                self._latest_drive_status_payload = payload
        except Exception as exc:
            self.get_logger().error(f'Error parsing /drive/status msg: {exc}')

    def _on_sensor_status(self, msg: SensorStatus):
        """scout2map_msgs/msg/SensorStatus 토픽 콜백 -> 최신 데이터 저장

        sensor_bridge가 보는 Pico 센서-퓨전 MCU 링크/센서 인식 상태. 이걸
        놓치면 (2026-08-29 BH1750 미인식 사례처럼) 조도 센서가 부팅 때부터
        한 번도 값을 못 보냈는데도 event_engine 쪽에서는 조용히
        illuminance_valid=false로만 남아 SSH 없이는 알아챌 방법이 없었다.
        """
        try:
            payload = json.dumps({
                'kind': 'sensor_mcu_status',
                'data': {
                    'port_open': bool(msg.port_open),
                    'link_ok': bool(msg.link_ok),
                    'last_line_age_s': round(float(msg.last_line_age_s), 2),
                    'mcu_reboot_count': msg.mcu_reboot_count,
                    'parse_errors': msg.parse_errors,
                    'framing_overflows': msg.framing_overflows,
                    'aht21_present': bool(msg.aht21_present),
                    'ens160_present': bool(msg.ens160_present),
                    'bh1750_present': bool(msg.bh1750_present),
                    'pms7003_seen': bool(msg.pms7003_seen),
                }
            })
            with self._telemetry_lock:
                self._latest_sensor_status_payload = payload
        except Exception as exc:
            self.get_logger().error(f'Error parsing /sensors/status msg: {exc}')

    _DIAG_LEVEL_NAMES = {
        DiagnosticStatus.OK: 'OK',
        DiagnosticStatus.WARN: 'WARN',
        DiagnosticStatus.ERROR: 'ERROR',
        DiagnosticStatus.STALE: 'STALE',
    }

    def _on_diagnostics(self, msg: DiagnosticArray):
        """diagnostic_msgs/DiagnosticArray 콜백 -> scout_vision 항목만 추출

        /diagnostics는 ekf_filter_node 등 여러 노드가 같이 쓰는 공용 버스라,
        vision_diagnostic_name('scout_vision/inference' 기본값)과 일치하는
        항목만 골라서 캐시한다. 카메라가 프레임을 한 번도 못 받은 상태
        (frame_age_s 없음, 2026-08-29 사례)를 SSH 없이 보여주기 위함.
        """
        try:
            target = None
            for status in msg.status:
                if status.name == self._vision_diagnostic_name:
                    target = status
                    break
            if target is None:
                return

            values = {kv.key: kv.value for kv in target.values}
            level_byte = (
                target.level[0]
                if isinstance(target.level, (bytes, bytearray))
                else target.level
            )
            payload = json.dumps({
                'kind': 'vision_camera_status',
                'data': {
                    'level': self._DIAG_LEVEL_NAMES.get(level_byte, 'UNKNOWN'),
                    'message': target.message,
                    'frame_age_s': values.get('frame_age_s'),
                    'last_latency_ms': values.get('last_latency_ms'),
                    'p95_latency_ms': values.get('p95_latency_ms'),
                    'model_sha256': values.get('model_sha256'),
                }
            })
            with self._telemetry_lock:
                self._latest_vision_status_payload = payload
        except Exception as exc:
            self.get_logger().error(f'Error parsing /diagnostics msg: {exc}')

    def _broadcast_robot_pose(self):
        with self._buffer_lock:
            if not self._ws_clients:
                return

        try:
            now = rclpy.time.Time()
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = self._yaw_from_quaternion(q)

            pose_msg = json.dumps({
                'kind': 'pose',
                'x': tx,
                'y': ty,
                'yaw': yaw
            })

            asyncio.run_coroutine_threadsafe(self._broadcast(pose_msg), self._loop)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        
    def _broadcast_telemetry(self):
        """설정한 주기마다 최신 배터리 및 드라이브 상태를 주기적으로 브로드캐스트"""
        with self._buffer_lock:
            if not self._ws_clients:
                return

        with self._telemetry_lock:
            batt_payload = self._latest_battery_payload
            status_payload = self._latest_drive_status_payload
            sensor_payload = self._latest_sensor_status_payload
            vision_payload = self._latest_vision_status_payload
            self._latest_battery_payload = None
            self._latest_drive_status_payload = None
            self._latest_sensor_status_payload = None
            self._latest_vision_status_payload = None

        if batt_payload:
            asyncio.run_coroutine_threadsafe(self._broadcast(batt_payload), self._loop)
        if status_payload:
            asyncio.run_coroutine_threadsafe(self._broadcast(status_payload), self._loop)
        if sensor_payload:
            asyncio.run_coroutine_threadsafe(self._broadcast(sensor_payload), self._loop)
        if vision_payload:
            asyncio.run_coroutine_threadsafe(self._broadcast(vision_payload), self._loop)

    def _next_seq(self):
        with self._buffer_lock:
            self._seq += 1
            return self._seq

    def _wrap(self, raw_json_str, replay, wall_ts, seq):
        try:
            data = json.loads(raw_json_str)
        except (json.JSONDecodeError, TypeError):
            data = raw_json_str
        return json.dumps({
            'kind': 'event',
            'seq': seq,
            'replay': replay,
            'relayed_at': wall_ts,
            'data': data,
        })

    def _on_event(self, msg: String):
        wall_ts = time.time()
        seq = self._next_seq()

        with self._buffer_lock:
            has_clients = bool(self._ws_clients)

        if has_clients:
            envelope = self._wrap(msg.data, replay=False, wall_ts=wall_ts, seq=seq)
            asyncio.run_coroutine_threadsafe(self._broadcast(envelope), self._loop)
            return

        # No one listening right now, hold onto it until someone reconnects
        with self._buffer_lock:
            self._buffer.append((time.monotonic(), wall_ts, seq, msg.data))
            self._prune_buffer_locked()

    def _next_map_seq(self):
        with self._map_lock:
            self._map_seq += 1
            return self._map_seq

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_return_home_pose(self, msg: PoseStamped):
        payload = json.dumps({
            'kind': 'return_home_goal',
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': self._yaw_from_quaternion(msg.pose.orientation),
        })
        with self._return_home_lock:
            self._latest_return_home_goal = payload

        with self._buffer_lock:
            has_clients = bool(self._ws_clients)
        if has_clients:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _on_return_home_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning('malformed /return_home/status frame, dropping it')
            return

        payload = json.dumps({
            'kind': 'return_home_status',
            'state': data.get('state'),
            'start_captured': data.get('start_captured'),
        })
        with self._return_home_lock:
            self._latest_return_home_status = payload

        with self._buffer_lock:
            has_clients = bool(self._ws_clients)
        if has_clients:
            asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        yaw = self._yaw_from_quaternion(info.origin.orientation)
        if abs(yaw) > 0.01:
            self.get_logger().warning(
                f'/map origin has a non-zero yaw ({yaw:.3f} rad) - the web '
                'marker overlay assumes an unrotated origin for now and '
                'will be slightly off until that is accounted for')

        # OccupancyGrid.data is int8 (-1, 0..100); repack as unsigned bytes
        # (two's complement) so it round-trips through zlib/WebSocket, and
        # let the client convert back (byte > 127 -> byte - 256).
        raw_bytes = bytes(v & 0xFF for v in msg.data)
        compressed = zlib.compress(raw_bytes, self._map_zlib_level)

        meta = {
            'kind': 'map_meta',
            'seq': self._next_map_seq(),
            'encoding': 'zlib-int8-rowmajor-bottomleft',
            'resolution': info.resolution,
            'width': info.width,
            'height': info.height,
            'origin': {
                'x': info.origin.position.x,
                'y': info.origin.position.y,
                'yaw': yaw,
            },
            'raw_bytes': len(raw_bytes),
            'compressed_bytes': len(compressed),
        }

        with self._map_lock:
            self._latest_map_meta = meta
            self._latest_map_bytes = compressed
            self._map_dirty = True

    def _maybe_broadcast_map(self):
        with self._map_lock:
            dirty = self._map_dirty
            self._map_dirty = False
        if dirty:
            asyncio.run_coroutine_threadsafe(self._broadcast_map(), self._loop)

    def _prune_buffer_locked(self):
        # Caller must hold self._buffer_lock
        cutoff = time.monotonic() - self._buffer_max_age_s
        while self._buffer and self._buffer[0][0] < cutoff:
            self._buffer.popleft()
            self._dropped_count += 1
        while len(self._buffer) > self._buffer_max_len:
            self._buffer.popleft()
            self._dropped_count += 1

    def _publish_heartbeat(self):
        # self._link_state is written on the asyncio thread (_on_client)
        # under self._buffer_lock, so read it under the same lock here on
        # the rclpy timer thread rather than racing it.
        with self._buffer_lock:
            link_ok = self._link_state == 'ok'
        if link_ok:
            self._heartbeat_pub.publish(Empty())

    def _publish_status(self):
        with self._buffer_lock:
            state = self._link_state
            client_count = len(self._ws_clients)
            buffered = len(self._buffer)
            dropped = self._dropped_count
            offline_s = (
                round(time.monotonic() - self._last_client_seen_mono, 1)
                if state == 'lost' and self._last_client_seen_mono is not None
                else 0.0
            )

        payload = {
            'state': state,
            'connected_clients': client_count,
            'buffered_count': buffered,
            'dropped_count': dropped,
            'offline_s': offline_s,
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:  # noqa: BLE001 - loop may already be gone
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CommRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()