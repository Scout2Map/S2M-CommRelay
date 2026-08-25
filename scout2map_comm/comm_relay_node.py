#!/usr/bin/env python3
# Bridges /events to Web-Monitoring over a small custom WebSocket protocol.
# Unlike rosbridge, this node buffers events while no client is connected and
# replays them in order once a client (re)connects, so a hazard raised during
# a network outage is never silently dropped.
import asyncio
import json
import math
import threading
import time
import zlib
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

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
        self.declare_parameter('ws_ping_interval_s', 10.0)
        self.declare_parameter('ws_ping_timeout_s', 10.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_relay_period_s', 2.0)
        self.declare_parameter('map_zlib_level', 6)

        self._events_topic = self.get_parameter('events_topic').value
        self._ws_host = self.get_parameter('ws_host').value
        self._ws_port = int(self.get_parameter('ws_port').value)
        self._buffer_max_len = int(self.get_parameter('buffer_max_len').value)
        self._buffer_max_age_s = float(self.get_parameter('buffer_max_age_s').value)
        self._link_status_topic = self.get_parameter('link_status_topic').value
        self._link_status_period_s = float(self.get_parameter('link_status_period_s').value)
        self._ping_interval_s = float(self.get_parameter('ws_ping_interval_s').value)
        self._ping_timeout_s = float(self.get_parameter('ws_ping_timeout_s').value)
        self._map_topic = self.get_parameter('map_topic').value
        self._map_relay_period_s = float(self.get_parameter('map_relay_period_s').value)
        self._map_zlib_level = int(self.get_parameter('map_zlib_level').value)

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

        self._status_pub = self.create_publisher(String, self._link_status_topic, 10)
        self._sub = self.create_subscription(
            String, self._events_topic, self._on_event, 50)
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
        self._status_timer = self.create_timer(
            self._link_status_period_s, self._publish_status)
        self._map_timer = self.create_timer(
            self._map_relay_period_s, self._maybe_broadcast_map)

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

            async for _ in websocket:
                # One-way relay (SBC -> web); inbound frames are ignored.
                pass

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
