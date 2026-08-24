#!/usr/bin/env python3
# Bridges /events to Web-Monitoring over a small custom WebSocket protocol.
# Unlike rosbridge, this node buffers events while no client is connected and
# replays them in order once a client (re)connects, so a hazard raised during
# a network outage is never silently dropped.
import asyncio
import json
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
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

        self._events_topic = self.get_parameter('events_topic').value
        self._ws_host = self.get_parameter('ws_host').value
        self._ws_port = int(self.get_parameter('ws_port').value)
        self._buffer_max_len = int(self.get_parameter('buffer_max_len').value)
        self._buffer_max_age_s = float(self.get_parameter('buffer_max_age_s').value)
        self._link_status_topic = self.get_parameter('link_status_topic').value
        self._link_status_period_s = float(self.get_parameter('link_status_period_s').value)
        self._ping_interval_s = float(self.get_parameter('ws_ping_interval_s').value)
        self._ping_timeout_s = float(self.get_parameter('ws_ping_timeout_s').value)

        # Buffer entries: (monotonic_ts, wall_ts, seq, raw_json_str)
        # Held only while no web client is connected; see README for the
        # single-logical-consumer assumption behind this design.
        self._buffer = deque()
        self._buffer_lock = threading.Lock()
        self._dropped_count = 0
        self._seq = 0

        self._clients = set()
        self._last_client_seen_mono = None
        self._link_state = 'lost'

        self._status_pub = self.create_publisher(String, self._link_status_topic, 10)
        self._sub = self.create_subscription(
            String, self._events_topic, self._on_event, 50)
        self._status_timer = self.create_timer(
            self._link_status_period_s, self._publish_status)

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
            self._clients.add(websocket)
            self._link_state = 'ok'

        self.get_logger().info(
            f'web client connected ({len(backlog)} buffered, '
            f'{dropped} dropped, offline {offline_s}s)')

        try:
            await websocket.send(json.dumps({
                'kind': 'status',
                'state': 'ok',
                'reconnected': offline_s is not None,
                'offline_s': offline_s,
                'buffered_count': len(backlog),
                'dropped_count': dropped,
            }))

            for _, wall_ts, seq, raw in backlog:
                await websocket.send(self._wrap(raw, replay=True, wall_ts=wall_ts, seq=seq))

            async for _ in websocket:
                # One-way relay (SBC -> web); inbound frames are ignored.
                pass

        except (ConnectionClosed, OSError):
            pass
        except Exception as exc:  # noqa: BLE001 - one bad client must not kill the server
            self.get_logger().warning(f'client session ended with error: {exc}')
        finally:
            with self._buffer_lock:
                self._clients.discard(websocket)
                if not self._clients:
                    self._last_client_seen_mono = time.monotonic()
                    self._link_state = 'lost'

    async def _broadcast(self, envelope):
        with self._buffer_lock:
            clients = list(self._clients)
        if not clients:
            return
        results = await asyncio.gather(
            *(c.send(envelope) for c in clients), return_exceptions=True)
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                with self._buffer_lock:
                    self._clients.discard(client)

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
            has_clients = bool(self._clients)

        if has_clients:
            envelope = self._wrap(msg.data, replay=False, wall_ts=wall_ts, seq=seq)
            asyncio.run_coroutine_threadsafe(self._broadcast(envelope), self._loop)
            return

        # No one listening right now, hold onto it until someone reconnects
        with self._buffer_lock:
            self._buffer.append((time.monotonic(), wall_ts, seq, msg.data))
            self._prune_buffer_locked()

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
            client_count = len(self._clients)
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
