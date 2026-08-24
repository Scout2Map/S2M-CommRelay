# Unit tests for CommRelayNode's buffer pruning logic.
# This only covers the pure buffering behaviour. The websocket server itself
# needs a live client to exercise the connect/replay path - see README
# "테스트 방법" for a manual wscat/browser-console check of that part.
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from scout2map_comm.comm_relay_node import CommRelayNode


@pytest.fixture
def node():
    rclpy.init()
    # ws_port=0 lets the OS pick a free port so repeated test runs never
    # collide with a leftover bind from a previous run.
    n = CommRelayNode(parameter_overrides=[Parameter('ws_port', value=0)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_buffer_keeps_entries_under_the_limits(node):
    node._buffer_max_len = 3
    node._buffer_max_age_s = 3600.0
    for i in range(3):
        with node._buffer_lock:
            node._buffer.append((time.monotonic(), time.time(), i, f'{{"seq":{i}}}'))
            node._prune_buffer_locked()
    assert len(node._buffer) == 3
    assert node._dropped_count == 0


def test_buffer_drops_oldest_over_max_len(node):
    node._buffer_max_len = 2
    node._buffer_max_age_s = 3600.0
    for i in range(5):
        with node._buffer_lock:
            node._buffer.append((time.monotonic(), time.time(), i, f'{{"seq":{i}}}'))
            node._prune_buffer_locked()
    assert len(node._buffer) == 2
    assert node._dropped_count == 3
    kept_seqs = [item[2] for item in node._buffer]
    assert kept_seqs == [3, 4]


def test_buffer_drops_stale_entries_by_age(node):
    node._buffer_max_len = 100
    node._buffer_max_age_s = 0.05
    with node._buffer_lock:
        node._buffer.append((time.monotonic() - 10.0, time.time() - 10.0, 0, '{}'))
        node._buffer.append((time.monotonic(), time.time(), 1, '{}'))
        node._prune_buffer_locked()
    assert len(node._buffer) == 1
    assert node._buffer[0][2] == 1


def test_wrap_marks_replay_flag(node):
    live = node._wrap('{"type": "HIGH_TEMP"}', replay=False, wall_ts=1.0, seq=1)
    replayed = node._wrap('{"type": "HIGH_TEMP"}', replay=True, wall_ts=1.0, seq=2)
    assert '"replay": false' in live
    assert '"replay": true' in replayed
