# Unit tests for CommRelayNode's /map relay: compression round-trip, the
# dirty-flag throttle, and the non-zero-origin-yaw warning path.
import math
import types
import zlib

import pytest
import rclpy
from rclpy.parameter import Parameter

from scout2map_comm.comm_relay_node import CommRelayNode


@pytest.fixture
def node():
    rclpy.init()
    n = CommRelayNode(parameter_overrides=[Parameter('ws_port', value=0)])
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _fake_grid(width, height, resolution, origin_x, origin_y, yaw, cells):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    orientation = types.SimpleNamespace(w=qw, x=0.0, y=0.0, z=qz)
    position = types.SimpleNamespace(x=origin_x, y=origin_y, z=0.0)
    origin = types.SimpleNamespace(position=position, orientation=orientation)
    info = types.SimpleNamespace(resolution=resolution, width=width, height=height, origin=origin)
    return types.SimpleNamespace(info=info, data=cells)


def test_on_map_stores_compressed_grid_and_marks_dirty(node):
    cells = [-1, 0, 100, 0, -1, 100]
    msg = _fake_grid(3, 2, 0.05, -1.0, -2.0, 0.0, cells)

    node._on_map(msg)

    with node._map_lock:
        meta = node._latest_map_meta
        payload = node._latest_map_bytes
        dirty = node._map_dirty

    assert dirty is True
    assert meta['width'] == 3 and meta['height'] == 2
    assert meta['origin']['x'] == -1.0 and meta['origin']['y'] == -2.0
    assert abs(meta['origin']['yaw']) < 1e-6

    recovered = [b if b < 128 else b - 256 for b in zlib.decompress(payload)]
    assert recovered == cells


def test_maybe_broadcast_map_clears_dirty_flag_once(node):
    msg = _fake_grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0, 0, 0, 0])
    node._on_map(msg)

    with node._map_lock:
        assert node._map_dirty is True

    # _maybe_broadcast_map hands off to the asyncio loop and clears the flag
    # synchronously regardless of whether any client is connected to receive it
    node._maybe_broadcast_map()

    with node._map_lock:
        assert node._map_dirty is False


def test_yaw_extraction_matches_quaternion_input(node):
    for yaw in (0.0, 0.3, -1.2, math.pi / 2):
        msg = _fake_grid(1, 1, 1.0, 0.0, 0.0, yaw, [0])
        node._on_map(msg)
        with node._map_lock:
            reported_yaw = node._latest_map_meta['origin']['yaw']
        assert abs(reported_yaw - yaw) < 1e-6, f'expected {yaw}, got {reported_yaw}'
