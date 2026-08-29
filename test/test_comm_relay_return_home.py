# Unit tests for CommRelayNode's return_home relay: payload shape, the
# cache-for-late-joiners behaviour, and the malformed-status guard.
#
# Like test_comm_relay_map.py, this constructs a real CommRelayNode and
# needs an actual rclpy - not runnable in a plain-Python sandbox with no
# ROS 2 install. The equivalent logic (payload construction, the
# no-clients-yet cache path, the malformed-JSON guard) was verified without
# rclpy by replicating _on_return_home_pose/_on_return_home_status against
# fake inputs before this file was written; see the session notes for that
# smoke test. Run for real via `colcon test` on the robot/dev laptop.
import json
import math
import types

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


def _fake_pose(x, y, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    orientation = types.SimpleNamespace(w=qw, x=0.0, y=0.0, z=qz)
    position = types.SimpleNamespace(x=x, y=y, z=0.0)
    pose = types.SimpleNamespace(position=position, orientation=orientation)
    return types.SimpleNamespace(pose=pose)


def test_return_home_pose_is_cached_even_with_no_clients(node):
    msg = _fake_pose(1.5, -2.25, math.pi / 2)

    node._on_return_home_pose(msg)

    with node._return_home_lock:
        cached = json.loads(node._latest_return_home_goal)
    assert cached['kind'] == 'return_home_goal'
    assert abs(cached['x'] - 1.5) < 1e-9
    assert abs(cached['y'] - (-2.25)) < 1e-9
    assert abs(cached['yaw'] - math.pi / 2) < 1e-6


def test_return_home_status_relays_state_and_start_captured(node):
    msg = types.SimpleNamespace(data=json.dumps({
        'state': 'RETURNING',
        'armed': True,
        'start_captured': True,
        'reason': 'operator requested',
    }))

    node._on_return_home_status(msg)

    with node._return_home_lock:
        cached = json.loads(node._latest_return_home_status)
    assert cached == {
        'kind': 'return_home_status',
        'state': 'RETURNING',
        'start_captured': True,
    }


def test_return_home_status_ignores_malformed_json(node):
    node._on_return_home_status(types.SimpleNamespace(data='not json'))

    with node._return_home_lock:
        assert node._latest_return_home_status is None


def test_return_home_status_malformed_json_does_not_clobber_previous_cache(node):
    good = types.SimpleNamespace(data=json.dumps({'state': 'NORMAL', 'start_captured': True}))
    node._on_return_home_status(good)

    node._on_return_home_status(types.SimpleNamespace(data='not json'))

    with node._return_home_lock:
        cached = json.loads(node._latest_return_home_status)
    assert cached['state'] == 'NORMAL'
