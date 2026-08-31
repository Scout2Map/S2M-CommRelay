# Unit tests for the link-quality auto-adjust logic (RTT tier classification
# + hysteresis, and the telemetry-broadcast skip counter). The actual ping
# round trip and the SetParameters calls to scout_vision need a live
# websocket client and a live scout_vision node respectively - both are
# monkeypatched out here so these tests stay fast and self-contained; see
# README "테스트 방법" for a manual end-to-end check of the full path.
import pytest
import rclpy
from rclpy.parameter import Parameter

from scout2map_comm.comm_relay_node import CommRelayNode


@pytest.fixture
def node():
    rclpy.init()
    n = CommRelayNode(parameter_overrides=[Parameter('ws_port', value=0)])
    n._link_quality_hysteresis_checks = 2
    n._link_rtt_good_ms = 250.0
    n._link_rtt_degraded_ms = 800.0
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_tier_does_not_flip_on_a_single_bad_reading(node, monkeypatch):
    applied = []
    monkeypatch.setattr(node, '_apply_link_quality_profile', applied.append)
    monkeypatch.setattr(node, '_broadcast_link_quality', lambda tier, rtt: None)

    node._update_link_quality_tier(900.0)

    assert node._link_quality_tier == 'good'
    assert applied == []


def test_tier_flips_to_degraded_after_the_hysteresis_streak(node, monkeypatch):
    applied = []
    monkeypatch.setattr(node, '_apply_link_quality_profile', applied.append)
    monkeypatch.setattr(node, '_broadcast_link_quality', lambda tier, rtt: None)

    node._update_link_quality_tier(900.0)
    node._update_link_quality_tier(900.0)

    assert node._link_quality_tier == 'degraded'
    assert applied == ['degraded']


def test_dead_zone_reading_resets_the_pending_streak(node, monkeypatch):
    monkeypatch.setattr(node, '_apply_link_quality_profile', lambda tier: None)
    monkeypatch.setattr(node, '_broadcast_link_quality', lambda tier, rtt: None)

    node._update_link_quality_tier(900.0)  # 1st bad reading
    node._update_link_quality_tier(500.0)  # dead zone - neither confirms nor holds the streak
    node._update_link_quality_tier(900.0)  # only the 1st again, not the 2nd

    assert node._link_quality_tier == 'good'


def test_dead_zone_reading_is_a_no_op_when_already_stable(node, monkeypatch):
    calls = []
    monkeypatch.setattr(node, '_apply_link_quality_profile', calls.append)
    monkeypatch.setattr(node, '_broadcast_link_quality', lambda tier, rtt: None)

    node._update_link_quality_tier(500.0)

    assert node._link_quality_tier == 'good'
    assert calls == []


def test_tier_recovers_to_good_after_the_hysteresis_streak(node, monkeypatch):
    applied = []
    monkeypatch.setattr(node, '_apply_link_quality_profile', applied.append)
    monkeypatch.setattr(node, '_broadcast_link_quality', lambda tier, rtt: None)
    node._link_quality_tier = 'degraded'

    node._update_link_quality_tier(100.0)
    assert node._link_quality_tier == 'degraded'
    node._update_link_quality_tier(100.0)

    assert node._link_quality_tier == 'good'
    assert applied == ['good']


def test_broadcast_telemetry_holds_the_payload_until_the_send_tick(node):
    # A fake, unregistered client: non-empty _ws_clients is enough to pass
    # _broadcast_telemetry's early-return guard. _broadcast()'s own
    # _send_frames() no-ops on a client with no entry in _client_locks, so
    # nothing actually tries to touch the network.
    node._ws_clients = {object()}
    node._telemetry_skip_every = 3
    node._telemetry_tick = 0
    node._latest_battery_payload = '{"kind": "drive_battery"}'

    node._broadcast_telemetry()  # tick 1/3
    with node._telemetry_lock:
        assert node._latest_battery_payload is not None

    node._broadcast_telemetry()  # tick 2/3
    with node._telemetry_lock:
        assert node._latest_battery_payload is not None

    node._broadcast_telemetry()  # tick 3/3 - reads and clears it
    with node._telemetry_lock:
        assert node._latest_battery_payload is None


def test_broadcast_telemetry_sends_every_tick_when_not_degraded(node):
    node._ws_clients = {object()}
    node._telemetry_skip_every = 1
    node._telemetry_tick = 0
    node._latest_battery_payload = '{"kind": "drive_battery"}'

    node._broadcast_telemetry()

    with node._telemetry_lock:
        assert node._latest_battery_payload is None
