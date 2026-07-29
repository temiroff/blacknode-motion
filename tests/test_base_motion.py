"""blacknode-motion — mobile-base motion contracts.

All tests run without roslibpy and without a robot: network-touching helpers
are monkeypatched, and the geometry/authorization logic is exercised pure.
"""
import json
import math
import time
from pathlib import Path

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes

_ADAPTER_NODES = Path(__file__).resolve().parents[1] / "components" / "base" / "adapters" / "ros2" / "nodes"
_before = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_motion.base.adapters.ros2", _ADAPTER_NODES)
_tag_new_package_nodes(_before, "blacknode-motion", _ADAPTER_NODES, "base", "ros2")

from blacknode.pkg.blacknode_motion.base.adapters.ros2 import base_motion as bm
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb

NEW_NODES = [
    "BaseSafetyGate",
    "ROS2BaseMove",
    "ROS2BaseStop",
    "ROS2LaserScanCheck",
    "ROS2OdomState",
]


def _fresh_authorization(**overrides):
    ctx = {"armed": True, "max_speed_mps": 0.2, "max_turn_rps": 0.6, "max_duration_s": 1.5}
    ctx.update(overrides)
    return _NODE_REGISTRY["BaseSafetyGate"](ctx)["authorization"]


def test_new_nodes_registered_with_category_and_package():
    for name in NEW_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "Motion"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-motion"


def test_base_template_validates():
    from blacknode.workflow import validate_workflow

    template_dir = _ADAPTER_NODES.parents[0] / "templates"
    for path in sorted(template_dir.glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        report = validate_workflow(workflow)
        assert report.ok, f"{path.name}: {report.to_dict()}"


# --- BaseSafetyGate ---------------------------------------------------------------

def test_gate_blocks_when_disarmed():
    result = _NODE_REGISTRY["BaseSafetyGate"]({})
    assert result["authorized"] is False
    assert result["authorization"]["authorized"] is False
    assert "disarmed" in result["report"]


def test_gate_blocks_on_emergency_stop_even_when_armed():
    result = _NODE_REGISTRY["BaseSafetyGate"]({"armed": True, "emergency_stop": True})
    assert result["authorized"] is False
    assert "emergency stop" in result["report"]


def test_gate_blocks_when_obstacle_inside_clearance():
    result = _NODE_REGISTRY["BaseSafetyGate"](
        {"armed": True, "min_clearance_m": 0.4, "clearance_m": 0.25}
    )
    assert result["authorized"] is False
    assert "0.25" in result["report"]


def test_gate_blocks_when_clearance_required_but_missing():
    result = _NODE_REGISTRY["BaseSafetyGate"]({"armed": True, "require_clearance": True})
    assert result["authorized"] is False
    assert "no clearance reading" in result["report"]


def test_gate_authorizes_and_clamps_caps_to_hard_limits():
    result = _NODE_REGISTRY["BaseSafetyGate"](
        {"armed": True, "max_speed_mps": 9.0, "max_turn_rps": 9.0, "max_duration_s": 99.0, "clearance_m": 1.2}
    )
    assert result["authorized"] is True
    auth = result["authorization"]
    assert auth["max_speed_mps"] == pytest.approx(bm.HARD_MAX_SPEED_MPS)
    assert auth["max_turn_rps"] == pytest.approx(bm.HARD_MAX_TURN_RPS)
    assert auth["max_duration_s"] == pytest.approx(bm.HARD_MAX_DURATION_S)
    assert auth["issued_at"] == pytest.approx(time.time(), abs=5.0)


# --- ROS2BaseMove -----------------------------------------------------------------

def test_base_move_blocks_without_authorization(monkeypatch):
    monkeypatch.setattr(bm, "stream_twist", lambda *a, **k: pytest.fail("must not touch the network"))
    result = _NODE_REGISTRY["ROS2BaseMove"]({})
    assert result["moved"] is False
    assert "no authorization" in result["report"]


def test_base_move_blocks_on_refused_authorization(monkeypatch):
    monkeypatch.setattr(bm, "stream_twist", lambda *a, **k: pytest.fail("must not touch the network"))
    refused = _NODE_REGISTRY["BaseSafetyGate"]({"armed": False})["authorization"]
    result = _NODE_REGISTRY["ROS2BaseMove"]({"authorization": refused})
    assert result["moved"] is False
    assert "authorization refused" in result["report"]


def test_base_move_blocks_on_stale_authorization(monkeypatch):
    monkeypatch.setattr(bm, "stream_twist", lambda *a, **k: pytest.fail("must not touch the network"))
    auth = _fresh_authorization()
    auth["issued_at"] = time.time() - 3600
    result = _NODE_REGISTRY["ROS2BaseMove"]({"authorization": auth})
    assert result["moved"] is False
    assert "stale" in result["report"]


def test_base_move_clamps_velocity_and_duration_then_streams(monkeypatch):
    calls = {}

    def fake_stream(host, port, topic, vx, vy, wz, duration_s, rate_hz, timeout=10.0):
        calls.update(host=host, port=port, topic=topic, vx=vx, vy=vy, wz=wz,
                     duration_s=duration_s, rate_hz=rate_hz)
        return {"ok": True, "sent": 10}

    monkeypatch.setattr(bm, "stream_twist", fake_stream)
    result = _NODE_REGISTRY["ROS2BaseMove"](
        {
            "authorization": _fresh_authorization(),
            "host": "robot.local",
            "cmd_vel_topic": "/controller/cmd_vel",
            "vx": 5.0,
            "vy": 5.0,
            "wz": -9.0,
            "duration_s": 99.0,
        }
    )
    assert result["moved"] is True
    assert calls["topic"] == "/controller/cmd_vel"
    assert math.hypot(calls["vx"], calls["vy"]) == pytest.approx(0.2)
    assert calls["wz"] == pytest.approx(-0.6)
    assert calls["duration_s"] == pytest.approx(1.5)
    assert "clamped" in result["report"]
    assert result["command"]["frames_sent"] == 10


def test_base_move_zero_velocity_stays_zero(monkeypatch):
    calls = {}

    def fake_stream(host, port, topic, vx, vy, wz, duration_s, rate_hz, timeout=10.0):
        calls.update(vx=vx, vy=vy, wz=wz)
        return {"ok": True, "sent": 5}

    monkeypatch.setattr(bm, "stream_twist", fake_stream)
    result = _NODE_REGISTRY["ROS2BaseMove"](
        {"authorization": _fresh_authorization(), "vx": 0.0, "vy": 0.0, "wz": 0.3}
    )
    assert result["moved"] is True
    assert calls["vx"] == 0.0 and calls["vy"] == 0.0
    assert calls["wz"] == pytest.approx(0.3)


def test_base_move_reports_stream_failure(monkeypatch):
    monkeypatch.setattr(bm, "stream_twist", lambda *a, **k: {"ok": False, "sent": 3, "error": "rosbridge disconnected mid-move"})
    result = _NODE_REGISTRY["ROS2BaseMove"]({"authorization": _fresh_authorization()})
    assert result["moved"] is False
    assert "disconnected" in result["report"]
    assert "stop was streamed" in result["report"]


def test_base_stop_needs_no_authorization(monkeypatch):
    calls = {}

    def fake_stop(host, port, topic, timeout=5.0):
        calls.update(topic=topic)
        return {"ok": True, "sent": 3}

    monkeypatch.setattr(bm, "publish_twist_stop", fake_stop)
    result = _NODE_REGISTRY["ROS2BaseStop"]({"cmd_vel_topic": "/controller/cmd_vel"})
    assert result["stopped"] is True
    assert calls["topic"] == "/controller/cmd_vel"


# --- scan / odom geometry ---------------------------------------------------------

def test_sector_min_range_picks_closest_front_beam():
    # 360 one-degree beams from -pi: index 180 is straight ahead (angle 0).
    ranges = [None] * 360
    ranges[180] = 2.0          # dead ahead
    ranges[190] = 0.8          # 10 degrees off — inside a 60 degree sector
    ranges[250] = 0.1          # 70 degrees off — outside the sector
    ranges[181] = float("inf")  # ignored
    message = {
        "ranges": ranges,
        "angle_min": -math.pi,
        "angle_increment": math.radians(1.0),
        "range_min": 0.05,
        "range_max": 12.0,
    }
    best, samples = bm._sector_min_range(message, 60.0)
    assert best == pytest.approx(0.8)
    assert samples == 2


def test_sector_min_range_handles_empty_scan():
    best, samples = bm._sector_min_range({"ranges": []}, 60.0)
    assert best is None
    assert samples == 0


def test_parse_odometry_extracts_pose_and_yaw():
    half = math.sqrt(0.5)
    message = {
        "pose": {"pose": {
            "position": {"x": 1.25, "y": -0.5, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": half, "w": half},  # yaw +90 deg
        }},
        "twist": {"twist": {
            "linear": {"x": 0.1, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": -0.2},
        }},
    }
    position, velocity = bm._parse_odometry(message)
    assert position == {"x": 1.25, "y": -0.5, "yaw_deg": pytest.approx(90.0)}
    assert velocity == {"vx": 0.1, "vy": 0.0, "wz": -0.2}
