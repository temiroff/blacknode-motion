"""blacknode-motion — arm motion contracts and ROS 2 control surfaces.

All tests run without rclpy/roslibpy and without a robot: transport helpers
are monkeypatched, and the arming/clamping/dashboard logic is exercised pure.
"""
import base64
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

import blacknode  # noqa: F401  triggers package discovery
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes

_ARM_NODES = (
    Path(__file__).resolve().parents[1]
    / "components" / "arm" / "trajectory" / "nodes"
)
_ADAPTER_NODES = Path(__file__).resolve().parents[1] / "components" / "arm" / "adapters" / "ros2" / "nodes"
_before = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_motion.arm", _ARM_NODES)
_tag_new_package_nodes(_before, "blacknode-motion", _ARM_NODES, "arm")
_before_adapter = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_motion.arm.adapters.ros2", _ADAPTER_NODES)
_tag_new_package_nodes(
    _before_adapter, "blacknode-motion", _ADAPTER_NODES, "arm", "ros2",
)

from blacknode.pkg.blacknode_motion.arm import motion_profiles as mp
from blacknode.pkg.blacknode_motion.arm.adapters.ros2 import joint_motion as jm
from blacknode.pkg.blacknode_ros2 import ros2_native_runtime as nr
from blacknode.pkg.blacknode_ros2 import rosbridge_runtime as rb

NEW_NODES = [
    "JointMotionProfile",
    "ROS2JointSliders",
    "ROS2JointState",
    "ROS2ManualMove",
    "ROS2MotionDashboard",
    "ROS2SetJoint",
]


@pytest.mark.parametrize("mode", ["direct", "linear", "trapezoidal", "minimum_jerk"])
def test_joint_motion_profile_node_previews_supported_modes(mode):
    result = _NODE_REGISTRY["JointMotionProfile"]({
        "mode": mode,
        "distance": 30.0,
        "units": "degrees",
        "min_duration": 0.8,
        "max_velocity": 90.0,
        "max_acceleration": 180.0,
        "rate_hz": 50.0,
    })

    assert result["profile"]["kind"] == "blacknode.joint-motion-profile"
    assert result["profile"]["mode"] == mode
    assert result["samples"][0]["position"] == pytest.approx(0.0)
    assert result["samples"][-1]["position"] == pytest.approx(30.0)
    assert result["preview"].startswith("data:image/svg+xml;base64,")
    assert mode in result["report"]


def test_trapezoidal_profile_is_monotonic_and_respects_limits():
    profile = mp.canonical_profile(
        mode="trapezoidal",
        units="degrees",
        min_duration=0.0,
        max_velocity=60.0,
        max_acceleration=120.0,
        rate_hz=100.0,
    )
    plan = mp.plan_profile(profile, math.radians(10.0))

    assert plan["alphas"][0] == 0.0
    assert plan["alphas"][-1] == 1.0
    assert all(a <= b for a, b in zip(plan["alphas"], plan["alphas"][1:]))
    assert plan["peak_velocity_rad_s"] <= math.radians(60.0) + 1e-9
    assert plan["peak_acceleration_rad_s2"] <= math.radians(120.0) + 1e-9


def test_minimum_jerk_profile_starts_and_ends_at_rest():
    profile = mp.canonical_profile(
        mode="minimum_jerk",
        units="radians",
        min_duration=1.0,
        max_velocity=2.0,
        max_acceleration=4.0,
        rate_hz=100.0,
    )
    plan = mp.plan_profile(profile, 0.5)

    assert plan["samples"][0]["velocity"] == pytest.approx(0.0)
    assert plan["samples"][0]["acceleration"] == pytest.approx(0.0)
    assert plan["samples"][-1]["velocity"] == pytest.approx(0.0)
    assert plan["samples"][-1]["acceleration"] == pytest.approx(0.0)


def test_joint_sliders_read_joints_and_move_only_when_armed(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "no rclpy"))
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: {"shoulder_pan": 0.0, "gripper": math.radians(10.0)})
    monkeypatch.setattr(rb, "read_config", lambda *a, **k: {"joints": {
        "shoulder_pan": {"lower": math.radians(-90), "upper": math.radians(90)},
    }})
    moves = []
    monkeypatch.setattr(rb, "stream_motion",
                        lambda host, port, cmd, names, start, target, **k:
                        moves.append(dict(target)) or {"ok": True})

    # Cook the node (disarmed) — it reports joints + limits for the UI.
    out = _NODE_REGISTRY["ROS2JointSliders"]({
        "run_id": "t_sliders", "host": "h", "port": 9090, "units": "degrees", "armed": False,
    })
    names = [j["name"] for j in out["joints"]]
    assert names == ["shoulder_pan", "gripper"]
    pan = next(j for j in out["joints"] if j["name"] == "shoulder_pan")
    assert pan["min"] == pytest.approx(-90.0) and pan["max"] == pytest.approx(90.0)

    # Disarmed: a slider push is refused.
    blocked = jm.set_joint_slider_targets("t_sliders", {"shoulder_pan": 30.0})
    assert blocked["ok"] is False and moves == []

    # Arm, then a push moves the joint, clamped to limits.
    jm.set_joint_slider_armed("t_sliders", True)
    ok = jm.set_joint_slider_targets("t_sliders", {"shoulder_pan": 200.0})
    assert ok["ok"] is True and len(moves) == 1
    assert moves[0]["shoulder_pan"] == pytest.approx(math.radians(90.0))  # clamped to upper limit


def test_new_nodes_registered_with_category_and_package():
    for name in NEW_NODES:
        assert name in _NODE_REGISTRY, name
        assert _NODE_REGISTRY[name]._bn_category == "Motion"
        assert _NODE_REGISTRY[name]._bn_package == "blacknode-motion"


# --- ROS2SetJoint / ROS2JointState ------------------------------------------------

def test_generic_set_joint_auto_rosbridge_preview_never_writes(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "missing rclpy"))
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "read_config", lambda *args, **kwargs: {
        "commands_allowed": True,
        "limits": {"shoulder_pan": {"lower": -1.0, "upper": 1.0}},
    })
    monkeypatch.setattr(rb, "read_pose", lambda *args, **kwargs: {"shoulder_pan": 0.25})
    monkeypatch.setattr(rb, "stream_motion", lambda *args, **kwargs: pytest.fail("disarmed preview must not write"))

    result = _NODE_REGISTRY["ROS2SetJoint"]({
        "transport": "auto",
        "joint": "shoulder_pan",
        "position": 30.0,
        "units": "degrees",
        "armed": False,
        "config_topic": "/joint_config",
    })

    assert result["moved"] is False
    assert result["before"]["shoulder_pan"] == pytest.approx(math.degrees(0.25))
    assert result["target"]["shoulder_pan"] == pytest.approx(30.0)
    assert "PREVIEW (not armed)" in result["report"]


def test_set_joint_uses_wired_robot_transport_and_topics(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "missing rclpy"))
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    captured = {}
    poses = iter([
        {"shoulder_pan": 0.0},
        {"shoulder_pan": math.radians(20.0)},
    ])

    def fake_read_config(host, port, topic, timeout):
        captured["config"] = (host, port, topic)
        return {
            "commands_allowed": True,
            "joints": {
                "shoulder_pan": {
                    "lower": math.radians(-90.0),
                    "upper": math.radians(90.0),
                },
            },
        }

    def fake_read_pose(host, port, topic, timeout):
        captured.setdefault("poses", []).append((host, port, topic))
        return next(poses)

    def fake_stream(host, port, topic, names, start, target, **kwargs):
        captured["command"] = (host, port, topic)
        captured["motion"] = kwargs
        return {"ok": True, "sent": 10}

    monkeypatch.setattr(rb, "read_config", fake_read_config)
    monkeypatch.setattr(rb, "read_pose", fake_read_pose)
    monkeypatch.setattr(rb, "stream_motion", fake_stream)

    result = _NODE_REGISTRY["ROS2SetJoint"]({
        "robot": {
            "ready": True,
            "host": "robot.local",
            "port": 9191,
            "state_topic": "/arm/state",
            "command_topic": "/arm/command",
            "config_topic": "/arm/config",
            "units": "degrees",
            "interface": {"kind": "rosbridge"},
        },
        "joint": "shoulder_pan",
        "position": 20.0,
        "armed": True,
    })

    assert result["moved"] is True
    assert captured["config"] == ("robot.local", 9191, "/arm/config")
    assert captured["poses"] == [
        ("robot.local", 9191, "/arm/state"),
        ("robot.local", 9191, "/arm/state"),
    ]
    assert captured["command"] == ("robot.local", 9191, "/arm/command")
    assert captured["motion"]["alphas"][0] == 0.0
    assert captured["motion"]["alphas"][-1] == 1.0
    assert captured["motion"]["rate_hz"] == 30.0


def test_set_joint_consumes_wired_trapezoidal_profile(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "missing rclpy"))
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    poses = iter([{"shoulder_pan": 0.0}, {"shoulder_pan": math.radians(30.0)}])
    monkeypatch.setattr(rb, "read_pose", lambda *args, **kwargs: next(poses))
    monkeypatch.setattr(rb, "read_config", lambda *args, **kwargs: {
        "commands_allowed": True,
        "joints": {
            "shoulder_pan": {
                "lower": math.radians(-90.0),
                "upper": math.radians(90.0),
            },
        },
    })
    captured = {}

    def fake_stream(*args, **kwargs):
        captured.update(kwargs)
        return {"ok": True, "sent": len(kwargs["alphas"])}

    monkeypatch.setattr(rb, "stream_motion", fake_stream)
    profile = mp.canonical_profile(
        mode="trapezoidal",
        units="degrees",
        min_duration=0.5,
        max_velocity=60.0,
        max_acceleration=120.0,
        rate_hz=50.0,
    )
    result = _NODE_REGISTRY["ROS2SetJoint"]({
        "transport": "rosbridge",
        "joint": "shoulder_pan",
        "position": 30.0,
        "units": "degrees",
        "config_topic": "/joint_config",
        "motion_profile": profile,
        "armed": True,
    })

    assert result["moved"] is True
    assert captured["alphas"][0] == 0.0
    assert captured["alphas"][-1] == 1.0
    assert captured["rate_hz"] == 50.0
    assert "trapezoidal profile" in result["report"]


def test_set_joint_blocks_when_wired_robot_driver_is_not_ready(monkeypatch):
    monkeypatch.setattr(rb, "stream_motion", lambda *args, **kwargs: pytest.fail("unready robot must not write"))

    result = _NODE_REGISTRY["ROS2SetJoint"]({
        "robot": {
            "ready": False,
            "error": "robot driver is not running",
            "interface": {"kind": "rosbridge"},
        },
        "joint": "shoulder_pan",
        "position": 20.0,
        "armed": True,
    })

    assert result["moved"] is False
    assert "robot driver is not running" in result["report"]
    assert "Start the Robot node" in result["report"]


def test_native_joint_state_reads_pose(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {"gripper": math.radians(45.0)})

    result = jm.ros2_native_joint_state({"units": "degrees"})

    assert result["pose"]["gripper"] == 45.0
    assert result["names"] == ["gripper"]
    assert "native rclpy" in result["report"]


def test_native_set_joint_previews_live_pose_when_disarmed(monkeypatch):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("disarmed must never stream motion commands")

    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {
        "shoulder_pan": math.radians(-11.6015625), "gripper": math.radians(25.0),
    })
    monkeypatch.setattr(nr, "stream_motion", fail_if_called)

    result = jm.ros2_native_set_joint({
        "joint": "shoulder_pan",
        "position": 0.0,
        "units": "degrees",
        "armed": False,
    })

    assert result["moved"] is False
    assert result["report"].startswith("PREVIEW")
    assert math.isclose(result["before"]["shoulder_pan"], -11.6015625, abs_tol=1e-6)
    assert result["after"] == result["before"]
    assert math.isclose(result["target"]["shoulder_pan"], 0.0, abs_tol=1e-6)


def test_native_set_joint_streams_absolute_target(monkeypatch):
    start = {"shoulder_pan": 0.0, "gripper": math.radians(25.0)}
    after = {"shoulder_pan": 0.0, "gripper": math.radians(60.0)}
    poses = iter([start, after])
    captured = {}

    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: next(poses))
    monkeypatch.setattr(nr, "read_config", lambda *a, **k: {
        "commands_allowed": True,
        "joints": {
            "gripper": {
                "lower": math.radians(0.0),
                "upper": math.radians(90.0),
            },
        },
    })

    def fake_stream(command_topic, names, s, t, **kwargs):
        captured["command_topic"] = command_topic
        captured["names"] = names
        captured["target"] = t
        return {"ok": True, "sent": 40}

    monkeypatch.setattr(nr, "stream_motion", fake_stream)

    result = jm.ros2_native_set_joint({
        "joint": "gripper",
        "position": 60.0,
        "units": "degrees",
        "config_topic": "/joint_config",
        "armed": True,
    })

    assert result["moved"] is True
    assert captured["command_topic"] == "/joint_commands"
    assert captured["names"] == ["shoulder_pan", "gripper"]
    assert math.isclose(captured["target"]["gripper"], math.radians(60.0), abs_tol=1e-6)
    assert "native set gripper" in result["report"]


def test_joint_state_derives_connection_from_wired_robot(monkeypatch):
    # One 'robot' edge should drive host/port/topic/units — no hand-matched params.
    monkeypatch.setattr(nr, "available", lambda: (False, "missing rclpy"))
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    captured = {}
    monkeypatch.setattr(rb, "read_pose", lambda host, port, topic, timeout: captured.update(
        host=host, port=port, topic=topic) or {"shoulder_pan": math.radians(12.0)})

    result = _NODE_REGISTRY["ROS2JointState"]({
        "robot": {
            "host": "192.168.1.50", "port": 9091, "state_topic": "/arm/joint_states",
            "units": "degrees", "interface": {"kind": "rosbridge"},
        },
    })

    assert captured == {"host": "192.168.1.50", "port": 9091, "topic": "/arm/joint_states"}
    assert result["pose"]["shoulder_pan"] == pytest.approx(12.0)  # degrees, from robot.units


def test_nodes_structured_error_without_transports(monkeypatch):
    monkeypatch.setattr(nr, "available", lambda: (False, "rclpy is not importable"))
    monkeypatch.setattr(rb, "available", lambda: (False, "roslibpy is not installed"))

    state = _NODE_REGISTRY["ROS2JointState"]({})
    assert state["pose"] == {}
    assert "FAILED" in state["report"]

    set_joint = _NODE_REGISTRY["ROS2SetJoint"]({"joint": "gripper", "armed": True})
    assert set_joint["moved"] is False
    assert "FAILED" in set_joint["report"]


# --- ROS2ManualMove ---------------------------------------------------------------

def test_manual_move_releases_torque_and_keeps_pose_visible(monkeypatch):
    published = []
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "publish_string", lambda topic, value, timeout: published.append((topic, value)) or {"ok": True})
    monkeypatch.setattr(nr, "read_config", lambda *a, **k: {
        "commands_allowed": False,
        "torque_enabled": False,
        "teach_mode": True,
        "last_error": "",
    })
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {"shoulder_pan": math.radians(12.5)})

    result = _NODE_REGISTRY["ROS2ManualMove"]({
        "action": "release",
        "transport": "native",
        "units": "degrees",
    })

    assert result["live"] is True
    assert result["mode"] == "released"
    assert result["torque_enabled"] is False
    assert math.isclose(result["pose"]["shoulder_pan"], 12.5)
    assert json.loads(published[0][1]) == {"action": "enter_teach"}
    assert "support the arm" in result["report"]


def test_manual_move_hold_reports_safe_acknowledgement(monkeypatch):
    monkeypatch.setattr(rb, "available", lambda: (True, ""))
    monkeypatch.setattr(rb, "publish_string", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(rb, "read_config", lambda *a, **k: {
        "commands_allowed": True,
        "torque_enabled": True,
        "teach_mode": False,
        "last_error": "",
    })
    monkeypatch.setattr(rb, "read_pose", lambda *a, **k: {"gripper": math.radians(20.0)})

    result = _NODE_REGISTRY["ROS2ManualMove"]({"action": "hold", "transport": "rosbridge"})

    assert result["live"] is True
    assert result["mode"] == "hold"
    assert result["torque_enabled"] is True
    assert "live pose monitoring is active" in result["report"]


def test_manual_move_dashboard_separates_control_from_robot_state():
    dashboard = jm._teach_dashboard(
        {"shoulder_pan": 12.5},
        "degrees",
        False,
        "Holding current pose.",
        action="check",
        live=True,
    )
    svg = base64.b64decode(dashboard.split(",", 1)[1]).decode("utf-8")

    assert "CONTROL: MONITOR ONLY • LIVE" in svg
    assert "ROBOT: HOLDING • TORQUE ON" in svg


def test_existing_manual_monitor_receives_confirmed_release_config(monkeypatch):
    seeded = []
    session = SimpleNamespace(seed_config=lambda config: seeded.append(dict(config)))
    item = {
        "ctx": {"action": "hold"},
        "outputs": {"torque_enabled": True},
        "dashboard_baselines": {"dashboard": {"shoulder_pan": 0.0}},
        "session": session,
    }
    monkeypatch.setattr(jm, "_teach_monitors", {"manual": item})
    confirmed = {"torque_enabled": False, "teach_mode": True, "mode": "teach"}

    jm._start_teach_monitor(
        "manual",
        {"action": "release"},
        {"torque_enabled": False, "mode": "released"},
        confirmed,
    )

    assert seeded == [confirmed]
    assert item["confirmed_config"] == confirmed
    assert item["ctx"]["action"] == "release"
    assert item["outputs"]["torque_enabled"] is False
    assert item["dashboard_baselines"] == {}


def test_manual_monitor_discards_stale_joint_subscription(monkeypatch):
    class StopAfterOneTick:
        def __init__(self):
            self.calls = 0

        def wait(self, _timeout):
            self.calls += 1
            return self.calls > 1

    session = SimpleNamespace(
        snapshot=lambda: ({"shoulder_pan": 0.0}, {"torque_enabled": False}, 9.0),
    )
    released = []
    monkeypatch.setattr(
        rb,
        "release_joint_stream",
        lambda value, **kwargs: released.append((value, kwargs)),
    )
    item = {
        "ctx": {"transport": "rosbridge"},
        "session": session,
        "stop": StopAfterOneTick(),
        "outputs": {},
    }

    jm._teach_monitor_worker("manual", item)

    assert released == [(session, {"discard": True})]
    assert item["session"] is None
    assert "subscription stale" in item["error"]


def test_native_manual_monitor_reuses_one_read_only_subscription(monkeypatch):
    class StopAfterTwoTicks:
        def __init__(self):
            self.calls = 0

        def wait(self, _timeout):
            self.calls += 1
            return self.calls > 2

    session = SimpleNamespace(
        wait_for_pose=lambda _timeout: {"shoulder_pan": 0.0},
        wait_for_config=lambda _timeout: {"torque_enabled": False},
        seed_config=lambda _config: None,
        snapshot=lambda: (
            {"shoulder_pan": 0.0},
            {"torque_enabled": False},
            0.01,
        ),
    )
    acquired = []
    released = []
    monkeypatch.setattr(
        nr,
        "acquire_joint_stream",
        lambda *args, **kwargs: acquired.append((args, kwargs)) or session,
    )
    monkeypatch.setattr(
        nr,
        "release_joint_stream",
        lambda value, **kwargs: released.append((value, kwargs)),
    )
    monkeypatch.setattr(
        nr,
        "read_pose",
        lambda *args, **kwargs: pytest.fail("monitor must not create one-shot read nodes"),
    )
    monkeypatch.setattr(
        nr,
        "read_config",
        lambda *args, **kwargs: pytest.fail("monitor must not create one-shot read nodes"),
    )
    item = {
        "ctx": {
            "transport": "native",
            "state_topic": "/leader/joint_states",
            "config_topic": "/leader/joint_config",
        },
        "session": None,
        "stop": StopAfterTwoTicks(),
        "outputs": {},
    }

    jm._teach_monitor_worker("leader_pose", item)
    jm._release_teach_session(item)

    assert acquired == [(
        (
            "/leader/joint_states",
            "",
            "/leader/joint_config",
        ),
        {
            "timeout": 2.0,
            "node_name": "blacknode_manual_move_leader_pose",
        },
    )]
    assert released == [(session, {"discard": False})]


def test_manual_move_run_once_returns_snapshot_without_monitor(monkeypatch):
    started = []
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "read_config", lambda *a, **k: {"torque_enabled": False, "last_error": ""})
    monkeypatch.setattr(nr, "read_pose", lambda *a, **k: {"shoulder_pan": math.radians(7.0)})
    monkeypatch.setattr(jm, "_start_teach_monitor", lambda *a, **k: started.append(a))
    monkeypatch.setattr(jm, "_stop_teach_monitor", lambda *a, **k: None)

    result = _NODE_REGISTRY["ROS2ManualMove"]({
        "action": "check",
        "transport": "native",
        "units": "degrees",
        "__run_mode__": "once",
    })

    assert result["live"] is False
    assert result["updated_at"].startswith("snapshot ")
    assert math.isclose(result["pose"]["shoulder_pan"], 7.0)
    assert "one-time pose snapshot" in result["report"]
    assert started == []


def test_manual_move_can_release_without_starting_a_live_monitor(monkeypatch):
    started = []
    monkeypatch.setattr(nr, "available", lambda: (True, ""))
    monkeypatch.setattr(nr, "publish_string", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(
        nr,
        "read_config",
        lambda *a, **k: {"torque_enabled": False, "last_error": ""},
    )
    monkeypatch.setattr(
        nr,
        "read_pose",
        lambda *a, **k: {"shoulder_pan": math.radians(7.0)},
    )
    monkeypatch.setattr(jm, "_start_teach_monitor", lambda *a, **k: started.append(a))
    monkeypatch.setattr(jm, "_stop_teach_monitor", lambda *a, **k: None)

    result = _NODE_REGISTRY["ROS2ManualMove"]({
        "action": "release",
        "transport": "native",
        "live_monitor": False,
    })

    assert result["live"] is False
    assert "one-time pose snapshot" in result["report"]
    assert started == []


# --- ROS2MotionDashboard ----------------------------------------------------------

def test_motion_dashboard_renders_before_after():
    before = {"shoulder_pan": 0.0, "gripper": 25.0}
    after = {"shoulder_pan": 0.0, "gripper": 60.0}
    result = _NODE_REGISTRY["ROS2MotionDashboard"]({
        "joint": "gripper",
        "before": before,
        "after": after,
        "target": {"gripper": 60.0},
        "moved": True,
        "units": "degrees",
    })
    assert result["dashboard"].startswith("data:image/svg+xml;base64,")
    assert result["summary"]["delta"] == 35.0
    assert result["summary"]["moved"] is True


def test_motion_dashboard_shows_live_pose_when_motion_data_is_empty():
    pose = {"shoulder_pan": 12.5, "gripper": 25.0}
    result = _NODE_REGISTRY["ROS2MotionDashboard"]({
        "joint": "",
        "pose": pose,
        "before": {},
        "after": {},
        "target": {},
        "moved": False,
        "units": "degrees",
    })
    assert result["dashboard"].startswith("data:image/svg+xml;base64,")
    assert result["summary"]["moved"] is False
    assert result["summary"]["joints"] == ["gripper", "shoulder_pan"]
    assert result["summary"]["positions"] == pose
    assert result["summary"]["before_values"] == pose
    assert result["summary"]["after_values"] == pose


def test_motion_dashboard_renders_live_pose_state():
    pose = {"shoulder_pan": 12.5, "gripper": 25.0}
    result = _NODE_REGISTRY["ROS2MotionDashboard"]({
        "pose": pose,
        "before": {"shoulder_pan": 10.0, "gripper": 20.0},
        "units": "degrees",
        "__live_pose__": True,
    })

    assert result["live"] is True
    assert result["summary"]["live"] is True
    assert result["summary"]["positions"] == pose
    assert result["summary"]["before_values"] == {"shoulder_pan": 10.0, "gripper": 20.0}


def test_live_motion_dashboard_keeps_first_pose_as_baseline():
    graph = type("GraphStub", (), {})()
    graph._edges = [{"from": "manual", "from_port": "pose", "to": "dashboard", "to_port": "pose"}]
    graph._nodes = {"dashboard": {"type": "ROS2MotionDashboard", "params": {}}}
    ctx = {"__graph__": graph, "__node_id__": "manual"}
    item = {}

    jm._live_motion_dashboard_outputs(ctx, {"shoulder_pan": 10.0}, "degrees", False, item)
    outputs = jm._live_motion_dashboard_outputs(ctx, {"shoulder_pan": 15.0}, "degrees", False, item)

    summary = outputs["dashboard"]["summary"]
    assert summary["live"] is True
    assert summary["mode"] == "released"
    assert summary["torque_enabled"] is False
    assert summary["joint"] == "shoulder_pan"
    assert summary["delta"] == 5.0
    assert summary["before_values"] == {"shoulder_pan": 10.0}
    assert summary["positions"] == {"shoulder_pan": 15.0}
    svg = base64.b64decode(outputs["dashboard"]["dashboard"].split(",", 1)[1]).decode("utf-8")
    assert "MOST CHANGED JOINT" in svg
    assert "CHANGE SINCE LIVE START" in svg
    assert "TARGET" not in svg


def test_live_pose_pushes_into_robot_connection_dashboard(monkeypatch):
    received = []
    monkeypatch.setitem(
        jm._NODE_REGISTRY,
        "RobotConnectionDashboard",
        lambda ctx: received.append(dict(ctx)) or {
            "dashboard": "data:image/svg+xml;base64,dGVzdA==",
            "summary": {"profile_id": "my_so_arm101", "pose": dict(ctx["pose"])},
        },
    )
    graph = type("GraphStub", (), {})()
    graph._edges = [
        {"from": "manual", "from_port": "pose", "to": "connection", "to_port": "pose"},
        {"from": "discovery", "from_port": "robot", "to": "connection", "to_port": "robot"},
        {"from": "status", "from_port": "ready", "to": "connection", "to_port": "interface_ready"},
    ]
    graph._nodes = {
        "connection": {"type": "RobotConnectionDashboard", "params": {"connected": True}},
    }
    graph._cache = {
        ("discovery", "robot"): {"driver": {"profile_id": "my_so_arm101"}},
        ("status", "ready"): True,
    }
    ctx = {"__graph__": graph, "__node_id__": "manual"}

    outputs = jm._live_motion_dashboard_outputs(
        ctx, {"shoulder_pan": 21.5}, "degrees", False, {}
    )

    assert outputs["connection"]["summary"]["pose"] == {"shoulder_pan": 21.5}
    assert received[0]["robot"]["driver"]["profile_id"] == "my_so_arm101"
    assert received[0]["interface_ready"] is True


def test_live_pose_pushes_into_connected_robot_calibration_recorder(monkeypatch):
    received = []
    monkeypatch.setitem(
        jm._NODE_REGISTRY,
        "RobotCalibrationRecorder",
        lambda ctx: received.append(dict(ctx)) or {
            "active": True,
            "samples": 3,
            "observed": {"shoulder_pan": {"min_deg": -5.0, "max_deg": 10.0}},
            "report": "recording",
        },
    )
    graph = type("GraphStub", (), {})()
    graph._edges = [
        {"from": "manual", "from_port": "pose", "to": "calibration", "to_port": "pose"},
        {"from": "profile", "from_port": "profile", "to": "calibration", "to_port": "profile"},
        {"from": "usb", "from_port": "recommended", "to": "calibration", "to_port": "hardware"},
    ]
    graph._nodes = {
        "calibration": {
            "type": "RobotCalibrationRecorder",
            "params": {"run_id": "custom_robot_calibration", "safety_margin_deg": 4.0},
        },
    }
    graph._cache = {
        ("profile", "profile"): {"id": "my_robot", "joints": []},
        ("usb", "recommended"): {"serial": "ABC123"},
    }
    ctx = {"__graph__": graph, "__node_id__": "manual"}

    outputs = jm._live_motion_dashboard_outputs(
        ctx, {"shoulder_pan": 12.0}, "degrees", False, {}
    )

    assert outputs["calibration"]["active"] is True
    assert received[0]["action"] == "_sample"
    assert received[0]["pose"] == {"shoulder_pan": 12.0}
    assert received[0]["torque_enabled"] is False
    assert received[0]["profile"]["id"] == "my_robot"
    assert received[0]["hardware"]["serial"] == "ABC123"


def test_templates_validate():
    from blacknode.workflow import validate_workflow

    template_dir = _ADAPTER_NODES.parents[0] / "templates"
    for path in sorted(template_dir.glob("*.json")):
        workflow = json.loads(path.read_text(encoding="utf-8"))
        assert "ROS2DemoPublisher" not in {
            meta.get("type")
            for meta in workflow.get("node_meta", {}).values()
        }
        report = validate_workflow(workflow)
        assert report.ok, f"{path.name}: {report.to_dict()}"


def test_move_joint_template_uses_real_driver_and_explicit_basic_inputs():
    template = json.loads(
        (_ADAPTER_NODES.parents[0] / "templates" / "ros2-robot-joint-move.json").read_text(encoding="utf-8")
    )
    nodes = template["node_meta"]
    edges = template["edges"]

    assert "sim_robot" not in nodes
    assert nodes["robot"]["type"] == "Robot"
    assert nodes["robot"]["params"]["action"] == "start"
    assert nodes["joint_name"]["type"] == "Text"
    assert nodes["joint_name"]["params"]["value"] == "shoulder_pan"
    assert nodes["arm_motion"]["type"] == "Bool"
    assert nodes["arm_motion"]["params"]["value"] is False
    assert nodes["arm_motion"]["params"]["label"] == "Armed"
    assert nodes["position_angle"]["type"] == "Float"
    assert nodes["position_angle"]["params"] == {
        "value": 35.0,
        "label": "Position angle",
    }
    assert nodes["motion_profile"]["type"] == "JointMotionProfile"
    assert nodes["motion_profile"]["params"]["mode"] == "trapezoidal"
    assert nodes["move"]["params"]["armed"] is False
    assert {
        (edge["from"], edge["from_port"], edge["to"], edge["to_port"])
        for edge in edges
    } >= {
        ("robot", "robot", "pose", "robot"),
        ("robot", "robot", "move", "robot"),
        ("joint_name", "value", "move", "joint"),
        ("arm_motion", "value", "move", "armed"),
        ("position_angle", "value", "move", "position"),
        ("motion_profile", "profile", "move", "motion_profile"),
    }
