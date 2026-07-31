import time

import blacknode  # noqa: F401
import pytest
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_motion.arm import servo_control
from blacknode.pkg.blacknode_motion.core import arbitration


class FakeMotionSession:
    def __init__(self, *, warnings=None):
        self.pose = {"shoulder": 10.0, "elbow": -5.0}
        self.warnings = list(warnings or [])
        self.armed = False
        self.closed = False
        self.commands = []

    def sample(self):
        return {
            "pose": dict(self.pose),
            "torque_enabled": self.armed,
            "errors": [],
            "warnings": list(self.warnings),
        }

    def hold(self):
        self.armed = True
        return self.sample()

    def command(self, positions_deg, *, deadline):
        assert deadline > time.monotonic()
        assert self.armed is True
        self.commands.append(dict(positions_deg))
        self.pose.update(positions_deg)
        return self.sample()

    def release(self):
        self.armed = False
        return self.sample()

    def close(self):
        self.closed = True


def _context():
    return {
        "robot_id": "local-usb-test",
        "hardware_id": "usb:test-arm",
        "profile": {
            "id": "test_arm",
            "joints": [
                {"id": "shoulder", "safe_min_deg": -90.0, "safe_max_deg": 90.0},
                {"id": "elbow", "safe_min_deg": -80.0, "safe_max_deg": 80.0},
            ],
            "capability_bindings": {
                "joint_group": {
                    "provider": {
                        "package": "test-motion",
                        "component": "mock-bus",
                    },
                    "configuration": {},
                },
            },
        },
        "calibration": {
            "profile_id": "test_arm",
            "hardware_id": "usb:test-arm",
            "joints": {
                "shoulder": {"home_deg": 0.0},
                "elbow": {"home_deg": 0.0},
            },
        },
    }


@pytest.fixture
def motion_provider():
    sessions = []

    def provider_node(_ctx):
        return {}

    def open_session(_ctx):
        session = FakeMotionSession()
        sessions.append(session)
        return session

    provider_node._bn_robot_joint_motion_provider = {
        "package": "test-motion",
        "component": "mock-bus",
        "capability": "joint_group",
        "open_session": open_session,
    }
    _NODE_REGISTRY["_TestServoMotionProvider"] = provider_node
    yield sessions
    servo_control.stop_runtime_services()
    arbitration.command_arbiter.reset()
    _NODE_REGISTRY.pop("_TestServoMotionProvider", None)


def test_servo_motion_arms_at_feedback_and_commands_one_joint(motion_provider):
    armed = servo_control.arm_servo_motion("servo:test", _context())
    session = motion_provider[0]

    assert armed["armed"] is True
    assert session.armed is True
    moved = servo_control.command_servo_motion("servo:test", {
        "kind": "blacknode.joint-command-request",
        "schema_version": 1,
        "joint_name": "shoulder",
        "servo_id": 1,
        "position_rad": 0.5,
        "issued_at": time.time(),
        "requires_motion_authorization": True,
    })

    assert moved["ok"] is True
    assert moved["armed"] is True
    assert session.commands[-1]["shoulder"] == pytest.approx(28.6478898)
    assert "elbow" not in session.commands[-1]
    shared = servo_control.sample_servo_motion_for_robot("local-usb-test")
    assert shared["torque_enabled"] is True
    assert shared["pose"]["shoulder"] == pytest.approx(28.6478898)

    disarmed = servo_control.disarm_servo_motion("servo:test")
    assert disarmed["ok"] is True
    assert session.armed is False
    assert session.closed is True


def test_servo_motion_rejects_stale_command_without_writing(motion_provider):
    servo_control.arm_servo_motion("servo:stale", _context())
    session = motion_provider[0]

    blocked = servo_control.command_servo_motion("servo:stale", {
        "kind": "blacknode.joint-command-request",
        "schema_version": 1,
        "joint_name": "shoulder",
        "position_rad": 0.25,
        "issued_at": time.time() - 2.0,
        "requires_motion_authorization": True,
    })

    assert blocked["ok"] is False
    assert "stale" in blocked["report"]
    assert session.commands == []
    assert session.armed is False
    assert session.closed is True


def test_servo_motion_requires_hardware_bound_complete_calibration(motion_provider):
    ctx = _context()
    del ctx["calibration"]["joints"]["elbow"]

    with pytest.raises(ValueError, match="cover every configured joint"):
        servo_control.arm_servo_motion("servo:uncalibrated", ctx)

    assert motion_provider == []


def test_servo_motion_warning_blocks_torque_enable():
    session = FakeMotionSession(warnings=["servo shoulder undervoltage"])

    def provider_node(_ctx):
        return {}

    provider_node._bn_robot_joint_motion_provider = {
        "package": "test-motion",
        "component": "mock-bus",
        "capability": "joint_group",
        "open_session": lambda _ctx: session,
    }
    _NODE_REGISTRY["_TestWarningServoMotionProvider"] = provider_node
    try:
        with pytest.raises(RuntimeError, match="hardware warning blocks motion"):
            servo_control.arm_servo_motion("servo:warning", _context())
        assert session.armed is False
        assert session.closed is True
    finally:
        _NODE_REGISTRY.pop("_TestWarningServoMotionProvider", None)
