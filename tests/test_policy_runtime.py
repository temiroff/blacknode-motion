"""Policy artifact, safety gate, and managed control contracts."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

import blacknode  # noqa: F401 - discover extension packages
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes

_POLICY_NODES = Path(__file__).resolve().parents[1] / "components" / "policy" / "nodes"
_ADAPTER_NODES = Path(__file__).resolve().parents[1] / "components" / "policy" / "adapters" / "ros2" / "nodes"
_before = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_motion.policy", _POLICY_NODES)
_import_nodes_module("blacknode.pkg.blacknode_motion.policy.adapters.ros2", _ADAPTER_NODES)
_tag_new_package_nodes(_before, "blacknode-motion", _ADAPTER_NODES, "policy", "ros2")

from blacknode.pkg.blacknode_motion.policy import policy_runtime


def _artifact() -> dict:
    return {
        "kind": "blacknode.policy-artifact", "schema_version": 1,
        "policy_type": "act", "backend": "blacknode-native",
        "action_mode": "absolute_joint_position", "units": "radians",
        "joint_names": ["shoulder", "gripper"], "camera_names": ["front"],
        "state_dim": 2, "action_dim": 2, "path": "ignored-by-fake-loader",
    }


def _robot(*, calibrated: bool = True) -> dict:
    return {
        "host": "127.0.0.1", "port": 9090,
        "state_topic": "/follower/joint_states", "command_topic": "/follower/joint_commands",
        "control_topic": "/follower/robot_control",
        "driver": {
            "running": True,
            "calibration_path": "robots/so_arm101/calibrations/serial.json" if calibrated else "",
            "joints": [
                {"id": "shoulder", "safe_min_deg": -90.0, "safe_max_deg": 90.0},
                {"id": "gripper", "safe_min_deg": 0.0, "safe_max_deg": 60.0},
            ],
        },
    }


def _cameras() -> list[dict]:
    return [{"kind": "blacknode.frame-stream", "schema_version": 1, "stream_id": "front", "snapshot_url": "http://camera/snapshot.jpg"}]


def _safety(tmp_path: Path | None = None) -> dict:
    return {
        "kind": "blacknode.policy-safety-gate", "schema_version": 1,
        "max_velocity_deg_s": 30.0, "max_step_deg": 3.0,
        "stale_after": 0.5, "loop_hz": 10.0, "request_timeout": 1.0,
        "require_calibration": True, "workspace_topic": "", "workspace_limits": {},
        "log_dir": str(tmp_path) if tmp_path else "",
    }


def test_policy_nodes_are_registered_and_disarmed_by_default():
    assert "PolicySafetyGate" in _NODE_REGISTRY
    assert "PolicyRuntime" in _NODE_REGISTRY
    assert _NODE_REGISTRY["PolicyRuntime"]._bn_input_defaults["action"] == "status"


def test_contract_requires_calibration_and_exact_camera_and_joint_order():
    contract = policy_runtime.validate_deployment_contract(_artifact(), _robot(), _cameras(), _safety())
    assert contract["joint_names"] == ["shoulder", "gripper"]
    with pytest.raises(ValueError, match="calibration"):
        policy_runtime.validate_deployment_contract(_artifact(), _robot(calibrated=False), _cameras(), _safety())
    with pytest.raises(ValueError, match="missing: front"):
        policy_runtime.validate_deployment_contract(_artifact(), _robot(), [], _safety())


def test_safety_gate_clamps_joint_velocity_and_workspace():
    contract = policy_runtime.validate_deployment_contract(_artifact(), _robot(), _cameras(), _safety())
    gate = policy_runtime.SafetyGate(contract["joint_names"], contract["joint_specs"], _safety())
    result = gate.apply([math.pi, -1.0], {"shoulder": 0.0, "gripper": 0.0}, dt=0.1)
    assert result["ok"]
    assert result["action"]["shoulder"] == pytest.approx(math.radians(3.0))
    assert result["action"]["gripper"] == pytest.approx(0.0)
    assert any("joint_limit" in item for item in result["clamped"])
    workspace_safety = {**_safety(), "workspace_limits": {"x": [0.0, 0.5], "y": [-0.5, 0.5], "z": [0.0, 1.0]}}
    workspace_gate = policy_runtime.SafetyGate(contract["joint_names"], contract["joint_specs"], workspace_safety)
    blocked = workspace_gate.apply([0.0, 0.0], {"shoulder": 0.0, "gripper": 0.0}, dt=0.1, workspace={"x": 0.8, "y": 0.0, "z": 0.2})
    assert not blocked["ok"]
    assert "workspace x" in blocked["reason"]


class _FakePolicy:
    def predict(self, qpos, images):
        assert qpos == [0.0, 0.0]
        assert images["front"].shape == (8, 8, 3)
        return {"kind": "blacknode.policy-prediction", "joint_names": ["shoulder", "gripper"], "action": [1.0, 0.5]}


class _FakeIO:
    def __init__(self, _contract, _cameras, _safety):
        self.commands = []
        self.controls = []

    def start(self):
        return None

    def snapshot(self):
        return {
            "pose": {"shoulder": 0.0, "gripper": 0.0}, "pose_age": 0.01,
            "images": {"front": np.zeros((8, 8, 3), dtype=np.uint8)}, "camera_ages": {"front": 0.01},
            "workspace": {}, "workspace_age": 0.0,
        }

    def publish(self, action):
        self.commands.append(dict(action))

    def control(self, action):
        self.controls.append(action)
        return {"ok": True}

    def close(self):
        return None


class _RejectHoldIO(_FakeIO):
    def control(self, action):
        self.controls.append(action)
        return {"ok": action != "exit_teach", "error": "hold rejected"}


def test_disarmed_preview_arm_sync_estop_and_replay_log(tmp_path: Path):
    run = policy_runtime.PolicyRun(
        "test", _artifact(), _robot(), _cameras(), _safety(tmp_path), device="cpu",
        policy_loader=lambda _artifact, _device: _FakePolicy(), io_factory=_FakeIO,
    )
    run.phase = "preview"
    preview = run.step()
    assert not preview["commanded"]
    assert not run.io.commands
    run.control("arm")
    synchronized = run.step()
    assert synchronized["commanded"]
    assert run.io.commands[-1] == {"shoulder": 0.0, "gripper": 0.0}
    run.last_command_at -= 0.1
    run.step()
    assert run.io.commands[-1]["shoulder"] == pytest.approx(math.radians(3.0))
    run.control("estop")
    assert not run.armed and run.estop
    assert "enter_teach" in run.io.controls
    assert run.log_path.exists()
    assert '"event":"estop"' in run.log_path.read_text(encoding="utf-8")


def test_arm_fails_closed_when_driver_rejects_hold(tmp_path: Path):
    run = policy_runtime.PolicyRun(
        "reject", _artifact(), _robot(), _cameras(), _safety(tmp_path), device="cpu",
        policy_loader=lambda _artifact, _device: _FakePolicy(), io_factory=_RejectHoldIO,
    )
    run.phase = "preview"
    with pytest.raises(RuntimeError, match="hold rejected"):
        run.control("arm")
    assert not run.armed
    assert not run.io.commands
