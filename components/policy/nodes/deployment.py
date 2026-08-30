"""Policy qualification and physical-deployment approval contracts."""
from __future__ import annotations

import hashlib
import json
import time
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Text, node


_CATEGORY = "Motion"


def _canonical_digest(value: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def qualification_digest(qualification: dict[str, Any]) -> str:
    payload = {
        key: value for key, value in qualification.items()
        if key not in {"qualification_digest", "path"}
    }
    return _canonical_digest(payload)


def robot_identity(robot: dict[str, Any]) -> dict[str, Any]:
    driver = robot.get("driver") if isinstance(robot.get("driver"), dict) else {}
    joints = driver.get("joints") if isinstance(driver.get("joints"), list) else []
    joint_names = [
        str(item.get("id")) for item in joints
        if isinstance(item, dict) and str(item.get("id") or "").strip()
    ]
    calibration_path = str(driver.get("calibration_path") or "").strip()
    if not joint_names:
        raise ValueError("connect a configured robot with ordered joints")
    if not calibration_path:
        raise ValueError("physical deployment requires hardware-bound calibration")
    identity = {
        "profile": str(
            robot.get("profile") or robot.get("robot_profile")
            or driver.get("profile") or driver.get("robot_profile") or ""
        ),
        "hardware_id": str(
            robot.get("hardware_id") or driver.get("hardware_id")
            or driver.get("serial") or calibration_path
        ),
        "calibration_path": calibration_path,
        "joint_names": joint_names,
    }
    identity["digest"] = _canonical_digest(identity)
    return identity


def safety_digest(safety: dict[str, Any]) -> str:
    if safety.get("kind") != "blacknode.policy-safety-gate":
        raise ValueError("connect a blacknode.policy-safety-gate")
    return _canonical_digest(dict(safety))


def create_deployment_authorization(
    artifact: dict[str, Any],
    qualification: dict[str, Any],
    robot: dict[str, Any],
    safety: dict[str, Any],
) -> dict[str, Any]:
    if artifact.get("kind") != "blacknode.policy-artifact":
        raise ValueError("connect a blacknode.policy-artifact")
    artifact_digest = str(artifact.get("artifact_digest") or "")
    if len(artifact_digest) != 64:
        raise ValueError("policy artifact must include its model-and-contract digest")
    artifact_safety = artifact.get("safety") if isinstance(artifact.get("safety"), dict) else {}
    if (
        artifact_safety.get("simulation_only") is not True
        or artifact_safety.get("physical_motion_authorized") is not False
    ):
        raise ValueError("deployment approval expects an immutable, simulation-only policy artifact")
    if qualification.get("kind") != "blacknode.policy-qualification":
        raise ValueError("connect a blacknode.policy-qualification")
    if qualification.get("passed") is not True:
        raise ValueError("policy qualification has not passed")
    if str(qualification.get("artifact_digest") or "") != artifact_digest:
        raise ValueError("qualification belongs to a different policy artifact")
    expected_qualification_digest = qualification_digest(qualification)
    if str(qualification.get("qualification_digest") or "") != expected_qualification_digest:
        raise ValueError("qualification digest is invalid")
    identity = robot_identity(robot)
    artifact_joints = [str(name) for name in artifact.get("joint_names") or []]
    if identity["joint_names"] != artifact_joints:
        raise ValueError("robot joint order does not match the qualified policy")
    artifact_profile = str(artifact.get("robot_profile") or "")
    if artifact_profile and identity["profile"] and artifact_profile != identity["profile"]:
        raise ValueError("robot profile does not match the qualified policy")
    authorization = {
        "kind": "blacknode.policy-deployment-authorization", "schema_version": 1,
        "issued_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "artifact_digest": artifact_digest,
        "qualification_digest": expected_qualification_digest,
        "robot_identity": identity,
        "safety_digest": safety_digest(safety),
        "physical_motion_authorized": True,
        "starts_disarmed": True,
    }
    authorization["authorization_digest"] = _canonical_digest(authorization)
    return authorization


def validate_deployment_authorization(
    authorization: dict[str, Any], artifact: dict[str, Any], robot: dict[str, Any], safety: dict[str, Any]
) -> dict[str, Any]:
    if authorization.get("kind") != "blacknode.policy-deployment-authorization":
        raise ValueError("physical PPO deployment requires explicit deployment authorization")
    payload = {key: value for key, value in authorization.items() if key != "authorization_digest"}
    if str(authorization.get("authorization_digest") or "") != _canonical_digest(payload):
        raise ValueError("deployment authorization digest is invalid")
    if authorization.get("physical_motion_authorized") is not True:
        raise ValueError("deployment authorization does not permit physical motion")
    if str(authorization.get("artifact_digest") or "") != str(artifact.get("artifact_digest") or ""):
        raise ValueError("deployment authorization belongs to a different policy")
    identity = robot_identity(robot)
    authorized_identity = dict(authorization.get("robot_identity") or {})
    if authorized_identity.get("digest") != identity["digest"]:
        raise ValueError("deployment authorization belongs to a different robot or calibration")
    if str(authorization.get("safety_digest") or "") != safety_digest(safety):
        raise ValueError("deployment authorization belongs to a different safety configuration")
    return dict(authorization)


@node(
    name="PolicyDeploymentAuthorize", component="policy", category=_CATEGORY,
    description=(
        "Approve one qualified policy for one calibrated robot and safety configuration. "
        "The runtime still starts disarmed and requires a separate arm action."
    ),
    inputs={
        "trigger": AnyPort, "action": Enum(["check", "authorize"], default="check"),
        "artifact": Dict(default={}), "qualification": Dict(default={}),
        "robot": Dict(default={}), "safety": Dict(default={}),
        "authorize_physical_motion": Bool(default=False),
    },
    outputs={"ok": Bool, "authorized": Bool, "authorization": Dict, "report": Text},
    primary_inputs=["trigger", "artifact", "qualification", "robot", "safety"],
    primary_outputs=["authorization", "report"],
)
def policy_deployment_authorize(ctx: dict[str, Any]) -> dict[str, Any]:
    try:
        action = str(ctx.get("action") or "check").lower()
        artifact = dict(ctx.get("artifact") or {})
        qualification = dict(ctx.get("qualification") or {})
        robot = dict(ctx.get("robot") or {})
        safety = dict(ctx.get("safety") or {})
        if action == "check":
            create_deployment_authorization(artifact, qualification, robot, safety)
            return {
                "ok": True, "authorized": False, "authorization": {},
                "report": "qualified deployment is ready; choose authorize and confirm physical motion",
            }
        if not bool(ctx.get("authorize_physical_motion", False)):
            raise ValueError("authorize_physical_motion must be explicitly enabled")
        authorization = create_deployment_authorization(
            artifact, qualification, robot, safety
        )
        return {
            "ok": True, "authorized": True, "authorization": authorization,
            "report": "policy approved for this robot and safety configuration; runtime remains disarmed",
        }
    except Exception as exc:  # noqa: BLE001
        return {
            "ok": False, "authorized": False, "authorization": {},
            "report": f"policy deployment authorization BLOCKED: {exc}",
        }
