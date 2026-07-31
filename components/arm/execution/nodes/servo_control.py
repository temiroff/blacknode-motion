"""Managed single-servo motion behind the RobotServo editor control surface."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Mapping

from blacknode.node import _NODE_REGISTRY

from . import arm_controller


_PROVIDER_ATTRIBUTE = "_bn_robot_joint_motion_provider"
_SESSION_LOCK = threading.Lock()
_SESSIONS: dict[str, dict[str, Any]] = {}


def _profile_from_ctx(ctx: Mapping[str, Any]) -> dict[str, Any]:
    profile = ctx.get("profile") if isinstance(ctx.get("profile"), Mapping) else {}
    return dict(profile)


def _provider_binding(profile: Mapping[str, Any]) -> dict[str, Any]:
    bindings = (
        profile.get("capability_bindings")
        if isinstance(profile.get("capability_bindings"), Mapping)
        else {}
    )
    for capability in ("joint_group", "calibration_control", "position_feedback"):
        binding = bindings.get(capability)
        if not isinstance(binding, Mapping):
            continue
        provider = (
            binding.get("provider")
            if isinstance(binding.get("provider"), Mapping)
            else {}
        )
        if provider.get("package") and provider.get("component"):
            return {
                "capability": capability,
                "package": str(provider["package"]),
                "component": str(provider["component"]),
                "configuration": dict(binding.get("configuration") or {}),
            }
    raise ValueError("robot profile does not bind a joint motion provider")


def _provider_specs() -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for fn in _NODE_REGISTRY.values():
        spec = getattr(fn, _PROVIDER_ATTRIBUTE, None)
        if isinstance(spec, Mapping) and callable(spec.get("open_session")):
            result.append(dict(spec))
    return result


def _open_provider_session(ctx: Mapping[str, Any]) -> tuple[Any, dict[str, Any]]:
    binding = _provider_binding(_profile_from_ctx(ctx))
    for provider in _provider_specs():
        if (
            str(provider.get("package") or "") == binding["package"]
            and str(provider.get("component") or "") == binding["component"]
        ):
            provider_ctx = dict(ctx)
            provider_ctx["provider_config"] = {
                **binding["configuration"],
                **dict(ctx.get("provider_config") or {}),
            }
            return provider["open_session"](provider_ctx), {
                "package": binding["package"],
                "component": binding["component"],
                "capability": str(provider.get("capability") or "joint_group"),
                "bound_via": binding["capability"],
            }
    raise RuntimeError(
        "joint motion provider is unavailable for "
        f"{binding['package']}/{binding['component']}"
    )


def _joint_specs(profile: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        str(joint.get("id")): dict(joint)
        for joint in (profile.get("joints") or [])
        if isinstance(joint, Mapping) and str(joint.get("id") or "").strip()
    }


def _validate_calibration(ctx: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    profile = _profile_from_ctx(ctx)
    calibration = (
        ctx.get("calibration")
        if isinstance(ctx.get("calibration"), Mapping)
        else profile.get("calibration")
        if isinstance(profile.get("calibration"), Mapping)
        else {}
    )
    calibration = dict(calibration)
    profile_id = str(profile.get("id") or "").strip()
    hardware_id = str(ctx.get("hardware_id") or "").strip()
    if not profile_id or not hardware_id or not calibration:
        raise ValueError("hardware-bound calibration is required before motion")
    if str(calibration.get("profile_id") or "") != profile_id:
        raise ValueError("calibration does not match the selected robot profile")
    if str(calibration.get("hardware_id") or "") != hardware_id:
        raise ValueError("calibration does not match the connected robot hardware")
    calibrated_joints = calibration.get("joints")
    if not isinstance(calibrated_joints, Mapping) or set(calibrated_joints) != set(_joint_specs(profile)):
        raise ValueError("calibration does not cover every configured joint")
    return hardware_id, calibration


def _sample_error(sample: Mapping[str, Any]) -> str:
    errors = [str(value) for value in (sample.get("errors") or []) if value]
    warnings = [str(value) for value in (sample.get("warnings") or []) if value]
    if errors:
        return "; ".join(errors)
    if warnings:
        return "hardware warning blocks motion: " + "; ".join(warnings)
    return ""


def arm_servo_motion(run_id: str, ctx: Mapping[str, Any]) -> dict[str, Any]:
    """Open the selected provider and enable torque at the measured pose."""
    disarm_servo_motion(run_id)
    profile = _profile_from_ctx(ctx)
    hardware_id, _calibration = _validate_calibration(ctx)
    joints = _joint_specs(profile)
    session, provider = _open_provider_session(ctx)
    try:
        before = dict(session.sample() or {})
        error = _sample_error(before)
        pose = dict(before.get("pose") or {})
        if error:
            raise RuntimeError(error)
        if set(pose) != set(joints):
            raise RuntimeError("complete fresh feedback is required before arming")
        held = dict(session.hold() or {})
        error = _sample_error(held)
        if error:
            raise RuntimeError(error)
        if held.get("torque_enabled") is not True:
            raise RuntimeError("holding torque could not be verified")
    except Exception:
        try:
            session.release()
        except Exception:
            pass
        try:
            session.close()
        except Exception:
            pass
        raise
    item = {
        "session": session,
        "lock": threading.RLock(),
        "armed": True,
        "profile": profile,
        "joints": joints,
        "hardware_id": hardware_id,
        "robot_id": str(ctx.get("robot_id") or ""),
        "provider": provider,
        "last_command_at": time.monotonic(),
        "sample": held,
    }
    with _SESSION_LOCK:
        _SESSIONS[run_id] = item
    return {
        "ok": True,
        "armed": True,
        "pose": dict(held.get("pose") or {}),
        "provider": provider,
        "report": "Servo motion armed at the current measured pose.",
    }


def command_servo_motion(run_id: str, command: Mapping[str, Any]) -> dict[str, Any]:
    """Execute one fresh canonical RobotServo request through motion safety."""
    def blocked(report: str) -> dict[str, Any]:
        disarm_servo_motion(run_id)
        return {"ok": False, "armed": False, "report": f"BLOCKED: {report}"}

    request = dict(command or {})
    try:
        schema_version = int(request.get("schema_version") or 0)
    except (TypeError, ValueError):
        return blocked("invalid joint command")
    if (
        request.get("kind") != "blacknode.joint-command-request"
        or schema_version != 1
        or request.get("requires_motion_authorization") is not True
    ):
        return blocked("invalid joint command")
    try:
        issued_at = float(request["issued_at"])
        requested_rad = float(request["position_rad"])
    except (KeyError, TypeError, ValueError):
        return blocked("invalid joint target")
    if not math.isfinite(requested_rad) or not 0.0 <= time.time() - issued_at <= 0.75:
        return blocked("joint command is stale")
    joint = str(request.get("joint_name") or "").strip()
    with _SESSION_LOCK:
        item = _SESSIONS.get(run_id)
    if item is None or not item.get("armed"):
        return {"ok": False, "armed": False, "report": "BLOCKED: arm Servo motion first"}
    if joint not in item["joints"]:
        return blocked(f"unknown joint '{joint}'")

    with item["lock"]:
        with _SESSION_LOCK:
            if _SESSIONS.get(run_id) is not item or not item.get("armed"):
                return {"ok": False, "armed": False, "report": "BLOCKED: Servo motion is disarmed"}
        sample = dict(item["session"].sample() or {})
        error = _sample_error(sample)
        pose_deg = dict(sample.get("pose") or {})
        if error or sample.get("torque_enabled") is not True or set(pose_deg) != set(item["joints"]):
            disarm_servo_motion(run_id)
            return {
                "ok": False,
                "armed": False,
                "report": "BLOCKED: live feedback or torque verification failed"
                + (f": {error}" if error else ""),
            }
        spec = item["joints"][joint]
        lower_deg = float(spec["safe_min_deg"])
        upper_deg = float(spec["safe_max_deg"])
        lower_rad, upper_rad = sorted((math.radians(lower_deg), math.radians(upper_deg)))
        current_rad = {name: math.radians(float(value)) for name, value in pose_deg.items()}
        elapsed = max(0.001, time.monotonic() - float(item["last_command_at"]))
        velocity_deg = float(spec.get("velocity_limit") or 0.0)

        def publish(safe_target: dict[str, float]) -> Mapping[str, Any]:
            degrees = {
                name: math.degrees(value)
                for name, value in safe_target.items()
            }
            result = dict(
                item["session"].command(
                    degrees,
                    deadline=time.monotonic() + 0.5,
                ) or {}
            )
            provider_error = _sample_error(result)
            if provider_error or result.get("torque_enabled") is not True:
                return {"ok": False, "error": provider_error or "torque verification failed"}
            item["sample"] = result
            return {"ok": True}

        result = arm_controller.execute_joint_target(
            publish,
            resource=item["hardware_id"],
            owner=f"ui:robot-servo:{run_id}",
            current=current_rad,
            target={joint: requested_rad},
            limits={joint: (lower_rad, upper_rad)},
            armed=True,
            feedback_age=0.0,
            max_velocity=(
                math.radians(velocity_deg)
                if velocity_deg > 0.0
                else math.inf
            ),
            interval=elapsed,
        )
        item["last_command_at"] = time.monotonic()
        if not result.get("ok"):
            disarm_servo_motion(run_id)
            return {
                "ok": False,
                "armed": False,
                "report": f"joint move FAILED: {result.get('error', 'unknown error')}",
            }
        target_deg = math.degrees(float(result["target"][joint]))
        return {
            "ok": True,
            "armed": True,
            "commanded": True,
            "joint": joint,
            "target": target_deg,
            "clamped": joint in result.get("clamped", []),
            "report": f"moved {joint} to {target_deg:.2f}°",
        }


def disarm_servo_motion(run_id: str) -> dict[str, Any]:
    """Release torque and close the provider session."""
    with _SESSION_LOCK:
        item = _SESSIONS.pop(run_id, None)
    if item is None:
        return {"ok": True, "armed": False, "report": "Servo motion is disarmed."}
    arm_controller.release_motion_owner(f"ui:robot-servo:{run_id}")
    verified = False
    error = ""
    with item["lock"]:
        try:
            sample = dict(item["session"].release() or {})
            verified = sample.get("torque_enabled") is False
            if not verified:
                error = _sample_error(sample) or "torque release could not be verified"
        except Exception as exc:  # noqa: BLE001
            error = str(exc)
        finally:
            item["armed"] = False
            try:
                item["session"].close()
            except Exception as exc:  # noqa: BLE001
                error = "; ".join(
                    value for value in (error, f"provider close failed: {exc}") if value
                )
                verified = False
    return {
        "ok": verified,
        "armed": False,
        "report": (
            "Servo motion disarmed; torque released."
            if verified
            else f"Servo motion disarmed, but torque release needs attention: {error}"
        ),
    }


def servo_motion_status(run_id: str) -> dict[str, Any]:
    with _SESSION_LOCK:
        item = _SESSIONS.get(run_id)
        if item is None:
            return {"ok": True, "armed": False, "report": "Servo motion is disarmed."}
        return {
            "ok": True,
            "armed": bool(item.get("armed")),
            "provider": dict(item.get("provider") or {}),
            "sample": dict(item.get("sample") or {}),
            "report": "Servo motion is armed.",
        }


def sample_servo_motion_for_robot(robot_id: str) -> dict[str, Any] | None:
    """Share the armed provider session with RobotMonitor to avoid bus races."""
    with _SESSION_LOCK:
        match = next(
            (
                (run_id, item)
                for run_id, item in _SESSIONS.items()
                if str(item.get("robot_id") or "") == str(robot_id or "")
            ),
            None,
        )
    if match is None:
        return None
    run_id, item = match
    try:
        with item["lock"]:
            sample = dict(item["session"].sample() or {})
            item["sample"] = sample
    except Exception as exc:  # noqa: BLE001
        disarm_servo_motion(run_id)
        return {
            "data_ready": False,
            "command_ok": False,
            "pose": {},
            "torque_enabled": False,
            "errors": [str(exc)],
            "warnings": [],
            "servos": {},
            "diagnostics": {},
            "report": f"Servo motion feedback failed; motion was disarmed: {exc}",
        }
    error = _sample_error(sample)
    if error:
        disarm_servo_motion(run_id)
        sample["torque_enabled"] = False
    return {
        **sample,
        "data_ready": bool(sample.get("pose")),
        "command_ok": not error,
        "report": (
            f"{error}; Servo motion was disarmed."
            if error
            else "Servo motion armed; live provider feedback is shared with RobotMonitor."
        ),
    }


def stop_runtime_services() -> dict[str, Any]:
    with _SESSION_LOCK:
        run_ids = list(_SESSIONS)
    failures = [
        result["report"]
        for result in (disarm_servo_motion(run_id) for run_id in run_ids)
        if not result.get("ok")
    ]
    return {
        "ok": not failures,
        "stopped": {"managed_runs": len(run_ids)},
        "report": (
            f"stopped {len(run_ids)} Servo motion session(s)"
            + (f"; {'; '.join(failures)}" if failures else "")
        ),
    }
