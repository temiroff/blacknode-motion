"""Blacknode policy safety and managed deployment nodes."""
from __future__ import annotations

import base64
import html
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, List, Text, node

from blacknode.pkg.blacknode_motion.policy import policy_runtime

_CATEGORY = "Motion"


def _dashboard(status: dict[str, Any]) -> str:
    phase = str(status.get("phase") or "unknown").upper()
    color = "#ef4444" if status.get("emergency_stop") or phase == "FAULT" else "#22c55e" if status.get("armed") else "#f59e0b"
    error = html.escape(str(status.get("last_error") or "ready"))[:100]
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="720" height="300" viewBox="0 0 720 300">
<rect width="720" height="300" rx="22" fill="#0b1020"/><rect x="20" y="20" width="680" height="72" rx="16" fill="#172033" stroke="{color}" stroke-width="2"/>
<circle cx="50" cy="56" r="10" fill="{color}"/><text x="72" y="64" fill="#f8fafc" font-family="sans-serif" font-size="24" font-weight="800">POLICY RUNTIME · {html.escape(phase)}</text>
<text x="34" y="132" fill="#94a3b8" font-family="sans-serif" font-size="13">INFERENCES</text><text x="34" y="164" fill="#f8fafc" font-family="monospace" font-size="24">{int(status.get('inference_count') or 0)}</text>
<text x="230" y="132" fill="#94a3b8" font-family="sans-serif" font-size="13">COMMANDS</text><text x="230" y="164" fill="#f8fafc" font-family="monospace" font-size="24">{int(status.get('command_count') or 0)}</text>
<text x="420" y="132" fill="#94a3b8" font-family="sans-serif" font-size="13">MEAN INFERENCE</text><text x="420" y="164" fill="#f8fafc" font-family="monospace" font-size="24">{float(status.get('mean_inference_ms') or 0):.1f} ms</text>
<text x="34" y="214" fill="{color}" font-family="sans-serif" font-size="18" font-weight="800">{'ARMED' if status.get('armed') else 'MOTION DISARMED'}</text>
<text x="34" y="252" fill="#fca5a5" font-family="sans-serif" font-size="13">{error}</text>
<text x="686" y="276" text-anchor="end" fill="#64748b" font-family="sans-serif" font-size="12">Stop, e-stop, takeover, and faults release torque</text></svg>'''
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode("utf-8")).decode("ascii")


@node(
    name="PolicySafetyGate", category=_CATEGORY,
    description="Configure the deployment safety gate: calibrated joint limits, velocity/step bounds, freshness, optional workspace bounds, and replay logs.",
    inputs={
        "trigger": AnyPort,
        "robot": Dict(default={}),
        "max_velocity_deg_s": Float(default=30.0),
        "max_step_deg": Float(default=3.0),
        "stale_after": Float(default=0.5),
        "loop_hz": Float(default=10.0),
        "request_timeout": Float(default=1.0),
        "require_calibration": Bool(default=True),
        "workspace_topic": Text(default=""),
        "workspace_limits": Dict(default={}),
        "log_dir": Text(default=""),
    },
    outputs={"ok": Bool, "safety": Dict, "report": Text},
    primary_inputs=["trigger", "robot"], primary_outputs=["safety", "report"],
)
def policy_safety_gate(ctx: dict) -> dict:
    safety = {
        "kind": "blacknode.policy-safety-gate", "schema_version": 1,
        "max_velocity_deg_s": max(0.0, float(ctx.get("max_velocity_deg_s") or 0.0)),
        "max_step_deg": max(0.0, float(ctx.get("max_step_deg") or 0.0)),
        "stale_after": max(0.05, float(ctx.get("stale_after") or 0.5)),
        "loop_hz": max(1.0, min(60.0, float(ctx.get("loop_hz") or 10.0))),
        "request_timeout": max(0.1, float(ctx.get("request_timeout") or 1.0)),
        "require_calibration": bool(ctx.get("require_calibration", True)),
        "workspace_topic": str(ctx.get("workspace_topic") or "").strip(),
        "workspace_limits": dict(ctx.get("workspace_limits") or {}),
        "log_dir": str(ctx.get("log_dir") or "").strip(),
    }
    robot = dict(ctx.get("robot") or {})
    driver = robot.get("driver") if isinstance(robot.get("driver"), dict) else {}
    calibrated = bool(str(driver.get("calibration_path") or "").strip())
    ok = bool(driver.get("joints")) and (calibrated or not safety["require_calibration"])
    report = "safety gate ready; policy motion remains disarmed until an explicit arm action"
    if not driver.get("joints"):
        report = "safety gate BLOCKED: connect a configured Robot"
    elif safety["require_calibration"] and not calibrated:
        report = "safety gate BLOCKED: save a hardware-bound calibration before deployment"
    return {"ok": ok, "safety": safety, "report": report}


@node(
    name="PolicyRuntime", live=True, category=_CATEGORY,
    description="Preview or continuously execute a policy through a ROS robot interface with explicit arm, disarm, emergency-stop, and human-takeover controls.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["status", "check", "start", "arm", "disarm", "estop", "reset_estop", "takeover", "reset_takeover", "stop"], default="status"),
        "run_id": Text(default="so_arm101_policy"),
        "artifact": Dict(default={}),
        "robot": Dict(default={}),
        "camera_streams": List(default=[]),
        "safety": Dict(default={}),
        "device": Enum(["auto", "cuda", "cpu"], default="auto"),
    },
    outputs={
        "ok": Bool, "running": Bool, "armed": Bool, "emergency_stop": Bool,
        "human_takeover": Bool, "phase": Text, "prediction": Dict, "action": Dict,
        "clamped": List, "metrics": Dict, "dashboard": Image, "log_path": Text, "report": Text,
    },
    primary_inputs=["trigger", "artifact", "robot", "camera_streams", "safety"],
    primary_outputs=["dashboard", "metrics", "report"],
)
def policy_runtime_node(ctx: dict) -> dict:
    run_id = str(ctx.get("run_id") or "so_arm101_policy").strip() or "so_arm101_policy"
    action = str(ctx.get("action") or "status").strip().lower()
    try:
        if action == "status":
            status = policy_runtime.policy_status(run_id)
        elif action == "check":
            contract = policy_runtime.validate_deployment_contract(
                dict(ctx.get("artifact") or {}), dict(ctx.get("robot") or {}),
                list(ctx.get("camera_streams") or []), dict(ctx.get("safety") or {}),
            )
            status = {**policy_runtime.policy_status(run_id), "phase": "ready", "joint_names": contract["joint_names"], "camera_names": contract["camera_names"]}
        elif action == "start":
            status = policy_runtime.start_policy(
                run_id, dict(ctx.get("artifact") or {}), dict(ctx.get("robot") or {}),
                list(ctx.get("camera_streams") or []), dict(ctx.get("safety") or {}),
                str(ctx.get("device") or "auto"),
            )
        else:
            status = policy_runtime.control_policy(run_id, action)
        ok = not bool(status.get("last_error")) and status.get("phase") != "fault"
        report = (
            f"policy {status.get('phase')}: {'ARMED' if status.get('armed') else 'motion disarmed'}; "
            f"{int(status.get('inference_count') or 0)} inference(s), {int(status.get('command_count') or 0)} command(s)"
        )
        if status.get("last_error"):
            report += f"; {status['last_error']}"
        return {
            "ok": ok, "running": bool(status.get("running")), "armed": bool(status.get("armed")),
            "emergency_stop": bool(status.get("emergency_stop")), "human_takeover": bool(status.get("human_takeover")),
            "phase": str(status.get("phase") or "unknown"), "prediction": dict(status.get("last_prediction") or {}),
            "action": dict(status.get("last_action") or {}), "clamped": list(status.get("clamped") or []),
            "metrics": status, "dashboard": _dashboard(status), "log_path": str(status.get("log_path") or ""),
            "report": report,
        }
    except Exception as exc:  # noqa: BLE001
        status = {**policy_runtime.policy_status(run_id), "phase": "fault", "last_error": str(exc)}
        return {
            "ok": False, "running": bool(status.get("running")), "armed": False,
            "emergency_stop": bool(status.get("emergency_stop")), "human_takeover": bool(status.get("human_takeover")),
            "phase": "fault", "prediction": {}, "action": {}, "clamped": [], "metrics": status,
            "dashboard": _dashboard(status), "log_path": str(status.get("log_path") or ""),
            "report": f"policy runtime FAILED: {exc}",
        }
