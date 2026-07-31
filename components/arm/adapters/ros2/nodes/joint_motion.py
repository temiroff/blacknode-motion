"""Universal joint-space robot control over ROS 2 transports.

These drive **any** robot that exposes ``sensor_msgs/msg/JointState`` over a
ROS 2 graph: native ``rclpy`` when available, rosbridge WebSocket otherwise.
Topics, joint name, and units are all inputs, so the same nodes work for any
joint-based robot — robot specifics live in templates, not in the nodes.

Motion is gated: command nodes do nothing unless explicitly armed, sync to the
current pose before moving, clamp to limits when a config topic provides them,
and stream a heartbeat so a robot driver's own timeout still applies.

Transport preflight (``ROS2Status``) and the graph/topic primitives live in
``blacknode-ros2/core``; this adapter only adds joint-space control on top.
"""
from __future__ import annotations

import base64
import html
import json
import math
import threading
import time
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, _NODE_REGISTRY, node

from blacknode.pkg.blacknode_motion.arm.motion_profiles import (
    canonical_profile,
    plan_profile,
)
from blacknode.pkg.blacknode_motion.arm import arm_controller


class _LazyNativeRuntime:
    """Delay ROS 2 imports so dependency discovery can load in any folder order."""

    def __getattr__(self, name: str):
        from blacknode.pkg.blacknode_ros2 import ros2_native_runtime

        return getattr(ros2_native_runtime, name)


class _LazyRosbridgeRuntime:
    def __getattr__(self, name: str):
        from blacknode.pkg.blacknode_ros2 import rosbridge_runtime

        return getattr(rosbridge_runtime, name)


nr = _LazyNativeRuntime()
rb = _LazyRosbridgeRuntime()

_CATEGORY = "Motion"
_teach_monitor_lock = threading.Lock()
_teach_monitors: dict[str, dict[str, Any]] = {}


def _apply_robot_descriptor(ctx: dict) -> dict:
    """Derive the ROS connection from a wired Robot descriptor.

    When a Robot node's ``robot`` output is connected, take its host, port,
    joint-state topic, units, and transport instead of hand-matched params — so
    one edge (robot.robot -> node.robot) is enough and can never drift from the
    Robot node's own settings. Falls through to the explicit params when nothing
    is wired.
    """
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    if not robot:
        return ctx
    merged = dict(ctx)
    if robot.get("host"):
        merged["host"] = robot["host"]
    if robot.get("port"):
        merged["port"] = robot["port"]
    if robot.get("state_topic"):
        merged["topic"] = robot["state_topic"]
        merged["state_topic"] = robot["state_topic"]
    if robot.get("command_topic"):
        merged["command_topic"] = robot["command_topic"]
    if robot.get("config_topic"):
        merged["config_topic"] = robot["config_topic"]
    if robot.get("units"):
        merged["units"] = robot["units"]
    interface = robot.get("interface") if isinstance(robot.get("interface"), dict) else {}
    kind = str(interface.get("kind") or "")
    if kind in {"native", "rosbridge"}:
        merged["transport"] = kind
    return merged


def _resolve_transport(ctx: dict) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    if requested in {"native", "rosbridge"}:
        return requested
    native_ok, _ = nr.available()
    return "native" if native_ok else "rosbridge"


def _transport_report(ctx: dict, resolved: str) -> str:
    requested = str(ctx.get("transport") or "auto").strip().lower()
    suffix = " (auto-selected)" if requested == "auto" else ""
    return f"transport: {resolved}{suffix}"


def _svg_text(value: Any, limit: int = 90) -> str:
    text = " ".join(str(value or "").split())
    if len(text) > limit:
        text = text[: limit - 3] + "..."
    return html.escape(text)


def _svg_data(svg: str) -> str:
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


def _teach_dashboard(
    pose: dict[str, Any],
    units: str,
    teach_mode: bool,
    report: str = "",
    *,
    action: str = "check",
    live: bool = False,
    updated_at: str = "",
) -> str:
    accent = "#f59e0b" if teach_mode else "#22c55e"
    robot_state = "RELEASED • TORQUE OFF" if teach_mode else "HOLDING • TORQUE ON"
    control = {
        "check": "MONITOR ONLY",
        "status": "MONITOR ONLY",
        "release": "RELEASE + LIVE POSE",
        "enter": "RELEASE + LIVE POSE",
        "hold": "HOLD POSITION",
        "exit": "HOLD POSITION",
    }.get(str(action or "check").strip().lower(), str(action or "check").strip().upper())
    runtime_state = "LIVE" if live else "SNAPSHOT"
    rows = []
    for index, (name, value) in enumerate(sorted(pose.items())[:8]):
        y = 190 + index * 48
        value_text = f"{value:.2f}" if isinstance(value, (int, float)) else "-"
        rows.append(
            f'<text x="54" y="{y}" fill="#f8fafc" font-family="monospace" font-size="17">{_svg_text(name, 24)}</text>'
            f'<text x="696" y="{y}" text-anchor="end" fill="{accent}" font-family="monospace" font-size="18" font-weight="700">{value_text}</text>'
        )
    if not rows:
        rows.append('<text x="380" y="260" text-anchor="middle" fill="#93a4b8" font-family="Arial,sans-serif" font-size="18">Waiting for joint positions…</text>')
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="760" height="620" viewBox="0 0 760 620">
<rect width="760" height="620" rx="24" fill="#0b1020"/>
<rect x="24" y="24" width="712" height="112" rx="18" fill="#172033" stroke="{accent}" stroke-width="2"/>
<text x="48" y="56" fill="#f8fafc" font-family="Arial,sans-serif" font-size="23" font-weight="800">MANUAL MOVE + LIVE POSE</text>
<text x="48" y="87" fill="#cbd5e1" font-family="Arial,sans-serif" font-size="15" font-weight="700">CONTROL: {_svg_text(control, 32)} • {runtime_state}</text>
<text x="48" y="116" fill="{accent}" font-family="Arial,sans-serif" font-size="16" font-weight="800">ROBOT: {robot_state}</text>
<text x="54" y="158" fill="#93a4b8" font-family="Arial,sans-serif" font-size="12" font-weight="700">JOINT</text>
<text x="696" y="158" text-anchor="end" fill="#93a4b8" font-family="Arial,sans-serif" font-size="12" font-weight="700">POSITION ({_svg_text(units, 12)})</text>
{''.join(rows)}
<text x="48" y="582" fill="#93a4b8" font-family="Arial,sans-serif" font-size="13">{_svg_text(report, 92)}</text>
<text x="712" y="582" text-anchor="end" fill="#64748b" font-family="monospace" font-size="12">{_svg_text(updated_at, 24)}</text>
</svg>"""
    return _svg_data(svg)


def _to_radians(value: float, units: str) -> float:
    return math.radians(value) if units == "degrees" else value


def _from_radians(value: float, units: str) -> float:
    return math.degrees(value) if units == "degrees" else value


def _motion_plan(ctx: dict, distance_rad: float) -> dict[str, Any]:
    """Resolve a wired profile contract or build one from this node's inputs."""
    wired = ctx.get("motion_profile")
    if isinstance(wired, dict) and wired:
        profile = dict(wired)
    else:
        profile = canonical_profile(
            mode=str(ctx.get("profile_mode") or "linear"),
            units=str(ctx.get("units") or "degrees"),
            min_duration=float(ctx.get("ramp_seconds") or 0.0),
            max_velocity=float(ctx.get("max_velocity") or 90.0),
            max_acceleration=float(ctx.get("max_acceleration") or 180.0),
            rate_hz=float(ctx.get("rate_hz") or 30.0),
        )
    return plan_profile(profile, distance_rad)


def ros2_native_joint_state(ctx: dict) -> dict:
    """Read the current pose from a JointState topic through native rclpy."""
    ok, err = nr.available()
    if not ok:
        return {"pose": {}, "names": [], "report": f"native joint state FAILED: {err}"}
    topic = str(ctx.get("topic") or "/joint_states")
    units = str(ctx.get("units") or "radians")
    timeout = float(ctx.get("timeout") or 10.0)
    try:
        pose_rad = nr.read_pose(topic, timeout)
    except Exception as exc:
        return {"pose": {}, "names": [], "report": f"native joint state FAILED: {exc}"}
    if not pose_rad:
        return {"pose": {}, "names": [], "report": f"no JointState on {topic} within {timeout:g}s - is the ROS 2 robot driver running?"}
    pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
    summary = ", ".join(f"{name} {pose[name]:.2f}" for name in pose)
    return {"pose": pose, "names": list(pose.keys()), "report": f"{len(pose)} joints ({units}) via native rclpy: {summary}"}


def ros2_native_set_joint(ctx: dict) -> dict:
    """Set one joint to an absolute position through native rclpy (armed-gated)."""
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    armed = bool(ctx.get("armed", False))
    blocked = {"moved": False, "joint": joint, "before": {}, "after": {}, "target": {}}
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to a joint name (discover them with ROS2JointState)."}
    ok, err = nr.available()
    if not ok:
        return {**blocked, "report": f"native set {joint} FAILED: {err}"}

    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    position = float(ctx.get("position") or 0.0)
    ramp_seconds = float(ctx.get("ramp_seconds") or 0.8)
    hold_seconds = float(ctx.get("hold_seconds") or 0.2)
    rate_hz = float(ctx.get("rate_hz") or 30.0)
    timeout = float(ctx.get("timeout") or 10.0)

    # Reading pose/config is a passive subscribe -- no motor command is ever
    # sent by it -- so it happens regardless of `armed`. This is what lets a
    # disarmed preview show real numbers instead of empty dicts. The only
    # operation actually gated behind `armed` below is nr.stream_motion(),
    # the one call that writes to the command topic.
    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = nr.read_config(config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "report": f"native set {joint} FAILED: {exc}"}

    try:
        start_rad = nr.read_pose(state_topic, timeout)
    except Exception as exc:
        return {**blocked, "report": f"native set {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "report": f"native set {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {**blocked, "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}"}

    names = list(start_rad.keys())
    raw_target_rad = _to_radians(position, units)
    limits = nr.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_rad_value = min(upper, max(lower, raw_target_rad))
    else:
        target_rad_value = raw_target_rad
    target_rad = dict(start_rad)
    target_rad[joint] = target_rad_value
    try:
        motion_plan = _motion_plan(ctx, abs(target_rad_value - start_rad[joint]))
    except (TypeError, ValueError) as exc:
        return {**blocked, "report": f"BLOCKED: invalid motion profile: {exc}"}

    before = {n: _from_radians(v, units) for n, v in start_rad.items()}
    target = {n: _from_radians(v, units) for n, v in target_rad.items()}
    clamp_note = "" if abs(raw_target_rad - target_rad_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"
    range_note = ""
    if joint in limits:
        lo = _from_radians(limits[joint][0], units)
        hi = _from_radians(limits[joint][1], units)
        range_note = f" Safe range for {joint}: {lo:.2f} .. {hi:.2f} {units}."

    if not armed:
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": (
                f"PREVIEW (not armed): {joint} currently {before[joint]:.2f} {units}, "
                f"would move to {target[joint]:.2f}{clamp_note} using {motion_plan['mode']} "
                f"({motion_plan['duration']:.3f}s, {len(motion_plan['alphas'])} targets)."
                f"{range_note} Set armed=true to actually move it."
            ),
        }

    if config and "commands_allowed" in config and not bool(config.get("commands_allowed")):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": "BLOCKED: the robot driver reports it is read-only (commands_allowed=false).",
        }

    resource = str(
        (robot.get("driver") or {}).get("hardware_id")
        or robot.get("device_id")
        or command_topic
    )
    result = arm_controller.execute_joint_target(
        lambda safe_target: nr.stream_motion(
            command_topic,
            names,
            start_rad,
            safe_target,
            ramp_seconds=motion_plan["duration"],
            hold_seconds=hold_seconds,
            rate_hz=motion_plan["rate_hz"],
            timeout=timeout,
            alphas=motion_plan["alphas"],
        ),
        resource=resource,
        owner=f"ui:set-joint:{joint}",
        current=start_rad,
        target=target_rad,
        limits=limits,
        armed=True,
        interval=max(0.001, motion_plan["duration"]),
    )
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": f"native set {joint} FAILED: {result.get('error', 'unknown error')}",
        }

    try:
        after_rad = nr.read_pose(state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {n: _from_radians(v, units) for n, v in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    report = (
        f"native set {joint}: {before[joint]:.2f} -> {after.get(joint, before[joint]):.2f} {units} "
        f"(target {target[joint]:.2f}{clamp_note}); {motion_plan['mode']} profile streamed "
        f"{result.get('sent', 0)} commands at {motion_plan['rate_hz']:g} Hz.{range_note}"
    )
    return {"moved": moved, "joint": joint, "before": before, "after": after, "target": target, "report": report}


@node(
    name="ROS2SetJoint",
    category=_CATEGORY,
    description="Set one joint to an absolute position using native ROS 2 or rosbridge automatically. Safe by default: disarmed.",
    inputs={
        "trigger": AnyPort,
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default=""),
        "manual_action": Text(default="check"),
        "joint": Text(default=""),
        "position": Float(default=0.0),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "motion_profile": Dict(default={}),
        "profile_mode": Enum(["direct", "linear", "trapezoidal", "minimum_jerk"], default="linear"),
        "ramp_seconds": Float(default=0.8),
        "max_velocity": Float(default=90.0),
        "max_acceleration": Float(default=180.0),
        "hold_seconds": Float(default=0.2),
        "rate_hz": Float(default=30.0),
        "armed": Bool(default=False),
        "timeout": Float(default=10.0),
    },
    outputs={"moved": Bool, "joint": Text, "before": Dict, "after": Dict, "target": Dict, "report": Text},
)
def ros2_set_joint(ctx: dict) -> dict:
    ctx = _apply_robot_descriptor(ctx)
    manual_action = str(ctx.get("manual_action") or ctx.get("teach_action") or "check").strip().lower()
    if manual_action not in {"status", "check"}:
        return {
            "moved": False,
            "joint": str(ctx.get("joint") or "").strip(),
            "before": {},
            "after": {},
            "target": {},
            "report": f"BLOCKED: manual-move action '{manual_action}' was requested in this run; recook with action=check before commanding motion.",
        }
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    joint = str(ctx.get("joint") or "").strip()
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    blocked = {"moved": False, "joint": joint, "before": {}, "after": {}, "target": {}}
    if not joint:
        return {**blocked, "report": "BLOCKED: set 'joint' to a joint name (discover them with ROS2JointState)."}
    if robot and robot.get("ready") is False:
        reason = str(robot.get("error") or "the selected robot driver is not running")
        return {
            **blocked,
            "report": (
                f"BLOCKED: robot is not ready: {reason}. "
                "Start the Robot node and confirm its report before arming motion."
            ),
        }

    transport = _resolve_transport(ctx)
    if transport == "native":
        result = ros2_native_set_joint(ctx)
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result

    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "").strip()
    position = float(ctx.get("position") or 0.0)
    ramp_seconds = float(ctx.get("ramp_seconds") or 0.8)
    hold_seconds = float(ctx.get("hold_seconds") or 0.2)
    rate_hz = float(ctx.get("rate_hz") or 30.0)
    timeout = float(ctx.get("timeout") or 10.0)

    ok, err = rb.available()
    if not ok:
        return {**blocked, "report": f"rosbridge set {joint} FAILED: {err}"}
    config: dict[str, Any] = {}
    if config_topic:
        try:
            config = rb.read_config(host, port, config_topic, timeout) or {}
        except Exception as exc:
            return {**blocked, "report": f"rosbridge set {joint} FAILED: {exc}"}
    try:
        start_rad = rb.read_pose(host, port, state_topic, timeout)
    except Exception as exc:
        return {**blocked, "report": f"rosbridge set {joint} FAILED: {exc}"}
    if not start_rad:
        return {**blocked, "report": f"rosbridge set {joint} FAILED: no JointState on {state_topic} within {timeout:g}s"}
    if joint not in start_rad:
        return {**blocked, "report": f"BLOCKED: joint '{joint}' not in {state_topic}. Available: {', '.join(start_rad)}"}

    raw_target_rad = _to_radians(position, units)
    target_value = raw_target_rad
    limits = rb.limits_radians(config)
    if joint in limits:
        lower, upper = limits[joint]
        target_value = min(upper, max(lower, raw_target_rad))
    target_rad = dict(start_rad)
    target_rad[joint] = target_value
    try:
        motion_plan = _motion_plan(ctx, abs(target_value - start_rad[joint]))
    except (TypeError, ValueError) as exc:
        return {**blocked, "report": f"BLOCKED: invalid motion profile: {exc}"}
    before = {name: _from_radians(value, units) for name, value in start_rad.items()}
    target = {name: _from_radians(value, units) for name, value in target_rad.items()}
    clamp_note = "" if abs(raw_target_rad - target_value) < 1e-9 else f" (clamped to {target[joint]:.2f})"
    range_note = ""
    if joint in limits:
        lo = _from_radians(limits[joint][0], units)
        hi = _from_radians(limits[joint][1], units)
        range_note = f" Safe range for {joint}: {lo:.2f} .. {hi:.2f} {units}."

    if not bool(ctx.get("armed", False)):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": (
                f"{_transport_report(ctx, transport)}\nPREVIEW (not armed): {joint} currently {before[joint]:.2f} {units}, "
                f"would move to {target[joint]:.2f}{clamp_note} using {motion_plan['mode']} "
                f"({motion_plan['duration']:.3f}s, {len(motion_plan['alphas'])} targets)."
                f"{range_note} Set armed=true to move."
            ),
        }
    if config and config.get("commands_allowed") is False:
        return {**blocked, "before": before, "after": before, "target": target, "report": "BLOCKED: robot reports commands_allowed=false."}

    resource = str(
        (robot.get("driver") or {}).get("hardware_id")
        or robot.get("device_id")
        or f"{host}:{port}{command_topic}"
    )
    result = arm_controller.execute_joint_target(
        lambda safe_target: rb.stream_motion(
            host,
            port,
            command_topic,
            list(start_rad),
            start_rad,
            safe_target,
            ramp_seconds=motion_plan["duration"],
            hold_seconds=hold_seconds,
            rate_hz=motion_plan["rate_hz"],
            timeout=timeout,
            alphas=motion_plan["alphas"],
        ),
        resource=resource,
        owner=f"ui:set-joint:{joint}",
        current=start_rad,
        target=target_rad,
        limits=limits,
        armed=True,
        interval=max(0.001, motion_plan["duration"]),
    )
    if not result.get("ok"):
        return {
            "moved": False,
            "joint": joint,
            "before": before,
            "after": before,
            "target": target,
            "report": f"rosbridge set {joint} FAILED: {result.get('error', 'unknown error')}",
        }
    try:
        after_rad = rb.read_pose(host, port, state_topic, timeout) or dict(start_rad)
    except Exception:
        after_rad = dict(start_rad)
    after = {name: _from_radians(value, units) for name, value in after_rad.items()}
    moved = abs(after_rad.get(joint, start_rad[joint]) - start_rad[joint]) >= math.radians(0.5)
    return {
        "moved": moved,
        "joint": joint,
        "before": before,
        "after": after,
        "target": target,
        "report": (
            f"{_transport_report(ctx, transport)}\nrosbridge set {joint}: {before[joint]:.2f} -> "
            f"{after.get(joint, before[joint]):.2f} {units} (target {target[joint]:.2f}{clamp_note}); "
            f"{motion_plan['mode']} profile streamed {result.get('sent', 0)} commands at "
            f"{motion_plan['rate_hz']:g} Hz.{range_note}"
        ),
    }


@node(
    name="ROS2JointState",
    category=_CATEGORY,
    primary_inputs=["robot", "command"],
    description=(
        "Read joint state using native rclpy when available, otherwise rosbridge. "
        "Wire a Robot node's 'robot' output into 'robot' and it uses that robot's "
        "host, port, joint-state topic, and units automatically."
    ),
    inputs={
        "trigger": AnyPort,
        "robot": Dict(default={}),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "topic": Text(default="/joint_states"),
        "units": Enum(["radians", "degrees"], default="radians"),
        "timeout": Float(default=10.0),
    },
    outputs={"pose": Dict, "names": List, "report": Text},
)
def ros2_joint_state(ctx: dict) -> dict:
    ctx = _apply_robot_descriptor(ctx)
    transport = _resolve_transport(ctx)
    if transport == "native":
        result = ros2_native_joint_state(ctx)
        result["report"] = f"{_transport_report(ctx, transport)}\n{result.get('report', '')}"
        return result
    ok, err = rb.available()
    if not ok:
        return {"pose": {}, "names": [], "report": f"joint state FAILED: {err}"}
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    topic = str(ctx.get("topic") or "/joint_states")
    units = str(ctx.get("units") or "radians")
    timeout = float(ctx.get("timeout") or 10.0)
    try:
        pose_rad = rb.read_pose(host, port, topic, timeout)
    except Exception as exc:
        return {"pose": {}, "names": [], "report": f"joint state FAILED: {exc}"}
    if not pose_rad:
        return {"pose": {}, "names": [], "report": f"no JointState on {topic} within {timeout:g}s — is the robot bridge running?"}
    pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
    summary = ", ".join(f"{name} {pose[name]:.2f}" for name in pose)
    return {
        "pose": pose,
        "names": list(pose.keys()),
        "report": f"{_transport_report(ctx, transport)}\n{len(pose)} joints ({units}): {summary}",
    }


def _live_motion_dashboard_outputs(
    ctx: dict[str, Any], pose: dict[str, float], units: str, torque_enabled: bool, item: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    """Push a live pose into supported directly connected pass-through nodes."""
    graph = ctx.get("__graph__")
    source_id = str(ctx.get("__node_id__") or "")
    if graph is None or not source_id or not pose:
        return {}
    outputs: dict[str, dict[str, Any]] = {}
    for edge in list(getattr(graph, "_edges", []) or []):
        if edge.get("from") != source_id or edge.get("from_port") != "pose":
            continue
        target_id = str(edge.get("to") or "")
        target = getattr(graph, "_nodes", {}).get(target_id) or {}
        target_type = str(target.get("type") or "")
        if target_type == "ROS2MotionDashboard":
            baselines = item.setdefault("dashboard_baselines", {})
            baseline = baselines.setdefault(target_id, dict(pose))
            dashboard_ctx = dict(target.get("params") or {})
            dashboard_ctx.update({
                "pose": dict(pose),
                "before": dict(baseline),
                "units": units,
                "__live_pose__": True,
                "__torque_enabled__": torque_enabled,
                "__manual_mode__": "holding" if torque_enabled else "released",
            })
            try:
                outputs[target_id] = dict(ros2_motion_dashboard(dashboard_ctx))
            except Exception:
                continue
        elif target_type == "RobotConnectionDashboard":
            dashboard_fn = _NODE_REGISTRY.get(target_type)
            if dashboard_fn is None:
                continue
            dashboard_ctx = dict(target.get("params") or {})
            cache = getattr(graph, "_cache", {}) or {}
            for incoming in list(getattr(graph, "_edges", []) or []):
                if incoming.get("to") != target_id or incoming.get("to_port") == "pose":
                    continue
                cache_key = (incoming.get("from"), incoming.get("from_port"))
                if cache_key in cache:
                    dashboard_ctx[str(incoming.get("to_port") or "")] = cache[cache_key]
            dashboard_ctx["pose"] = dict(pose)
            dashboard_ctx["__live_pose__"] = True
            try:
                outputs[target_id] = dict(dashboard_fn(dashboard_ctx))
            except Exception:
                continue
        elif target_type == "RobotCalibrationRecorder":
            calibration_fn = _NODE_REGISTRY.get(target_type)
            if calibration_fn is None:
                continue
            calibration_ctx = dict(target.get("params") or {})
            # Reuse values resolved during the original graph cook (profile,
            # hardware serial, and safety options) while replacing only the
            # live pose and torque state.
            cache = getattr(graph, "_cache", {}) or {}
            for incoming in list(getattr(graph, "_edges", []) or []):
                if incoming.get("to") != target_id or incoming.get("to_port") == "pose":
                    continue
                cache_key = (incoming.get("from"), incoming.get("from_port"))
                if cache_key in cache:
                    calibration_ctx[str(incoming.get("to_port") or "")] = cache[cache_key]
            calibration_ctx.update({
                "action": "_sample",
                "pose": dict(pose),
                "torque_enabled": torque_enabled,
                "__live_pose__": True,
            })
            try:
                outputs[target_id] = dict(calibration_fn(calibration_ctx))
            except Exception:
                continue
    return outputs


def _teach_monitor_worker(run_id: str, item: dict[str, Any]) -> None:
    while not item["stop"].wait(0.1):
        ctx = dict(item["ctx"])
        robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
        transport = _resolve_transport(ctx)
        host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
        port = int(ctx.get("port") or robot.get("port") or 9090)
        state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
        config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "/joint_config")
        command_topic = str(ctx.get("command_topic") or robot.get("command_topic") or "/joint_commands")
        units = str(ctx.get("units") or robot.get("units") or "degrees")
        try:
            session = item.get("session")
            if transport == "native":
                if session is None:
                    session = nr.acquire_joint_stream(
                        state_topic,
                        "",
                        config_topic,
                        timeout=2.0,
                        node_name=f"blacknode_manual_move_{run_id}",
                    )
                    item["session"] = session
                    item["session_runtime"] = nr
                    session.wait_for_pose(1.0)
                    session.wait_for_config(0.5)
            else:
                if session is None:
                    session = rb.acquire_joint_stream(host, port, state_topic, command_topic, config_topic, timeout=2.0)
                    item["session"] = session
                    item["session_runtime"] = rb
                    session.wait_for_pose(1.0)
                    session.wait_for_config(0.5)
            confirmed_config = item.pop("confirmed_config", None)
            if isinstance(confirmed_config, dict):
                session.seed_config(confirmed_config)
            pose_rad, config, state_age = session.snapshot()
            if state_age > 2.0:
                # A worker heartbeat is not proof that ROS is still delivering
                # JointState callbacks. Replace one stale managed subscription;
                # never create one-shot rclpy nodes inside this polling loop.
                _release_teach_session(item, discard=True)
                item["error"] = f"joint-state subscription stale ({state_age:.1f}s); reconnecting"
                continue
            if not config:
                previous = item.get("outputs") or {}
                config = {"torque_enabled": previous.get("torque_enabled", False)}
            torque_enabled = bool(config.get("torque_enabled", False))
            pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
            command_error = str(ctx.get("__manual_command_error__") or "")
            report = command_error or ("Move the supported arm by hand; live joint positions update here." if not torque_enabled else "Holding current pose.")
            updated_at = time.strftime("updated %H:%M:%S")
            outputs = {
                "action": str(ctx.get("action") or "check"),
                "live": True,
                "data_ready": bool(pose),
                "mode": "released" if not torque_enabled else "hold",
                "torque_enabled": torque_enabled,
                "command_ok": not bool(command_error),
                "pose": pose,
                "joints": list(pose),
                "updated_at": updated_at,
                "dashboard": _teach_dashboard(pose, units, not torque_enabled, report, action=str(ctx.get("action") or "check"), live=True, updated_at=updated_at),
                "report": report,
            }
            downstream_outputs = _live_motion_dashboard_outputs(ctx, pose, units, torque_enabled, item)
            with _teach_monitor_lock:
                current = _teach_monitors.get(run_id)
                if current is not item:
                    return
                item["outputs"] = outputs
                item["downstream_outputs"] = downstream_outputs
                item["downstream_types"] = {
                    str(edge.get("to") or ""): str((getattr(ctx.get("__graph__"), "_nodes", {}).get(str(edge.get("to") or "")) or {}).get("type") or "")
                    for edge in list(getattr(ctx.get("__graph__"), "_edges", []) or [])
                    if edge.get("from") == str(ctx.get("__node_id__") or "") and edge.get("from_port") == "pose"
                }
                item["updated_at"] = time.time()
                item["error"] = ""
        except Exception as exc:
            with _teach_monitor_lock:
                if _teach_monitors.get(run_id) is not item:
                    return
                item["error"] = f"{type(exc).__name__}: {exc}"


def _start_teach_monitor(
    run_id: str,
    ctx: dict,
    outputs: dict[str, Any],
    confirmed_config: dict[str, Any] | None = None,
) -> None:
    with _teach_monitor_lock:
        existing = _teach_monitors.get(run_id)
        if existing is not None:
            existing["ctx"] = dict(ctx)
            existing["outputs"] = dict(outputs)
            existing["dashboard_baselines"] = {}
            existing["confirmed_config"] = dict(confirmed_config or {})
            session = existing.get("session")
            if session is not None and confirmed_config:
                session.seed_config(confirmed_config)
            return
        item: dict[str, Any] = {
            "ctx": dict(ctx),
            "outputs": dict(outputs),
            "updated_at": time.time(),
            "error": "",
            "downstream_outputs": {},
            "dashboard_baselines": {},
            "confirmed_config": dict(confirmed_config or {}),
            "stop": threading.Event(),
        }
        _teach_monitors[run_id] = item
    thread = threading.Thread(target=_teach_monitor_worker, args=(run_id, item), name=f"blacknode-teach-{run_id}", daemon=True)
    item["thread"] = thread
    thread.start()


def _release_teach_session(
    item: dict[str, Any],
    *,
    discard: bool = False,
) -> None:
    session = item.get("session")
    item["session"] = None
    runtime = item.pop("session_runtime", None)
    if session is None:
        return
    if runtime is nr:
        nr.release_joint_stream(session, discard=discard)
    else:
        rb.release_joint_stream(session, discard=discard)


def _stop_teach_monitor(run_id: str) -> None:
    with _teach_monitor_lock:
        item = _teach_monitors.pop(run_id, None)
    if item is not None:
        item["stop"].set()
        _release_teach_session(item)


def runtime_status() -> dict[str, Any]:
    with _teach_monitor_lock:
        monitors = []
        for run_id, item in _teach_monitors.items():
            monitors.append({
                "run_id": run_id,
                "node_id": str(item.get("ctx", {}).get("__node_id__") or ""),
                "node_type": "ROS2ManualMove",
                "outputs": dict(item.get("outputs") or {}),
                "updated_at": item.get("updated_at"),
                "error": item.get("error") or "",
            })
            for node_id, outputs in dict(item.get("downstream_outputs") or {}).items():
                monitors.append({
                    "run_id": run_id,
                    "node_id": node_id,
                    "node_type": str(item.get("downstream_types", {}).get(node_id) or "live downstream"),
                    "outputs": dict(outputs or {}),
                    "updated_at": item.get("updated_at"),
                    "error": "",
                })
    try:
        from blacknode.pkg.blacknode_skills.follow.leader_follower_runtime import monitor_entries
        leader_follower_entries = monitor_entries()
    except Exception:
        leader_follower_entries = []
    monitors.extend(leader_follower_entries)
    return {
        "ok": True,
        "active": bool(monitors),
        "node_outputs": monitors,
        "managed_runs": (
            [{"run_id": run_id, "kind": "manual_move"} for run_id in _teach_monitors]
            + [{"run_id": entry["run_id"], "kind": "leader_follower"} for entry in leader_follower_entries]
        ),
        "report": f"{len(_teach_monitors)} manual-move monitor(s), {len(leader_follower_entries)} leader-follower controller(s) active",
    }


def stop_runtime_services() -> dict[str, Any]:
    with _teach_monitor_lock:
        items = list(_teach_monitors.values())
        _teach_monitors.clear()
    for item in items:
        item["stop"].set()
        _release_teach_session(item)
    try:
        from blacknode.pkg.blacknode_skills.follow.follow_runtime import stop_continuous_follow_services
        follow_result = stop_continuous_follow_services()
    except ModuleNotFoundError:
        follow_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        follow_result = {"ok": False, "stopped": 0, "error": str(exc)}
    stopped_follow = int(follow_result.get("stopped") or 0)
    try:
        from blacknode.pkg.blacknode_skills.follow.leader_follower_runtime import stop_leader_follower_services
        leader_follower_result = stop_leader_follower_services()
    except ModuleNotFoundError:
        leader_follower_result = {"ok": True, "stopped": 0, "error": ""}
    except Exception as exc:
        leader_follower_result = {"ok": False, "stopped": 0, "error": str(exc)}
    stopped_leader_follower = int(leader_follower_result.get("stopped") or 0)
    closed_sessions = rb.close_joint_streams()
    return {
        "ok": bool(follow_result.get("ok", True) and leader_follower_result.get("ok", True)),
        "stopped": {
            "streams": len(items),
            "managed_runs": stopped_follow + stopped_leader_follower,
            "detached": 0,
            "joint_streams": closed_sessions,
        },
        "report": (
            f"stopped {len(items)} manual-move monitor(s), {stopped_follow} follow controller(s), "
            f"and {stopped_leader_follower} leader-follower controller(s); "
            f"closed {closed_sessions} joint stream(s)"
        ),
    }


@node(
    name="ROS2ManualMove",
    category=_CATEGORY,
    live=True,
    description="Release torque for safe hand positioning or hold the current pose, with an explicit live joint monitor.",
    inputs={
        "trigger": AnyPort,
        "run_id": Text(default="robot_teach"),
        "action": Enum(["check", "release", "hold"], default="check"),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "robot": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "config_topic": Text(default="/joint_config"),
        "control_topic": Text(default="/robot_control"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "live_monitor": Bool(default=True),
        "timeout": Float(default=5.0),
    },
    outputs={
        "action": Text,
        "live": Bool,
        "data_ready": Bool,
        "mode": Text,
        "torque_enabled": Bool,
        "command_ok": Bool,
        "pose": Dict,
        "joints": List,
        "updated_at": Text,
        "dashboard": Image,
        "report": Text,
    },
)
def ros2_manual_move(ctx: dict) -> dict:
    robot = ctx.get("robot") if isinstance(ctx.get("robot"), dict) else {}
    raw_action = str(ctx.get("action") or "check").strip().lower()
    action = {"status": "check", "enter": "release", "exit": "hold"}.get(raw_action, raw_action)
    run_id = str(ctx.get("run_id") or "robot_teach").strip() or "robot_teach"
    transport = _resolve_transport(ctx)
    host = str(ctx.get("host") or robot.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or robot.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or robot.get("state_topic") or "/joint_states")
    config_topic = str(ctx.get("config_topic") or robot.get("config_topic") or "/joint_config")
    control_topic = str(ctx.get("control_topic") or robot.get("control_topic") or "/robot_control")
    units = str(ctx.get("units") or robot.get("units") or "degrees")
    timeout = max(0.5, float(ctx.get("timeout") or 5.0))
    # Direct/MCP calls predate graph execution modes and retain their live
    # behavior. The editor/server always supplies an explicit mode.
    run_mode = "once" if ctx.get("__run_mode__") == "once" else "live"
    base = {
        "action": action,
        "live": False,
        "data_ready": False,
        "mode": "unknown",
        "torque_enabled": False,
        "command_ok": False,
        "pose": {},
        "joints": [],
        "updated_at": "not run yet",
        "dashboard": _teach_dashboard({}, units, False, "Waiting for robot state", action=action),
    }

    if transport == "native":
        ok, error = nr.available()
        read_config = lambda wait: nr.read_config(config_topic, wait)
        read_pose = lambda wait: nr.read_pose(state_topic, wait)
        publish = lambda payload: nr.publish_string(control_topic, payload, min(timeout, 2.0))
    else:
        ok, error = rb.available()
        read_config = lambda wait: rb.read_config(host, port, config_topic, wait)
        read_pose = lambda wait: rb.read_pose(host, port, state_topic, wait)
        publish = lambda payload: rb.publish_string(host, port, control_topic, payload, min(timeout, 2.0))
    if not ok:
        return {**base, "report": f"manual move FAILED: {error}"}

    if action in {"release", "hold"}:
        requested = "enter_teach" if action == "release" else "exit_teach"
        published = publish(json.dumps({"action": requested}))
        if not published.get("ok"):
            return {**base, "report": f"manual move FAILED: {published.get('error', 'control command was not accepted')}"}

    desired_torque = action == "hold" if action != "check" else None
    deadline = time.monotonic() + timeout
    config: dict[str, Any] = {}
    while time.monotonic() < deadline:
        try:
            config = read_config(min(0.5, max(0.05, deadline - time.monotonic()))) or {}
        except Exception as exc:
            if action == "check":
                return {**base, "report": f"live pose check FAILED: {type(exc).__name__}: {exc}"}
        if "torque_enabled" in config and (
            desired_torque is None or bool(config.get("torque_enabled")) == desired_torque
        ):
            break
        time.sleep(0.05)

    if "torque_enabled" not in config:
        return {
            **base,
            "report": f"manual move unsupported: no torque state on {config_topic}; restart with the updated robot driver.",
        }
    torque_enabled = bool(config.get("torque_enabled"))
    teach_mode = not torque_enabled
    try:
        pose_rad = read_pose(min(timeout, 2.0)) or {}
    except Exception:
        pose_rad = {}
    pose = {name: _from_radians(value, units) for name, value in pose_rad.items()}
    last_error = str(config.get("last_error") or "")
    acknowledged = desired_torque is None or torque_enabled == desired_torque
    monitor_enabled = run_mode == "live" and bool(ctx.get("live_monitor", True))

    if not acknowledged:
        report = f"manual move FAILED: driver did not acknowledge '{action}' within {timeout:g}s"
    elif last_error:
        report = f"manual move WARNING: {last_error}; keep the arm supported and use Stop all if any joint still resists."
    elif teach_mode and monitor_enabled:
        report = (
            f"RELEASED: torque is off; support the arm and move it by hand. "
            f"LIVE monitor is reading {len(pose)} joint position(s) from {state_topic}."
        )
    elif teach_mode:
        report = "RELEASED: torque is off. This is a one-time pose snapshot; use Go live to watch hand movement."
    elif monitor_enabled:
        report = "HOLDING: live pose monitoring is active; torque is on. Release only when the arm is supported."
    else:
        report = "HOLDING: torque is on. This is a one-time pose snapshot; use Go live for continuous updates."
    updated_at = time.strftime("updated %H:%M:%S")
    is_live = monitor_enabled
    result = {
        "action": action,
        "live": is_live,
        "data_ready": bool(pose),
        "mode": "released" if teach_mode else "hold",
        "torque_enabled": torque_enabled,
        "command_ok": acknowledged,
        "pose": pose,
        "joints": list(pose),
        "updated_at": updated_at if is_live else f"snapshot {time.strftime('%H:%M:%S')}",
        "dashboard": _teach_dashboard(pose, units, teach_mode, report, action=action, live=is_live, updated_at=updated_at if is_live else "ONE-TIME SNAPSHOT"),
        "report": f"{_transport_report(ctx, transport)}\n{report}",
    }
    if is_live:
        monitor_ctx = {
            **ctx,
            "action": action,
            "__manual_command_error__": "" if acknowledged else report,
        }
        _start_teach_monitor(run_id, monitor_ctx, result, config)
    else:
        _stop_teach_monitor(run_id)
    return result


@node(
    name="ROS2MotionDashboard",
    category=_CATEGORY,
    live=True,
    description="Render live pose updates when connected to Manual Move, or a one-time before/after motion result.",
    inputs={
        "joint": Text(default=""),
        "pose": Dict,
        "before": Dict,
        "after": Dict,
        "target": Dict,
        "moved": Bool(default=False),
        "units": Text(default="radians"),
    },
    outputs={"dashboard": Image, "live": Bool, "summary": Dict},
)
def ros2_motion_dashboard(ctx: dict) -> dict:
    live_pose = bool(ctx.get("__live_pose__"))
    torque_enabled = bool(ctx.get("__torque_enabled__")) if live_pose else None
    manual_mode = str(ctx.get("__manual_mode__") or ("holding" if torque_enabled else "released")) if live_pose else "snapshot"
    requested_joint = str(ctx.get("joint") or "")
    joint = requested_joint
    pose = dict(ctx.get("pose") or {})
    before = (dict(ctx.get("before") or {}) or dict(pose))
    after = dict(pose) if live_pose else (dict(ctx.get("after") or {}) or dict(pose) or dict(before))
    target = {} if live_pose else dict(ctx.get("target") or {})
    moved = False if live_pose else bool(ctx.get("moved", False))
    units = str(ctx.get("units") or "radians")

    joint_names = sorted(set(pose) | set(before) | set(after) | set(target))
    if live_pose:
        changed_joints = [
            name for name in joint_names
            if isinstance(before.get(name), (int, float)) and isinstance(after.get(name), (int, float))
        ]
        if changed_joints:
            joint = max(changed_joints, key=lambda name: abs(float(after[name]) - float(before[name])))
    delta = (after.get(joint, 0.0) - before.get(joint, 0.0)) if joint in before and joint in after else 0.0
    summary = {
        "joint": joint,
        "requested_joint": requested_joint,
        "live": live_pose,
        "mode": manual_mode,
        "torque_enabled": torque_enabled,
        "moved": moved,
        "units": units,
        "before": before.get(joint),
        "after": after.get(joint),
        "target": target.get(joint),
        "delta": delta,
        "joints": joint_names,
        "positions": dict(after or before or pose),
        "before_values": before,
        "after_values": after,
        "target_values": target,
    }

    has_motion_request = bool(joint or target) and not live_pose
    verdict = "LIVE" if live_pose else ("MOVED" if moved else ("NO CHANGE" if has_motion_request else "POSE SNAPSHOT"))
    accent = "#22c55e" if live_pose or moved else ("#ef4444" if has_motion_request else "#2e9fe6")
    muted = "#93a4b8"
    panel = "#172033"
    target_value = target.get(joint)
    current_value = after.get(joint)
    target_text = f"{target_value:.2f}" if isinstance(target_value, (int, float)) else "-"
    current_text = f"{current_value:.2f}" if isinstance(current_value, (int, float)) else "-"
    side_joint_label = "MOST CHANGED JOINT" if live_pose else "COMMANDED JOINT"
    delta_label = "CHANGE SINCE LIVE START" if live_pose else f"DELTA ({units})"
    value_label = f"CURRENT ({units})" if live_pose else "TARGET"
    value_text = current_text if live_pose else target_text

    rows = []
    for index, name in enumerate(joint_names[:8]):
        y = 196 + index * 50
        b = before.get(name)
        a = after.get(name)
        is_target = name == joint
        b_text = f"{b:.2f}" if isinstance(b, (int, float)) else "-"
        a_text = f"{a:.2f}" if isinstance(a, (int, float)) else "-"
        moved_row = isinstance(b, (int, float)) and isinstance(a, (int, float)) and abs(a - b) >= 0.01
        a_color = accent if moved_row else "#f8fafc"
        name_color = "#f8fafc" if is_target else muted
        weight_attr = ' font-weight="700"' if is_target else ""
        if is_target:
            rows.append(f'<rect x="36" y="{y - 24}" width="688" height="40" rx="10" fill="#0f1a2e" stroke="{accent}"/>')
        rows.append(
            f'<text x="60" y="{y}" fill="{name_color}" font-family="monospace" font-size="16"{weight_attr}>{_svg_text(name, 20)}</text>'
            f'<text x="430" y="{y}" text-anchor="end" fill="{muted}" font-family="monospace" font-size="16">{b_text}</text>'
            f'<text x="500" y="{y}" text-anchor="middle" fill="{muted}" font-family="Arial" font-size="14">-&gt;</text>'
            f'<text x="700" y="{y}" text-anchor="end" fill="{a_color}" font-family="monospace" font-size="16" font-weight="700">{a_text}</text>'
        )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1120" height="640" viewBox="0 0 1120 640">
<rect width="1120" height="640" rx="28" fill="#0b1020"/>
<rect x="24" y="24" width="1072" height="86" rx="18" fill="{panel}" stroke="#2e9fe6" stroke-width="2"/>
<circle cx="68" cy="67" r="18" fill="#2e9fe6"/><circle cx="68" cy="67" r="8" fill="#0b1020"/>
<text x="104" y="58" fill="#f8fafc" font-family="Arial,sans-serif" font-size="26" font-weight="700">{'LIVE MOTION DASHBOARD' if live_pose else 'MOTION RESULT · SNAPSHOT'}</text>
<text x="104" y="86" fill="{muted}" font-family="Arial,sans-serif" font-size="15">{('continuously updated · ' + ('HOLDING · TORQUE ON' if torque_enabled else 'RELEASED · TORQUE OFF')) if live_pose else 'one-time before vs after result'} ({_svg_text(units, 12)})</text>
<rect x="900" y="40" width="170" height="52" rx="26" fill="{accent}"/>
<text x="985" y="74" text-anchor="middle" fill="#ffffff" font-family="Arial,sans-serif" font-size="22" font-weight="800">{verdict}</text>

<text x="60" y="158" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">JOINT</text>
<text x="430" y="158" text-anchor="end" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">BEFORE</text>
<text x="700" y="158" text-anchor="end" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">{'CURRENT' if live_pose else 'AFTER'}</text>
{''.join(rows)}

<rect x="760" y="150" width="324" height="440" rx="16" fill="{panel}"/>
<text x="784" y="190" fill="{muted}" font-family="Arial,sans-serif" font-size="13" font-weight="700">{_svg_text(side_joint_label, 28)}</text>
<text x="784" y="232" fill="#f8fafc" font-family="Arial,sans-serif" font-size="24" font-weight="800">{_svg_text(joint or "-", 18)}</text>
<text x="784" y="300" fill="{muted}" font-family="Arial,sans-serif" font-size="13">{_svg_text(delta_label, 30)}</text>
<text x="784" y="346" fill="{accent}" font-family="Arial,sans-serif" font-size="42" font-weight="800">{delta:+.2f}</text>
<text x="784" y="408" fill="{muted}" font-family="Arial,sans-serif" font-size="13">{_svg_text(value_label, 24)}</text>
<text x="784" y="448" fill="#f8fafc" font-family="monospace" font-size="22">{value_text}</text>
</svg>"""
    return {"dashboard": _svg_data(svg), "live": live_pose, "summary": summary}


# Live per-joint slider control. State is kept per run so the editor can push a
# new slider value while the node stays live, and the driver holds the last
# commanded pose between updates.
_JOINT_SLIDER_STATE: dict[str, dict[str, Any]] = {}
_JOINT_SLIDER_LOCK = threading.Lock()


def _slider_joint_specs(names, pose, limits, units):
    """One entry per joint for the editor to render a slider: name, bounds, and
    current value, all in display units. Falls back to a wide default range when
    the driver publishes no limits."""
    specs = []
    for name in names:
        if name in limits:
            lo = _from_radians(limits[name][0], units)
            hi = _from_radians(limits[name][1], units)
        else:
            lo, hi = (-180.0, 180.0) if units == "degrees" else (-math.pi, math.pi)
        specs.append({
            "name": name,
            "min": round(min(lo, hi), 3),
            "max": round(max(lo, hi), 3),
            "value": round(float(pose.get(name, 0.0)), 3),
        })
    return specs


@node(
    name="ROS2JointSliders",
    category=_CATEGORY,
    live=True,
    primary_inputs=["robot"],
    description=(
        "Live joint sliders: reads the robot's joints, limits, and current pose, "
        "then moving a slider commands that joint in real time over ROS 2. Wire a "
        "Robot descriptor in, Go Live, arm it, and drag a slider to move the robot. "
        "Motion stays disarmed until you enable it."
    ),
    inputs={
        "trigger": AnyPort,
        "robot": Dict,
        "command": Dict(default={}),
        "run_id": Text(default="joint_sliders"),
        "targets": Dict(default={}),
        "armed": Bool(default=False),
        "transport": Enum(["auto", "native", "rosbridge"], default="auto"),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "state_topic": Text(default="/joint_states"),
        "command_topic": Text(default="/joint_commands"),
        "config_topic": Text(default="/joint_config"),
        "units": Enum(["radians", "degrees"], default="degrees"),
        "ramp_seconds": Float(default=0.25),
        "timeout": Float(default=5.0),
    },
    outputs={
        "joints": List,
        "pose": Dict,
        "armed": Bool,
        "commanded": Bool,
        "live": Bool,
        "run_id": Text,
        "report": Text,
    },
)
def ros2_joint_sliders(ctx: dict) -> dict:
    ctx = _apply_robot_descriptor(ctx)
    run_id = str(ctx.get("run_id") or "joint_sliders").strip() or "joint_sliders"
    # A plain Run reads once; Go Live keeps the sliders driving the robot. The
    # 'live' output tells the editor to show LIVE, not "snapshot not updating".
    is_live = ctx.get("__run_mode__") != "once"
    transport = _resolve_transport(ctx)
    host = str(ctx.get("host") or "127.0.0.1")
    port = int(ctx.get("port") or 9090)
    state_topic = str(ctx.get("state_topic") or "/joint_states")
    command_topic = str(ctx.get("command_topic") or "/joint_commands")
    config_topic = str(ctx.get("config_topic") or "/joint_config")
    units = str(ctx.get("units") or "degrees")
    timeout = max(0.5, float(ctx.get("timeout") or 5.0))
    armed = bool(ctx.get("armed", False))
    ramp_seconds = max(0.0, float(ctx.get("ramp_seconds") or 0.25))
    blank = {"joints": [], "pose": {}, "armed": armed, "commanded": False, "live": is_live, "run_id": run_id}

    if transport == "native":
        ok, err = nr.available()
        read_pose = lambda: nr.read_pose(state_topic, min(timeout, 2.0))
        read_config = lambda: nr.read_config(config_topic, min(timeout, 2.0))
        limits_of = nr.limits_radians
    else:
        ok, err = rb.available()
        read_pose = lambda: rb.read_pose(host, port, state_topic, min(timeout, 2.0))
        read_config = lambda: rb.read_config(host, port, config_topic, min(timeout, 2.0))
        limits_of = rb.limits_radians
    if not ok:
        return {**blank, "report": f"joint sliders FAILED: {err}"}

    try:
        pose_rad = read_pose() or {}
    except Exception as exc:  # noqa: BLE001
        return {**blank, "report": f"joint sliders FAILED: {exc}"}
    if not pose_rad:
        return {**blank, "report": (
            f"joint sliders: no JointState on {state_topic} at {host}:{port} — "
            "is the robot's driver running?"
        )}
    try:
        config = read_config() or {}
    except Exception:  # noqa: BLE001
        config = {}
    limits = limits_of(config)
    names = list(pose_rad)
    pose = {name: round(_from_radians(value, units), 3) for name, value in pose_rad.items()}
    specs = _slider_joint_specs(names, pose, limits, units)

    with _JOINT_SLIDER_LOCK:
        _JOINT_SLIDER_STATE[run_id] = {
            "transport": transport, "host": host, "port": port,
            "state_topic": state_topic, "command_topic": command_topic,
            "units": units, "names": names,
            "limits": limits, "ramp_seconds": ramp_seconds, "timeout": timeout,
            "commands_allowed": config.get("commands_allowed", True),
            "armed": armed, "held_rad": dict(pose_rad),
            "command_lock": threading.Lock(),
        }
    report = (
        f"LIVE joint sliders on {state_topic} ({len(names)} joints). "
        + ("ARMED - drag a slider to move the robot." if armed
           else "DISARMED - enable Arm to move; sliders preview only.")
    )
    return {"joints": specs, "pose": pose, "armed": armed, "commanded": False,
            "live": is_live, "run_id": run_id, "report": report}


def set_joint_slider_targets(run_id: str, targets: dict[str, Any]) -> dict[str, Any]:
    """Command the robot from slider values pushed live by the editor. Clamps to
    limits, moves only from the held pose, and refuses unless armed."""
    with _JOINT_SLIDER_LOCK:
        state = _JOINT_SLIDER_STATE.get(run_id)
        if state is None:
            return {"ok": False, "report": f"joint sliders '{run_id}' is not running"}
        state = dict(state)
    if not state.get("armed"):
        return {"ok": False, "report": "BLOCKED: arm the sliders before moving"}
    if state.get("commands_allowed") is False:
        return {"ok": False, "report": "BLOCKED: the robot driver reports commands_allowed=false"}
    command_lock = state["command_lock"]
    with command_lock:
        with _JOINT_SLIDER_LOCK:
            live_state = _JOINT_SLIDER_STATE.get(run_id)
            if live_state is None or not live_state.get("armed"):
                return {"ok": False, "report": "BLOCKED: arm the sliders before moving"}
            state = dict(live_state)
        if state["transport"] == "native":
            try:
                current = nr.read_pose(
                    state["state_topic"],
                    min(state["timeout"], 2.0),
                ) or {}
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "report": f"joint move FAILED: live feedback unavailable: {exc}"}
        else:
            try:
                current = rb.read_pose(
                    state["host"],
                    state["port"],
                    state["state_topic"],
                    min(state["timeout"], 2.0),
                ) or {}
            except Exception as exc:  # noqa: BLE001
                return {"ok": False, "report": f"joint move FAILED: live feedback unavailable: {exc}"}
        names = state["names"]
        if not current or any(name not in current for name in names):
            return {"ok": False, "report": "BLOCKED: complete live joint feedback is unavailable"}
        held = {name: float(current[name]) for name in names}
        units = state["units"]
        limits = state["limits"]
        target = dict(held)
        changed = []
        for joint, value in (targets or {}).items():
            if joint not in names or not isinstance(value, (int, float)):
                continue
            rad = _to_radians(float(value), units)
            if joint in limits:
                lo, hi = limits[joint]
                rad = max(lo, min(hi, rad))
            target[joint] = rad
            changed.append(joint)
        if not changed:
            return {"ok": True, "report": "no in-range joint change"}

        if state["transport"] == "native":
            publish = lambda safe_target: nr.stream_motion(
                state["command_topic"], names, held, safe_target,
                ramp_seconds=state["ramp_seconds"], hold_seconds=0.05, rate_hz=30.0,
                timeout=state["timeout"])
            resource = state["command_topic"]
        else:
            publish = lambda safe_target: rb.stream_motion(
                state["host"], state["port"], state["command_topic"], names, held,
                safe_target, ramp_seconds=state["ramp_seconds"], hold_seconds=0.05,
                rate_hz=30.0, timeout=state["timeout"])
            resource = f"{state['host']}:{state['port']}{state['command_topic']}"
        result = arm_controller.execute_joint_target(
            publish,
            resource=resource,
            owner=f"ui:joint-sliders:{run_id}",
            current=held,
            target=target,
            limits=limits,
            armed=True,
            feedback_age=0.0,
            interval=max(0.001, state["ramp_seconds"]),
        )
        if not result.get("ok", True):
            return {"ok": False, "report": f"joint move FAILED: {result.get('error', 'unknown')}"}
        with _JOINT_SLIDER_LOCK:
            if run_id in _JOINT_SLIDER_STATE:
                _JOINT_SLIDER_STATE[run_id]["held_rad"] = target
        return {"ok": True, "report": f"moved {', '.join(changed)}", "commanded": True}


def set_joint_slider_command(run_id: str, command: dict[str, Any]) -> dict[str, Any]:
    """Consume the canonical command request emitted by ``RobotServo``.

    The graph edge expresses operator intent, while this live motion node still
    owns authorization, current-pose synchronization, calibrated limits, and
    transport publication.
    """
    request = dict(command or {})
    if request.get("kind") != "blacknode.joint-command-request":
        return {"ok": False, "report": "BLOCKED: unsupported joint command"}
    if int(request.get("schema_version") or 0) != 1:
        return {"ok": False, "report": "BLOCKED: unsupported joint command schema"}
    if request.get("requires_motion_authorization") is not True:
        return {"ok": False, "report": "BLOCKED: joint command did not request motion authorization"}
    joint = str(request.get("joint_name") or "").strip()
    if not joint:
        return {"ok": False, "report": "BLOCKED: joint command has no joint name"}
    try:
        position_rad = float(request["position_rad"])
        issued_at = float(request["issued_at"])
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "report": "BLOCKED: joint command position or timestamp is invalid"}
    if not math.isfinite(position_rad):
        return {"ok": False, "report": "BLOCKED: joint command position is not finite"}
    age = time.time() - issued_at
    if age < -1.0 or age > 0.75:
        return {"ok": False, "report": "BLOCKED: joint command is stale"}
    with _JOINT_SLIDER_LOCK:
        state = _JOINT_SLIDER_STATE.get(run_id)
        units = str(state.get("units") or "degrees") if state else "degrees"
    return set_joint_slider_targets(
        run_id,
        {joint: _from_radians(position_rad, units)},
    )


def set_joint_slider_armed(run_id: str, armed: bool) -> dict[str, Any]:
    """Toggle the armed gate for a live slider run without recooking."""
    with _JOINT_SLIDER_LOCK:
        state = _JOINT_SLIDER_STATE.get(run_id)
        if state is None:
            return {"ok": False, "report": f"joint sliders '{run_id}' is not running"}
        state["armed"] = bool(armed)
    return {"ok": True, "report": f"joint sliders {'armed' if armed else 'disarmed'}"}
