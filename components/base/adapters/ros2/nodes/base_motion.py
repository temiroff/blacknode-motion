"""Safety-gated mobile-base motion over rosbridge.

Companion to the joint-space nodes in ``ros2_live``: these nodes drive a
wheeled base that listens for ``geometry_msgs/msg/Twist`` velocity commands
(mecanum, differential, Ackermann — anything with a cmd_vel-style topic).

The chain mirrors the arm-side ``PolicySafetyGate`` design:

    [ROS2LaserScanCheck] -> clearance_m
                                |
    [BaseSafetyGate: armed?] -> authorization (caps + freshness)
                                |
    [ROS2BaseMove] -> clamped, time-boxed velocity stream, then zero Twist

``ROS2BaseMove`` refuses to run without a fresh authorization from
``BaseSafetyGate``, clamps every velocity to the gate's caps *and* to the
hard module limits below, and always streams zero-velocity stop frames
afterwards — even when the move errors mid-stream. ``ROS2BaseStop`` is the
standalone big red button and deliberately needs no authorization.

Every node returns a structured report instead of raising, so workflows stay
usable on machines without roslibpy or without a reachable robot.
"""
from __future__ import annotations

import math
import time
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Float, Int, Text, node



class _LazyRosbridgeRuntime:
    """Delay ROS 2 imports so dependency discovery can load in any folder order."""

    def __getattr__(self, name: str):
        from blacknode.pkg.blacknode_ros2 import rosbridge_runtime

        return getattr(rosbridge_runtime, name)


rb = _LazyRosbridgeRuntime()

_CATEGORY = "Motion"

TWIST_TYPE = "geometry_msgs/msg/Twist"
SCAN_TYPE = "sensor_msgs/msg/LaserScan"
ODOM_TYPE = "nav_msgs/msg/Odometry"

# Absolute ceilings, applied on top of whatever the gate authorizes. A gate
# param typo can never turn a bench test into a runaway robot.
HARD_MAX_SPEED_MPS = 0.5
HARD_MAX_TURN_RPS = 1.5
HARD_MAX_DURATION_S = 5.0
DEFAULT_AUTH_MAX_AGE_S = 30.0
_STOP_FRAMES = 3


def _float(ctx: dict, name: str, default: float) -> float:
    """Read a float input where 0.0 is meaningful (the ``or`` idiom would eat it)."""
    value = ctx.get(name)
    if value is None or value == "":
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _twist_message(vx: float, vy: float, wz: float):
    return rb.roslibpy.Message(
        {
            "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
        }
    )


def stream_twist(
    host: str,
    port: int,
    topic_name: str,
    vx: float,
    vy: float,
    wz: float,
    duration_s: float,
    rate_hz: float,
    timeout: float = 10.0,
) -> dict[str, Any]:
    """Publish a Twist at ``rate_hz`` for ``duration_s``, then always stop.

    The zero-velocity stop frames go out in ``finally`` so a mid-stream
    exception or disconnect still commands a halt. Returns ``{ok, sent, error?}``.
    """
    ok, err = rb.available()
    if not ok:
        return {"ok": False, "sent": 0, "error": err}
    try:
        ros = rb.get_connection(host, int(port), timeout)
    except Exception as exc:
        return {"ok": False, "sent": 0, "error": f"{type(exc).__name__}: {exc}"}

    rate = min(30.0, max(2.0, float(rate_hz)))
    period = 1.0 / rate
    frames = max(1, int(max(0.0, float(duration_s)) * rate))
    topic = rb.roslibpy.Topic(ros, topic_name, TWIST_TYPE)
    sent = 0
    error = ""
    try:
        topic.advertise()
        time.sleep(0.15)  # rosbridge advertisement is asynchronous
        for _ in range(frames):
            if not ros.is_connected:
                error = "rosbridge disconnected mid-move"
                break
            topic.publish(_twist_message(vx, vy, wz))
            sent += 1
            time.sleep(period)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            for _ in range(_STOP_FRAMES):
                topic.publish(_twist_message(0.0, 0.0, 0.0))
                time.sleep(0.05)
        except Exception:
            error = error or "could not stream zero-velocity stop frames"
        try:
            topic.unadvertise()
        except Exception:
            pass
    if error:
        return {"ok": False, "sent": sent, "error": error}
    return {"ok": True, "sent": sent}


def publish_twist_stop(host: str, port: int, topic_name: str, timeout: float = 5.0) -> dict[str, Any]:
    """Stream a short burst of zero-velocity Twists (idempotent halt)."""
    return stream_twist(host, port, topic_name, 0.0, 0.0, 0.0, duration_s=0.3, rate_hz=10.0, timeout=timeout)


def _sector_min_range(message: dict, sector_deg: float) -> tuple[float | None, int]:
    """Closest finite range within ±sector/2 around the scan's forward axis.

    Returns ``(min_range_m or None, beams_considered)``. Rosbridge encodes
    infinite ranges as ``null``; those and out-of-band readings are skipped.
    """
    ranges = message.get("ranges") or []
    angle_min = _float(message, "angle_min", 0.0)
    increment = _float(message, "angle_increment", 0.0)
    range_min = _float(message, "range_min", 0.0)
    range_max = _float(message, "range_max", 0.0)
    half = math.radians(min(360.0, max(1.0, float(sector_deg)))) / 2.0
    best: float | None = None
    samples = 0
    for index, raw in enumerate(ranges):
        try:
            value = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(value):
            continue
        angle = angle_min + index * increment
        angle = math.atan2(math.sin(angle), math.cos(angle))  # normalize to [-pi, pi]
        if abs(angle) > half:
            continue
        if value < max(range_min, 0.01):
            continue
        if range_max > 0.0 and value > range_max:
            continue
        samples += 1
        best = value if best is None else min(best, value)
    return best, samples


def _yaw_degrees(orientation: dict) -> float:
    x = _float(orientation, "x", 0.0)
    y = _float(orientation, "y", 0.0)
    z = _float(orientation, "z", 0.0)
    w = _float(orientation, "w", 1.0)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _parse_odometry(message: dict) -> tuple[dict[str, float], dict[str, float]]:
    pose = ((message.get("pose") or {}).get("pose")) or {}
    point = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    twist = ((message.get("twist") or {}).get("twist")) or {}
    linear = twist.get("linear") or {}
    angular = twist.get("angular") or {}
    position = {
        "x": round(_float(point, "x", 0.0), 4),
        "y": round(_float(point, "y", 0.0), 4),
        "yaw_deg": round(_yaw_degrees(orientation), 2),
    }
    velocity = {
        "vx": round(_float(linear, "x", 0.0), 4),
        "vy": round(_float(linear, "y", 0.0), 4),
        "wz": round(_float(angular, "z", 0.0), 4),
    }
    return position, velocity


def _authorization_error(authorization: Any) -> str:
    """Empty string when the authorization is present, positive, and fresh."""
    if not isinstance(authorization, dict) or not authorization:
        return "no authorization (wire BaseSafetyGate.authorization into this node)"
    if not authorization.get("authorized"):
        reason = str(authorization.get("reason") or "the gate did not authorize motion")
        return f"authorization refused: {reason}"
    try:
        age = time.time() - float(authorization.get("issued_at"))
    except (TypeError, ValueError):
        return "authorization has no issued_at timestamp; re-run BaseSafetyGate"
    max_age = _float(authorization, "max_age_s", DEFAULT_AUTH_MAX_AGE_S)
    if age > max_age:
        return f"authorization is stale ({age:.0f}s old, limit {max_age:.0f}s); re-run BaseSafetyGate"
    return ""


@node(
    name="BaseSafetyGate",
    category=_CATEGORY,
    description=(
        "Authorize bounded mobile-base motion: arm/disarm switch, speed/turn/duration caps, "
        "optional LiDAR clearance requirement, and an emergency stop. ROS2BaseMove refuses to "
        "run without this node's fresh authorization."
    ),
    inputs={
        "armed": Bool(default=False),
        "max_speed_mps": Float(default=0.2),
        "max_turn_rps": Float(default=0.6),
        "max_duration_s": Float(default=1.5),
        "min_clearance_m": Float(default=0.35),
        "clearance_m": Float(default=-1.0),
        "require_clearance": Bool(default=False),
        "emergency_stop": Bool(default=False),
    },
    outputs={"authorization": Dict, "authorized": Bool, "report": Text},
)
def base_safety_gate(ctx: dict) -> dict:
    armed = bool(ctx.get("armed", False))
    emergency_stop = bool(ctx.get("emergency_stop", False))
    require_clearance = bool(ctx.get("require_clearance", False))
    max_speed = min(HARD_MAX_SPEED_MPS, abs(_float(ctx, "max_speed_mps", 0.2)))
    max_turn = min(HARD_MAX_TURN_RPS, abs(_float(ctx, "max_turn_rps", 0.6)))
    max_duration = min(HARD_MAX_DURATION_S, abs(_float(ctx, "max_duration_s", 1.5)))
    min_clearance = max(0.0, _float(ctx, "min_clearance_m", 0.35))
    clearance = _float(ctx, "clearance_m", -1.0)

    reason = ""
    if emergency_stop:
        reason = "emergency stop engaged"
    elif not armed:
        reason = "gate is disarmed (set armed=true after checking the robot's surroundings)"
    elif 0.0 <= clearance < min_clearance:
        reason = f"obstacle at {clearance:.2f} m is inside the {min_clearance:.2f} m minimum clearance"
    elif require_clearance and clearance < 0.0:
        reason = "no clearance reading (wire ROS2LaserScanCheck.clearance_m or set require_clearance=false)"

    authorized = reason == ""
    authorization = {
        "authorized": authorized,
        "max_speed_mps": max_speed,
        "max_turn_rps": max_turn,
        "max_duration_s": max_duration,
        "issued_at": time.time(),
        "max_age_s": DEFAULT_AUTH_MAX_AGE_S,
        "reason": reason or "authorized",
    }
    clearance_line = (
        f"clearance: {clearance:.2f} m (min {min_clearance:.2f} m)"
        if clearance >= 0.0
        else "clearance: no reading wired"
    )
    verdict = "base motion AUTHORIZED" if authorized else f"base motion BLOCKED: {reason}"
    report = "\n".join(
        [
            verdict,
            f"caps: {max_speed:g} m/s, {max_turn:g} rad/s, {max_duration:g} s per move",
            clearance_line,
            f"authorization valid for {DEFAULT_AUTH_MAX_AGE_S:g} s",
        ]
    )
    return {"authorization": authorization, "authorized": authorized, "report": report}


@node(
    name="ROS2BaseMove",
    category=_CATEGORY,
    description=(
        "One bounded velocity move over rosbridge: stream geometry_msgs/Twist to a cmd_vel topic "
        "for a fixed duration, clamped to BaseSafetyGate caps, always ending in a zero-velocity stop."
    ),
    inputs={
        "authorization": Dict,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "cmd_vel_topic": Text(default="/cmd_vel"),
        "vx": Float(default=0.1),
        "vy": Float(default=0.0),
        "wz": Float(default=0.0),
        "duration_s": Float(default=1.0),
        "rate_hz": Float(default=10.0),
        "timeout": Float(default=10.0),
    },
    outputs={"moved": Bool, "command": Dict, "report": Text},
)
def ros2_base_move(ctx: dict) -> dict:
    error = _authorization_error(ctx.get("authorization"))
    if error:
        return {"moved": False, "command": {}, "report": f"base move BLOCKED: {error}"}
    authorization = ctx["authorization"]

    requested_duration = _float(ctx, "duration_s", 1.0)
    if requested_duration <= 0.0:
        return {"moved": False, "command": {}, "report": "base move FAILED: set duration_s > 0"}

    max_speed = min(HARD_MAX_SPEED_MPS, abs(_float(authorization, "max_speed_mps", 0.2)))
    max_turn = min(HARD_MAX_TURN_RPS, abs(_float(authorization, "max_turn_rps", 0.6)))
    max_duration = min(HARD_MAX_DURATION_S, abs(_float(authorization, "max_duration_s", 1.5)))

    vx = _float(ctx, "vx", 0.1)
    vy = _float(ctx, "vy", 0.0)
    wz = _float(ctx, "wz", 0.0)
    clamped = False
    speed = math.hypot(vx, vy)
    if speed > max_speed and speed > 0.0:
        scale = max_speed / speed
        vx *= scale
        vy *= scale
        clamped = True
    if abs(wz) > max_turn:
        wz = math.copysign(max_turn, wz)
        clamped = True
    duration = min(requested_duration, max_duration)
    if duration < requested_duration:
        clamped = True

    host = str(ctx.get("host") or "127.0.0.1").strip()
    port = int(ctx.get("port") or 9090)
    topic = str(ctx.get("cmd_vel_topic") or "/cmd_vel").strip()
    rate = min(30.0, max(2.0, _float(ctx, "rate_hz", 10.0)))
    result = stream_twist(
        host, port, topic, vx, vy, wz, duration, rate,
        timeout=_float(ctx, "timeout", 10.0),
    )
    command = {
        "topic": topic,
        "vx": round(vx, 4),
        "vy": round(vy, 4),
        "wz": round(wz, 4),
        "duration_s": round(duration, 3),
        "rate_hz": rate,
        "frames_sent": int(result.get("sent", 0)),
    }
    note = " (clamped to gate caps)" if clamped else ""
    if not result.get("ok"):
        return {
            "moved": False,
            "command": command,
            "report": (
                f"base move FAILED: {result.get('error', 'unknown error')} "
                f"(sent {command['frames_sent']} frame(s); zero-velocity stop was streamed)"
            ),
        }
    return {
        "moved": True,
        "command": command,
        "report": (
            f"base move OK: vx={vx:.2f} m/s vy={vy:.2f} m/s wz={wz:.2f} rad/s "
            f"for {duration:g} s ({command['frames_sent']} frames) on {topic}, then stopped{note}"
        ),
    }


@node(
    name="ROS2BaseStop",
    category=_CATEGORY,
    description=(
        "Big red button: stream zero-velocity Twist messages to halt the base immediately. "
        "Deliberately requires no authorization."
    ),
    inputs={
        "trigger": AnyPort,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "cmd_vel_topic": Text(default="/cmd_vel"),
        "timeout": Float(default=5.0),
    },
    outputs={"stopped": Bool, "report": Text},
)
def ros2_base_stop(ctx: dict) -> dict:
    host = str(ctx.get("host") or "127.0.0.1").strip()
    port = int(ctx.get("port") or 9090)
    topic = str(ctx.get("cmd_vel_topic") or "/cmd_vel").strip()
    result = publish_twist_stop(host, port, topic, timeout=_float(ctx, "timeout", 5.0))
    if not result.get("ok"):
        return {"stopped": False, "report": f"base stop FAILED: {result.get('error', 'unknown error')}"}
    return {"stopped": True, "report": f"base stop OK: zero velocity streamed to {topic}"}


@node(
    name="ROS2LaserScanCheck",
    category=_CATEGORY,
    description=(
        "Read one sensor_msgs/LaserScan over rosbridge and report the closest obstacle inside a "
        "forward sector — wire clearance_m into BaseSafetyGate."
    ),
    inputs={
        "trigger": AnyPort,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "scan_topic": Text(default="/scan"),
        "sector_deg": Float(default=60.0),
        "min_clearance_m": Float(default=0.35),
        "timeout": Float(default=5.0),
    },
    outputs={"clearance_m": Float, "clear": Bool, "samples": Int, "report": Text},
)
def ros2_laser_scan_check(ctx: dict) -> dict:
    host = str(ctx.get("host") or "127.0.0.1").strip()
    port = int(ctx.get("port") or 9090)
    topic = str(ctx.get("scan_topic") or "/scan").strip()
    sector = _float(ctx, "sector_deg", 60.0)
    min_clearance = max(0.0, _float(ctx, "min_clearance_m", 0.35))
    timeout = max(0.5, _float(ctx, "timeout", 5.0))

    failed = {"clearance_m": -1.0, "clear": False, "samples": 0}
    ok, err = rb.available()
    if not ok:
        return {**failed, "report": f"laser scan check FAILED: {err}"}
    try:
        ros = rb.get_connection(host, port, timeout)
    except Exception as exc:
        return {**failed, "report": f"laser scan check FAILED: {type(exc).__name__}: {exc}"}
    message = rb._read_once(ros, topic, SCAN_TYPE, timeout)
    if message is None:
        return {**failed, "report": f"laser scan check FAILED: no scan on {topic} within {timeout:g} s"}
    best, samples = _sector_min_range(message, sector)
    if best is None:
        return {
            **failed,
            "report": f"laser scan check FAILED: scan received but no valid beams in the ±{sector / 2:g}° sector",
        }
    clear = best >= min_clearance
    verdict = "CLEAR" if clear else "BLOCKED"
    return {
        "clearance_m": round(best, 3),
        "clear": clear,
        "samples": samples,
        "report": (
            f"closest obstacle {best:.2f} m within ±{sector / 2:g}° ({samples} beams) — "
            f"{verdict} at min clearance {min_clearance:.2f} m"
        ),
    }


@node(
    name="ROS2OdomState",
    category=_CATEGORY,
    description="Read one nav_msgs/Odometry over rosbridge: planar pose (x, y, yaw) and current velocity.",
    inputs={
        "trigger": AnyPort,
        "host": Text(default="127.0.0.1"),
        "port": Int(default=9090),
        "odom_topic": Text(default="/odom"),
        "timeout": Float(default=5.0),
    },
    outputs={"position": Dict, "velocity": Dict, "report": Text},
)
def ros2_odom_state(ctx: dict) -> dict:
    host = str(ctx.get("host") or "127.0.0.1").strip()
    port = int(ctx.get("port") or 9090)
    topic = str(ctx.get("odom_topic") or "/odom").strip()
    timeout = max(0.5, _float(ctx, "timeout", 5.0))

    ok, err = rb.available()
    if not ok:
        return {"position": {}, "velocity": {}, "report": f"odom read FAILED: {err}"}
    try:
        ros = rb.get_connection(host, port, timeout)
    except Exception as exc:
        return {"position": {}, "velocity": {}, "report": f"odom read FAILED: {type(exc).__name__}: {exc}"}
    message = rb._read_once(ros, topic, ODOM_TYPE, timeout)
    if message is None:
        return {"position": {}, "velocity": {}, "report": f"odom read FAILED: no message on {topic} within {timeout:g} s"}
    position, velocity = _parse_odometry(message)
    return {
        "position": position,
        "velocity": velocity,
        "report": (
            f"pose x={position['x']:.2f} m y={position['y']:.2f} m yaw={position['yaw_deg']:.1f}° | "
            f"v=({velocity['vx']:.2f}, {velocity['vy']:.2f}) m/s wz={velocity['wz']:.2f} rad/s from {topic}"
        ),
    }
