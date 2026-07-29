"""Reusable point-to-point joint motion profiles.

The profile node is transport-neutral and belongs to the arm controller. It
produces a small canonical contract that any
motion adapter can consume: angular limits are stored in radians and the
generated trajectory is a normalized 0..1 path.
"""
from __future__ import annotations

import base64
import html
import math
from typing import Any

from blacknode.node import Dict, Enum, Float, Image, List, Text, node


_MODES = ("direct", "linear", "trapezoidal", "minimum_jerk")
_MAX_FRAMES = 10_000


def _positive(value: Any, fallback: float, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        result = fallback
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return result


def canonical_profile(
    *,
    mode: str,
    units: str,
    min_duration: float,
    max_velocity: float,
    max_acceleration: float,
    rate_hz: float,
) -> dict[str, Any]:
    """Return a validated profile contract using radians internally."""
    selected = str(mode or "trapezoidal").strip().lower()
    if selected not in _MODES:
        raise ValueError(f"unknown motion profile '{selected}'")
    selected_units = str(units or "degrees").strip().lower()
    if selected_units not in {"degrees", "radians"}:
        raise ValueError("units must be 'degrees' or 'radians'")

    duration = max(0.0, float(min_duration or 0.0))
    rate = _positive(rate_hz, 50.0, "rate_hz")
    velocity = _positive(max_velocity, 90.0, "max_velocity")
    acceleration = _positive(max_acceleration, 180.0, "max_acceleration")
    if selected_units == "degrees":
        velocity = math.radians(velocity)
        acceleration = math.radians(acceleration)

    return {
        "kind": "blacknode.joint-motion-profile",
        "schema_version": 1,
        "mode": selected,
        "min_duration": duration,
        "max_velocity_rad_s": velocity,
        "max_acceleration_rad_s2": acceleration,
        "rate_hz": rate,
    }


def _profile_duration(profile: dict[str, Any], distance: float) -> tuple[float, dict[str, float]]:
    mode = str(profile["mode"])
    minimum = max(0.0, float(profile.get("min_duration") or 0.0))
    velocity = _positive(profile.get("max_velocity_rad_s"), math.radians(90.0), "max_velocity")
    acceleration = _positive(
        profile.get("max_acceleration_rad_s2"), math.radians(180.0), "max_acceleration"
    )
    distance = abs(float(distance))

    if distance <= 1e-12 or mode == "direct":
        return 0.0, {"base_duration": 0.0}
    if mode == "linear":
        base = distance / velocity
        return max(minimum, base), {"base_duration": base}
    if mode == "minimum_jerk":
        by_velocity = 1.875 * distance / velocity
        by_acceleration = math.sqrt((10.0 / math.sqrt(3.0)) * distance / acceleration)
        base = max(by_velocity, by_acceleration)
        return max(minimum, base), {"base_duration": base}

    # Rest-to-rest trapezoid. Short moves naturally become triangular.
    t_accel = velocity / acceleration
    d_accel = 0.5 * acceleration * t_accel * t_accel
    if 2.0 * d_accel >= distance:
        t_accel = math.sqrt(distance / acceleration)
        t_flat = 0.0
        peak_velocity = acceleration * t_accel
    else:
        t_flat = (distance - 2.0 * d_accel) / velocity
        peak_velocity = velocity
    base = 2.0 * t_accel + t_flat
    return max(minimum, base), {
        "base_duration": base,
        "t_accel": t_accel,
        "t_flat": t_flat,
        "peak_velocity": peak_velocity,
    }


def plan_profile(profile: dict[str, Any], distance_rad: float) -> dict[str, Any]:
    """Sample a normalized point-to-point profile for an angular distance."""
    if profile.get("kind") != "blacknode.joint-motion-profile":
        raise ValueError("motion profile has an unsupported kind")
    mode = str(profile.get("mode") or "")
    if mode not in _MODES:
        raise ValueError(f"unknown motion profile '{mode}'")
    distance = abs(float(distance_rad))
    if not math.isfinite(distance):
        raise ValueError("distance must be finite")
    rate = _positive(profile.get("rate_hz"), 50.0, "rate_hz")
    duration, shape = _profile_duration(profile, distance)

    if distance <= 1e-12:
        return {
            "mode": mode,
            "duration": 0.0,
            "rate_hz": rate,
            "alphas": [0.0, 1.0],
            "samples": [
                {"time": 0.0, "position": 0.0, "velocity": 0.0, "acceleration": 0.0}
            ],
            "peak_velocity_rad_s": 0.0,
            "peak_acceleration_rad_s2": 0.0,
        }
    if mode == "direct":
        return {
            "mode": mode,
            "duration": 0.0,
            "rate_hz": rate,
            "alphas": [0.0, 1.0],
            "samples": [
                {"time": 0.0, "position": 0.0, "velocity": 0.0, "acceleration": 0.0},
                {"time": 0.0, "position": distance, "velocity": 0.0, "acceleration": 0.0},
            ],
            "peak_velocity_rad_s": 0.0,
            "peak_acceleration_rad_s2": 0.0,
        }

    frames = max(1, int(math.ceil(duration * rate)))
    if frames > _MAX_FRAMES:
        raise ValueError(
            f"profile needs {frames} frames; reduce duration/rate below {_MAX_FRAMES} frames"
        )
    alphas: list[float] = []
    samples: list[dict[str, float]] = []
    base_duration = float(shape["base_duration"])
    stretch = duration / base_duration if base_duration > 0.0 else 1.0
    velocity_limit = float(profile["max_velocity_rad_s"])
    acceleration_limit = float(profile["max_acceleration_rad_s2"])

    for frame in range(frames + 1):
        t = duration * frame / frames
        u = frame / frames
        if mode == "linear":
            alpha = u
            velocity = distance / duration
            acceleration = 0.0
        elif mode == "minimum_jerk":
            alpha = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
            velocity = distance * (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / duration
            acceleration = distance * (60.0 * u - 180.0 * u**2 + 120.0 * u**3) / duration**2
        else:
            base_t = min(base_duration, t / stretch)
            t_accel = float(shape["t_accel"])
            t_flat = float(shape["t_flat"])
            peak = float(shape["peak_velocity"])
            accel = acceleration_limit
            if base_t <= t_accel:
                position = 0.5 * accel * base_t**2
                base_velocity = accel * base_t
                base_acceleration = accel
            elif base_t <= t_accel + t_flat:
                position = 0.5 * accel * t_accel**2 + peak * (base_t - t_accel)
                base_velocity = peak
                base_acceleration = 0.0
            else:
                td = base_t - t_accel - t_flat
                position = (
                    0.5 * accel * t_accel**2
                    + peak * t_flat
                    + peak * td
                    - 0.5 * accel * td**2
                )
                base_velocity = max(0.0, peak - accel * td)
                base_acceleration = -accel
            alpha = position / distance
            velocity = base_velocity / stretch
            acceleration = base_acceleration / stretch**2

        alpha = min(1.0, max(0.0, alpha))
        alphas.append(alpha)
        samples.append(
            {
                "time": t,
                "position": distance * alpha,
                "velocity": velocity,
                "acceleration": acceleration,
            }
        )

    alphas[0], alphas[-1] = 0.0, 1.0
    samples[0]["position"], samples[-1]["position"] = 0.0, distance
    peak_velocity = max(abs(item["velocity"]) for item in samples)
    peak_acceleration = max(abs(item["acceleration"]) for item in samples)
    # Floating-point sampling can land a few ulps above the declared bounds.
    peak_velocity = min(peak_velocity, velocity_limit)
    peak_acceleration = min(peak_acceleration, acceleration_limit)
    return {
        "mode": mode,
        "duration": duration,
        "rate_hz": rate,
        "alphas": alphas,
        "samples": samples,
        "peak_velocity_rad_s": peak_velocity,
        "peak_acceleration_rad_s2": peak_acceleration,
    }


def _svg_data(plan: dict[str, Any]) -> str:
    samples = list(plan.get("samples") or [])
    width, height = 620, 250
    if not samples:
        return ""
    duration = max(float(plan.get("duration") or 0.0), 1e-9)
    max_position = max((abs(float(item["position"])) for item in samples), default=1.0) or 1.0
    max_velocity = max((abs(float(item["velocity"])) for item in samples), default=1.0) or 1.0

    def points(key: str, maximum: float, top: float, bottom: float) -> str:
        values = []
        for item in samples:
            x = 42.0 + 550.0 * float(item["time"]) / duration
            y = bottom - (bottom - top) * max(0.0, float(item[key])) / maximum
            values.append(f"{x:.1f},{y:.1f}")
        return " ".join(values)

    mode = html.escape(str(plan.get("mode") or "profile"))
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">
<rect width="100%" height="100%" rx="12" fill="#17191d"/>
<text x="24" y="28" fill="#f3f4f6" font-family="sans-serif" font-size="15" font-weight="700">{mode}</text>
<text x="596" y="28" text-anchor="end" fill="#9ca3af" font-family="sans-serif" font-size="12">{plan.get("duration", 0.0):.3f} s</text>
<line x1="42" y1="210" x2="592" y2="210" stroke="#4b5563"/>
<line x1="42" y1="44" x2="42" y2="210" stroke="#4b5563"/>
<polyline points="{points("position", max_position, 54, 205)}" fill="none" stroke="#3ddc97" stroke-width="3"/>
<polyline points="{points("velocity", max_velocity, 54, 205)}" fill="none" stroke="#60a5fa" stroke-width="2"/>
<text x="52" y="232" fill="#3ddc97" font-family="sans-serif" font-size="11">position</text>
<text x="116" y="232" fill="#60a5fa" font-family="sans-serif" font-size="11">velocity</text>
</svg>"""
    encoded = base64.b64encode(svg.encode("utf-8")).decode("ascii")
    return f"data:image/svg+xml;base64,{encoded}"


@node(
    name="JointMotionProfile",
    category="Motion",
    description=(
        "Build and preview a reusable point-to-point joint motion profile. "
        "Wire profile into ROS2SetJoint; motion remains gated by that node's armed input."
    ),
    inputs={
        "mode": Enum(list(_MODES), default="trapezoidal"),
        "distance": Float(default=30.0),
        "units": Enum(["degrees", "radians"], default="degrees"),
        "min_duration": Float(default=0.8),
        "max_velocity": Float(default=90.0),
        "max_acceleration": Float(default=180.0),
        "rate_hz": Float(default=50.0),
    },
    outputs={
        "profile": Dict,
        "samples": List,
        "duration": Float,
        "preview": Image,
        "report": Text,
    },
)
def joint_motion_profile(ctx: dict) -> dict:
    units = str(ctx.get("units") or "degrees")
    try:
        profile = canonical_profile(
            mode=str(ctx.get("mode") or "trapezoidal"),
            units=units,
            min_duration=float(ctx.get("min_duration") or 0.0),
            max_velocity=float(ctx.get("max_velocity") or 0.0),
            max_acceleration=float(ctx.get("max_acceleration") or 0.0),
            rate_hz=float(ctx.get("rate_hz") or 0.0),
        )
        distance = abs(float(ctx.get("distance") or 0.0))
        distance_rad = math.radians(distance) if units == "degrees" else distance
        plan = plan_profile(profile, distance_rad)
    except (TypeError, ValueError) as exc:
        return {
            "profile": {},
            "samples": [],
            "duration": 0.0,
            "preview": "",
            "report": f"motion profile BLOCKED: {exc}",
        }

    scale = 180.0 / math.pi if units == "degrees" else 1.0
    samples = [
        {
            "time": item["time"],
            "position": item["position"] * scale,
            "velocity": item["velocity"] * scale,
            "acceleration": item["acceleration"] * scale,
        }
        for item in plan["samples"]
    ]
    if plan["mode"] == "direct":
        report = (
            f"direct profile: {distance:g} {units}; current pose is synchronized, "
            "then the final target is sent with no bounded intermediate ramp."
        )
    elif plan["mode"] == "linear":
        report = (
            f"linear profile: {distance:g} {units} in {plan['duration']:.3f}s, "
            f"{len(plan['alphas'])} targets at {plan['rate_hz']:g} Hz; "
            f"constant velocity {plan['peak_velocity_rad_s'] * scale:.2f} {units}/s "
            "with discontinuous endpoint acceleration."
        )
    else:
        report = (
            f"{plan['mode']} profile: {distance:g} {units} in {plan['duration']:.3f}s, "
            f"{len(plan['alphas'])} targets at {plan['rate_hz']:g} Hz; "
            f"peak velocity {plan['peak_velocity_rad_s'] * scale:.2f} {units}/s, "
            f"peak acceleration {plan['peak_acceleration_rad_s2'] * scale:.2f} {units}/s²."
        )
    return {
        "profile": profile,
        "samples": samples,
        "duration": plan["duration"],
        "preview": _svg_data(plan),
        "report": report,
    }
