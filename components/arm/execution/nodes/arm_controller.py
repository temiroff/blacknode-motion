"""Canonical arm execution gateway used by UI surfaces and robot skills."""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Mapping

from blacknode.pkg.blacknode_motion.core.arbitration import (
    MotionAuthorizationError,
    command_arbiter,
)


def execute_joint_target(
    publish: Callable[[dict[str, float]], Mapping[str, Any] | None],
    *,
    resource: str,
    owner: str,
    current: Mapping[str, float],
    target: Mapping[str, float],
    limits: Mapping[str, tuple[float, float]],
    armed: bool,
    feedback_age: float = 0.0,
    stale_after: float = 0.75,
    max_velocity: float = math.inf,
    interval: float = 1.0,
    priority: int = 0,
    emergency_stop: bool = False,
    collision_check: Callable[[Mapping[str, float]], bool] | None = None,
) -> dict[str, Any]:
    """Arbitrate, safety-check, and publish one joint target.

    Positions and velocities use radians and seconds. The transport callback is
    deliberately injected here so UI and skill modules never publish to a
    physical driver's topic themselves.
    """
    try:
        lease = command_arbiter.authorize(
            resource,
            owner,
            armed=armed,
            priority=priority,
            ttl=max(0.1, stale_after),
            emergency_stop=emergency_stop,
        )
    except MotionAuthorizationError as exc:
        return {"ok": False, "error": str(exc)}
    if feedback_age < 0.0 or feedback_age > max(0.05, stale_after):
        return {"ok": False, "error": "joint feedback is stale"}
    current_values = {
        str(name): float(value)
        for name, value in current.items()
    }
    requested = {
        str(name): float(value)
        for name, value in target.items()
    }
    if not requested or set(requested) - set(current_values):
        return {"ok": False, "error": "joint target does not match current state"}
    moving = {
        name
        for name, value in requested.items()
        if abs(value - current_values[name]) > 1e-12
    }
    missing_limits = sorted(moving - set(limits))
    if missing_limits:
        return {
            "ok": False,
            "error": (
                "calibrated joint limits are missing for: "
                + ", ".join(missing_limits)
            ),
        }
    safe_target: dict[str, float] = {}
    clamped: list[str] = []
    max_delta = (
        max(0.0, float(max_velocity)) * max(0.001, float(interval))
        if math.isfinite(float(max_velocity))
        else math.inf
    )
    for name, value in requested.items():
        if name in limits:
            lower, upper = limits[name]
            bounded = min(float(upper), max(float(lower), value))
        else:
            bounded = current_values[name]
        start = current_values[name]
        velocity_bounded = min(start + max_delta, max(start - max_delta, bounded))
        if abs(velocity_bounded - value) > 1e-12:
            clamped.append(name)
        safe_target[name] = velocity_bounded
    if collision_check is not None:
        try:
            collision_free = bool(collision_check(safe_target))
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": f"collision check failed: {exc}"}
        if not collision_free:
            return {"ok": False, "error": "collision check rejected joint target"}
    try:
        result = dict(publish(safe_target) or {})
    except Exception as exc:  # noqa: BLE001 - transport errors are structured
        return {"ok": False, "error": str(exc)}
    if result.get("ok") is False:
        return result
    return {
        **result,
        "ok": True,
        "target": safe_target,
        "clamped": sorted(set(clamped)),
        "authorization": lease.as_dict(),
        "issued_at": time.time(),
    }


def release_motion_owner(owner: str) -> None:
    command_arbiter.release_owner(owner)
