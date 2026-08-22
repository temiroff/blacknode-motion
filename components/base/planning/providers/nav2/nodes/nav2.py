"""Session-scoped Nav2 provider and safety-gated NavigateToPose operation."""
from __future__ import annotations

import json
import math
import re
import shlex
import time
from pathlib import Path
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Int, Text, node


class _LazyROS2Runtime:
    def __getattr__(self, name: str):
        from blacknode.pkg.blacknode_ros2 import ros2_runtime

        return getattr(ros2_runtime, name)


rt = _LazyROS2Runtime()
_CATEGORY = "Motion"
_ACTION_TYPE = "nav2_msgs/action/NavigateToPose"
_HARD_MAX_GOAL_TIMEOUT_S = 300.0


def _safe_id(value: Any, prefix: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_-]+", "_", str(value or "rosorin").strip()).strip("_")
    return f"{prefix}-{clean or 'rosorin'}"[:64].rstrip("_-")


def _float(ctx: dict, name: str, default: float) -> float:
    try:
        return float(ctx.get(name) if ctx.get(name) not in (None, "") else default)
    except (TypeError, ValueError):
        return float(default)


def _actions() -> dict[str, Any]:
    result = rt.run_ros2(["action", "list", "-t"], timeout=15.0)
    discovered: dict[str, str] = {}
    if result.get("ok"):
        for raw in str(result.get("stdout") or "").splitlines():
            match = re.match(r"^(\S+)\s+\[([^\]]+)]\s*$", raw.strip())
            if match:
                discovered[match.group(1)] = match.group(2).strip()
            elif raw.strip():
                discovered[raw.strip().split()[0]] = ""
    return {**result, "actions": discovered}


def _nav_status(action_name: str, run_id: str, lifecycle: str) -> dict[str, Any]:
    actions = _actions()
    action_type = (actions.get("actions") or {}).get(action_name, "")
    owned = rt.ros2_managed_status(run_id)
    available = action_name in (actions.get("actions") or {}) and (
        not action_type or action_type == _ACTION_TYPE
    )
    return {
        "kind": "blacknode.navigation-provider-state",
        "schema_version": 1,
        "provider": "nav2",
        "transport": "ros2",
        "lifecycle": lifecycle,
        "run_id": run_id,
        "action_name": action_name,
        "action_type": action_type,
        "ready": available,
        "running": bool(available or owned.get("running")),
        "owned_by_blacknode": bool(owned.get("running")),
        "backend": str(actions.get("backend") or owned.get("backend") or "none"),
        "error": str(actions.get("error") or owned.get("error") or ""),
        "observed_at": time.time(),
    }


def _goal_runtime(run_id: str) -> tuple[dict[str, Any], list[str]]:
    status = rt.ros2_managed_status(run_id)
    logs: list[str] = []
    try:
        for item in rt.runtime_status().get("node_outputs") or []:
            if str(item.get("run_id") or "") == run_id:
                logs = [str(line) for line in ((item.get("outputs") or {}).get("logs") or [])]
                break
    except Exception:
        pass
    return status, logs


def _last_navigation_event(logs: list[str]) -> dict[str, Any]:
    for line in reversed(logs):
        try:
            event = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            continue
        if isinstance(event, dict) and event.get("kind") == "blacknode.navigation-event":
            return event
    return {}


def _last_exploration_event(logs: list[str]) -> dict[str, Any]:
    for line in reversed(logs):
        try:
            event = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            continue
        if isinstance(event, dict) and event.get("kind") == "blacknode.exploration-event":
            return event
    return {}


def _authorization_error(authorization: Any) -> str:
    if not isinstance(authorization, dict) or not authorization:
        return "no authorization (wire BaseSafetyGate.authorization into this motion node)"
    if not authorization.get("authorized"):
        return f"authorization refused: {authorization.get('reason') or 'base motion is disarmed'}"
    try:
        issued_at = float(authorization.get("issued_at"))
        max_age = float(authorization.get("max_age_s") or 30.0)
    except (TypeError, ValueError):
        return "authorization has no valid freshness timestamp"
    if time.time() - issued_at > max_age:
        return "authorization is stale; re-run BaseSafetyGate"
    return ""


def _deep_merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _prepare_nav2_params(params_file: str, controller_params_file: str, run_id: str) -> tuple[str, str]:
    """Build a Blacknode-owned Nav2 parameter overlay without editing vendor files."""
    source = Path(params_file).expanduser().resolve()
    if not source.is_file():
        return "", f"Nav2 params file does not exist: {source}"
    if not controller_params_file:
        return str(source), ""
    controller = Path(controller_params_file).expanduser().resolve()
    if not controller.is_file():
        return "", f"Nav2 controller params file does not exist: {controller}"
    try:
        import yaml

        main_payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
        controller_payload = yaml.safe_load(controller.read_text(encoding="utf-8")) or {}
        if not isinstance(main_payload, dict) or not isinstance(controller_payload, dict):
            return "", "Nav2 parameter files must contain YAML objects"
        merged = _deep_merge(main_payload, controller_payload)
        output_dir = Path.home() / "Blacknode" / "runtime" / "nav2"
        output_dir.mkdir(parents=True, exist_ok=True)
        output = output_dir / f"{run_id}-params.yaml"
        output.write_text(yaml.safe_dump(merged, sort_keys=False), encoding="utf-8")
        return str(output), ""
    except Exception as exc:
        return "", f"could not prepare Nav2 parameter overlay: {type(exc).__name__}: {exc}"


def _zero_twist(topic: str) -> None:
    request = {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
    }
    try:
        rt.run_ros2(
            ["topic", "pub", "--once", topic, "geometry_msgs/msg/Twist", json.dumps(request)],
            timeout=5.0,
        )
    except Exception:
        pass


@node(
    name="NavigationSession",
    category=_CATEGORY,
    description=(
        "Attach to an existing Nav2 stack or start and stop a Blacknode-owned Nav2 session "
        "against a persisted map. Vendor boot services remain unchanged."
    ),
    inputs={
        "trigger": AnyPort,
        "action": Enum(["status", "start", "stop"], default="status"),
        "lifecycle": Enum(["existing", "managed"], default="existing"),
        "map_mode": Enum(["saved", "live_slam"], default="saved"),
        "run_id": Text(default="rosorin-nav2"),
        "action_name": Text(default="/navigate_to_pose"),
        "map_artifact": Dict,
        "map_yaml": Text(default=""),
        "params_file": Text(default=""),
        "controller_params_file": Text(default=""),
        "launch_package": Text(default="nav2_bringup"),
        "launch_file": Text(default="bringup_launch.py"),
        "launch_arguments": Text(default="use_sim_time:=false autostart:=true"),
        "wait_seconds": Float(default=15.0),
    },
    outputs={"ready": Bool, "running": Bool, "owned": Bool, "provider": Dict, "report": Text},
    primary_inputs=["trigger", "action", "lifecycle", "map_mode", "map_artifact"],
    primary_outputs=["provider", "report"],
)
def navigation_session(ctx: dict) -> dict:
    action = str(ctx.get("action") or "status").strip().lower()
    lifecycle = str(ctx.get("lifecycle") or "existing").strip().lower()
    map_mode = str(ctx.get("map_mode") or "saved").strip().lower()
    action_name = str(ctx.get("action_name") or "/navigate_to_pose").strip()
    run_id = _safe_id(ctx.get("run_id"), "blacknode-nav2")
    state = _nav_status(action_name, run_id, lifecycle)

    if action == "start" and lifecycle == "managed" and not state["ready"]:
        artifact = ctx.get("map_artifact") if isinstance(ctx.get("map_artifact"), dict) else {}
        map_yaml = str(ctx.get("map_yaml") or artifact.get("map_yaml") or "").strip()
        params_file = str(ctx.get("params_file") or "").strip()
        if (map_mode == "saved" and not map_yaml) or not params_file:
            return {
                "ready": False,
                "running": False,
                "owned": False,
                "provider": state,
                "report": (
                    "navigation start FAILED: managed saved-map lifecycle requires map_yaml/map_artifact and params_file"
                    if map_mode == "saved"
                    else "navigation start FAILED: managed live-SLAM lifecycle requires params_file"
                ),
            }
        resolved_map = Path(map_yaml).expanduser().resolve() if map_mode == "saved" and map_yaml else None
        if map_mode == "saved" and (resolved_map is None or not resolved_map.is_file()):
            return {
                "ready": False,
                "running": False,
                "owned": False,
                "provider": state,
                "report": f"navigation start FAILED: saved map does not exist: {resolved_map}",
            }
        effective_params, params_error = _prepare_nav2_params(
            params_file,
            str(ctx.get("controller_params_file") or "").strip(),
            run_id,
        )
        if params_error:
            return {
                "ready": False,
                "running": False,
                "owned": False,
                "provider": state,
                "report": f"navigation start FAILED: {params_error}",
            }
        try:
            extra = shlex.split(str(ctx.get("launch_arguments") or ""))
        except ValueError as exc:
            return {"ready": False, "running": False, "owned": False, "provider": state, "report": f"navigation start FAILED: invalid launch_arguments: {exc}"}
        requested_launch_file = str(ctx.get("launch_file") or "").strip()
        if map_mode == "live_slam" and requested_launch_file in {"", "bringup_launch.py"}:
            launch_file = "navigation_launch.py"
        else:
            launch_file = requested_launch_file or "bringup_launch.py"
        launch_args = [
            "launch",
            str(ctx.get("launch_package") or "nav2_bringup").strip(),
            launch_file,
        ]
        if resolved_map is not None:
            launch_args.append(f"map:={resolved_map}")
        launch_args.extend([f"params_file:={effective_params}", *extra])
        result = rt.run_ros2_managed(
            run_id,
            launch_args,
        )
        if not result.get("ok"):
            return {"ready": False, "running": False, "owned": False, "provider": state, "report": f"navigation start FAILED: {result.get('error') or 'could not launch Nav2'}"}
        deadline = time.monotonic() + max(0.0, min(60.0, _float(ctx, "wait_seconds", 15.0)))
        while time.monotonic() < deadline:
            state = _nav_status(action_name, run_id, lifecycle)
            if state["ready"]:
                break
            time.sleep(0.5)
        state = _nav_status(action_name, run_id, lifecycle)
        state = {
            **state,
            "map_mode": map_mode,
            "map_yaml": str(resolved_map) if resolved_map is not None else "",
            "params_file": effective_params,
        }
        return {
            "ready": bool(state["ready"]),
            "running": bool(state["running"]),
            "owned": bool(state["owned_by_blacknode"]),
            "provider": state,
            "report": (
                f"navigation running: Blacknode owns {run_id} and {action_name} is ready"
                if state["ready"]
                else f"navigation launched as {run_id}, but {action_name} is not ready yet"
            ),
        }

    if action == "start" and state["ready"]:
        return {
            "ready": True,
            "running": True,
            "owned": bool(state["owned_by_blacknode"]),
            "provider": state,
            "report": f"navigation attached: {action_name} is already available; existing ROSOrin bringup was left unchanged",
        }

    if action == "stop":
        stopped = 0
        owned = rt.ros2_managed_status(run_id)
        if owned.get("running"):
            result = rt.stop_ros2_managed(run_id)
            if not result.get("ok"):
                return {"ready": bool(state["ready"]), "running": bool(state["running"]), "owned": True, "provider": state, "report": f"navigation stop FAILED: {result.get('error') or 'owned process could not be stopped'}"}
            stopped = int(result.get("stopped") or 0)
        state = _nav_status(action_name, run_id, lifecycle)
        return {
            "ready": bool(state["ready"]),
            "running": bool(state["running"]),
            "owned": bool(state["owned_by_blacknode"]),
            "provider": state,
            "report": (
                f"stopped {stopped} Blacknode-owned Nav2 process; ROSOrin bringup was left unchanged"
                if stopped
                else "no Blacknode-owned Nav2 process was running; ROSOrin bringup was left unchanged"
            ),
        }

    return {
        "ready": bool(state["ready"]),
        "running": bool(state["running"]),
        "owned": bool(state["owned_by_blacknode"]),
        "provider": state,
        "report": (
            f"navigation ready: {action_name} provides {_ACTION_TYPE}"
            if state["ready"]
            else f"navigation unavailable: {state['error'] or f'{action_name} is not available'}"
        ),
    }


@node(
    name="NavigateTo",
    category=_CATEGORY,
    description=(
        "Preview, send, monitor, or cancel one Blacknode-owned Nav2 goal. Sending requires a "
        "fresh BaseSafetyGate authorization and cancellation ends with a zero Twist."
    ),
    inputs={
        "trigger": AnyPort,
        "action": Enum(["preview", "send", "status", "cancel"], default="preview"),
        "authorization": Dict,
        "goal_id": Text(default="rosorin-goal"),
        "action_name": Text(default="/navigate_to_pose"),
        "frame_id": Text(default="map"),
        "x_m": Float(default=0.0),
        "y_m": Float(default=0.0),
        "yaw_deg": Float(default=0.0),
        "timeout_s": Float(default=180.0),
        "cmd_vel_topic": Text(default="/cmd_vel"),
        "speed_limit_topic": Text(default="/speed_limit"),
    },
    outputs={"accepted": Bool, "running": Bool, "succeeded": Bool, "goal": Dict, "status": Dict, "report": Text},
    primary_inputs=["trigger", "action", "authorization", "x_m", "y_m", "yaw_deg"],
    primary_outputs=["goal", "status", "report"],
)
def navigate_to(ctx: dict) -> dict:
    action = str(ctx.get("action") or "preview").strip().lower()
    run_id = _safe_id(ctx.get("goal_id"), "blacknode-goal")
    action_name = str(ctx.get("action_name") or "/navigate_to_pose").strip()
    frame_id = str(ctx.get("frame_id") or "map").strip()
    x_m = _float(ctx, "x_m", 0.0)
    y_m = _float(ctx, "y_m", 0.0)
    yaw_deg = _float(ctx, "yaw_deg", 0.0)
    goal = {
        "kind": "blacknode.navigation-goal",
        "schema_version": 1,
        "goal_id": run_id,
        "provider": "nav2",
        "frame_id": frame_id,
        "x_m": x_m,
        "y_m": y_m,
        "yaw_deg": yaw_deg,
        "action_name": action_name,
    }

    if action == "preview":
        return {
            "accepted": False,
            "running": False,
            "succeeded": False,
            "goal": goal,
            "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": "preview", "motion_commanded": False},
            "report": f"navigation preview: ({x_m:.2f}, {y_m:.2f}) m at {yaw_deg:.1f}° in {frame_id}; no motion commanded",
        }

    if action == "cancel":
        before, _ = _goal_runtime(run_id)
        result = rt.stop_ros2_python_node(run_id)
        _zero_twist(str(ctx.get("cmd_vel_topic") or "/cmd_vel").strip())
        stopped = int(result.get("stopped") or 0)
        return {
            "accepted": False,
            "running": False,
            "succeeded": False,
            "goal": goal,
            "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": "cancelled" if stopped else "idle", "motion_commanded": False},
            "report": (
                "navigation goal cancelled and zero velocity published"
                if stopped or before.get("running")
                else "no Blacknode-owned navigation goal was running; zero velocity published"
            ),
        }

    if action == "status":
        runtime, logs = _goal_runtime(run_id)
        event = _last_navigation_event(logs)
        state = str(event.get("state") or ("running" if runtime.get("running") else "idle"))
        return {
            "accepted": state in {"accepted", "running", "succeeded"},
            "running": bool(runtime.get("running")),
            "succeeded": state == "succeeded",
            "goal": goal,
            "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": state, "event": event, "logs": logs[-20:]},
            "report": f"navigation goal {run_id}: {state}",
        }

    error = _authorization_error(ctx.get("authorization"))
    if error:
        return {
            "accepted": False,
            "running": False,
            "succeeded": False,
            "goal": goal,
            "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": "blocked", "error": error},
            "report": f"navigation BLOCKED: {error}",
        }
    nav = _actions()
    action_type = (nav.get("actions") or {}).get(action_name, "")
    if action_name not in (nav.get("actions") or {}) or (action_type and action_type != _ACTION_TYPE):
        return {
            "accepted": False,
            "running": False,
            "succeeded": False,
            "goal": goal,
            "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": "unavailable", "error": str(nav.get("error") or "Nav2 action is unavailable")},
            "report": f"navigation FAILED: {action_name} does not provide {_ACTION_TYPE}",
        }
    timeout_s = max(1.0, min(_HARD_MAX_GOAL_TIMEOUT_S, _float(ctx, "timeout_s", 180.0)))
    max_speed_mps = max(0.01, min(0.5, _float(ctx.get("authorization") or {}, "max_speed_mps", 0.15)))
    script = Path(__file__).with_name("_nav2_goal_runner.py")
    started = rt.start_ros2_python_node(
        run_id=run_id,
        source_mode="file",
        script_path=str(script),
        code="",
        arguments=[
            "--action-name", action_name,
            "--frame-id", frame_id,
            "--x", str(x_m),
            "--y", str(y_m),
            "--yaw-deg", str(yaw_deg),
            "--timeout", str(timeout_s),
            "--speed-limit-topic", str(ctx.get("speed_limit_topic") or "/speed_limit").strip(),
            "--max-speed", str(max_speed_mps),
        ],
    )
    if not started.get("ok"):
        _zero_twist(str(ctx.get("cmd_vel_topic") or "/cmd_vel").strip())
        return {
            "accepted": False,
            "running": False,
            "succeeded": False,
            "goal": goal,
            "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": "failed", "error": str(started.get("error") or "goal client failed")},
            "report": f"navigation FAILED: {started.get('error') or 'could not start the managed Nav2 goal client'}",
        }
    return {
        "accepted": True,
        "running": True,
        "succeeded": False,
        "goal": goal,
        "status": {"kind": "blacknode.navigation-status", "schema_version": 1, "state": "starting", "backend": started.get("backend"), "timeout_s": timeout_s, "max_speed_mps": max_speed_mps},
        "report": f"navigation goal started: ({x_m:.2f}, {y_m:.2f}) m at {yaw_deg:.1f}° in {frame_id}; max speed {max_speed_mps:g} m/s; timeout {timeout_s:g}s",
    }


@node(
    name="ExploreEnvironment",
    category=_CATEGORY,
    description=(
        "Explore live SLAM frontiers through Nav2, stop on stale map/LiDAR/localization, save the "
        "completed map, and report when localization is ready for navigation commands."
    ),
    inputs={
        "trigger": AnyPort,
        "action": Enum(["preview", "start", "status", "pause", "resume", "stop"], default="preview"),
        "authorization": Dict,
        "run_id": Text(default="environment"),
        "action_name": Text(default="/navigate_to_pose"),
        "map_topic": Text(default="/map"),
        "scan_topic": Text(default="/scan"),
        "map_frame": Text(default="map"),
        "base_frame": Text(default="base_link"),
        "cmd_vel_topic": Text(default="/cmd_vel"),
        "speed_limit_topic": Text(default="/speed_limit"),
        "min_clearance_m": Float(default=0.45),
        "safety_sector_deg": Float(default=80.0),
        "map_stale_after_s": Float(default=5.0),
        "scan_stale_after_s": Float(default=1.0),
        "min_frontier_cells": Int(default=8),
        "obstacle_clearance_m": Float(default=0.4),
        "min_goal_distance_m": Float(default=0.5),
        "no_frontier_timeout_s": Float(default=12.0),
        "goal_timeout_s": Float(default=120.0),
        "session_timeout_s": Float(default=1800.0),
        "max_failed_goals": Int(default=8),
        "save_directory": Text(default="~/Blacknode/maps"),
        "map_name": Text(default="environment"),
        "save_map_service": Text(default="/slam_toolbox/save_map"),
        "serialize_service": Text(default="/slam_toolbox/serialize_map"),
    },
    outputs={
        "running": Bool,
        "complete": Bool,
        "ready_for_commands": Bool,
        "progress": Dict,
        "map_artifact": Dict,
        "status": Dict,
        "report": Text,
    },
    primary_inputs=["trigger", "action", "authorization", "map_name"],
    primary_outputs=["progress", "status", "report"],
    live=True,
)
def explore_environment(ctx: dict) -> dict:
    action = str(ctx.get("action") or "preview").strip().lower()
    run_id = _safe_id(ctx.get("run_id") or "environment", "blacknode-explore")
    cmd_vel_topic = str(ctx.get("cmd_vel_topic") or "/cmd_vel").strip()

    def current() -> tuple[dict[str, Any], dict[str, Any]]:
        runtime, logs = _goal_runtime(run_id)
        return runtime, _last_exploration_event(logs)

    def result_for(runtime: dict[str, Any], event: dict[str, Any], report: str = "") -> dict[str, Any]:
        state = str(event.get("state") or ("running" if runtime.get("running") else "idle"))
        complete = state == "ready"
        ready = complete and bool(event.get("ready_for_commands"))
        return {
            "running": bool(runtime.get("running")),
            "complete": complete,
            "ready_for_commands": ready,
            "progress": {
                key: event[key]
                for key in (
                    "coverage", "known_cells", "frontier_cells", "reached_goals",
                    "failed_goals", "candidates", "goal", "pose",
                )
                if key in event
            },
            "map_artifact": dict(event.get("map_artifact") or {}),
            "status": {
                "kind": "blacknode.exploration-status",
                "schema_version": 1,
                "state": state,
                "motion_commanded": bool(event.get("motion_commanded", False)),
                "event": event,
                "run_id": run_id,
                "backend": str(runtime.get("backend") or "none"),
            },
            "report": report or str(event.get("report") or f"environment exploration {state}"),
        }

    runtime, event = current()
    if action == "preview":
        preview = {
            "kind": "blacknode.exploration-event",
            "state": "preview",
            "motion_commanded": False,
            "ready_for_commands": False,
        }
        return result_for(
            {"running": False, "backend": runtime.get("backend")},
            preview,
            "exploration preview: SLAM frontiers will be mapped and navigated after explicit arming; no motion commanded",
        )
    if action in {"stop", "pause"}:
        stopped = rt.stop_ros2_python_node(run_id)
        _zero_twist(cmd_vel_topic)
        state = "paused" if action == "pause" else "stopped"
        stopped_event = {
            "kind": "blacknode.exploration-event",
            "state": state,
            "motion_commanded": False,
            "ready_for_commands": False,
        }
        return result_for(
            {"running": False, "backend": stopped.get("backend")},
            stopped_event,
            f"environment exploration {state}; navigation goal cancelled and zero velocity published",
        )
    if action == "status":
        return result_for(runtime, event)
    error = _authorization_error(ctx.get("authorization"))
    if error:
        blocked = {
            "kind": "blacknode.exploration-event",
            "state": "blocked",
            "motion_commanded": False,
            "ready_for_commands": False,
            "error": error,
        }
        return result_for(
            {"running": False, "backend": runtime.get("backend")},
            blocked,
            f"environment exploration BLOCKED: {error}",
        )
    nav = _actions()
    action_name = str(ctx.get("action_name") or "/navigate_to_pose").strip()
    action_type = (nav.get("actions") or {}).get(action_name, "")
    if action_name not in (nav.get("actions") or {}) or (action_type and action_type != _ACTION_TYPE):
        unavailable = {
            "kind": "blacknode.exploration-event",
            "state": "unavailable",
            "motion_commanded": False,
            "ready_for_commands": False,
            "error": str(nav.get("error") or "Nav2 action is unavailable"),
        }
        return result_for(
            {"running": False, "backend": nav.get("backend")},
            unavailable,
            f"environment exploration FAILED: {action_name} does not provide {_ACTION_TYPE}",
        )
    if runtime.get("running"):
        return result_for(runtime, event, "environment exploration is already running")
    authorization = ctx.get("authorization") if isinstance(ctx.get("authorization"), dict) else {}
    max_speed_mps = max(0.01, min(0.5, _float(authorization, "max_speed_mps", 0.15)))
    script = Path(__file__).with_name("_frontier_exploration_runner.py")
    helper = Path(__file__).with_name("_frontier.py")
    try:
        runner_code = script.read_text(encoding="utf-8")
        helper_code = helper.read_text(encoding="utf-8").replace("from __future__ import annotations\n", "")
        runner_code = runner_code.replace("from _frontier import select_frontier_goal\n", helper_code)
    except Exception as exc:
        failed = {
            "kind": "blacknode.exploration-event",
            "state": "failed",
            "motion_commanded": False,
            "ready_for_commands": False,
            "error": f"could not prepare exploration worker: {type(exc).__name__}: {exc}",
        }
        return result_for({"running": False, "backend": runtime.get("backend")}, failed)
    arguments = [
        "--action-name", action_name,
        "--map-topic", str(ctx.get("map_topic") or "/map").strip(),
        "--scan-topic", str(ctx.get("scan_topic") or "/scan").strip(),
        "--map-frame", str(ctx.get("map_frame") or "map").strip(),
        "--base-frame", str(ctx.get("base_frame") or "base_link").strip(),
        "--cmd-vel-topic", cmd_vel_topic,
        "--speed-limit-topic", str(ctx.get("speed_limit_topic") or "/speed_limit").strip(),
        "--max-speed", str(max_speed_mps),
        "--min-clearance", str(max(0.1, _float(ctx, "min_clearance_m", 0.45))),
        "--safety-sector-deg", str(max(10.0, min(180.0, _float(ctx, "safety_sector_deg", 80.0)))),
        "--map-stale-after", str(max(0.5, min(30.0, _float(ctx, "map_stale_after_s", 5.0)))),
        "--scan-stale-after", str(max(0.1, min(5.0, _float(ctx, "scan_stale_after_s", 1.0)))),
        "--min-frontier-cells", str(max(1, int(_float(ctx, "min_frontier_cells", 8)))),
        "--obstacle-clearance", str(max(0.1, _float(ctx, "obstacle_clearance_m", 0.4))),
        "--min-goal-distance", str(max(0.1, _float(ctx, "min_goal_distance_m", 0.5))),
        "--no-frontier-timeout", str(max(3.0, _float(ctx, "no_frontier_timeout_s", 12.0))),
        "--goal-timeout", str(max(10.0, min(300.0, _float(ctx, "goal_timeout_s", 120.0)))),
        "--session-timeout", str(max(60.0, min(3600.0, _float(ctx, "session_timeout_s", 1800.0)))),
        "--max-failed-goals", str(max(1, int(_float(ctx, "max_failed_goals", 8)))),
        "--save-directory", str(ctx.get("save_directory") or "~/Blacknode/maps").strip(),
        "--map-name", str(ctx.get("map_name") or "environment").strip(),
        "--save-map-service", str(ctx.get("save_map_service") or "/slam_toolbox/save_map").strip(),
        "--serialize-service", str(ctx.get("serialize_service") or "/slam_toolbox/serialize_map").strip(),
    ]
    started = rt.start_ros2_python_node(
        run_id=run_id,
        source_mode="inline",
        script_path="",
        code=runner_code,
        arguments=arguments,
    )
    if not started.get("ok"):
        _zero_twist(cmd_vel_topic)
        failed = {
            "kind": "blacknode.exploration-event",
            "state": "failed",
            "motion_commanded": False,
            "ready_for_commands": False,
            "error": str(started.get("error") or "exploration worker failed"),
        }
        return result_for(
            {"running": False, "backend": started.get("backend")},
            failed,
            f"environment exploration FAILED: {failed['error']}",
        )
    starting = {
        "kind": "blacknode.exploration-event",
        "state": "starting",
        "motion_commanded": False,
        "ready_for_commands": False,
    }
    return result_for(
        {"running": True, "backend": started.get("backend")},
        starting,
        f"environment exploration started at up to {max_speed_mps:g} m/s; live map, LiDAR, localization, timeout, and stop safeguards are active",
    )
