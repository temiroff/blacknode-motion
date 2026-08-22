import json
import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _import_nodes_module, _tag_new_package_nodes


_NODES = Path(__file__).resolve().parents[1] / "components" / "base" / "planning" / "providers" / "nav2" / "nodes"
_before = dict(_NODE_REGISTRY)
_import_nodes_module("blacknode.pkg.blacknode_motion.base.adapters.ros2_nav2", _NODES)
_tag_new_package_nodes(_before, "blacknode-motion", _NODES, "base", "ros2")

from blacknode.pkg.blacknode_motion.base.adapters.ros2_nav2 import nav2

_FRONTIER_SPEC = importlib.util.spec_from_file_location("blacknode_motion_frontier", _NODES / "_frontier.py")
assert _FRONTIER_SPEC and _FRONTIER_SPEC.loader
frontier = importlib.util.module_from_spec(_FRONTIER_SPEC)
_FRONTIER_SPEC.loader.exec_module(frontier)


def _runtime(*, nav_ready=True, owned=False):
    calls = []
    action_output = "/navigate_to_pose [nav2_msgs/action/NavigateToPose]\n" if nav_ready else ""
    return SimpleNamespace(
        calls=calls,
        run_ros2=lambda args, timeout=15.0: calls.append(("run", args, timeout)) or {
            "ok": True, "backend": "native", "stdout": action_output, "stderr": ""
        },
        ros2_managed_status=lambda _run_id: {"ok": True, "running": owned, "backend": "native"},
        run_ros2_managed=lambda run_id, args: calls.append(("start", run_id, args)) or {"ok": True, "backend": "native"},
        stop_ros2_managed=lambda run_id: calls.append(("stop", run_id)) or {"ok": True, "stopped": 1},
        start_ros2_python_node=lambda **kwargs: calls.append(("goal", kwargs)) or {"ok": True, "running": True, "backend": "native"},
        stop_ros2_python_node=lambda run_id: calls.append(("cancel", run_id)) or {"ok": True, "stopped": 1},
        runtime_status=lambda: {"node_outputs": []},
    )


def _authorization():
    return {"authorized": True, "issued_at": time.time(), "max_age_s": 30.0, "reason": "authorized"}


def test_nav2_nodes_are_registered_under_base_ros2_adapter():
    for name in ("NavigationSession", "NavigateTo", "ExploreEnvironment"):
        fn = _NODE_REGISTRY[name]
        assert fn._bn_package == "blacknode-motion"
        assert fn._bn_component == "base"
        assert fn._bn_adapter == "ros2"


def test_navigation_session_attaches_without_launching_vendor_stack(monkeypatch):
    fake = _runtime(nav_ready=True, owned=False)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigationSession"]({"action": "start", "lifecycle": "existing"})

    assert result["ready"] is True
    assert result["owned"] is False
    assert not [call for call in fake.calls if call[0] == "start"]
    assert "left unchanged" in result["report"]


def test_managed_navigation_requires_map_and_params(monkeypatch):
    fake = _runtime(nav_ready=False, owned=False)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigationSession"]({"action": "start", "lifecycle": "managed"})

    assert result["ready"] is False
    assert "requires map_yaml" in result["report"]
    assert not [call for call in fake.calls if call[0] == "start"]


def test_managed_live_slam_navigation_does_not_require_saved_map(monkeypatch, tmp_path):
    fake = _runtime(nav_ready=False, owned=False)
    status_calls = {"count": 0}

    def run_ros2(args, timeout=15.0):
        fake.calls.append(("run", args, timeout))
        if args == ["action", "list", "-t"]:
            status_calls["count"] += 1
            ready = status_calls["count"] > 1
            return {"ok": True, "backend": "native", "stdout": "/navigate_to_pose [nav2_msgs/action/NavigateToPose]\n" if ready else "", "stderr": ""}
        return {"ok": True, "backend": "native", "stdout": "", "stderr": ""}

    fake.run_ros2 = run_ros2
    fake.ros2_managed_status = lambda _run_id: {"ok": True, "running": True, "backend": "native"}
    monkeypatch.setattr(nav2, "rt", fake)
    params = tmp_path / "nav2.yaml"
    params.write_text("navigator:\n  ros__parameters: {}\n", encoding="utf-8")

    result = _NODE_REGISTRY["NavigationSession"]({
        "action": "start",
        "lifecycle": "managed",
        "map_mode": "live_slam",
        "params_file": str(params),
        "wait_seconds": 0,
    })

    launch = [call for call in fake.calls if call[0] == "start"][0][2]
    assert launch[:3] == ["launch", "nav2_bringup", "navigation_launch.py"]
    assert not any(argument.startswith("map:=") for argument in launch)
    assert result["provider"]["map_mode"] == "live_slam"


def test_managed_navigation_builds_owned_rosorin_params_overlay(monkeypatch, tmp_path):
    fake = _runtime(nav_ready=False, owned=False)
    status_calls = {"count": 0}

    def run_ros2(args, timeout=15.0):
        fake.calls.append(("run", args, timeout))
        if args == ["action", "list", "-t"]:
            status_calls["count"] += 1
            stdout = "/navigate_to_pose [nav2_msgs/action/NavigateToPose]\n" if status_calls["count"] > 1 else ""
            return {"ok": True, "backend": "native", "stdout": stdout, "stderr": ""}
        return {"ok": True, "backend": "native", "stdout": "", "stderr": ""}

    fake.run_ros2 = run_ros2
    fake.ros2_managed_status = lambda _run_id: {"ok": True, "running": True, "backend": "native"}
    monkeypatch.setattr(nav2, "rt", fake)
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path / "home"))
    map_yaml = tmp_path / "map.yaml"
    main_params = tmp_path / "nav2.yaml"
    controller_params = tmp_path / "controller.yaml"
    map_yaml.write_text("image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    main_params.write_text("planner_server:\n  ros__parameters:\n    expected_planner_frequency: 20.0\n", encoding="utf-8")
    controller_params.write_text("controller_server:\n  ros__parameters:\n    controller_frequency: 10.0\n", encoding="utf-8")

    result = _NODE_REGISTRY["NavigationSession"]({
        "action": "start",
        "lifecycle": "managed",
        "map_yaml": str(map_yaml),
        "params_file": str(main_params),
        "controller_params_file": str(controller_params),
        "wait_seconds": 0,
    })

    launch = [call for call in fake.calls if call[0] == "start"][0][2]
    effective = Path(next(arg.split(":=", 1)[1] for arg in launch if arg.startswith("params_file:=")))
    assert effective.is_file()
    payload = effective.read_text(encoding="utf-8")
    assert "planner_server:" in payload
    assert "controller_server:" in payload
    assert launch[:3] == ["launch", "nav2_bringup", "bringup_launch.py"]
    assert result["owned"] is True


def test_navigation_stop_only_targets_owned_process(monkeypatch):
    fake = _runtime(nav_ready=True, owned=False)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigationSession"]({"action": "stop", "lifecycle": "existing"})

    assert not [call for call in fake.calls if call[0] == "stop"]
    assert "left unchanged" in result["report"]


def test_navigate_to_preview_never_touches_ros(monkeypatch):
    fake = _runtime(nav_ready=True)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigateTo"]({"action": "preview", "x_m": 1.0, "y_m": 2.0})

    assert result["running"] is False
    assert result["status"]["motion_commanded"] is False
    assert fake.calls == []


def test_navigate_to_send_requires_fresh_authorization(monkeypatch):
    fake = _runtime(nav_ready=True)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigateTo"]({"action": "send", "authorization": {}})

    assert result["accepted"] is False
    assert result["status"]["state"] == "blocked"
    assert fake.calls == []


def test_navigate_to_starts_owned_cancellable_goal(monkeypatch):
    fake = _runtime(nav_ready=True)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigateTo"]({
        "action": "send",
        "authorization": _authorization(),
        "x_m": 1.25,
        "y_m": -0.5,
        "yaw_deg": 90.0,
    })

    assert result["accepted"] is True
    goal_call = [call for call in fake.calls if call[0] == "goal"][0][1]
    assert goal_call["source_mode"] == "file"
    assert goal_call["script_path"].endswith("_nav2_goal_runner.py")
    assert "--x" in goal_call["arguments"]
    assert "--max-speed" in goal_call["arguments"]


def test_cancel_stops_only_owned_goal_and_publishes_zero(monkeypatch):
    fake = _runtime(nav_ready=True, owned=True)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["NavigateTo"]({"action": "cancel", "goal_id": "cube-search"})

    assert result["status"]["state"] == "cancelled"
    assert any(call[0] == "cancel" for call in fake.calls)
    publishes = [call for call in fake.calls if call[0] == "run" and call[1][:3] == ["topic", "pub", "--once"]]
    assert len(publishes) == 1
    payload = json.loads(publishes[0][1][-1])
    assert payload["linear"]["x"] == 0.0


def test_goal_runner_cancels_on_sigterm_and_timeout():
    source = (_NODES / "_nav2_goal_runner.py").read_text(encoding="utf-8")
    compile(source, str(_NODES / "_nav2_goal_runner.py"), "exec")
    assert "signal.SIGTERM" in source
    assert "cancel_goal_async" in source
    assert "SpeedLimit" in source
    assert "restored.speed_limit = 0.0" in source


def test_exploration_runner_has_freshness_clearance_and_shutdown_safeguards():
    source = (_NODES / "_frontier_exploration_runner.py").read_text(encoding="utf-8")
    compile(source, str(_NODES / "_frontier_exploration_runner.py"), "exec")
    assert "qos_profile_sensor_data" in source
    assert "map_stale_after" in source
    assert "scan_stale_after" in source
    assert "forward_clearance < args.min_clearance" in source
    assert "cancel_goal_async" in source
    assert "self.cmd_vel.publish(Twist())" in source
    assert "Environment mapped. Localization ready. Waiting for command." in source


def test_frontier_selector_finds_unknown_boundary_and_respects_exclusions():
    width = height = 12
    data = [-1] * (width * height)
    for row in range(3, 9):
        for column in range(3, 9):
            data[row * width + column] = 0
    selected = frontier.select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=0.25,
        origin_x=-1.5,
        origin_y=-1.5,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        min_frontier_cells=2,
        obstacle_clearance_m=0.1,
        min_goal_distance_m=0.1,
    )

    assert selected["goal"] is not None
    assert selected["frontier_cells"] > 0
    goal = selected["goal"]
    excluded = frontier.select_frontier_goal(
        data,
        width=width,
        height=height,
        resolution=0.25,
        origin_x=-1.5,
        origin_y=-1.5,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        min_frontier_cells=2,
        obstacle_clearance_m=0.1,
        min_goal_distance_m=0.1,
        excluded_goals=[(goal["x_m"], goal["y_m"])],
        excluded_radius_m=10.0,
    )
    assert excluded["goal"] is None


def test_exploration_preview_and_start_are_safety_gated(monkeypatch):
    fake = _runtime(nav_ready=True)
    monkeypatch.setattr(nav2, "rt", fake)

    preview = _NODE_REGISTRY["ExploreEnvironment"]({"action": "preview"})
    blocked = _NODE_REGISTRY["ExploreEnvironment"]({"action": "start", "authorization": {}})

    assert preview["status"]["state"] == "preview"
    assert preview["status"]["motion_commanded"] is False
    assert blocked["status"]["state"] == "blocked"
    assert not [call for call in fake.calls if call[0] == "goal"]


def test_exploration_starts_one_managed_inline_worker(monkeypatch):
    fake = _runtime(nav_ready=True)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["ExploreEnvironment"]({
        "action": "start",
        "authorization": _authorization(),
        "map_name": "office",
    })

    assert result["running"] is True
    call = [item for item in fake.calls if item[0] == "goal"][0][1]
    assert call["source_mode"] == "inline"
    assert "select_frontier_goal" in call["code"]
    compile(call["code"], "frontier_exploration_inline.py", "exec")
    assert "--map-name" in call["arguments"]
    assert "office" in call["arguments"]


def test_exploration_stop_cancels_owned_worker_and_publishes_zero(monkeypatch):
    fake = _runtime(nav_ready=True, owned=True)
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["ExploreEnvironment"]({"action": "stop", "run_id": "office"})

    assert result["status"]["state"] == "stopped"
    assert any(call[0] == "cancel" for call in fake.calls)
    assert any(call[0] == "run" and call[1][:3] == ["topic", "pub", "--once"] for call in fake.calls)


def test_exploration_status_reports_saved_ready_state(monkeypatch):
    fake = _runtime(nav_ready=True)
    ready_event = json.dumps({
        "kind": "blacknode.exploration-event",
        "state": "ready",
        "ready_for_commands": True,
        "motion_commanded": False,
        "report": "Environment mapped. Localization ready. Waiting for command.",
        "coverage": 0.72,
        "map_artifact": {"kind": "blacknode.map-artifact", "map_yaml": "/maps/office.yaml"},
    })
    fake.runtime_status = lambda: {"node_outputs": [{
        "run_id": "blacknode-explore-environment",
        "outputs": {"logs": [ready_event]},
    }]}
    monkeypatch.setattr(nav2, "rt", fake)

    result = _NODE_REGISTRY["ExploreEnvironment"]({"action": "status"})

    assert result["complete"] is True
    assert result["ready_for_commands"] is True
    assert result["map_artifact"]["map_yaml"] == "/maps/office.yaml"


def test_rosorin_navigation_template_validates_and_starts_disarmed():
    from blacknode.workflow import validate_workflow

    path = _NODES.parent / "templates" / "rosorin-navigate-saved-map.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    report = validate_workflow(workflow)
    assert report.ok, report.to_dict()
    assert workflow["node_meta"]["navigate"]["params"]["action"] == "send"
    assert workflow["node_meta"]["gate"]["params"]["armed"] is False
    assert workflow["node_meta"]["navigation"]["params"]["lifecycle"] == "managed"


def test_rosorin_familiarization_template_validates_and_starts_disarmed():
    from blacknode.workflow import validate_workflow

    path = _NODES.parent / "templates" / "rosorin-familiarize-environment.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    report = validate_workflow(workflow)
    assert report.ok, report.to_dict()
    assert workflow["node_meta"]["mapping"]["params"]["action"] == "start"
    assert workflow["node_meta"]["navigation"]["params"]["map_mode"] == "live_slam"
    assert workflow["node_meta"]["gate"]["params"]["armed"] is False
    assert workflow["node_meta"]["explore"]["params"]["action"] == "start"
