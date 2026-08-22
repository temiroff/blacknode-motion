"""Managed ROS 2 frontier exploration worker with explicit stop behavior."""
from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from _frontier import select_frontier_goal


_shutdown_requested = False


def _request_shutdown(_signum, _frame) -> None:
    global _shutdown_requested
    _shutdown_requested = True


def _emit(state: str, **values: Any) -> None:
    print(json.dumps({"kind": "blacknode.exploration-event", "state": state, **values}), flush=True)


def _yaw_from_quaternion(value: Any) -> float:
    return math.atan2(
        2.0 * (float(value.w) * float(value.z) + float(value.x) * float(value.y)),
        1.0 - 2.0 * (float(value.y) ** 2 + float(value.z) ** 2),
    )


class FrontierExplorer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("blacknode_frontier_explorer")
        self.args = args
        self.map_message: OccupancyGrid | None = None
        self.map_received_at = 0.0
        self.scan_received_at = 0.0
        self.forward_clearance = math.inf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, map_qos)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.action_client = ActionClient(self, NavigateToPose, args.action_name)
        self.cmd_vel = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.speed_limit = self.create_publisher(SpeedLimit, args.speed_limit_topic, 10)
        self.goal_handle = None
        self.goal_result = None

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message
        self.map_received_at = time.monotonic()

    def _on_scan(self, message: LaserScan) -> None:
        half_sector = math.radians(self.args.safety_sector_deg) / 2.0
        best = math.inf
        for index, value in enumerate(message.ranges):
            angle = float(message.angle_min) + index * float(message.angle_increment)
            if abs(angle) > half_sector:
                continue
            distance = float(value)
            if math.isfinite(distance) and float(message.range_min) <= distance <= float(message.range_max):
                best = min(best, distance)
        self.forward_clearance = best
        self.scan_received_at = time.monotonic()

    def pose(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException:
            return None
        return float(transform.transform.translation.x), float(transform.transform.translation.y)

    def publish_speed_limit(self, speed_mps: float) -> None:
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = False
        message.speed_limit = float(speed_mps)
        for _ in range(3):
            self.speed_limit.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop_motion(self) -> None:
        if self.goal_handle is not None:
            try:
                pending = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, pending, timeout_sec=2.0)
            except Exception:
                pass
        self.goal_handle = None
        self.goal_result = None
        for _ in range(3):
            self.cmd_vel.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_speed_limit(0.0)

    def restore_speed_limit(self) -> None:
        restored = SpeedLimit()
        restored.header.stamp = self.get_clock().now().to_msg()
        restored.percentage = False
        restored.speed_limit = 0.0
        self.speed_limit.publish(restored)

    def send_goal(self, goal: dict[str, Any]) -> bool:
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = self.args.map_frame
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = float(goal["x_m"])
        request.pose.pose.position.y = float(goal["y_m"])
        half_yaw = float(goal["yaw_rad"]) / 2.0
        request.pose.pose.orientation.z = math.sin(half_yaw)
        request.pose.pose.orientation.w = math.cos(half_yaw)
        sent = self.action_client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        self.goal_handle = sent.result() if sent.done() else None
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.goal_handle = None
            return False
        self.goal_result = self.goal_handle.get_result_async()
        return True


def _save_map(args: argparse.Namespace) -> tuple[bool, dict[str, Any], str]:
    directory = Path(args.save_directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=True)
    safe_name = "".join(character if character.isalnum() or character in "_.-" else "-" for character in args.map_name).strip(".-") or "environment"
    stem = directory / safe_name
    save_request = json.dumps({"name": {"data": str(stem)}}, separators=(",", ":"))
    try:
        saved = subprocess.run(
            ["ros2", "service", "call", args.save_map_service, "slam_toolbox/srv/SaveMap", save_request],
            capture_output=True,
            text=True,
            timeout=args.service_timeout,
            check=False,
        )
    except Exception as exc:
        return False, {}, f"map save failed: {type(exc).__name__}: {exc}"
    if saved.returncode != 0:
        return False, {}, f"map save failed: {(saved.stderr or saved.stdout).strip()}"
    serialize_error = ""
    try:
        serialized = subprocess.run(
            [
                "ros2", "service", "call", args.serialize_service,
                "slam_toolbox/srv/SerializePoseGraph",
                json.dumps({"filename": str(stem)}, separators=(",", ":")),
            ],
            capture_output=True,
            text=True,
            timeout=args.service_timeout,
            check=False,
        )
        serialized_ok = serialized.returncode == 0
        if not serialized_ok:
            serialize_error = (serialized.stderr or serialized.stdout).strip()
    except Exception as exc:
        serialized_ok = False
        serialize_error = f"{type(exc).__name__}: {exc}"
    artifact = {
        "kind": "blacknode.map-artifact",
        "schema_version": 1,
        "provider": "slam_toolbox",
        "map_name": safe_name,
        "directory": str(directory),
        "map_yaml": str(stem.with_suffix(".yaml")),
        "map_image": str(stem.with_suffix(".pgm")),
        "pose_graph": str(stem),
        "pose_graph_serialized": serialized_ok,
        "frame_id": args.map_frame,
        "map_topic": args.map_topic,
        "created_at": time.time(),
    }
    warning = "" if serialized_ok else f"pose graph serialization failed: {serialize_error}"
    return True, artifact, warning


def run(args: argparse.Namespace) -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    signal.signal(signal.SIGINT, _request_shutdown)
    rclpy.init()
    node = FrontierExplorer(args)
    started_at = time.monotonic()
    no_frontier_since: float | None = None
    current_goal: dict[str, Any] | None = None
    current_goal_started = 0.0
    excluded: list[tuple[float, float]] = []
    failed_goals = 0
    reached_goals = 0
    last_progress_emit = 0.0
    inputs_ever_ready = False
    try:
        if not node.action_client.wait_for_server(timeout_sec=15.0):
            _emit("failed", error=f"{args.action_name} is unavailable")
            return 2
        node.publish_speed_limit(args.max_speed)
        _emit("starting", motion_commanded=False, map_topic=args.map_topic, scan_topic=args.scan_topic)
        while rclpy.ok() and not _shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - started_at > args.session_timeout:
                _emit("failed", error="exploration session timed out", reached_goals=reached_goals)
                return 3
            map_fresh = node.map_message is not None and now - node.map_received_at <= args.map_stale_after
            scan_fresh = node.scan_received_at > 0.0 and now - node.scan_received_at <= args.scan_stale_after
            pose = node.pose() if map_fresh else None
            if not map_fresh or not scan_fresh or pose is None:
                if not inputs_ever_ready and now - started_at > args.startup_timeout:
                    _emit(
                        "failed",
                        error="live map, LiDAR, and map-to-base localization were not all available before startup timeout",
                    )
                    return 7
                if current_goal is not None:
                    node.stop_motion()
                    current_goal = None
                    _emit("blocked", reason="map, LiDAR, or localization became stale; motion stopped")
                continue
            inputs_ever_ready = True
            if node.forward_clearance < args.min_clearance:
                if current_goal is not None:
                    node.stop_motion()
                    excluded.append((float(current_goal["x_m"]), float(current_goal["y_m"])))
                    current_goal = None
                _emit("blocked", reason=f"obstacle at {node.forward_clearance:.2f} m; waiting for clearance", motion_commanded=False)
                continue
            if current_goal is not None:
                if node.goal_result is not None and node.goal_result.done():
                    wrapped = node.goal_result.result()
                    status = int(getattr(wrapped, "status", 0)) if wrapped is not None else 0
                    excluded.append((float(current_goal["x_m"]), float(current_goal["y_m"])))
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        reached_goals += 1
                        _emit("goal_reached", goal=current_goal, reached_goals=reached_goals)
                    else:
                        failed_goals += 1
                        _emit("goal_failed", goal=current_goal, failed_goals=failed_goals, action_status=status)
                    current_goal = None
                    node.goal_handle = None
                    node.goal_result = None
                    if failed_goals >= args.max_failed_goals:
                        _emit("failed", error="too many navigation goals failed", failed_goals=failed_goals)
                        return 4
                    continue
                if now - current_goal_started > args.goal_timeout:
                    node.stop_motion()
                    excluded.append((float(current_goal["x_m"]), float(current_goal["y_m"])))
                    failed_goals += 1
                    _emit("goal_failed", goal=current_goal, failed_goals=failed_goals, reason="goal timeout")
                    current_goal = None
                continue
            message = node.map_message
            assert message is not None
            orientation = message.info.origin.orientation
            selection = select_frontier_goal(
                message.data,
                width=int(message.info.width),
                height=int(message.info.height),
                resolution=float(message.info.resolution),
                origin_x=float(message.info.origin.position.x),
                origin_y=float(message.info.origin.position.y),
                origin_yaw=_yaw_from_quaternion(orientation),
                robot_x=pose[0],
                robot_y=pose[1],
                min_frontier_cells=args.min_frontier_cells,
                obstacle_clearance_m=args.obstacle_clearance,
                min_goal_distance_m=args.min_goal_distance,
                excluded_goals=excluded,
                excluded_radius_m=args.excluded_radius,
            )
            if now - last_progress_emit >= 2.0:
                _emit(
                    "mapping",
                    motion_commanded=False,
                    reached_goals=reached_goals,
                    failed_goals=failed_goals,
                    known_cells=selection["known_cells"],
                    frontier_cells=selection["frontier_cells"],
                    coverage=selection["coverage"],
                    candidates=selection["candidates"],
                    pose={"x_m": pose[0], "y_m": pose[1]},
                )
                last_progress_emit = now
            goal = selection["goal"]
            if goal is None:
                if selection["known_cells"] < args.min_known_cells:
                    no_frontier_since = None
                    continue
                no_frontier_since = no_frontier_since or now
                if now - no_frontier_since < args.no_frontier_timeout:
                    continue
                node.stop_motion()
                saved, artifact, warning = _save_map(args)
                if not saved:
                    _emit("failed", error=warning, reached_goals=reached_goals)
                    return 5
                refresh_deadline = time.monotonic() + max(2.0, args.scan_stale_after * 2.0)
                while rclpy.ok() and time.monotonic() < refresh_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                refreshed_at = time.monotonic()
                final_pose = node.pose()
                final_map_fresh = refreshed_at - node.map_received_at <= args.map_stale_after
                final_scan_fresh = refreshed_at - node.scan_received_at <= args.scan_stale_after
                if final_pose is None or not final_map_fresh or not final_scan_fresh:
                    _emit("failed", error="map saved but live map, LiDAR, or localization readiness could not be verified")
                    return 6
                _emit(
                    "ready",
                    ready_for_commands=True,
                    motion_commanded=False,
                    report="Environment mapped. Localization ready. Waiting for command.",
                    map_artifact=artifact,
                    warning=warning,
                    pose={"x_m": final_pose[0], "y_m": final_pose[1]},
                    reached_goals=reached_goals,
                    known_cells=selection["known_cells"],
                    coverage=selection["coverage"],
                )
                return 0
            no_frontier_since = None
            if not node.send_goal(goal):
                excluded.append((float(goal["x_m"]), float(goal["y_m"])))
                failed_goals += 1
                _emit("goal_failed", goal=goal, failed_goals=failed_goals, reason="goal rejected")
                continue
            current_goal = goal
            current_goal_started = now
            _emit("navigating", motion_commanded=True, goal=goal, reached_goals=reached_goals)
        _emit("stopped", ready_for_commands=False, motion_commanded=False)
        return 0
    finally:
        node.stop_motion()
        node.restore_speed_limit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--speed-limit-topic", default="/speed_limit")
    parser.add_argument("--max-speed", type=float, default=0.15)
    parser.add_argument("--min-clearance", type=float, default=0.45)
    parser.add_argument("--safety-sector-deg", type=float, default=80.0)
    parser.add_argument("--map-stale-after", type=float, default=5.0)
    parser.add_argument("--scan-stale-after", type=float, default=1.0)
    parser.add_argument("--min-frontier-cells", type=int, default=8)
    parser.add_argument("--obstacle-clearance", type=float, default=0.4)
    parser.add_argument("--min-goal-distance", type=float, default=0.5)
    parser.add_argument("--excluded-radius", type=float, default=0.75)
    parser.add_argument("--no-frontier-timeout", type=float, default=12.0)
    parser.add_argument("--goal-timeout", type=float, default=120.0)
    parser.add_argument("--session-timeout", type=float, default=1800.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--max-failed-goals", type=int, default=8)
    parser.add_argument("--min-known-cells", type=int, default=400)
    parser.add_argument("--save-directory", default="~/Blacknode/maps")
    parser.add_argument("--map-name", default="environment")
    parser.add_argument("--save-map-service", default="/slam_toolbox/save_map")
    parser.add_argument("--serialize-service", default="/slam_toolbox/serialize_map")
    parser.add_argument("--service-timeout", type=float, default=30.0)
    return parser


if __name__ == "__main__":
    raise SystemExit(run(_parser().parse_args()))
