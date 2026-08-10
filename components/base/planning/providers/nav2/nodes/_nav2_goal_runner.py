"""Managed NavigateToPose client that cancels its own goal during shutdown."""
from __future__ import annotations

import argparse
import json
import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.node import Node


_cancel_requested = False


def _request_cancel(_signum=None, _frame=None) -> None:
    global _cancel_requested
    _cancel_requested = True


def _emit(state: str, **values) -> None:
    print(json.dumps({"kind": "blacknode.navigation-event", "state": state, **values}), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--action-name", default="/navigate_to_pose")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--speed-limit-topic", default="/speed_limit")
    parser.add_argument("--max-speed", type=float, required=True)
    args = parser.parse_args()

    signal.signal(signal.SIGTERM, _request_cancel)
    signal.signal(signal.SIGINT, _request_cancel)
    rclpy.init()
    node = Node("blacknode_nav2_goal")
    client = ActionClient(node, NavigateToPose, args.action_name)
    speed_limit = node.create_publisher(SpeedLimit, args.speed_limit_topic, 10)
    goal_handle = None
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            _emit("unavailable", error=f"action server {args.action_name} is unavailable")
            return 2

        limited = SpeedLimit()
        limited.header.stamp = node.get_clock().now().to_msg()
        limited.percentage = False
        limited.speed_limit = max(0.01, float(args.max_speed))
        for _ in range(3):
            speed_limit.publish(limited)
            rclpy.spin_once(node, timeout_sec=0.05)
        _emit("speed_limited", max_speed_mps=limited.speed_limit, topic=args.speed_limit_topic)

        pose = PoseStamped()
        pose.header.frame_id = args.frame_id
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(args.x)
        pose.pose.position.y = float(args.y)
        yaw = math.radians(float(args.yaw_deg))
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)
        goal_handle = sent.result() if sent.done() else None
        if goal_handle is None or not goal_handle.accepted:
            _emit("rejected")
            return 3

        _emit("accepted", x=args.x, y=args.y, yaw_deg=args.yaw_deg, frame_id=args.frame_id)
        result = goal_handle.get_result_async()
        deadline = time.monotonic() + max(1.0, float(args.timeout))
        while rclpy.ok() and not result.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if _cancel_requested or time.monotonic() >= deadline:
                cancelled = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancelled, timeout_sec=2.0)
                _emit("cancelled", reason="shutdown" if _cancel_requested else "timeout")
                return 4

        wrapped = result.result() if result.done() else None
        status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        if status == GoalStatus.STATUS_SUCCEEDED:
            _emit("succeeded")
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            _emit("cancelled", reason="server")
            return 4
        _emit("failed", status=status)
        return 5
    finally:
        if _cancel_requested and goal_handle is not None:
            try:
                pending = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, pending, timeout_sec=1.0)
            except Exception:
                pass
        # Restore the provider's configured maximum instead of leaving a
        # Blacknode session limit active after success, cancellation, or error.
        restored = SpeedLimit()
        restored.header.stamp = node.get_clock().now().to_msg()
        restored.percentage = False
        restored.speed_limit = 0.0
        try:
            for _ in range(3):
                speed_limit.publish(restored)
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
