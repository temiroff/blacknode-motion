"""Managed Blacknode policy inference and safety-gated rosbridge execution."""
from __future__ import annotations

import atexit
import concurrent.futures
import io
import json
import math
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any, Callable



def _rosbridge_runtime():
    """Load the optional ROS 2 execution transport only when it is used."""
    from blacknode.pkg.blacknode_ros2 import rosbridge_runtime

    return rosbridge_runtime

try:
    import numpy as np
except Exception:  # pragma: no cover - package health reports this
    np = None

try:
    from PIL import Image as PILImage
except Exception:  # pragma: no cover - package health reports this
    PILImage = None

try:
    import msgpack
    import websockets.sync.client as websocket_client
except Exception:  # pragma: no cover - package health reports this
    msgpack = None
    websocket_client = None


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _camera_handles(values: list[Any], expected: list[str]) -> dict[str, dict[str, Any]]:
    handles = {
        str(item.get("stream_id") or "").strip(): dict(item)
        for item in values
        if isinstance(item, dict) and item.get("kind") == "blacknode.frame-stream"
    }
    missing = [name for name in expected if name not in handles]
    if missing:
        raise ValueError("camera stream names do not match the policy artifact; missing: " + ", ".join(missing))
    return {name: handles[name] for name in expected}


def _robot_joint_specs(robot: dict[str, Any]) -> dict[str, dict[str, Any]]:
    driver = robot.get("driver") if isinstance(robot.get("driver"), dict) else {}
    joints = driver.get("joints") if isinstance(driver.get("joints"), list) else []
    return {
        str(item.get("id")): dict(item)
        for item in joints
        if isinstance(item, dict) and str(item.get("id") or "").strip()
    }


def validate_deployment_contract(
    artifact: dict[str, Any], robot: dict[str, Any], camera_streams: list[Any], safety: dict[str, Any]
) -> dict[str, Any]:
    if artifact.get("kind") != "blacknode.policy-artifact":
        raise ValueError("connect a blacknode.policy-artifact")
    action_mode = str(artifact.get("action_mode") or "")
    units = str(artifact.get("units") or "")
    simulator_delta_policy = (
        action_mode == "bounded_joint_position_delta" and units == "normalized"
    )
    if not simulator_delta_policy and (
        action_mode != "absolute_joint_position" or units != "radians"
    ):
        raise ValueError(
            "policy must emit absolute joint positions in radians or a supported "
            "simulation-only normalized joint delta"
        )
    joint_names = [str(name) for name in artifact.get("joint_names") or []]
    camera_names = [str(name) for name in artifact.get("camera_names") or []]
    if not joint_names:
        raise ValueError("policy artifact must declare ordered joints")
    driver = robot.get("driver") if isinstance(robot.get("driver"), dict) else {}
    if simulator_delta_policy:
        artifact_safety = artifact.get("safety") if isinstance(artifact.get("safety"), dict) else {}
        if (
            artifact_safety.get("simulation_only") is not True
            or artifact_safety.get("physical_motion_authorized") is not False
            or driver.get("simulation_only") is not True
        ):
            raise ValueError(
                "normalized joint-delta policies require a simulation-only artifact and provider"
            )
    elif not camera_names:
        raise ValueError("absolute joint-position policy artifact must declare ordered cameras")
    specs = _robot_joint_specs(robot)
    if list(specs) != joint_names:
        raise ValueError(
            "robot joint order does not match policy artifact: "
            f"robot={list(specs)}, policy={joint_names}"
        )
    _camera_handles(camera_streams, camera_names)
    if not bool(driver.get("running")):
        raise ValueError("robot driver is not running")
    if bool(safety.get("require_calibration", True)) and not str(driver.get("calibration_path") or "").strip():
        raise ValueError("a hardware-bound robot calibration is required before policy deployment")
    for name, spec in specs.items():
        lower = _finite(spec.get("safe_min_deg", spec.get("min_deg")))
        upper = _finite(spec.get("safe_max_deg", spec.get("max_deg")))
        if lower is None or upper is None or lower >= upper:
            raise ValueError(f"robot joint {name} has no valid calibrated limit")
    workspace = safety.get("workspace_limits") if isinstance(safety.get("workspace_limits"), dict) else {}
    if workspace and not str(safety.get("workspace_topic") or "").strip():
        raise ValueError("workspace_limits require a workspace_topic with live x/y/z telemetry")
    return {
        "joint_names": joint_names,
        "camera_names": camera_names,
        "policy_type": str(artifact.get("policy_type") or ""),
        "physical_motion_authorized": artifact.get("physical_motion_authorized") is not False,
        "action_mode": action_mode,
        "simulation_only": simulator_delta_policy,
        "joint_specs": specs,
        "host": str(robot.get("host") or driver.get("host") or "127.0.0.1"),
        "port": int(robot.get("port") or driver.get("port") or 9090),
        "state_topic": str(robot.get("state_topic") or driver.get("state_topic") or "/joint_states"),
        "command_topic": str(robot.get("command_topic") or driver.get("command_topic") or "/joint_commands"),
        "control_topic": str(robot.get("control_topic") or driver.get("control_topic") or "/robot_control"),
    }


class SafetyGate:
    def __init__(self, joint_names: list[str], joint_specs: dict[str, dict[str, Any]], config: dict[str, Any]) -> None:
        self.joint_names = list(joint_names)
        self.config = dict(config)
        self.limits = {
            name: (
                math.radians(float(joint_specs[name].get("safe_min_deg", joint_specs[name].get("min_deg")))),
                math.radians(float(joint_specs[name].get("safe_max_deg", joint_specs[name].get("max_deg")))),
            )
            for name in self.joint_names
        }

    def apply(
        self,
        predicted: list[float],
        current: dict[str, float],
        *,
        dt: float,
        workspace: dict[str, float] | None = None,
    ) -> dict[str, Any]:
        if len(predicted) != len(self.joint_names):
            return {"ok": False, "action": {}, "clamped": [], "reason": "prediction dimension mismatch"}
        workspace_limits = self.config.get("workspace_limits") if isinstance(self.config.get("workspace_limits"), dict) else {}
        if workspace_limits:
            if not workspace or any(axis not in workspace for axis in ("x", "y", "z")):
                return {"ok": False, "action": {}, "clamped": [], "reason": "workspace telemetry is missing"}
            for axis in ("x", "y", "z"):
                bounds = workspace_limits.get(axis)
                if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                    return {"ok": False, "action": {}, "clamped": [], "reason": f"workspace limit {axis} must be [min,max]"}
                value = _finite(workspace.get(axis))
                if value is None or value < float(bounds[0]) or value > float(bounds[1]):
                    return {"ok": False, "action": {}, "clamped": [], "reason": f"workspace {axis} is outside the safe range"}
        velocity = max(0.0, math.radians(float(self.config.get("max_velocity_deg_s") or 0.0)))
        max_step = max(0.0, math.radians(float(self.config.get("max_step_deg") or 0.0)))
        allowed_delta = min(
            value for value in (velocity * max(dt, 1e-3) if velocity else float("inf"), max_step or float("inf"))
        )
        action: dict[str, float] = {}
        clamped: list[str] = []
        for index, name in enumerate(self.joint_names):
            value = _finite(predicted[index])
            current_value = _finite(current.get(name))
            if value is None or current_value is None:
                return {"ok": False, "action": {}, "clamped": clamped, "reason": f"non-finite or missing joint value: {name}"}
            lower, upper = self.limits[name]
            bounded = min(upper, max(lower, value))
            if bounded != value:
                clamped.append(f"{name}:joint_limit")
            delta = bounded - current_value
            if math.isfinite(allowed_delta) and abs(delta) > allowed_delta:
                bounded = current_value + math.copysign(allowed_delta, delta)
                clamped.append(f"{name}:velocity")
            action[name] = bounded
        if clamped and bool(self.config.get("reject_on_clamp", False)):
            return {
                "ok": False, "action": {}, "clamped": clamped,
                "reason": "raw policy action requires safety clamping",
            }
        return {"ok": True, "action": action, "clamped": clamped, "reason": ""}


class RosbridgePolicyIO:
    def __init__(self, contract: dict[str, Any], cameras: dict[str, dict[str, Any]], safety: dict[str, Any]) -> None:
        if np is None or PILImage is None:
            raise RuntimeError("numpy and Pillow are required for policy camera inference")
        rb = _rosbridge_runtime()
        if rb.roslibpy is None:
            raise RuntimeError("roslibpy is required for policy deployment")
        self.contract = contract
        self.rb = rb
        self.cameras = cameras
        self.safety = safety
        self.lock = threading.RLock()
        self.pose: dict[str, float] = {}
        self.pose_at = 0.0
        self.workspace: dict[str, float] = {}
        self.workspace_at = 0.0
        self.ros = None
        self.state_subscriber = None
        self.workspace_subscriber = None
        self.command_publisher = None

    def start(self) -> None:
        self.ros = self.rb.get_connection(self.contract["host"], self.contract["port"], 10.0)
        self.state_subscriber = self.rb.roslibpy.Topic(
            self.ros, self.contract["state_topic"], self.rb.JOINT_STATE_TYPE,
        )

        def on_state(message: dict[str, Any]) -> None:
            pose = {
                str(name): float(value)
                for name, value in zip(message.get("name") or [], message.get("position") or [])
                if _finite(value) is not None
            }
            with self.lock:
                self.pose = pose
                self.pose_at = time.monotonic()

        self.state_subscriber.subscribe(on_state)
        workspace_topic = str(self.safety.get("workspace_topic") or "").strip()
        if workspace_topic:
            self.workspace_subscriber = self.rb.roslibpy.Topic(
                self.ros, workspace_topic, "geometry_msgs/PoseStamped",
            )

            def on_workspace(message: dict[str, Any]) -> None:
                position = ((message.get("pose") or {}).get("position") or {})
                values = {axis: _finite(position.get(axis)) for axis in ("x", "y", "z")}
                if all(value is not None for value in values.values()):
                    with self.lock:
                        self.workspace = {axis: float(value) for axis, value in values.items()}
                        self.workspace_at = time.monotonic()

            self.workspace_subscriber.subscribe(on_workspace)
        self.command_publisher = self.rb.roslibpy.Topic(
            self.ros, self.contract["command_topic"], self.rb.JOINT_STATE_TYPE,
        )
        self.command_publisher.advertise()

    def _image(self, handle: dict[str, Any], timeout: float) -> tuple[Any, float]:
        url = str(handle.get("snapshot_url") or "").strip()
        if not url:
            raise ValueError("camera stream is missing snapshot_url")
        with urllib.request.urlopen(url, timeout=timeout) as response:  # noqa: S310 - graph-supplied local stream
            content = response.read()
            captured_ns = int(response.headers.get("X-Blacknode-Captured-At-Ns") or 0)
        age = max(0.0, (time.time_ns() - captured_ns) / 1e9) if captured_ns else float("inf")
        image = np.asarray(PILImage.open(io.BytesIO(content)).convert("RGB"), dtype=np.uint8)
        return image, age

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            pose = dict(self.pose)
            pose_age = time.monotonic() - self.pose_at if self.pose_at else float("inf")
            workspace = dict(self.workspace)
            workspace_age = time.monotonic() - self.workspace_at if self.workspace_at else float("inf")
        timeout = max(0.1, float(self.safety.get("request_timeout") or 1.0))
        images: dict[str, Any] = {}
        camera_ages: dict[str, float] = {}
        for name, handle in self.cameras.items():
            images[name], camera_ages[name] = self._image(handle, timeout)
        return {
            "pose": pose, "pose_age": pose_age, "images": images,
            "camera_ages": camera_ages, "workspace": workspace, "workspace_age": workspace_age,
        }

    def publish(self, action: dict[str, float]) -> None:
        if self.command_publisher is None or self.ros is None or not self.ros.is_connected:
            raise RuntimeError("rosbridge command publisher is disconnected")
        self.command_publisher.publish(self.rb._joint_command_message(self.contract["joint_names"], action))

    def control(self, action: str) -> dict[str, Any]:
        return self.rb.publish_string(
            self.contract["host"], self.contract["port"], self.contract["control_topic"],
            json.dumps({"action": action}), timeout=2.0,
        )

    def close(self) -> None:
        for topic, method in (
            (self.state_subscriber, "unsubscribe"),
            (self.workspace_subscriber, "unsubscribe"),
            (self.command_publisher, "unadvertise"),
        ):
            try:
                if topic is not None:
                    getattr(topic, method)()
            except Exception:
                pass


def _pack_openpi_array(value: Any) -> Any:
    if np is not None and isinstance(value, np.ndarray):
        if value.dtype.kind in ("V", "O", "c"):
            raise ValueError(f"unsupported OpenPI array dtype: {value.dtype}")
        return {
            b"__ndarray__": True, b"data": value.tobytes(),
            b"dtype": value.dtype.str, b"shape": value.shape,
        }
    if np is not None and isinstance(value, np.generic):
        return {
            b"__npgeneric__": True, b"data": value.item(),
            b"dtype": value.dtype.str,
        }
    return value


def _unpack_openpi_array(value: dict) -> Any:
    if np is not None and b"__ndarray__" in value:
        return np.ndarray(
            buffer=value[b"data"], dtype=np.dtype(value[b"dtype"]),
            shape=value[b"shape"],
        )
    if np is not None and b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


class _OpenPIWireClient:
    def __init__(self, host: str, port: int, timeout: float) -> None:
        if msgpack is None or websocket_client is None:
            raise RuntimeError("msgpack and websockets are required for OpenPI remote inference")
        self.connection = websocket_client.connect(
            f"ws://{host}:{port}", compression=None, max_size=None,
            open_timeout=timeout, close_timeout=timeout,
        )
        self.packer = msgpack.Packer(default=_pack_openpi_array)
        self.metadata = msgpack.unpackb(
            self.connection.recv(), object_hook=_unpack_openpi_array
        )

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        self.connection.send(self.packer.pack(observation))
        response = self.connection.recv()
        if isinstance(response, str):
            raise RuntimeError(f"OpenPI inference server error: {response}")
        return msgpack.unpackb(response, object_hook=_unpack_openpi_array)

    def close(self) -> None:
        self.connection.close()


class OpenPIRemotePolicy:
    """Asynchronous action-chunk adapter for an official OpenPI server."""

    def __init__(
        self,
        artifact: dict[str, Any],
        _device: str = "auto",
        *,
        client_factory: Callable[[str, int, float], Any] = _OpenPIWireClient,
    ) -> None:
        del _device
        if np is None or PILImage is None:
            raise RuntimeError("numpy and Pillow are required for OpenPI inference")
        self.info = dict(artifact)
        server = artifact.get("server") if isinstance(artifact.get("server"), dict) else {}
        self.prompt = str(server.get("prompt") or artifact.get("task") or "").strip()
        if not self.prompt:
            raise ValueError("OpenPI artifact must declare server.prompt")
        self.horizon = int(server.get("action_horizon") or 0)
        self.execute_count = int(server.get("execute_actions_per_replan") or 0)
        self.async_start = int(server.get("async_replan_after_actions") or 0)
        if not 1 <= self.async_start <= self.execute_count <= self.horizon:
            raise ValueError("invalid OpenPI action-chunk schedule")
        self.image_size = int(server.get("image_size") or 224)
        cameras = list(artifact.get("camera_names") or [])
        self.image_camera = str(server.get("image_camera") or cameras[0])
        self.wrist_camera = str(server.get("wrist_camera") or "").strip()
        if self.image_camera not in cameras:
            raise ValueError("OpenPI image_camera is not declared in camera_names")
        if self.wrist_camera and self.wrist_camera not in cameras:
            raise ValueError("OpenPI wrist_camera is not declared in camera_names")
        calibration = artifact.get("joint_calibration")
        if not isinstance(calibration, dict) or calibration.get("kind") != "affine_robot_to_policy":
            raise ValueError("OpenPI artifact requires affine_robot_to_policy joint_calibration")
        joint_names = list(artifact.get("joint_names") or [])
        if list(calibration.get("joint_names") or []) != joint_names:
            raise ValueError("OpenPI joint calibration order does not match artifact joint_names")
        self.robot_to_policy_scale = np.asarray(
            calibration.get("robot_to_policy_scale"), dtype=np.float32
        )
        self.robot_to_policy_offset = np.asarray(
            calibration.get("robot_to_policy_offset"), dtype=np.float32
        )
        expected_calibration_shape = (len(joint_names),)
        if (
            self.robot_to_policy_scale.shape != expected_calibration_shape
            or self.robot_to_policy_offset.shape != expected_calibration_shape
            or not np.isfinite(self.robot_to_policy_scale).all()
            or not np.isfinite(self.robot_to_policy_offset).all()
            or np.any(np.abs(self.robot_to_policy_scale) < 1e-4)
        ):
            raise ValueError("OpenPI joint calibration is invalid or non-invertible")
        timeout = max(0.1, float(server.get("timeout_s") or 10.0))
        self.client = client_factory(
            str(server.get("host") or "127.0.0.1"),
            int(server.get("port") or 8000), timeout,
        )
        metadata = dict(getattr(self.client, "metadata", {}) or {})
        advertised = metadata.get("action_horizon")
        if advertised is not None and int(advertised) != self.horizon:
            raise ValueError(
                f"OpenPI server horizon {advertised} does not match artifact {self.horizon}"
            )
        self.executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="blacknode-openpi"
        )
        self.lock = threading.RLock()
        self.chunk: np.ndarray | None = None
        self.index = 0
        self.future: concurrent.futures.Future | None = None
        self.last_action: np.ndarray | None = None

    def _image(self, value: Any) -> np.ndarray:
        image = np.asarray(value, dtype=np.uint8)
        if image.ndim != 3 or image.shape[-1] != 3:
            raise ValueError(f"OpenPI camera image must be HWC RGB, got {image.shape}")
        if image.shape[:2] == (self.image_size, self.image_size):
            return image
        pil = PILImage.fromarray(image)
        width, height = pil.size
        ratio = max(width / self.image_size, height / self.image_size)
        resized = pil.resize(
            (int(width / ratio), int(height / ratio)), resample=PILImage.Resampling.BILINEAR
        )
        canvas = PILImage.new("RGB", (self.image_size, self.image_size), 0)
        canvas.paste(
            resized,
            ((self.image_size - resized.width) // 2, (self.image_size - resized.height) // 2),
        )
        return np.asarray(canvas, dtype=np.uint8)

    def _infer(self, qpos: list[float], images: dict[str, Any]) -> np.ndarray:
        robot_state = np.asarray(qpos, dtype=np.float32)
        policy_state = (
            robot_state * self.robot_to_policy_scale + self.robot_to_policy_offset
        )
        observation = {
            "observation/image": self._image(images[self.image_camera]),
            "observation/state": policy_state,
            "prompt": self.prompt,
        }
        if self.wrist_camera:
            observation["observation/wrist_image"] = self._image(images[self.wrist_camera])
        result = self.client.infer(observation)
        actions = np.asarray(result["actions"], dtype=np.float32)
        expected = (self.horizon, len(self.info["joint_names"]))
        if actions.shape != expected or not np.isfinite(actions).all():
            raise RuntimeError(
                f"OpenPI returned invalid action chunk {actions.shape}; expected {expected}"
            )
        return (
            (actions - self.robot_to_policy_offset) / self.robot_to_policy_scale
        ).astype(np.float32)

    def _accept_finished_future(self) -> None:
        if self.future is not None and self.future.done():
            self.chunk = self.future.result()
            self.future = None
            self.index = 0

    def predict(
        self,
        qpos: list[float],
        images: dict[str, Any],
        context: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        del context
        with self.lock:
            self._accept_finished_future()
            if self.chunk is None:
                self.chunk = self._infer(qpos, images)
                self.index = 0
            waiting = self.index >= self.execute_count
            if waiting:
                action = self.last_action if self.last_action is not None else self.chunk[-1]
            else:
                action = self.chunk[self.index]
                self.last_action = action.copy()
                self.index += 1
            if self.index >= self.async_start and self.future is None:
                qcopy = [float(value) for value in qpos]
                icopy = {name: np.asarray(value).copy() for name, value in images.items()}
                self.future = self.executor.submit(self._infer, qcopy, icopy)
            return {
                "kind": "blacknode.policy-prediction",
                "schema_version": 1,
                "joint_names": list(self.info["joint_names"]),
                "action": np.asarray(action, dtype=float).tolist(),
                "chunk_index": int(self.index),
                "chunk_waiting": bool(waiting),
                "backend": "openpi-remote",
            }

    def reset(self) -> None:
        with self.lock:
            if self.future is not None:
                if not self.future.cancel():
                    self.future.result()
            self.future = None
            self.chunk = None
            self.index = 0
            self.last_action = None

    def close(self) -> None:
        self.executor.shutdown(wait=True, cancel_futures=True)
        self.client.close()


def _load_policy(artifact: dict[str, Any], device: str) -> Any:
    if artifact.get("policy_type") == "openpi-pi05" and artifact.get("backend") == "openpi-remote":
        return OpenPIRemotePolicy(artifact, device)
    try:
        if artifact.get("policy_type") == "ppo-so101-reach":
            from blacknode.pkg.blacknode_training.ppo_runtime import PPOPolicy

            return PPOPolicy(artifact, device)
        from blacknode.pkg.blacknode_training.runtime import ACTPolicy
    except Exception as exc:
        raise RuntimeError("blacknode-training is required to load the policy artifact") from exc
    return ACTPolicy(artifact, device)


class PolicyRun:
    def __init__(
        self,
        run_id: str,
        artifact: dict[str, Any],
        robot: dict[str, Any],
        camera_streams: list[Any],
        safety: dict[str, Any],
        *,
        device: str = "auto",
        policy_loader: Callable[[dict[str, Any], str], Any] = _load_policy,
        io_factory: Callable[[dict[str, Any], dict[str, dict[str, Any]], dict[str, Any]], Any] = RosbridgePolicyIO,
    ) -> None:
        self.run_id = run_id
        self.artifact = dict(artifact)
        self.robot = dict(robot)
        self.safety = dict(safety)
        self.contract = validate_deployment_contract(self.artifact, self.robot, camera_streams, self.safety)
        self.cameras = _camera_handles(camera_streams, self.contract["camera_names"])
        self.policy = policy_loader(self.artifact, device)
        self.io = io_factory(self.contract, self.cameras, self.safety)
        self.gate = SafetyGate(self.contract["joint_names"], self.contract["joint_specs"], self.safety)
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.thread = threading.Thread(target=self._loop, daemon=True, name=f"blacknode-policy-{run_id}")
        self.armed = False
        self.estop = False
        self.takeover = False
        self.synchronized = False
        self.phase = "starting"
        self.last_error = ""
        self.last_prediction: dict[str, Any] = {}
        self.last_action: dict[str, float] = {}
        self.last_clamped: list[str] = []
        self.last_command_at = 0.0
        self.inference_count = 0
        self.command_count = 0
        self.blocked_count = 0
        self.inference_ms: deque[float] = deque(maxlen=100)
        self.started_at = time.time()
        configured_log = str(self.safety.get("log_dir") or "").strip()
        log_dir = Path(configured_log).expanduser().resolve() if configured_log else Path.cwd() / ".blacknode" / "policy-runs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"{run_id}.jsonl"

    def start(self) -> None:
        self.io.start()
        self.phase = "preview"
        self.thread.start()

    def _write_log(self, value: dict[str, Any]) -> None:
        payload = {"recorded_at_ns": time.time_ns(), "run_id": self.run_id, **value}
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, separators=(",", ":"), sort_keys=True) + "\n")

    def _safe_release(self) -> None:
        try:
            result = self.io.control("enter_teach")
            if isinstance(result, dict) and not result.get("ok", False):
                raise RuntimeError(str(result.get("error") or "robot driver rejected torque release"))
        except Exception as exc:
            with self.lock:
                self.last_error = f"torque release failed: {type(exc).__name__}: {exc}"

    def control(self, action: str) -> None:
        action = str(action or "").lower()
        with self.lock:
            if action == "arm":
                if self.estop:
                    raise RuntimeError("reset the latched emergency stop before arming")
                if self.takeover:
                    raise RuntimeError("reset human takeover before arming")
                if self.phase not in {"preview", "running"}:
                    raise RuntimeError("policy runtime is not ready to arm")
                if not self.contract["physical_motion_authorized"]:
                    raise RuntimeError("policy artifact has not been authorized for physical motion")
                result = self.io.control("exit_teach")
                if isinstance(result, dict) and not result.get("ok", False):
                    raise RuntimeError(str(result.get("error") or "robot driver rejected hold request"))
                self.armed = True
                self.synchronized = False
                self.phase = "running"
            elif action == "disarm":
                self.armed = False
                self.synchronized = False
                self.phase = "preview"
                self._safe_release()
            elif action == "estop":
                self.estop = True
                self.armed = False
                self.synchronized = False
                self.phase = "emergency_stopped"
                self._safe_release()
            elif action == "takeover":
                self.takeover = True
                self.armed = False
                self.synchronized = False
                self.phase = "human_takeover"
                self._safe_release()
            elif action == "reset_estop":
                self.estop = False
                self.phase = "human_takeover" if self.takeover else "preview"
            elif action == "reset_takeover":
                self.takeover = False
                self.phase = "emergency_stopped" if self.estop else "preview"
            else:
                raise ValueError(f"unknown policy control action: {action}")
        self._write_log({"event": action, "phase": self.phase, "armed": self.armed})

    def step(self) -> dict[str, Any]:
        snapshot = self.io.snapshot()
        stale_after = max(0.05, float(self.safety.get("stale_after") or 0.5))
        ages = [float(snapshot["pose_age"]), *[float(value) for value in snapshot["camera_ages"].values()]]
        if self.safety.get("workspace_limits"):
            ages.append(float(snapshot["workspace_age"]))
        if any(age > stale_after for age in ages):
            raise RuntimeError(f"source data is stale ({max(ages):.3f}s > {stale_after:.3f}s)")
        pose = dict(snapshot["pose"])
        qpos = [pose[name] for name in self.contract["joint_names"]]
        started = time.perf_counter()
        prediction = self.policy.predict(qpos, snapshot["images"], context=snapshot)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        now = time.monotonic()
        dt = now - self.last_command_at if self.last_command_at else 1.0 / max(1.0, float(self.safety.get("loop_hz") or 10.0))
        gated = self.gate.apply(prediction.get("action") or [], pose, dt=dt, workspace=snapshot.get("workspace"))
        commanded = False
        action = dict(gated.get("action") or {})
        with self.lock:
            self.inference_count += 1
            self.inference_ms.append(elapsed_ms)
            self.last_prediction = dict(prediction)
            self.last_action = action
            self.last_clamped = list(gated.get("clamped") or [])
            allowed = bool(gated.get("ok")) and self.armed and not self.estop and not self.takeover
            if allowed and not self.synchronized:
                action = {name: float(pose[name]) for name in self.contract["joint_names"]}
                self.synchronized = True
            if allowed:
                self.io.publish(action)
                self.last_command_at = now
                self.command_count += 1
                commanded = True
            elif not gated.get("ok"):
                self.blocked_count += 1
                if self.armed:
                    raise RuntimeError(f"safety gate blocked motion: {gated.get('reason') or 'unknown reason'}")
        event = {
            "event": "inference", "phase": self.phase, "armed": self.armed,
            "commanded": commanded, "source_age_ms": max(ages) * 1000.0,
            "inference_ms": elapsed_ms, "prediction": prediction.get("action") or [],
            "action": action if commanded else {}, "clamped": list(gated.get("clamped") or []),
            "blocked_reason": str(gated.get("reason") or ""),
        }
        self._write_log(event)
        return event

    def _loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                try:
                    self.step()
                    with self.lock:
                        self.last_error = ""
                        if self.phase == "fault" and not self.armed and not self.estop and not self.takeover:
                            self.phase = "preview"
                except Exception as exc:  # noqa: BLE001 - surfaced in status and replay log
                    with self.lock:
                        self.last_error = f"{type(exc).__name__}: {exc}"
                        self.blocked_count += 1
                        was_armed = self.armed
                        self.armed = False
                        self.synchronized = False
                        if self.phase not in {"emergency_stopped", "human_takeover"}:
                            self.phase = "fault"
                    if was_armed:
                        self._safe_release()
                    self._write_log({"event": "fault", "error": self.last_error, "commands_suppressed": True})
                period = 1.0 / max(1.0, min(60.0, float(self.safety.get("loop_hz") or 10.0)))
                self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))
        finally:
            self._safe_release()
            self.io.close()
            with self.lock:
                self.armed = False
                self.phase = "stopped"

    def stop(self) -> None:
        self.stop_event.set()
        self._safe_release()
        if self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)
        self.io.close()
        try:
            close_policy = getattr(self.policy, "close", None)
            if callable(close_policy):
                close_policy()
        except Exception:
            pass

    def status(self) -> dict[str, Any]:
        with self.lock:
            inference_ms = list(self.inference_ms)
            return {
                "kind": "blacknode.policy-runtime",
                "schema_version": 1,
                "run_id": self.run_id,
                "running": self.thread.is_alive(),
                "phase": self.phase,
                "armed": self.armed,
                "emergency_stop": self.estop,
                "human_takeover": self.takeover,
                "synchronized": self.synchronized,
                "inference_count": self.inference_count,
                "command_count": self.command_count,
                "blocked_count": self.blocked_count,
                "mean_inference_ms": sum(inference_ms) / len(inference_ms) if inference_ms else 0.0,
                "last_prediction": dict(self.last_prediction),
                "last_action": dict(self.last_action),
                "clamped": list(self.last_clamped),
                "last_error": self.last_error,
                "log_path": str(self.log_path),
                "elapsed_seconds": max(0.0, time.time() - self.started_at),
            }


_runs: dict[str, PolicyRun] = {}
_lock = threading.RLock()


def start_policy(
    run_id: str,
    artifact: dict[str, Any],
    robot: dict[str, Any],
    camera_streams: list[Any],
    safety: dict[str, Any],
    device: str,
) -> dict[str, Any]:
    with _lock:
        current = _runs.get(run_id)
        if current and current.thread.is_alive():
            raise RuntimeError(f"policy runtime {run_id!r} is already active")
        run = PolicyRun(run_id, artifact, robot, camera_streams, safety, device=device)
        _runs[run_id] = run
    try:
        run.start()
    except Exception:
        with _lock:
            _runs.pop(run_id, None)
        run.io.close()
        raise
    return run.status()


def policy_status(run_id: str) -> dict[str, Any]:
    with _lock:
        run = _runs.get(run_id)
    if run is None:
        return {
            "kind": "blacknode.policy-runtime", "schema_version": 1, "run_id": run_id,
            "running": False, "phase": "not_started", "armed": False,
            "emergency_stop": False, "human_takeover": False, "inference_count": 0,
            "command_count": 0, "blocked_count": 0, "mean_inference_ms": 0.0,
            "last_prediction": {}, "last_action": {}, "clamped": [], "last_error": "", "log_path": "",
        }
    return run.status()


def control_policy(run_id: str, action: str) -> dict[str, Any]:
    with _lock:
        run = _runs.get(run_id)
    if run is None:
        raise ValueError(f"policy runtime {run_id!r} was not found")
    if action == "stop":
        run.stop()
    else:
        run.control(action)
    return run.status()


def runtime_status() -> list[dict[str, Any]]:
    with _lock:
        return [run.status() for run in _runs.values() if run.thread.is_alive()]


def stop_policy_services() -> dict[str, Any]:
    with _lock:
        runs = list(_runs.values())
    errors = []
    for run in runs:
        try:
            run.stop()
        except Exception as exc:  # pragma: no cover - defensive shutdown
            errors.append(f"{run.run_id}: {exc}")
    return {"ok": not errors, "stopped": len(runs), "errors": errors}


atexit.register(stop_policy_services)
