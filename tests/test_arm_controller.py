import blacknode  # noqa: F401
import pytest
from blacknode.pkg.blacknode_motion.arm import arm_controller
from blacknode.pkg.blacknode_motion.core import arbitration


def setup_function():
    arbitration.command_arbiter.reset()


def test_arm_gateway_requires_arming_limits_and_fresh_feedback():
    published = []
    publish = lambda target: published.append(target) or {"ok": True}
    common = {
        "publish": publish,
        "resource": "arm-01",
        "owner": "ui:test",
        "current": {"shoulder": 0.0},
        "target": {"shoulder": 0.5},
        "limits": {"shoulder": (-1.0, 1.0)},
    }

    assert arm_controller.execute_joint_target(**common, armed=False)["ok"] is False
    assert arm_controller.execute_joint_target(
        **{**common, "limits": {}},
        armed=True,
    )["error"].startswith("calibrated joint limits")
    assert arm_controller.execute_joint_target(
        **common,
        armed=True,
        feedback_age=1.0,
        stale_after=0.5,
    )["error"] == "joint feedback is stale"
    assert published == []


def test_arm_gateway_arbitrates_owners_and_clamps_velocity():
    published = []
    first = arm_controller.execute_joint_target(
        lambda target: published.append(target) or {"ok": True},
        resource="arm-01",
        owner="skill:follow",
        current={"shoulder": 0.0},
        target={"shoulder": 1.5},
        limits={"shoulder": (-2.0, 2.0)},
        armed=True,
        max_velocity=0.5,
        interval=0.2,
    )
    blocked = arm_controller.execute_joint_target(
        lambda _target: {"ok": True},
        resource="arm-01",
        owner="ui:sliders",
        current={"shoulder": 0.0},
        target={"shoulder": 0.2},
        limits={"shoulder": (-2.0, 2.0)},
        armed=True,
    )

    assert first["ok"] is True
    assert published == [{"shoulder": pytest.approx(0.1)}]
    assert first["authorization"]["owner"] == "skill:follow"
    assert blocked["ok"] is False
    assert "owned by 'skill:follow'" in blocked["error"]


def test_emergency_stop_releases_motion_resource():
    arm_controller.execute_joint_target(
        lambda _target: {"ok": True},
        resource="arm-01",
        owner="policy",
        current={"shoulder": 0.0},
        target={"shoulder": 0.1},
        limits={"shoulder": (-1.0, 1.0)},
        armed=True,
    )
    stopped = arm_controller.execute_joint_target(
        lambda _target: {"ok": True},
        resource="arm-01",
        owner="policy",
        current={"shoulder": 0.0},
        target={"shoulder": 0.1},
        limits={"shoulder": (-1.0, 1.0)},
        armed=True,
        emergency_stop=True,
    )

    assert stopped == {"ok": False, "error": "emergency stop is active"}


def test_collision_provider_can_reject_before_transport_publish():
    published = []
    result = arm_controller.execute_joint_target(
        lambda target: published.append(target) or {"ok": True},
        resource="arm-01",
        owner="planner",
        current={"shoulder": 0.0},
        target={"shoulder": 0.2},
        limits={"shoulder": (-1.0, 1.0)},
        armed=True,
        collision_check=lambda _target: False,
    )

    assert result == {
        "ok": False,
        "error": "collision check rejected joint target",
    }
    assert published == []
