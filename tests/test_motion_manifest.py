import blacknode  # noqa: F401
from blacknode.node import _NODE_REGISTRY
from blacknode.packages import _PACKAGE_REGISTRY


def test_motion_layer_exposes_domain_components():
    info = _PACKAGE_REGISTRY["blacknode-motion"]
    assert info.ok
    assert info.layer == "motion"
    assert info.component_mode is True
    assert info.enabled_components == ["core", "arm", "policy", "safety"]
    assert set(info.components) == {"core", "arm", "base", "policy", "safety"}
    assert info.components["core"]["internal"] is True
    assert info.components["arm"]["aliases"] == ["joint-control"]
    assert info.components["base"]["aliases"] == ["mobile-base"]
    for component_name in ("arm", "base", "policy"):
        assert info.components[component_name]["requirements"] == [
            {
                "package": "",
                "component": "core",
                "version": ">=0.6.0,<1.0.0",
            },
            {
                "package": "",
                "component": "safety",
                "version": ">=0.6.0,<1.0.0",
            },
        ]
    assert info.components["safety"]["requirements"] == [{
        "package": "",
        "component": "core",
        "version": ">=0.6.0,<1.0.0",
    }]


def test_arm_owns_profiles_and_ros2_control_surfaces():
    info = _PACKAGE_REGISTRY["blacknode-motion"]
    arm = info.components["arm"]
    adapter = arm["adapters"]["ros2"]
    assert adapter["enabled"] is True
    assert arm["node_types"] == ["JointMotionProfile"]
    assert set(adapter["node_types"]) == {
        "ROS2JointSliders", "ROS2JointState", "ROS2ManualMove",
        "ROS2MotionDashboard", "ROS2SetJoint",
    }
    # the adapter must rest on the ROS 2 integration layer, never the reverse
    assert adapter["requirements"][0]["package"] == "blacknode-ros2"
    assert _NODE_REGISTRY["JointMotionProfile"]._bn_component == "arm"
    assert _NODE_REGISTRY["JointMotionProfile"]._bn_adapter == ""
    for node_name in ("ROS2JointSliders", "ROS2MotionDashboard", "ROS2SetJoint"):
        assert _NODE_REGISTRY[node_name]._bn_component == "arm"
        assert _NODE_REGISTRY[node_name]._bn_adapter == "ros2"
