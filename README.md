# blacknode-motion

This package owns robot motion planning, trajectory generation, execution,
learned motion policies, arbitration, and motion safety over stable capability
contracts. Vendor SDKs and physical transports remain in `blacknode-drivers`.

```text
blacknode-motion
├── core
├── arm
│   ├── planning
│   │   └── providers
│   │       ├── moveit
│   │       └── cumotion
│   ├── trajectory
│   └── execution
├── base
│   ├── planning
│   │   └── providers
│   │       └── nav2
│   └── execution
├── policy
└── safety
```

## Public components

| Component | Purpose |
|---|---|
| `core` | Motion ownership, priority, and arbitration contracts |
| `arm` | Arm planning, trajectories, execution, and ROS 2 control surfaces |
| `base` | Base planning, navigation providers, execution, and ROS 2 control |
| `policy` | Learned-motion policy execution and lifecycle |
| `safety` | Motion freshness, limits, stop, and shutdown supervision |

## Arm control

`JointMotionProfile` is transport-neutral and belongs to `arm/trajectory`. It
generates canonical direct, linear, trapezoidal, or minimum-jerk joint
trajectories.

The nested `arm/ros2` adapter provides `ROS2JointSliders`,
`ROS2MotionDashboard`, `ROS2SetJoint`, `ROS2JointState`, and
`ROS2ManualMove`. These nodes are control surfaces: they submit bounded,
explicitly armed commands through the arm-controller path. They do not own a
physical servo bus.

Motion remains disarmed by default. Armed moves synchronize to current pose,
apply calibrated limits, and preserve driver heartbeat safeguards.

## Base and policy control

The `base/ros2` adapter owns bounded base commands, stop, odometry, and scan
checks. Navigation implementations such as Nav2 belong under
`base/planning/providers/`.

The `policy` component owns learned-policy lifecycle and safety-gated
execution. Model architecture and training remain in `blacknode-training`.

## Provider placement

MoveIt and cuMotion integrations belong under `arm/planning/providers/`. A provider is
added to the manifest when its usable implementation, dependencies, lifecycle,
unavailable-state reporting, and tests exist.

ROS integration stays nested under the domain that owns it:

```text
blacknode-motion/arm/adapters/ros2
blacknode-motion/base/adapters/ros2
blacknode-motion/policy/adapters/ros2
```

## Verification

From the Blacknode repository root:

```powershell
python -m pytest packages/blacknode-motion/tests
```
