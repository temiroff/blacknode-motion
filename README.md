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
| `arm` | Arm planning, trajectories, execution, and ROS 2 control surfaces |
| `base` | Base planning, navigation providers, execution, and ROS 2 control |
| `policy` | Learned-motion policy execution and lifecycle |
| `safety` | Motion freshness, limits, stop, and shutdown supervision |

`core` is an internal component. `arm`, `base`, `policy`, and `safety`
activate it automatically; users do not select command arbitration separately.
Arm, base, and policy also activate `safety`.

## Arm control

`JointMotionProfile` is transport-neutral and belongs to `arm/trajectory`. It
generates canonical direct, linear, trapezoidal, or minimum-jerk joint
trajectories.

The nested `arm/ros2` adapter provides `ROS2JointSliders`,
`ROS2MotionDashboard`, `ROS2SetJoint`, `ROS2JointState`, and
`ROS2ManualMove`. These nodes are control surfaces. They submit requests to the
arm execution gateway, which applies command ownership and motion safety before
the ROS adapter publishes to a driver-owned endpoint.

`RobotServo` uses the generic arm execution gateway for standalone local USB
control. Its on-node **Arm** action selects the joint-motion provider from the
robot profile, seeds every joint from fresh feedback, and keeps arbitration and
calibrated bounds in the motion layer.

`ROS2JointSliders.command` also accepts the canonical command request produced
by a `RobotServo` for advanced workflows. Connect `command` to `command`, press
**Go Live**, verify the reported joint and limits, and explicitly arm the
motion node.

Motion remains disarmed by default. Armed moves synchronize to current pose,
apply calibrated limits, and preserve driver heartbeat safeguards.

```text
UI or skill
    -> motion/arm execution gateway
    -> core ownership arbitration
    -> motion safety
    -> profile-selected concrete driver
```

UI and skill modules must not call a driver adapter or raw command publisher.
The execution gateway is the only application-level command writer.

## Layered safety

| Layer | Responsibility |
|---|---|
| Driver | Communication timeout, vendor faults, temperature limits, final command clamping, and torque disable |
| Motion | Joint and velocity limits, collision-provider checks, freshness, command ownership, and authorization |
| Physical/firmware | Emergency-stop circuit and firmware-level shutdown |

Each layer remains effective when an upstream layer fails. Motion safety does
not replace driver or physical safeguards.

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

## Deprecated component names

| Deprecated | Replacement | Removal |
|---|---|---|
| `joint-control` | `arm` | `1.0.0` |
| `mobile-base` | `base` | `1.0.0` |

Selecting a deprecated name emits a warning that includes its replacement and
planned removal version.

## Verification

From the Blacknode repository root:

```powershell
python -m pytest packages/blacknode-motion/tests
```
