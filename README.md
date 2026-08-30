# blacknode-motion

`blacknode-motion` owns robot motion planning, trajectories, execution, learned-policy control, arbitration, and safety over stable capability contracts.

## Components

| Component | Default | Purpose |
|---|---:|---|
| `core` | On | Internal command ownership, priority, and arbitration |
| `arm` | On | Arm trajectories, execution, manipulation, and ROS 2 control surfaces |
| `base` | Off | Mobile-base planning, execution, odometry, scan checks, and stop |
| `policy` | On | Learned-policy lifecycle and safety-gated execution |
| `safety` | On | Freshness, calibrated limits, stop, and shutdown supervision |

`joint-control` is a deprecated alias for `arm`; `mobile-base` is a deprecated alias for `base`. Both are planned for removal in version 1.0.0.

## Main interfaces

- `JointMotionProfile` creates direct, linear, trapezoidal, or minimum-jerk joint trajectories.
- The arm ROS 2 adapter provides joint state, sliders, dashboards, manual moves, and bounded joint commands.
- The base ROS 2 adapter provides bounded base moves, stop, odometry, LaserScan safety checks, session-scoped Nav2, cancellable `NavigateTo` goals, and managed frontier exploration over a live SLAM map.
- The policy adapter starts in prediction preview and requires a separate arm action before commands can flow.
- Simulation providers can evaluate compatible normalized joint-delta PPO artifacts. Physical deployment requires a passed qualification plus `PolicyDeploymentAuthorize`, bound to the exact artifact, calibrated robot identity, and safety configuration.
- `PolicyRuntime` accepts the resulting authorization, starts disarmed, and supports semantic observation fields from live joint state, workspace state, and an explicit observation context.

Applications and skills submit requests through the motion gateway. The gateway applies ownership, freshness, limits, and authorization before a profile-selected driver receives a command.

## Safety

- Motion is disarmed by default.
- The first armed target synchronizes to current feedback.
- Stale state, emergency stop, faults, takeover, or shutdown suppress commands.
- Driver-level clamping and physical emergency stops remain independent safety layers.

## ROSOrin navigation

`NavigationSession` attaches to an existing `/navigate_to_pose` server or can
start a separately configured Blacknode-owned `nav2_bringup` process.
The ROSOrin workflow creates a Blacknode-owned parameter overlay from its main
Nav2 and DWB controller YAML files, then launches Nav2 against the saved map.
The source vendor files and base-driver lifecycle remain unchanged. `NavigateTo` begins in preview mode and accepts a goal
only with a fresh `BaseSafetyGate` authorization. The managed goal client
cancels its own goal on timeout, explicit cancel, Runtime shutdown, or workflow
replacement. It applies the authorization's linear-speed cap through Nav2 and
restores the provider maximum when the goal finishes.

The `ROSOrin Navigate Saved Map` workflow starts disarmed. Set its map path and
goal, confirm the LiDAR clearance, and then explicitly arm the gate. Stopping
the workflow leaves the vendor ROS workspace and boot services unchanged.

## Autonomous environment familiarization

`ExploreEnvironment` selects boundaries between known free space and unknown
space from the live occupancy grid, asks Nav2 to reach safe candidates, and
continues until no usable frontier remains. The managed worker requires a fresh
`BaseSafetyGate` authorization before it starts and continuously checks map,
LiDAR, and localization freshness. It cancels the active goal and publishes a
zero velocity when those inputs become stale, an obstacle violates the direct
clearance threshold, the operator pauses or stops the session, or Runtime shuts
down.

When exploration completes, the worker saves the SLAM Toolbox occupancy map and
pose graph, verifies the `map` to `base_link` transform, and reports:
`Environment mapped. Localization ready. Waiting for command.` The
`ROSOrin Familiarize Environment` workflow starts SLAM, brings up Nav2 in
live-SLAM mode, checks current LiDAR clearance, and starts exploration only
after its safety gate is explicitly armed.

## Install and verify

```powershell
blacknode packages install https://github.com/temiroff/blacknode-motion.git
python -m pytest packages/blacknode-motion/tests
```

See [AGENTS.md](AGENTS.md) for ownership and controller rules.
