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
- The base ROS 2 adapter provides bounded base moves, stop, odometry, and LaserScan safety checks.
- The policy adapter starts in prediction preview and requires a separate arm action before commands can flow.

Applications and skills submit requests through the motion gateway. The gateway applies ownership, freshness, limits, and authorization before a profile-selected driver receives a command.

## Safety

- Motion is disarmed by default.
- The first armed target synchronizes to current feedback.
- Stale state, emergency stop, faults, takeover, or shutdown suppress commands.
- Driver-level clamping and physical emergency stops remain independent safety layers.

## Install and verify

```powershell
blacknode packages install https://github.com/temiroff/blacknode-motion.git
python -m pytest packages/blacknode-motion/tests
```

See [AGENTS.md](AGENTS.md) for ownership and controller rules.
