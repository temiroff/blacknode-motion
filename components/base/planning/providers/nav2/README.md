# Nav2 provider

`NavigationSession` attaches to an existing Nav2 action server or starts a
session-scoped Nav2 bringup against a persisted map. `NavigateTo` previews,
sends, monitors, and cancels one goal through a managed `rclpy` action client.

Managed sessions never edit or disable robot-vendor launch files, systemd
units, workspaces, or boot configuration. Stop targets only the process handle
Blacknode started. The goal client cancels its own accepted goal on timeout,
explicit cancel, Runtime shutdown, or workflow replacement, then the adapter
publishes a zero `Twist` on the configured command topic.

`NavigateTo` starts in `preview` and requires a fresh authorization from
`BaseSafetyGate` before `send` can create a physical navigation goal. Its
managed action client applies the gate's linear-speed cap through Nav2's
`SpeedLimit` topic and restores the provider maximum when the goal ends.

The ROSOrin workflow combines the vendor `nav2_params.yaml` and tuned DWB
controller YAML into a Blacknode-owned runtime overlay. It launches standard
`nav2_bringup` with the saved Blacknode map and never starts a second robot base
driver. The source ROSOrin YAML files remain unchanged.
