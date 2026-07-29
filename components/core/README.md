# Core

This internal component owns motion leases, priority, and command
authorization. Arm, base, policy, and safety activate it automatically.

One physical motion resource has one fresh owner. A competing command is
rejected unless its priority permits takeover. Disarming, lease expiry, stop,
and emergency stop remove command authority.
