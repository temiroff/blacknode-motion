# Safety

This component owns motion-level freshness, calibrated joint and velocity
limits, collision-provider checks, command authorization, stop, and shutdown
supervision.

Driver packages independently retain communication timeouts, vendor-fault and
temperature handling, final limit clamps, and torque disable. Physical or
firmware emergency stop remains independent of both software layers.
