# blacknode-motion Agent Instructions

This is an independent Blacknode extension-package repository.

Keep motion arbitration in `core`; arm planning, trajectories, and execution in
`arm`; base planning and execution in `base`; learned-policy execution in
`policy`; and cross-motion supervision in `safety`. Planning providers belong
under the owning domain's `planning/providers` directory. Consume stable state
and command capabilities; never import vendor hardware SDKs. Motion stays
disarmed by default. Enforce
freshness, calibrated limits, idempotence, ownership, emergency stop, and
explicit shutdown at every controller boundary. Test with mock or replay
providers before supported hardware providers.

Run package tests with `python -m pytest packages/blacknode-motion/tests`.
