from pathlib import Path

# Arm is one component module even though its implementation is organized into
# trajectory and execution directories. Extend this component package's module
# search path so both remain available under the stable
# ``blacknode.pkg.blacknode_motion.arm`` namespace.
__path__.append(
    str(Path(__file__).resolve().parents[2] / "execution" / "nodes")
)

from . import arm_controller, motion_profiles, servo_control  # noqa: E402,F401
