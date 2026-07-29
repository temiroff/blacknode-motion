"""Transport-neutral motion ownership and command authorization."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any


class MotionAuthorizationError(RuntimeError):
    """Raised when motion ownership or authorization rejects a command."""


@dataclass(frozen=True)
class MotionLease:
    resource: str
    owner: str
    priority: int
    issued_at: float
    expires_at: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "kind": "blacknode.motion-authorization",
            "schema_version": 1,
            "resource": self.resource,
            "owner": self.owner,
            "priority": self.priority,
            "issued_at": self.issued_at,
            "expires_at": self.expires_at,
            "authorized": True,
        }


class CommandArbiter:
    """Grant one fresh owner at a time for each physical motion resource."""

    def __init__(self) -> None:
        self._leases: dict[str, MotionLease] = {}
        self._lock = threading.Lock()

    def authorize(
        self,
        resource: str,
        owner: str,
        *,
        armed: bool,
        priority: int = 0,
        ttl: float = 1.0,
        emergency_stop: bool = False,
        now: float | None = None,
    ) -> MotionLease:
        current = time.monotonic() if now is None else float(now)
        resource = str(resource).strip()
        owner = str(owner).strip()
        if not resource or not owner:
            raise MotionAuthorizationError(
                "motion resource and owner must be explicit"
            )
        if emergency_stop:
            self.release_resource(resource)
            raise MotionAuthorizationError("emergency stop is active")
        if not armed:
            raise MotionAuthorizationError("motion is disarmed")
        expires_at = current + max(0.05, min(float(ttl), 5.0))
        with self._lock:
            existing = self._leases.get(resource)
            if (
                existing is not None
                and existing.expires_at > current
                and existing.owner != owner
                and existing.priority >= int(priority)
            ):
                raise MotionAuthorizationError(
                    f"motion resource '{resource}' is owned by "
                    f"'{existing.owner}'"
                )
            lease = MotionLease(
                resource=resource,
                owner=owner,
                priority=int(priority),
                issued_at=current,
                expires_at=expires_at,
            )
            self._leases[resource] = lease
            return lease

    def release_owner(self, owner: str) -> None:
        with self._lock:
            self._leases = {
                resource: lease
                for resource, lease in self._leases.items()
                if lease.owner != owner
            }

    def release_resource(self, resource: str) -> None:
        with self._lock:
            self._leases.pop(str(resource), None)

    def reset(self) -> None:
        with self._lock:
            self._leases.clear()


command_arbiter = CommandArbiter()

