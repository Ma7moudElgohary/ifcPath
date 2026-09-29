from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from ..model import Vec3


@dataclass(slots=True, frozen=True)
class ResidentBatchInfo:
    key: str
    byte_size: int
    center: Vec3


@dataclass(slots=True, frozen=True)
class ResidencyPlan:
    upload: tuple[str, ...]
    evict: tuple[str, ...]
    resident: tuple[str, ...]
    resident_bytes: int
    visible_resident: int
    visible_requested: int
    visible_dropped: tuple[str, ...]
    target_pending: tuple[str, ...]


class GpuResidentSet:
    """Deterministic memory-budget policy for static GPU geometry batches.

    The CPU-side scene remains authoritative. This class decides which static
    batches deserve GPU residency for the current camera. Visible batches are
    always highest priority, followed by nearby prefetch candidates. Existing
    non-visible residents receive a small retention bias to avoid churn during
    small camera movements.

    Static memory and upload work are independently bounded: the resident-set
    target obeys ``budget_bytes`` while each plan admits only a limited number / 
    number of bytes of new uploads. Large camera jumps therefore converge over a
    few frames instead of blocking the UI with a burst of GPU allocation.
    """

    def __init__(
        self,
        budget_bytes: int,
        *,
        prefetch_batches: int = 12,
        retention_bias: float = 0.80,
        max_upload_batches: int = 8,
        max_upload_bytes: int = 64 * 1024 * 1024,
    ) -> None:
        self.budget_bytes = max(1, int(budget_bytes))
        self.prefetch_batches = max(0, int(prefetch_batches))
        self.retention_bias = min(1.0, max(0.1, float(retention_bias)))
        self.max_upload_batches = max(1, int(max_upload_batches))
        self.max_upload_bytes = max(1, int(max_upload_bytes))
        self._catalog: dict[str, ResidentBatchInfo] = {}
        self._resident: set[str] = set()

    @property
    def resident_keys(self) -> set[str]:
        return set(self._resident)

    @property
    def resident_bytes(self) -> int:
        return sum(self._catalog[key].byte_size for key in self._resident if key in self._catalog)

    def reset(self, batches: Iterable[ResidentBatchInfo]) -> None:
        self._catalog = {batch.key: batch for batch in batches}
        self._resident.clear()

    def set_budget(self, budget_bytes: int) -> None:
        self.budget_bytes = max(1, int(budget_bytes))

    def plan(self, *, visible_keys: Iterable[str], camera_position: Vec3) -> ResidencyPlan:
        visible = {key for key in visible_keys if key in self._catalog}
        ranked_visible = sorted(
            visible,
            key=lambda key: (self._distance_sq(self._catalog[key], camera_position), key),
        )

        invisible = [key for key in self._catalog if key not in visible]
        invisible.sort(
            key=lambda key: (
                self._distance_sq(self._catalog[key], camera_position)
                * (self.retention_bias if key in self._resident else 1.0),
                key,
            )
        )
        ranked = ranked_visible + invisible[: self.prefetch_batches]

        target: list[str] = []
        target_bytes = 0
        for key in ranked:
            size = max(0, self._catalog[key].byte_size)
            if not target and size > self.budget_bytes:
                target.append(key)
                target_bytes += size
                continue
            if target_bytes + size > self.budget_bytes:
                continue
            target.append(key)
            target_bytes += size

        target_set = set(target)
        evict = tuple(sorted(self._resident - target_set))
        retained = self._resident & target_set

        uploads: list[str] = []
        upload_bytes = 0
        for key in target:
            if key in retained:
                continue
            size = max(0, self._catalog[key].byte_size)
            if uploads and (
                len(uploads) >= self.max_upload_batches
                or upload_bytes + size > self.max_upload_bytes
            ):
                continue
            # Always admit at least the highest-priority pending batch even if a
            # single chunk exceeds the per-frame upload target.
            uploads.append(key)
            upload_bytes += size
            if len(uploads) >= self.max_upload_batches:
                break

        resident = retained | set(uploads)
        pending = tuple(key for key in target if key not in resident)
        dropped = tuple(key for key in ranked_visible if key not in resident)
        self._resident = resident
        used = sum(self._catalog[key].byte_size for key in resident)

        return ResidencyPlan(
            upload=tuple(uploads),
            evict=evict,
            resident=tuple(key for key in target if key in resident),
            resident_bytes=used,
            visible_resident=sum(1 for key in visible if key in resident),
            visible_requested=len(visible),
            visible_dropped=dropped,
            target_pending=pending,
        )

    @staticmethod
    def _distance_sq(batch: ResidentBatchInfo, camera_position: Vec3) -> float:
        return sum((batch.center[axis] - camera_position[axis]) ** 2 for axis in range(3))


def batch_center(bounds_min: Vec3, bounds_max: Vec3) -> Vec3:
    return tuple((bounds_min[axis] + bounds_max[axis]) * 0.5 for axis in range(3))  # type: ignore[return-value]


def budget_bytes_from_mb(value: float) -> int:
    if not math.isfinite(value):
        value = 512.0
    return max(1, int(max(1.0, value) * 1024 * 1024))
