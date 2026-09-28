from __future__ import annotations

import math
from dataclasses import dataclass, field

from .model import Vec3

_EPSILON = 1e-9


@dataclass(slots=True)
class RouteWalker:
    """Deterministic distance-based walker over a 3D route polyline.

    The walker is intentionally UI-independent. Time is supplied by the caller
    and converted into travelled metres, so stairs/ramps naturally take longer
    than their horizontal projection and the same state machine can be reused by
    desktop, tests, or another host.
    """

    speed_mps: float = 1.35
    points: list[Vec3] = field(default_factory=list)
    distance_m: float = 0.0
    running: bool = False

    def set_route(
        self,
        points: list[Vec3] | tuple[Vec3, ...],
        *,
        start_distance_m: float = 0.0,
        running: bool | None = None,
    ) -> None:
        self.points = _dedupe_points(list(points))
        self.distance_m = min(max(0.0, float(start_distance_m)), self.total_length_m)
        if running is not None:
            self.running = bool(running) and not self.finished
        elif self.finished:
            self.running = False

    def clear(self) -> None:
        self.points.clear()
        self.distance_m = 0.0
        self.running = False

    def reset(self) -> None:
        self.distance_m = 0.0
        self.running = False

    def play(self) -> None:
        if len(self.points) >= 2 and not self.finished:
            self.running = True

    def pause(self) -> None:
        self.running = False

    def advance(self, delta_seconds: float) -> Vec3 | None:
        if self.running and delta_seconds > 0.0 and not self.finished:
            self.distance_m = min(
                self.total_length_m,
                self.distance_m + max(0.0, float(delta_seconds)) * max(0.0, self.speed_mps),
            )
            if self.finished:
                self.running = False
        return self.position

    @property
    def total_length_m(self) -> float:
        return polyline_length(self.points)

    @property
    def progress(self) -> float:
        total = self.total_length_m
        if total <= _EPSILON:
            return 0.0
        return min(1.0, self.distance_m / total)

    @property
    def finished(self) -> bool:
        total = self.total_length_m
        return bool(self.points) and self.distance_m >= total - _EPSILON

    @property
    def position(self) -> Vec3 | None:
        sample = sample_polyline(self.points, self.distance_m)
        return sample[0] if sample is not None else None

    @property
    def forward_xy(self) -> tuple[float, float]:
        sample = sample_polyline(self.points, self.distance_m)
        return sample[1] if sample is not None else (0.0, 1.0)

    @property
    def remaining_points(self) -> list[Vec3]:
        """Return a route beginning exactly at the current walker position."""
        if not self.points:
            return []
        sample = sample_polyline(self.points, self.distance_m, include_segment=True)
        if sample is None:
            return []
        position, _forward, segment_index = sample
        result = [position]
        for point in self.points[segment_index + 1 :]:
            if _distance(result[-1], point) > _EPSILON:
                result.append(point)
        return result


def polyline_length(points: list[Vec3] | tuple[Vec3, ...]) -> float:
    return sum(_distance(a, b) for a, b in zip(points, points[1:]))


def sample_polyline(
    points: list[Vec3] | tuple[Vec3, ...],
    distance_m: float,
    *,
    include_segment: bool = False,
):
    """Sample position and XY facing direction at travelled 3D distance."""
    if not points:
        return None
    if len(points) == 1:
        result = (points[0], (0.0, 1.0), 0)
        return result if include_segment else result[:2]

    remaining = max(0.0, float(distance_m))
    last_forward = (0.0, 1.0)
    for index, (a, b) in enumerate(zip(points, points[1:])):
        segment_length = _distance(a, b)
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        horizontal = math.hypot(dx, dy)
        if horizontal > _EPSILON:
            last_forward = (dx / horizontal, dy / horizontal)
        if segment_length <= _EPSILON:
            continue
        if remaining <= segment_length or index == len(points) - 2:
            t = min(1.0, remaining / segment_length)
            position: Vec3 = (
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            )
            result = (position, last_forward, index)
            return result if include_segment else result[:2]
        remaining -= segment_length

    result = (points[-1], last_forward, len(points) - 2)
    return result if include_segment else result[:2]


def _distance(a: Vec3, b: Vec3) -> float:
    return math.dist(a, b)


def _dedupe_points(points: list[Vec3]) -> list[Vec3]:
    result: list[Vec3] = []
    for point in points:
        value: Vec3 = (float(point[0]), float(point[1]), float(point[2]))
        if not result or _distance(result[-1], value) > _EPSILON:
            result.append(value)
    return result
