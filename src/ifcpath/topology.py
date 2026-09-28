from __future__ import annotations

import math

from .models import Level, Space, Vec3


def nearest_level_id(levels: list[Level], point: Vec3) -> str | None:
    if not levels:
        return None
    return min(levels, key=lambda l: abs(l.elevation_m - point[2])).id


def containing_space_id(spaces: list[Space], point: Vec3, tolerance_m: float = 0.08) -> str | None:
    x, y, z = point
    candidates: list[tuple[float, str]] = []
    for space in spaces:
        if not space.bounds_m:
            continue
        lo, hi = space.bounds_m
        if (
            lo[0] - tolerance_m <= x <= hi[0] + tolerance_m
            and lo[1] - tolerance_m <= y <= hi[1] + tolerance_m
            and lo[2] - tolerance_m <= z <= hi[2] + tolerance_m
        ):
            if space.centroid_m:
                candidates.append((math.dist(point, space.centroid_m), space.id))
            else:
                candidates.append((0.0, space.id))
    if not candidates:
        return None
    candidates.sort(key=lambda x: x[0])
    return candidates[0][1]
