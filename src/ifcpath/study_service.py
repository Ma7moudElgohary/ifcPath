from __future__ import annotations

import math
from typing import Any

from .evacuation import EvacuationAgentSpec
from .exporter import model_from_dict
from .hierarchical_routing import HierarchicalRouteOptions
from .hybrid_evacuation import HybridEvacuationConfig, HybridEvacuationSimulator
from .model import InavModel, NavCell, Vec3
from .simulation_playback import record_playback
from .surface_nav import closest_cell


class StudyRequestError(ValueError):
    """Raised when a live evacuation-study request is invalid."""


def run_study(payload: dict[str, Any]) -> dict[str, Any]:
    """Run one browser-authored evacuation study and return portable playback."""
    raw_model = payload.get("model")
    if not isinstance(raw_model, dict):
        raise StudyRequestError("study request requires an INAV model object")
    model = model_from_dict(raw_model)
    if not model.cells:
        raise StudyRequestError("INAV model has no navigation cells")

    backend = str(payload.get("backend", "kinematic")).strip().lower()
    if backend not in {"kinematic", "jupedsim"}:
        raise StudyRequestError(f"unsupported study backend: {backend}")

    groups = payload.get("groups", [])
    explicit_agents = payload.get("agents", [])
    if not isinstance(groups, list) or not isinstance(explicit_agents, list):
        raise StudyRequestError("groups and agents must be arrays")

    agents: list[EvacuationAgentSpec] = []
    for index, group in enumerate(groups):
        if not isinstance(group, dict):
            raise StudyRequestError(f"population group {index} must be an object")
        agents.extend(_expand_group(model, group, index))
    for index, item in enumerate(explicit_agents):
        if not isinstance(item, dict):
            raise StudyRequestError(f"agent {index} must be an object")
        agents.append(_agent_spec(item, f"agent:{index}"))
    if not agents:
        raise StudyRequestError("study has no agents or population groups")

    blocked_portals = {str(value) for value in payload.get("blocked_portals", []) if str(value)}
    blocked_spaces = {str(value) for value in payload.get("blocked_spaces", []) if str(value)}
    raw_multipliers = payload.get("space_cost_multipliers", {}) or {}
    if not isinstance(raw_multipliers, dict):
        raise StudyRequestError("space_cost_multipliers must be an object")
    multipliers = {
        str(key): max(1.0, float(value))
        for key, value in raw_multipliers.items()
        if str(key)
    }

    options = HierarchicalRouteOptions(
        blocked_portals=blocked_portals,
        blocked_spaces=blocked_spaces,
        space_cost_multipliers=multipliers,
    )
    simulator = HybridEvacuationSimulator(
        model,
        agents,
        options=options,
        hybrid_config=HybridEvacuationConfig(local_backend=backend),
    )
    playback = record_playback(
        simulator,
        frame_interval_s=float(payload.get("frame_interval_s", 0.20)),
        max_time_s=float(payload.get("max_time_s", 900.0)),
    )
    playback["scenario"] = {
        "agent_count": len(agents),
        "group_count": len(groups),
        "blocked_portals": sorted(blocked_portals),
        "blocked_spaces": sorted(blocked_spaces),
        "space_cost_multipliers": dict(sorted(multipliers.items())),
    }
    return playback


def _agent_spec(item: dict[str, Any], fallback_id: str) -> EvacuationAgentSpec:
    position = _vec3(item.get("start_m"), "agent start_m")
    return EvacuationAgentSpec(
        id=str(item.get("id") or fallback_id),
        start_m=position,
        speed_mps=max(0.05, float(item.get("speed_mps", 1.2))),
    )


def _expand_group(model: InavModel, group: dict[str, Any], index: int) -> list[EvacuationAgentSpec]:
    center = _vec3(group.get("position_m"), f"population group {index} position_m")
    count = int(group.get("count", 1))
    if count < 1 or count > 5000:
        raise StudyRequestError(f"population group {index} count must be between 1 and 5000")
    speed = max(0.05, float(group.get("speed_mps", 1.2)))
    spacing = max(0.20, float(group.get("spacing_m", 0.45)))
    prefix = str(group.get("id") or f"group:{index}")

    nearest = closest_cell(model.cells, center)
    if nearest is None:
        raise StudyRequestError(f"population group {index} is not near walkable navigation geometry")
    anchor_cell, projected, distance_m = nearest
    if distance_m > float(group.get("max_snap_distance_m", 3.0)):
        raise StudyRequestError(
            f"population group {index} is {distance_m:.2f} m from the navigation surface"
        )

    domain = [
        cell
        for cell in model.cells
        if (anchor_cell.space_id is None or cell.space_id == anchor_cell.space_id)
        and (anchor_cell.level_id is None or cell.level_id == anchor_cell.level_id)
        and cell.terrain == "open"
    ]
    if not domain:
        domain = [anchor_cell]

    candidates: list[Vec3] = [projected]
    for cell in sorted(domain, key=lambda item: (_distance(_centroid(item), projected), item.id)):
        candidates.extend(_triangle_candidates(cell))
    candidates = _dedupe_candidates(candidates)

    chosen: list[Vec3] = []
    for candidate in sorted(candidates, key=lambda point: (_distance(point, projected), point)):
        if all(_distance(candidate, existing) + 1e-9 >= spacing for existing in chosen):
            chosen.append(candidate)
            if len(chosen) >= count:
                break
    if len(chosen) < count:
        raise StudyRequestError(
            f"population group {index} cannot place {count} agents with {spacing:.2f} m spacing "
            f"inside space {anchor_cell.space_id or '<unowned>'}; placed {len(chosen)}"
        )

    return [
        EvacuationAgentSpec(
            id=f"{prefix}:{agent_index + 1}",
            start_m=point,
            speed_mps=speed,
        )
        for agent_index, point in enumerate(chosen)
    ]


def _triangle_candidates(cell: NavCell, grid: int = 8) -> list[Vec3]:
    a, b, c = cell.vertices_m
    result: list[Vec3] = [_centroid(cell)]
    # Interior barycentric lattice. Excluding exact edges prevents starts from
    # landing on walls/portal boundaries in clearance-eroded CDT spaces.
    for i in range(1, grid):
        for j in range(1, grid - i):
            k = grid - i - j
            if k <= 0:
                continue
            wa, wb, wc = i / grid, j / grid, k / grid
            result.append(
                (
                    a[0] * wa + b[0] * wb + c[0] * wc,
                    a[1] * wa + b[1] * wb + c[1] * wc,
                    a[2] * wa + b[2] * wb + c[2] * wc,
                )
            )
    return result


def _dedupe_candidates(points: list[Vec3]) -> list[Vec3]:
    seen: set[tuple[int, int, int]] = set()
    result: list[Vec3] = []
    for point in points:
        key = tuple(round(value * 10000) for value in point)
        if key in seen:
            continue
        seen.add(key)
        result.append(point)
    return result


def _centroid(cell: NavCell) -> Vec3:
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )


def _distance(a: Vec3, b: Vec3) -> float:
    return math.dist(a, b)


def _vec3(value: Any, label: str) -> Vec3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise StudyRequestError(f"{label} must be [x, y, z]")
    return tuple(float(component) for component in value)  # type: ignore[return-value]
