from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from .model import InavModel, NavCell, SemanticTransition, Vec3
from .surface_funnel import SurfaceFunnelRoute, find_surface_funnel_route
from .surface_nav import cell_centroid

SURFACE_VERTICAL_TERRAINS = {"stair", "ramp", "escalator"}


@dataclass(frozen=True, slots=True)
class SurfaceLanding:
    space_id: str
    level_id: str | None
    vertical_cell_id: str
    open_cell_id: str
    point: Vec3
    attachment_distance_m: float


@dataclass(frozen=True, slots=True)
class SurfaceVerticalComponent:
    kind: str
    cell_ids: tuple[str, ...]
    resource_id: str
    landings: tuple[SurfaceLanding, ...]


@dataclass(frozen=True, slots=True)
class SurfaceVerticalTransfer:
    points: tuple[Vec3, ...]
    cell_ids: tuple[str, ...]
    resource_id: str
    kind: str

    @property
    def length_m(self) -> float:
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))


def surface_vertical_components(model: InavModel) -> list[SurfaceVerticalComponent]:
    """Return connected walkable stair/ramp/escalator surface components.

    A component is defined by explicit ``NavCell.neighbor_ids`` adjacency, not by
    a radius search. Landing ownership is taken from adjacent ``open`` cells that
    carry semantic ``space_id``/``level_id`` values.
    """
    by_id = {cell.id: cell for cell in model.cells}
    vertical_ids = {
        cell.id for cell in model.cells if cell.terrain in SURFACE_VERTICAL_TERRAINS
    }
    remaining = set(vertical_ids)
    components: list[SurfaceVerticalComponent] = []

    while remaining:
        start = min(remaining)
        remaining.remove(start)
        ids = {start}
        queue = deque([start])
        while queue:
            current_id = queue.popleft()
            current = by_id[current_id]
            for neighbor_id in current.neighbor_ids:
                if neighbor_id not in remaining:
                    continue
                neighbor = by_id.get(neighbor_id)
                if neighbor is None or neighbor.terrain not in SURFACE_VERTICAL_TERRAINS:
                    continue
                remaining.remove(neighbor_id)
                ids.add(neighbor_id)
                queue.append(neighbor_id)

        cells = [by_id[cell_id] for cell_id in sorted(ids)]
        kinds = {cell.terrain for cell in cells}
        kind = next(iter(kinds)) if len(kinds) == 1 else "vertical"
        landings = _component_landings(cells, by_id)
        components.append(
            SurfaceVerticalComponent(
                kind=kind,
                cell_ids=tuple(sorted(ids)),
                resource_id=_surface_resource_id(kind, cells),
                landings=tuple(landings),
            )
        )

    return sorted(components, key=lambda item: item.resource_id)


def ensure_surface_vertical_transitions(model: InavModel) -> list[SemanticTransition]:
    """Infer adjacent-storey semantic transitions from continuous surface contact.

    This runs before the legacy sampled-node inference in normal load/export and
    routing paths. Existing exact ``kind + space-pair`` transitions are preserved.
    Elevators are intentionally absent because they remain explicit resources.
    """
    existing = {
        (transition.kind, *sorted((transition.from_space_id, transition.to_space_id)))
        for transition in model.transitions
        if transition.to_space_id
    }
    level_elevation = {level.id: level.elevation_m for level in model.levels}
    added: list[SemanticTransition] = []

    for component_index, component in enumerate(surface_vertical_components(model)):
        by_level: dict[str, SurfaceLanding] = {}
        for landing in component.landings:
            if not landing.level_id:
                continue
            current = by_level.get(landing.level_id)
            candidate_key = (
                landing.attachment_distance_m,
                landing.space_id,
                landing.vertical_cell_id,
                landing.open_cell_id,
            )
            current_key = (
                current.attachment_distance_m,
                current.space_id,
                current.vertical_cell_id,
                current.open_cell_id,
            ) if current is not None else None
            if current is None or candidate_key < current_key:
                by_level[landing.level_id] = landing

        if len(by_level) < 2:
            continue
        ordered_levels = sorted(
            by_level,
            key=lambda level_id: (
                level_elevation.get(level_id, _landing_z(by_level[level_id])),
                level_id,
            ),
        )
        for lower_level, upper_level in zip(ordered_levels, ordered_levels[1:]):
            lower = by_level[lower_level]
            upper = by_level[upper_level]
            if lower.space_id == upper.space_id:
                continue
            key = (component.kind, *sorted((lower.space_id, upper.space_id)))
            if key in existing:
                continue
            transition = SemanticTransition(
                id=(
                    f"transition:surface:{component.kind}:{component_index}:"
                    f"{lower.space_id}:{upper.space_id}"
                ),
                kind=component.kind,
                from_space_id=lower.space_id,
                to_space_id=upper.space_id,
                portal_id=None,
                from_level_id=lower_level,
                to_level_id=upper_level,
                bidirectional=True,
                source="surface_vertical_touch",
                resource_id=component.resource_id,
            )
            model.transitions.append(transition)
            added.append(transition)
            existing.add(key)

    if added:
        model.metadata["surface_vertical_transition_count"] = sum(
            1 for transition in model.transitions
            if transition.source == "surface_vertical_touch"
        )
    return added


def find_surface_vertical_transfer(
    model: InavModel,
    transition: SemanticTransition,
) -> SurfaceVerticalTransfer | None:
    """Resolve one semantic vertical transition through actual walkable terrain."""
    if not transition.to_space_id or transition.kind == "elevator":
        return None

    best: SurfaceVerticalTransfer | None = None
    by_id = {cell.id: cell for cell in model.cells}
    for component in surface_vertical_components(model):
        if transition.resource_id and transition.source == "surface_vertical_touch":
            if component.resource_id != transition.resource_id:
                continue
        if transition.kind != "vertical" and component.kind not in {transition.kind, "vertical"}:
            continue

        from_landings = [
            landing for landing in component.landings
            if landing.space_id == transition.from_space_id
            and _level_matches(landing.level_id, transition.from_level_id)
        ]
        to_landings = [
            landing for landing in component.landings
            if landing.space_id == transition.to_space_id
            and _level_matches(landing.level_id, transition.to_level_id)
        ]
        if not from_landings or not to_landings:
            continue

        component_cells = [by_id[cell_id] for cell_id in component.cell_ids if cell_id in by_id]
        for from_landing in from_landings:
            for to_landing in to_landings:
                route = find_surface_funnel_route(
                    component_cells,
                    from_landing.point,
                    to_landing.point,
                )
                if route is None or not route.points:
                    continue
                candidate = _as_transfer(route, component)
                if best is None or (candidate.length_m, candidate.cell_ids) < (
                    best.length_m,
                    best.cell_ids,
                ):
                    best = candidate
    return best


def _component_landings(
    cells: list[NavCell],
    by_id: dict[str, NavCell],
) -> list[SurfaceLanding]:
    best: dict[tuple[str | None, str], SurfaceLanding] = {}
    component_ids = {cell.id for cell in cells}
    for cell in cells:
        for neighbor_id in cell.neighbor_ids:
            if neighbor_id in component_ids:
                continue
            neighbor = by_id.get(neighbor_id)
            if neighbor is None or neighbor.terrain != "open" or not neighbor.space_id:
                continue
            point = _portal_midpoint(cell, neighbor)
            landing = SurfaceLanding(
                space_id=neighbor.space_id,
                level_id=neighbor.level_id,
                vertical_cell_id=cell.id,
                open_cell_id=neighbor.id,
                point=point,
                attachment_distance_m=math.dist(cell_centroid(cell), cell_centroid(neighbor)),
            )
            key = (neighbor.level_id, neighbor.space_id)
            current = best.get(key)
            if current is None or (
                landing.attachment_distance_m,
                landing.vertical_cell_id,
                landing.open_cell_id,
            ) < (
                current.attachment_distance_m,
                current.vertical_cell_id,
                current.open_cell_id,
            ):
                best[key] = landing
    return sorted(
        best.values(),
        key=lambda item: (item.level_id or "", item.space_id, item.vertical_cell_id),
    )


def _portal_midpoint(a: NavCell, b: NavCell) -> Vec3:
    segment = a.portals.get(b.id) or b.portals.get(a.id)
    if segment is not None:
        p, q = segment
        return (
            (p[0] + q[0]) * 0.5,
            (p[1] + q[1]) * 0.5,
            (p[2] + q[2]) * 0.5,
        )
    ca, cb = cell_centroid(a), cell_centroid(b)
    return (
        (ca[0] + cb[0]) * 0.5,
        (ca[1] + cb[1]) * 0.5,
        (ca[2] + cb[2]) * 0.5,
    )


def _surface_resource_id(kind: str, cells: list[NavCell]) -> str:
    guid_candidates: set[str] = set()
    patterned = 0
    for cell in cells:
        parts = cell.id.split(":")
        if len(parts) >= 4 and parts[0] == "cell" and parts[1] in SURFACE_VERTICAL_TERRAINS:
            patterned += 1
            guid_candidates.add(parts[2])
    if patterned == len(cells) and len(guid_candidates) == 1:
        return f"surface:{kind}:{next(iter(guid_candidates))}"
    return f"surface:{kind}:{min(cell.id for cell in cells)}"


def _as_transfer(
    route: SurfaceFunnelRoute,
    component: SurfaceVerticalComponent,
) -> SurfaceVerticalTransfer:
    return SurfaceVerticalTransfer(
        points=tuple(route.points),
        cell_ids=tuple(route.cell_ids),
        resource_id=component.resource_id,
        kind=component.kind,
    )


def _landing_z(landing: SurfaceLanding) -> float:
    return landing.point[2]


def _level_matches(actual: str | None, requested: str | None) -> bool:
    return requested is None or actual is None or actual == requested
