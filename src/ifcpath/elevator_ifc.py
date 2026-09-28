from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .model import InavModel, NavEdge, NavNode, Portal


@dataclass(frozen=True, slots=True)
class ElevatorExtractionStats:
    detected: int = 0
    connected: int = 0
    landings: int = 0
    rejected: int = 0


@dataclass(frozen=True, slots=True)
class _LandingCandidate:
    level_id: str
    portal: Portal
    walk_node: NavNode
    footprint_distance_m: float
    walk_distance_m: float


def add_ifc_elevator_connectors(
    ifc_model,
    out: InavModel,
    *,
    bbox_provider: Callable[[object], tuple[float, float, float, float, float, float] | None],
    door_search_distance_m: float = 1.0,
    landing_connect_distance_m: float = 2.5,
) -> ElevatorExtractionStats:
    """Add conservative metric elevator components from IFC transport elements.

    An ``IfcTransportElement(ELEVATOR)`` only tells us that an elevator exists;
    its representation is not guaranteed to be the full shaft. IFCPath therefore
    does **not** infer served storeys from the element's Z extent.

    Instead, the element's world-space XY footprint identifies the shaft/car
    location and already-qualified IFC door portals provide candidate landings.
    A connector is emitted only when at least two distinct levels have both:

    - a door close to that footprint; and
    - an existing walk node in one of the door's connected semantic spaces.

    This intentionally fails closed on ambiguous BIM. The inferred vertical nodes
    keep the elevator IFC GUID in their IDs so semantic post-processing can assign
    one stable shared resource ID across all adjacent-level transitions.
    """
    detected = 0
    connected = 0
    landing_count = 0
    rejected = 0

    try:
        transport_elements = list(ifc_model.by_type("IfcTransportElement"))
    except Exception:
        transport_elements = []

    level_elevations = {level.id: float(level.elevation_m) for level in out.levels}
    walk_nodes = [
        node
        for node in out.nodes
        if node.kind == "walk" and node.level_id and node.space_id
    ]

    for entity in transport_elements:
        if _predefined_type(entity) != "ELEVATOR":
            continue
        detected += 1
        bbox = bbox_provider(entity)
        if bbox is None:
            rejected += 1
            continue

        center_xy = ((bbox[0] + bbox[3]) * 0.5, (bbox[1] + bbox[4]) * 0.5)
        candidates_by_level: dict[str, list[_LandingCandidate]] = {}
        for portal in out.portals:
            if not portal.level_id or portal.level_id not in level_elevations:
                continue
            footprint_distance = _distance_xy_to_box(portal.position_m, bbox)
            if footprint_distance > max(0.0, door_search_distance_m):
                continue

            connected_spaces = {
                space_id
                for space_id in (portal.from_space_id, portal.to_space_id)
                if space_id
            }
            if not connected_spaces:
                continue
            nearest = [
                (_dist(portal.position_m, node.position_m), node)
                for node in walk_nodes
                if node.level_id == portal.level_id and node.space_id in connected_spaces
            ]
            if not nearest:
                continue
            walk_distance, walk_node = min(nearest, key=lambda item: (item[0], item[1].id))
            if walk_distance > max(0.0, landing_connect_distance_m):
                continue
            candidates_by_level.setdefault(portal.level_id, []).append(
                _LandingCandidate(
                    level_id=portal.level_id,
                    portal=portal,
                    walk_node=walk_node,
                    footprint_distance_m=footprint_distance,
                    walk_distance_m=walk_distance,
                )
            )

        selected: list[_LandingCandidate] = []
        for level_id, candidates in candidates_by_level.items():
            selected.append(
                min(
                    candidates,
                    key=lambda item: (
                        item.footprint_distance_m,
                        item.walk_distance_m,
                        item.portal.id,
                    ),
                )
            )
        selected.sort(
            key=lambda item: (
                level_elevations.get(item.level_id, item.walk_node.position_m[2]),
                item.level_id,
            )
        )

        if len(selected) < 2:
            rejected += 1
            continue

        guid = str(getattr(entity, "GlobalId", "") or f"entity-{entity.id()}")
        elevator_nodes: list[NavNode] = []
        for landing in selected:
            node = NavNode(
                id=f"elevator:{guid}:{landing.level_id}",
                position_m=(
                    float(center_xy[0]),
                    float(center_xy[1]),
                    float(landing.walk_node.position_m[2]),
                ),
                kind="elevator",
                level_id=landing.level_id,
            )
            elevator_nodes.append(node)
            out.nodes.append(node)
            out.edges.append(
                NavEdge(
                    a=node.id,
                    b=landing.walk_node.id,
                    distance_m=_dist(node.position_m, landing.walk_node.position_m),
                    kind="elevator",
                    portal_id=landing.portal.id,
                )
            )
            # A one-sided landing door is not an exterior building exit merely
            # because the elevator shaft/cab is not represented as IfcSpace.
            landing.portal.is_exit = False

        for lower, upper in zip(elevator_nodes, elevator_nodes[1:]):
            out.edges.append(
                NavEdge(
                    a=lower.id,
                    b=upper.id,
                    distance_m=_dist(lower.position_m, upper.position_m),
                    kind="elevator",
                )
            )

        connected += 1
        landing_count += len(elevator_nodes)

    return ElevatorExtractionStats(
        detected=detected,
        connected=connected,
        landings=landing_count,
        rejected=rejected,
    )


def _predefined_type(entity) -> str:
    try:
        from ifcopenshell.util.element import get_predefined_type

        value = get_predefined_type(entity)
    except Exception:
        value = getattr(entity, "PredefinedType", None)
    return str(value or "").upper()


def _distance_xy_to_box(
    point: tuple[float, float, float],
    box: tuple[float, float, float, float, float, float],
) -> float:
    dx = max(box[0] - point[0], 0.0, point[0] - box[3])
    dy = max(box[1] - point[1], 0.0, point[1] - box[4])
    return (dx * dx + dy * dy) ** 0.5


def _dist(a, b) -> float:
    return (
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    ) ** 0.5
