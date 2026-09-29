from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

Vec3 = tuple[float, float, float]


@dataclass(slots=True)
class Level:
    id: str
    name: str
    elevation_m: float


@dataclass(slots=True)
class Space:
    id: str
    name: str
    level_id: str | None
    centroid_m: Vec3 | None = None
    ifc_guid: str | None = None
    is_external: bool = False
    # Whether this space must participate in occupant-egress readiness. Service
    # domains such as roof/plenum/void spaces remain routable for maintenance,
    # but do not make an otherwise qualified occupant model fail readiness.
    egress_required: bool = True


@dataclass(slots=True)
class Portal:
    id: str
    kind: str
    position_m: Vec3
    from_space_id: str | None = None
    to_space_id: str | None = None
    level_id: str | None = None
    width_m: float | None = None
    ifc_guid: str | None = None
    is_exit: bool = False


@dataclass(slots=True)
class SemanticTransition:
    id: str
    kind: str
    from_space_id: str
    to_space_id: str | None = None
    portal_id: str | None = None
    from_level_id: str | None = None
    to_level_id: str | None = None
    bidirectional: bool = True
    source: str = "ifc"
    resource_id: str | None = None


@dataclass(slots=True)
class NavCell:
    """Portable triangular navigation-mesh cell.

    Cells preserve the constrained triangulation rather than only its centroid
    graph. ``portals`` stores the geometric crossing segment for each adjacent
    cell. ``portal_ids`` binds an adjacency to an IFC semantic portal (normally
    a door), allowing dynamic closure/blocking without modifying the mesh.
    """

    id: str
    vertices_m: tuple[Vec3, Vec3, Vec3]
    space_id: str | None = None
    level_id: str | None = None
    neighbor_ids: list[str] = field(default_factory=list)
    terrain: str = "open"
    portals: dict[str, tuple[Vec3, Vec3]] = field(default_factory=dict)
    portal_ids: dict[str, str] = field(default_factory=dict)


@dataclass(slots=True)
class NavNode:
    id: str
    position_m: Vec3
    kind: str = "walk"
    level_id: str | None = None
    space_id: str | None = None
    portal_id: str | None = None
    cell_id: str | None = None


@dataclass(slots=True)
class NavEdge:
    a: str
    b: str
    distance_m: float
    kind: str = "walk"
    portal_id: str | None = None


@dataclass(slots=True)
class InavModel:
    schema: str = "ifcpath.inav/0.3"
    units: str = "m"
    up_axis: str = "Z"
    levels: list[Level] = field(default_factory=list)
    spaces: list[Space] = field(default_factory=list)
    portals: list[Portal] = field(default_factory=list)
    transitions: list[SemanticTransition] = field(default_factory=list)
    cells: list[NavCell] = field(default_factory=list)
    nodes: list[NavNode] = field(default_factory=list)
    edges: list[NavEdge] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
