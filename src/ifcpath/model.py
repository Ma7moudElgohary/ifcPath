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
class NavNode:
    id: str
    position_m: Vec3
    kind: str = "walk"
    level_id: str | None = None
    space_id: str | None = None
    portal_id: str | None = None


@dataclass(slots=True)
class NavEdge:
    a: str
    b: str
    distance_m: float
    kind: str = "walk"
    portal_id: str | None = None


@dataclass(slots=True)
class InavModel:
    schema: str = "ifcpath.inav/0.1"
    units: str = "m"
    up_axis: str = "Z"
    levels: list[Level] = field(default_factory=list)
    spaces: list[Space] = field(default_factory=list)
    portals: list[Portal] = field(default_factory=list)
    nodes: list[NavNode] = field(default_factory=list)
    edges: list[NavEdge] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
