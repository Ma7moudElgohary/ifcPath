from __future__ import annotations

from enum import Enum
from pydantic import BaseModel, Field

Vec3 = tuple[float, float, float]
BBox3 = tuple[Vec3, Vec3]


class NodeKind(str, Enum):
    WALK = "walk"
    DOOR = "door"
    STAIR = "stair"
    EXIT = "exit"


class Level(BaseModel):
    id: str
    name: str | None = None
    elevation_m: float = 0.0
    ifc_guid: str | None = None


class Space(BaseModel):
    id: str
    name: str | None = None
    level_id: str | None = None
    ifc_guid: str | None = None
    centroid_m: Vec3 | None = None
    bounds_m: BBox3 | None = None


class Portal(BaseModel):
    id: str
    kind: str
    position_m: Vec3
    ifc_guid: str | None = None
    name: str | None = None
    level_id: str | None = None
    from_space_id: str | None = None
    to_space_id: str | None = None
    width_m: float | None = None
    enabled: bool = True
    is_external: bool = False


class NavNode(BaseModel):
    id: str
    position_m: Vec3
    kind: NodeKind = NodeKind.WALK
    level_id: str | None = None
    space_id: str | None = None
    source_ifc_guid: str | None = None


class NavEdge(BaseModel):
    a: str
    b: str
    length_m: float
    kind: str = "walk"
    portal_id: str | None = None
    enabled: bool = True
    cost_multiplier: float = 1.0


class NavigationModel(BaseModel):
    schema_version: str = "0.2.0"
    units: str = "m"
    up_axis: str = "Z"
    levels: list[Level] = Field(default_factory=list)
    spaces: list[Space] = Field(default_factory=list)
    portals: list[Portal] = Field(default_factory=list)
    nodes: list[NavNode] = Field(default_factory=list)
    edges: list[NavEdge] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)
