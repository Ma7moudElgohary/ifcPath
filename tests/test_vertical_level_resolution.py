from __future__ import annotations

from dataclasses import dataclass, field

from ifcpath.ifc_loader import _spatial_level_id


@dataclass
class _Rel:
    RelatingObject: object | None = None
    RelatingStructure: object | None = None


@dataclass
class _Entity:
    entity_id: int
    type_name: str
    GlobalId: str = ""
    Decomposes: list[object] = field(default_factory=list)
    ContainedInStructure: list[object] = field(default_factory=list)

    def id(self) -> int:
        return self.entity_id

    def is_a(self, type_name: str) -> bool:
        return self.type_name == type_name


def test_stair_flight_inherits_storey_through_parent_stair() -> None:
    storey = _Entity(1, "IfcBuildingStorey", GlobalId="STOREY")
    stair = _Entity(2, "IfcStair", ContainedInStructure=[_Rel(RelatingStructure=storey)])
    flight = _Entity(3, "IfcStairFlight", Decomposes=[_Rel(RelatingObject=stair)])

    level = _spatial_level_id(flight, {}, {"STOREY": "level:STOREY"})

    assert level == "level:STOREY"


def test_nested_decomposition_can_use_precomputed_parent_containment() -> None:
    stair = _Entity(2, "IfcStair")
    flight = _Entity(3, "IfcStairFlight", Decomposes=[_Rel(RelatingObject=stair)])

    level = _spatial_level_id(flight, {2: "level:L1"}, {})

    assert level == "level:L1"


def test_decomposition_cycles_do_not_loop() -> None:
    a = _Entity(1, "IfcStair")
    b = _Entity(2, "IfcStairFlight")
    a.Decomposes = [_Rel(RelatingObject=b)]
    b.Decomposes = [_Rel(RelatingObject=a)]

    assert _spatial_level_id(b, {}, {}) is None
