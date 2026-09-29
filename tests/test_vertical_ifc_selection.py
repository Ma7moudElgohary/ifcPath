from __future__ import annotations

from dataclasses import dataclass, field

from ifcpath.ifc_loader import _vertical_walk_entities


@dataclass
class _Rel:
    RelatedObjects: list[object]


@dataclass
class _Entity:
    type_name: str
    name: str
    IsDecomposedBy: list[_Rel] = field(default_factory=list)

    def is_a(self, type_name: str) -> bool:
        return self.type_name == type_name


class _Model:
    def __init__(self, entities: list[_Entity]) -> None:
        self.entities = entities

    def by_type(self, type_name: str):
        return [entity for entity in self.entities if entity.type_name == type_name]


def test_decomposed_stair_uses_flights_but_keeps_unrelated_monolithic_stair() -> None:
    flight_a = _Entity("IfcStairFlight", "A-flight")
    stair_a = _Entity("IfcStair", "A", [_Rel([flight_a])])
    stair_b = _Entity("IfcStair", "B")
    selected = _vertical_walk_entities(
        _Model([stair_a, flight_a, stair_b]), "IfcStair", "IfcStairFlight"
    )
    assert {entity.name for entity in selected} == {"A-flight", "B"}


def test_decomposed_ramp_is_not_sampled_twice() -> None:
    flight = _Entity("IfcRampFlight", "ramp-flight")
    ramp = _Entity("IfcRamp", "ramp", [_Rel([flight])])
    selected = _vertical_walk_entities(
        _Model([ramp, flight]), "IfcRamp", "IfcRampFlight"
    )
    assert [entity.name for entity in selected] == ["ramp-flight"]


def test_monolithic_vertical_element_is_preserved_when_no_flights_exist() -> None:
    stair = _Entity("IfcStair", "monolithic")
    selected = _vertical_walk_entities(
        _Model([stair]), "IfcStair", "IfcStairFlight"
    )
    assert selected == [stair]
