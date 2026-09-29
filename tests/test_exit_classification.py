from __future__ import annotations

from dataclasses import dataclass, field

from ifcpath.ifc_loader import _boundary_space_info, _door_is_exit
from ifcpath.model import Space


@dataclass
class _Value:
    wrappedValue: object


@dataclass
class _Property:
    Name: str
    NominalValue: object


@dataclass
class _Pset:
    Name: str
    HasProperties: list[object]


@dataclass
class _DefinedBy:
    RelatingPropertyDefinition: object


@dataclass
class _Door:
    IsDefinedBy: list[object] = field(default_factory=list)
    IsTypedBy: list[object] = field(default_factory=list)


@dataclass
class _Element:
    entity_id: int
    def id(self) -> int:
        return self.entity_id


@dataclass
class _SpaceEntity:
    entity_id: int
    def id(self) -> int:
        return self.entity_id


@dataclass
class _Boundary:
    RelatingSpace: object
    RelatedBuildingElement: object
    InternalOrExternalBoundary: str


class _Model:
    def __init__(self, boundaries: list[object]) -> None:
        self.boundaries = boundaries

    def by_type(self, type_name: str):
        return self.boundaries if type_name == "IfcRelSpaceBoundary" else []


def _door_with_external(value: bool) -> _Door:
    pset = _Pset("Pset_DoorCommon", [_Property("IsExternal", _Value(value))])
    return _Door(IsDefinedBy=[_DefinedBy(pset)])


def test_door_common_is_external_true_overrides_space_count() -> None:
    assert _door_is_exit(_door_with_external(True), {10}, set(), 2)


def test_door_common_is_external_false_prevents_single_space_false_positive() -> None:
    assert not _door_is_exit(_door_with_external(False), {10}, set(), 1)


def test_external_space_boundary_marks_exit_when_property_is_unknown() -> None:
    assert _door_is_exit(_Door(), {10, 20}, {20}, 2)


def test_single_space_remains_fallback_for_imperfect_ifc() -> None:
    assert _door_is_exit(_Door(), {10}, set(), 1)


def test_boundary_info_records_explicit_external_element() -> None:
    space_entity = _SpaceEntity(1)
    element = _Element(10)
    model = _Model([_Boundary(space_entity, element, "EXTERNAL")])
    space = Space("S1", "Room", "L1")
    mapping, external = _boundary_space_info(model, {1: space})
    assert mapping[10] == [space]
    assert external == {10}
