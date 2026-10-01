from __future__ import annotations

from ifcpath.model import InavModel
from ifcpath.semantic_skeleton import (
    _has_elevator_transport,
    build_semantic_skeleton_from_ifc,
)


class _FakeIfc:
    def __init__(self, by_type: dict[str, list[object]] | None = None) -> None:
        self._by_type = by_type or {}

    def by_type(self, name: str):
        return list(self._by_type.get(name, ()))


class _FakeTransport:
    def __init__(self, predefined_type: str) -> None:
        self.PredefinedType = predefined_type


def test_semantic_skeleton_has_no_legacy_graph_for_empty_ifc(monkeypatch) -> None:
    fake = _FakeIfc()
    monkeypatch.setattr("ifcpath.semantic_skeleton.ifcopenshell.open", lambda _: fake)
    monkeypatch.setattr("ifcpath.semantic_skeleton._levels", lambda _: [])
    monkeypatch.setattr("ifcpath.semantic_skeleton._contained_levels", lambda *_: [])
    monkeypatch.setattr(
        "ifcpath.semantic_skeleton._boundary_space_info",
        lambda *_: ({}, set()),
    )

    model = build_semantic_skeleton_from_ifc("empty.ifc")

    assert model.metadata["semantic_import_mode"] == "semantics-only"
    assert model.nodes == []
    assert model.edges == []
    assert model.cells == []
    assert model.spaces == []
    assert model.portals == []


def test_elevator_transport_is_detected() -> None:
    fake = _FakeIfc(
        {"IfcTransportElement": [_FakeTransport("ESCALATOR"), _FakeTransport("ELEVATOR")]}
    )
    assert _has_elevator_transport(fake) is True


def test_elevator_ifc_falls_back_to_full_importer(monkeypatch) -> None:
    fake = _FakeIfc({"IfcTransportElement": [_FakeTransport("ELEVATOR")]})
    full = InavModel(metadata={"marker": "full"})
    monkeypatch.setattr("ifcpath.semantic_skeleton.ifcopenshell.open", lambda _: fake)
    monkeypatch.setattr("ifcpath.semantic_skeleton.build_from_ifc", lambda *_: full)

    model = build_semantic_skeleton_from_ifc("elevator.ifc")

    assert model is full
    assert model.metadata["semantic_import_mode"] == "full-legacy-elevator"
    assert model.metadata["marker"] == "full"
