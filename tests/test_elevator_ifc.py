from __future__ import annotations

from dataclasses import dataclass

from ifcpath.elevator_ifc import add_ifc_elevator_connectors
from ifcpath.model import InavModel, Level, NavNode, Portal, Space
from ifcpath.semantic import ensure_semantic_transitions


@dataclass
class _FakeTransportElement:
    GlobalId: str
    PredefinedType: str
    entity_id: int = 1

    def id(self) -> int:
        return self.entity_id


class _FakeIfcModel:
    def __init__(self, elements: list[_FakeTransportElement]) -> None:
        self._elements = list(elements)

    def by_type(self, type_name: str):
        if type_name == "IfcTransportElement":
            return list(self._elements)
        return []


def _three_floor_lobbies() -> InavModel:
    model = InavModel(
        levels=[
            Level("L1", "Ground", 0.0),
            Level("L2", "First", 3.0),
            Level("L3", "Second", 6.0),
        ],
        spaces=[
            Space("S1", "Lobby 1", "L1"),
            Space("S2", "Lobby 2", "L2"),
            Space("S3", "Lobby 3", "L3"),
            Space("OUT", "Exterior-like room", "L1"),
        ],
        nodes=[
            NavNode("walk:L1", (1.8, 2.0, 0.0), level_id="L1", space_id="S1"),
            NavNode("walk:L2", (1.8, 2.0, 3.0), level_id="L2", space_id="S2"),
            NavNode("walk:L3", (1.8, 2.0, 6.0), level_id="L3", space_id="S3"),
            NavNode("walk:far", (20.0, 0.0, 0.0), level_id="L1", space_id="OUT"),
        ],
        portals=[
            Portal(
                "door:elevator:L1",
                "door",
                (1.4, 2.0, 0.0),
                from_space_id="S1",
                level_id="L1",
                is_exit=True,
            ),
            Portal(
                "door:elevator:L2",
                "door",
                (1.4, 2.0, 3.0),
                from_space_id="S2",
                level_id="L2",
                is_exit=True,
            ),
            Portal(
                "door:elevator:L3",
                "door",
                (1.4, 2.0, 6.0),
                from_space_id="S3",
                level_id="L3",
                is_exit=True,
            ),
            Portal(
                "door:unrelated-exit",
                "door",
                (20.0, 0.0, 0.0),
                from_space_id="OUT",
                level_id="L1",
                is_exit=True,
            ),
        ],
    )
    return model


def _bbox_provider(_entity):
    # Compact world-space car/shaft footprint centered on x=1, y=2. The Z extent
    # is deliberately tiny: extraction must derive served levels from landings,
    # never from this bbox's height.
    return (0.5, 1.5, 0.0, 1.5, 2.5, 1.0)


def test_elevator_requires_and_connects_multiple_qualified_landings() -> None:
    model = _three_floor_lobbies()
    ifc = _FakeIfcModel([_FakeTransportElement("ELV-GUID", "ELEVATOR")])

    stats = add_ifc_elevator_connectors(
        ifc,
        model,
        bbox_provider=_bbox_provider,
        door_search_distance_m=0.25,
        landing_connect_distance_m=1.0,
    )

    assert stats.detected == 1
    assert stats.connected == 1
    assert stats.landings == 3
    assert stats.rejected == 0

    elevator_nodes = [node for node in model.nodes if node.kind == "elevator"]
    assert {node.id for node in elevator_nodes} == {
        "elevator:ELV-GUID:L1",
        "elevator:ELV-GUID:L2",
        "elevator:ELV-GUID:L3",
    }
    # Two shaft edges plus one landing attachment per served level.
    elevator_edges = [edge for edge in model.edges if edge.kind == "elevator"]
    assert len(elevator_edges) == 5
    assert sum(edge.portal_id is None for edge in elevator_edges) == 2
    assert sum(edge.portal_id is not None for edge in elevator_edges) == 3

    # Landing doors are internal circulation, not accidental exterior exits.
    landing_portals = [portal for portal in model.portals if portal.id.startswith("door:elevator:")]
    assert landing_portals and all(not portal.is_exit for portal in landing_portals)
    unrelated = next(portal for portal in model.portals if portal.id == "door:unrelated-exit")
    assert unrelated.is_exit

    transitions = [
        transition
        for transition in ensure_semantic_transitions(model)
        if transition.kind == "elevator"
    ]
    assert len(transitions) == 2
    assert {transition.resource_id for transition in transitions} == {"elevator:ELV-GUID"}
    assert {(item.from_level_id, item.to_level_id) for item in transitions} == {
        ("L1", "L2"),
        ("L2", "L3"),
    }


def test_single_landing_is_detected_but_rejected_fail_closed() -> None:
    model = _three_floor_lobbies()
    model.portals = [model.portals[0], model.portals[-1]]
    ifc = _FakeIfcModel([_FakeTransportElement("ELV-GUID", "ELEVATOR")])

    stats = add_ifc_elevator_connectors(
        ifc,
        model,
        bbox_provider=_bbox_provider,
        door_search_distance_m=0.25,
        landing_connect_distance_m=1.0,
    )

    assert stats.detected == 1
    assert stats.connected == 0
    assert stats.landings == 0
    assert stats.rejected == 1
    assert not [node for node in model.nodes if node.kind == "elevator"]
    assert not [edge for edge in model.edges if edge.kind == "elevator"]
    # A rejected candidate is not silently reclassified as internal circulation.
    assert model.portals[0].is_exit


def test_non_elevator_transport_elements_are_ignored() -> None:
    model = _three_floor_lobbies()
    ifc = _FakeIfcModel(
        [
            _FakeTransportElement("ESC-GUID", "ESCALATOR"),
            _FakeTransportElement("LIFT-GUID", "LIFTINGGEAR", entity_id=2),
        ]
    )

    stats = add_ifc_elevator_connectors(ifc, model, bbox_provider=_bbox_provider)

    assert stats.detected == 0
    assert stats.connected == 0
    assert stats.rejected == 0
    assert not [node for node in model.nodes if node.kind == "elevator"]


def test_geometryless_elevator_is_reported_as_rejected() -> None:
    model = _three_floor_lobbies()
    ifc = _FakeIfcModel([_FakeTransportElement("ELV-GUID", "ELEVATOR")])

    stats = add_ifc_elevator_connectors(ifc, model, bbox_provider=lambda _entity: None)

    assert stats.detected == 1
    assert stats.connected == 0
    assert stats.rejected == 1
