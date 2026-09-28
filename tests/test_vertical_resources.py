from __future__ import annotations

from ifcpath.exporter import load_inav, save_inav
from ifcpath.model import InavModel, Level, NavEdge, NavNode, Space
from ifcpath.semantic import ensure_semantic_transitions


def _three_floor_elevator_component() -> InavModel:
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
        ],
        nodes=[
            NavNode("landing-1", (2.0, 2.0, 0.0), kind="walk", level_id="L1", space_id="S1"),
            NavNode("elevator:ELV-GUID:L1", (2.0, 2.0, 0.1), kind="elevator"),
            NavNode("elevator:ELV-GUID:L2", (2.0, 2.0, 3.0), kind="elevator"),
            NavNode("elevator:ELV-GUID:L3", (2.0, 2.0, 5.9), kind="elevator"),
            NavNode("landing-2", (2.0, 2.0, 3.0), kind="walk", level_id="L2", space_id="S2"),
            NavNode("landing-3", (2.0, 2.0, 6.0), kind="walk", level_id="L3", space_id="S3"),
        ],
        edges=[
            NavEdge("landing-1", "elevator:ELV-GUID:L1", 0.1),
            NavEdge("elevator:ELV-GUID:L1", "elevator:ELV-GUID:L2", 2.9),
            NavEdge("elevator:ELV-GUID:L2", "elevator:ELV-GUID:L3", 2.9),
            NavEdge("elevator:ELV-GUID:L2", "landing-2", 0.1),
            NavEdge("elevator:ELV-GUID:L3", "landing-3", 0.1),
        ],
    )
    return model


def test_adjacent_elevator_transitions_share_stable_ifc_resource_id() -> None:
    model = _three_floor_elevator_component()
    transitions = [
        transition
        for transition in ensure_semantic_transitions(model)
        if transition.kind == "elevator"
    ]

    assert len(transitions) == 2
    assert {(item.from_level_id, item.to_level_id) for item in transitions} == {
        ("L1", "L2"),
        ("L2", "L3"),
    }
    assert {transition.resource_id for transition in transitions} == {"elevator:ELV-GUID"}


def test_vertical_resource_id_survives_inav_roundtrip(tmp_path) -> None:
    model = _three_floor_elevator_component()
    ensure_semantic_transitions(model)
    path = save_inav(model, tmp_path / "elevator.inav")

    loaded = load_inav(path)
    transitions = [item for item in loaded.transitions if item.kind == "elevator"]

    assert len(transitions) == 2
    assert {item.resource_id for item in transitions} == {"elevator:ELV-GUID"}


def test_legacy_elevator_node_ids_keep_deterministic_fallback_resource() -> None:
    model = InavModel(
        levels=[Level("L1", "Ground", 0.0), Level("L2", "First", 3.0)],
        spaces=[Space("S1", "S1", "L1"), Space("S2", "S2", "L2")],
        nodes=[
            NavNode("walk-1", (0.0, 0.0, 0.0), level_id="L1", space_id="S1"),
            NavNode("legacy-elevator-a", (0.0, 0.0, 0.1), kind="elevator"),
            NavNode("legacy-elevator-b", (0.0, 0.0, 2.9), kind="elevator"),
            NavNode("walk-2", (0.0, 0.0, 3.0), level_id="L2", space_id="S2"),
        ],
        edges=[
            NavEdge("walk-1", "legacy-elevator-a", 0.1),
            NavEdge("legacy-elevator-a", "legacy-elevator-b", 2.8),
            NavEdge("legacy-elevator-b", "walk-2", 0.1),
        ],
    )

    transition = next(
        item for item in ensure_semantic_transitions(model) if item.kind == "elevator"
    )
    assert transition.resource_id == "vertical:elevator:legacy-elevator-a"
