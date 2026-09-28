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
            NavNode("elevator-1", (2.0, 2.0, 0.1), kind="elevator"),
            NavNode("elevator-2", (2.0, 2.0, 3.0), kind="elevator"),
            NavNode("elevator-3", (2.0, 2.0, 5.9), kind="elevator"),
            NavNode("landing-2", (2.0, 2.0, 3.0), kind="walk", level_id="L2", space_id="S2"),
            NavNode("landing-3", (2.0, 2.0, 6.0), kind="walk", level_id="L3", space_id="S3"),
        ],
        edges=[
            NavEdge("landing-1", "elevator-1", 0.1),
            NavEdge("elevator-1", "elevator-2", 2.9),
            NavEdge("elevator-2", "elevator-3", 2.9),
            NavEdge("elevator-2", "landing-2", 0.1),
            NavEdge("elevator-3", "landing-3", 0.1),
        ],
    )
    return model


def test_adjacent_elevator_transitions_share_one_physical_resource_id() -> None:
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
    resource_ids = {transition.resource_id for transition in transitions}
    assert len(resource_ids) == 1
    resource_id = next(iter(resource_ids))
    assert resource_id is not None
    assert resource_id.startswith("vertical:elevator:")


def test_vertical_resource_id_survives_inav_roundtrip(tmp_path) -> None:
    model = _three_floor_elevator_component()
    ensure_semantic_transitions(model)
    path = save_inav(model, tmp_path / "elevator.inav")

    loaded = load_inav(path)
    transitions = [item for item in loaded.transitions if item.kind == "elevator"]

    assert len(transitions) == 2
    assert transitions[0].resource_id is not None
    assert {item.resource_id for item in transitions} == {transitions[0].resource_id}
