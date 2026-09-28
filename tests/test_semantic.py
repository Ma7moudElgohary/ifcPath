from __future__ import annotations

import json

from ifcpath.exporter import load_inav, save_inav
from ifcpath.model import InavModel, Portal, Space
from ifcpath.semantic import SemanticRouteOptions, ensure_semantic_transitions, find_space_path


def _semantic_model() -> InavModel:
    model = InavModel()
    model.spaces = [
        Space("space:A", "A", "level:L1"),
        Space("space:B", "B", "level:L1"),
        Space("space:C", "C", "level:L1"),
        Space("space:D", "D", "level:L1"),
    ]
    model.portals = [
        Portal("door:AB", "door", (1.0, 0.0, 0.0), "space:A", "space:B", "level:L1"),
        Portal("door:BC", "door", (2.0, 0.0, 0.0), "space:B", "space:C", "level:L1"),
        Portal("door:AD", "door", (1.0, 1.0, 0.0), "space:A", "space:D", "level:L1"),
        Portal("door:DC", "door", (2.0, 1.0, 0.0), "space:D", "space:C", "level:L1"),
        Portal("door:EXIT", "door", (3.0, 0.0, 0.0), "space:C", None, "level:L1", is_exit=True),
    ]
    return model


def test_derives_space_to_space_and_exit_transitions():
    model = _semantic_model()
    transitions = ensure_semantic_transitions(model)

    assert len(transitions) == 5
    ab = next(x for x in transitions if x.portal_id == "door:AB")
    assert ab.from_space_id == "space:A"
    assert ab.to_space_id == "space:B"
    assert ab.from_level_id == "level:L1"
    assert ab.to_level_id == "level:L1"

    exit_transition = next(x for x in transitions if x.portal_id == "door:EXIT")
    assert exit_transition.from_space_id == "space:C"
    assert exit_transition.to_space_id is None


def test_semantic_route_reacts_to_blocked_portals_and_space_costs():
    model = _semantic_model()
    assert find_space_path(model, "space:A", "space:C") == ["space:A", "space:B", "space:C"]

    blocked = find_space_path(
        model,
        "space:A",
        "space:C",
        SemanticRouteOptions(blocked_portals={"door:BC"}),
    )
    assert blocked == ["space:A", "space:D", "space:C"]

    penalized = find_space_path(
        model,
        "space:A",
        "space:C",
        SemanticRouteOptions(space_cost_multipliers={"space:B": 10.0}),
    )
    assert penalized == ["space:A", "space:D", "space:C"]


def test_blocked_start_space_can_escape_but_cannot_be_entered():
    model = _semantic_model()
    options = SemanticRouteOptions(blocked_spaces={"space:A"})
    assert find_space_path(model, "space:A", "space:C", options)
    assert find_space_path(model, "space:C", "space:A", options) == []


def test_old_inav_is_upgraded_with_semantic_transitions(tmp_path):
    legacy = {
        "schema": "ifcpath.inav/0.1",
        "units": "m",
        "up_axis": "Z",
        "levels": [],
        "spaces": [
            {"id": "space:A", "name": "A", "level_id": None, "centroid_m": None, "ifc_guid": None},
            {"id": "space:B", "name": "B", "level_id": None, "centroid_m": None, "ifc_guid": None},
        ],
        "portals": [
            {
                "id": "door:AB",
                "kind": "door",
                "position_m": [0.0, 0.0, 0.0],
                "from_space_id": "space:A",
                "to_space_id": "space:B",
                "level_id": None,
                "width_m": None,
                "ifc_guid": None,
                "is_exit": False,
            }
        ],
        "nodes": [],
        "edges": [],
        "metadata": {},
    }
    path = tmp_path / "legacy.inav"
    path.write_text(json.dumps(legacy), encoding="utf-8")

    loaded = load_inav(path)
    assert len(loaded.transitions) == 1
    assert loaded.transitions[0].from_space_id == "space:A"
    assert loaded.transitions[0].to_space_id == "space:B"

    roundtrip = tmp_path / "roundtrip.inav"
    save_inav(loaded, roundtrip)
    raw = json.loads(roundtrip.read_text(encoding="utf-8"))
    assert raw["transitions"][0]["portal_id"] == "door:AB"
