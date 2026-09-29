from __future__ import annotations

from ifcpath.egress_domains import classify_egress_domains
from ifcpath.model import InavModel, Level, Portal, SemanticTransition, Space


def test_isolated_roof_service_space_is_not_required_for_occupant_egress() -> None:
    model = InavModel(
        levels=[Level("roof", "Roof", 6.0)],
        spaces=[Space("service", "R301", "roof")],
    )
    stats = classify_egress_domains(model)
    assert not model.spaces[0].egress_required
    assert stats.exempt_service == 1
    assert stats.required == 0


def test_accessible_roof_space_remains_in_occupant_egress_domain() -> None:
    model = InavModel(
        levels=[Level("l1", "Level 1", 0.0), Level("roof", "Roof", 6.0)],
        spaces=[Space("room", "Room", "l1"), Space("terrace", "Terrace", "roof")],
        transitions=[SemanticTransition(
            id="stair",
            kind="stair",
            from_space_id="room",
            to_space_id="terrace",
            from_level_id="l1",
            to_level_id="roof",
        )],
    )
    stats = classify_egress_domains(model)
    assert model.spaces[1].egress_required
    assert stats.exempt_service == 0
    assert stats.required == 2


def test_isolated_normal_room_is_not_silently_exempted() -> None:
    model = InavModel(
        levels=[Level("l1", "Level 1", 0.0)],
        spaces=[Space("room", "Room", "l1")],
    )
    classify_egress_domains(model)
    assert model.spaces[0].egress_required


def test_external_ifc_space_is_egress_exempt_but_preserved() -> None:
    space = Space("outside", "Exterior", "l1", is_external=True)
    model = InavModel(levels=[Level("l1", "Level 1", 0.0)], spaces=[space])
    stats = classify_egress_domains(model)
    assert space.is_external
    assert not space.egress_required
    assert stats.exempt_external == 1
