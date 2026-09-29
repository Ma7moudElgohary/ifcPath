from __future__ import annotations

from ifcpath.model import InavModel, Level, NavCell, Portal, Space
from ifcpath.validation import validate_model


def test_external_surface_does_not_block_internal_egress_readiness() -> None:
    model = InavModel(
        levels=[Level("L1", "Interior", 0.0), Level("ROOF", "Roof", 3.0)],
        spaces=[
            Space("inside", "Inside", "L1"),
            Space("roof", "Roof volume", "ROOF", is_external=True),
        ],
        portals=[
            Portal(
                id="door:exit",
                kind="door",
                position_m=(0.2, 0.2, 0.0),
                from_space_id="inside",
                level_id="L1",
                is_exit=True,
            )
        ],
        cells=[
            NavCell(
                id="inside:0",
                vertices_m=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                space_id="inside",
                level_id="L1",
                terrain="open",
            ),
            NavCell(
                id="roof:0",
                vertices_m=((5.0, 0.0, 3.0), (6.0, 0.0, 3.0), (5.0, 1.0, 3.0)),
                space_id="roof",
                level_id="ROOF",
                terrain="open",
            ),
        ],
    )

    report = validate_model(model)

    assert report.valid
    assert report.stats["surface_space_count"] == 2
    assert report.stats["surface_internal_space_count"] == 1
    assert report.stats["surface_external_space_count"] == 1
    assert report.stats["surface_exit_unreachable_spaces"] == 0
    assert report.stats["navigation_ready"] is True
