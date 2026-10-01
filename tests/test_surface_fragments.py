from __future__ import annotations

from ifcpath.model import InavModel, NavCell, Portal
from ifcpath.surface_fragments import prune_tiny_space_fragments


def _cell(cell_id, vertices, *, space="S", terrain="open", neighbors=None):
    return NavCell(
        id=cell_id,
        vertices_m=vertices,
        space_id=space,
        terrain=terrain,
        neighbor_ids=list(neighbors or ()),
    )


def test_tiny_relative_sampling_island_is_pruned_and_references_are_cleaned():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))  # 4 m2
    tiny = _cell("tiny", ((10, 0, 0), (10.2, 0, 0), (10, 0.2, 0)))  # 0.02 m2
    witness = _cell(
        "witness",
        ((11, 0, 0), (11.2, 0, 0), (11, 0.2, 0)),
        space=None,
        neighbors=["tiny"],
    )
    witness.portals["tiny"] = ((10.5, 0, 0), (10.5, 0.1, 0))
    witness.portal_ids["tiny"] = "P"
    cells = [main, tiny, witness]

    stats = prune_tiny_space_fragments(cells, InavModel())

    assert stats.removed_cells == 1
    assert stats.removed_components == 1
    assert {cell.id for cell in cells} == {"main", "witness"}
    assert witness.neighbor_ids == []
    assert witness.portals == {}
    assert witness.portal_ids == {}


def test_resolution_aware_area_limit_prunes_a_twelve_triangle_grid_island():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))  # 4 m2
    island = []
    for index in range(12):
        previous = f"tiny:{index - 1}" if index else None
        following = f"tiny:{index + 1}" if index < 11 else None
        neighbors = [item for item in (previous, following) if item]
        x = 10.0 + index * 0.25
        # Each triangle is 0.02 m2, so the component totals 0.24 m2: larger
        # than the legacy fixed 0.20 m2 cutoff but still only twelve triangles.
        island.append(
            _cell(
                f"tiny:{index}",
                ((x, 0, 0), (x + 0.2, 0, 0), (x, 0.2, 0)),
                neighbors=neighbors,
            )
        )

    without_resolution = [main, *island]
    fixed_stats = prune_tiny_space_fragments(without_resolution, InavModel())
    assert fixed_stats.removed_cells == 0

    # Rebuild because pruning is intentionally in-place.
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))
    island = []
    for index in range(12):
        previous = f"tiny:{index - 1}" if index else None
        following = f"tiny:{index + 1}" if index < 11 else None
        neighbors = [item for item in (previous, following) if item]
        x = 10.0 + index * 0.25
        island.append(
            _cell(
                f"tiny:{index}",
                ((x, 0, 0), (x + 0.2, 0, 0), (x, 0.2, 0)),
                neighbors=neighbors,
            )
        )
    cells = [main, *island]

    stats = prune_tiny_space_fragments(
        cells,
        InavModel(),
        sampling_cell_size_m=0.20,
    )

    assert stats.removed_cells == 12
    assert stats.removed_components == 1
    assert [cell.id for cell in cells] == ["main"]


def test_substantial_second_surface_is_not_hidden():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))  # 4 m2
    second = _cell("second", ((10, 0, 0), (12, 0, 0), (10, 1, 0)))  # 1 m2
    cells = [main, second]

    stats = prune_tiny_space_fragments(cells, InavModel())

    assert stats.removed_cells == 0
    assert {cell.id for cell in cells} == {"main", "second"}


def test_tiny_portal_threshold_surface_is_protected_before_binding():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))
    tiny = _cell("tiny", ((10, 0, 0), (10.2, 0, 0), (10, 0.2, 0)))
    model = InavModel(
        portals=[
            Portal(
                id="door",
                kind="door",
                position_m=(10.05, 0.05, 0.0),
                from_space_id="S",
                to_space_id="T",
            )
        ]
    )
    cells = [main, tiny]

    stats = prune_tiny_space_fragments(cells, model)

    assert stats.removed_cells == 0
    assert {cell.id for cell in cells} == {"main", "tiny"}


def test_unused_door_near_sliver_is_removed_after_binding_phase():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))
    tiny = _cell("tiny", ((10, 0, 0), (10.2, 0, 0), (10, 0.2, 0)))
    model = InavModel(
        portals=[
            Portal(
                id="door",
                kind="door",
                position_m=(10.05, 0.05, 0.0),
                from_space_id="S",
                to_space_id="T",
            )
        ]
    )
    cells = [main, tiny]

    stats = prune_tiny_space_fragments(
        cells,
        model,
        protect_portal_proximity=False,
        protect_bound_portals=True,
    )

    assert stats.removed_cells == 1
    assert {cell.id for cell in cells} == {"main"}


def test_actual_bound_portal_threshold_is_protected_after_binding_phase():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))
    tiny = _cell(
        "tiny",
        ((10, 0, 0), (10.2, 0, 0), (10, 0.2, 0)),
        neighbors=["other"],
    )
    other = _cell(
        "other",
        ((10.2, 0, 0), (10.4, 0, 0), (10.2, 0.2, 0)),
        space="T",
        neighbors=["tiny"],
    )
    tiny.portal_ids["other"] = "door"
    other.portal_ids["tiny"] = "door"
    model = InavModel(
        portals=[
            Portal(
                id="door",
                kind="door",
                position_m=(10.2, 0.05, 0.0),
                from_space_id="S",
                to_space_id="T",
            )
        ]
    )
    cells = [main, tiny, other]

    stats = prune_tiny_space_fragments(
        cells,
        model,
        protect_portal_proximity=False,
        protect_bound_portals=True,
    )

    assert stats.removed_cells == 0
    assert {cell.id for cell in cells} == {"main", "tiny", "other"}


def test_tiny_landing_attached_to_vertical_surface_is_protected():
    main = _cell("main", ((0, 0, 0), (4, 0, 0), (0, 2, 0)))
    tiny = _cell(
        "tiny",
        ((10, 0, 0), (10.2, 0, 0), (10, 0.2, 0)),
        neighbors=["stair"],
    )
    stair = _cell(
        "stair",
        ((10, 0, 0), (10.2, 0, 0.18), (10, 0.2, 0.18)),
        space=None,
        terrain="stair",
        neighbors=["tiny"],
    )
    cells = [main, tiny, stair]

    stats = prune_tiny_space_fragments(cells, InavModel())

    assert stats.removed_cells == 0
    assert {cell.id for cell in cells} == {"main", "tiny", "stair"}
