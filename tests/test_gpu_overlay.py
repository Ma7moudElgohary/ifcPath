from __future__ import annotations

from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space
from ifcpath.ui.gpu_overlay import build_navigation_overlay, build_scenario_overlay
from ifcpath.ui.gpu_scene import build_gpu_scene


def _model() -> InavModel:
    level = Level("L1", "Ground", 0.0)
    space = Space("space:A", "Hall", level.id)
    cell = NavCell(
        id="cell:A",
        vertices_m=((0.0, 0.0, 0.0), (8.0, 0.0, 0.0), (0.0, 6.0, 0.0)),
        space_id=space.id,
        level_id=level.id,
    )
    nodes = [
        NavNode("n0", (1.0, 1.0, 0.0), level_id=level.id, space_id=space.id, cell_id=cell.id),
        NavNode("n1", (3.0, 1.0, 0.0), level_id=level.id, space_id=space.id, cell_id=cell.id),
    ]
    portals = [
        Portal("door:1", "door", (2.0, 0.2, 0.0), from_space_id=space.id, level_id=level.id),
        Portal("exit:1", "door", (0.4, 2.0, 0.0), from_space_id=space.id, level_id=level.id, is_exit=True),
    ]
    return InavModel(
        levels=[level],
        spaces=[space],
        cells=[cell],
        nodes=nodes,
        edges=[NavEdge("n0", "n1", 2.0)],
        portals=portals,
    )


def test_navigation_overlay_contains_graph_portals_exits_and_endpoints() -> None:
    model = _model()
    scene = build_gpu_scene(model, None)

    overlay = build_navigation_overlay(
        model,
        scene,
        level_id="L1",
        show_graph=True,
        show_portals=True,
        show_exits=True,
        start=(1.0, 1.0, 0.0),
        goal=(4.0, 2.0, 0.0),
    )

    # One graph edge plus octahedral markers for door, exit, start and goal.
    assert overlay.line_vertex_count == 2
    assert overlay.triangle_vertex_count == 4 * 8 * 3


def test_scenario_overlay_contains_hazard_blocked_portal_and_people() -> None:
    model = _model()
    scene = build_gpu_scene(model, None)
    poses = {
        "a": ((1.0, 2.0, 0.0), (1.0, 0.0), "moving"),
        "b": ((2.0, 2.0, 0.0), (1.0, 0.0), "waiting"),
        "c": ((3.0, 2.0, 0.0), (1.0, 0.0), "trapped"),
    }

    overlay = build_scenario_overlay(
        model,
        scene,
        level_id="L1",
        blocked_portals={"door:1"},
        blocked_spaces={"space:A"},
        space_cost_multipliers={"space:A": 8.0},
        hazard_kinds={"space:A": "fire"},
        show_person=True,
        agent_position=(1.5, 1.5, 0.0),
        agent_forward=(0.0, 1.0),
        evacuation_poses=poses,
    )

    # Hazard triangle and four full low-poly people are triangles; blocked door is two lines.
    assert overlay.triangle_vertex_count > 4 * 100 * 3
    assert overlay.line_vertex_count == 4


def test_large_crowd_switches_excess_people_to_lightweight_markers() -> None:
    model = _model()
    scene = build_gpu_scene(model, None)
    poses = {
        f"agent:{index:03d}": ((1.0 + (index % 5) * 0.2, 2.0, 0.0), (0.0, 1.0), "moving")
        for index in range(50)
    }

    overlay = build_scenario_overlay(
        model,
        scene,
        level_id="L1",
        blocked_portals=set(),
        blocked_spaces=set(),
        space_cost_multipliers={},
        hazard_kinds={},
        show_person=False,
        agent_position=None,
        agent_forward=(0.0, 1.0),
        evacuation_poses=poses,
        max_full_people=36,
    )

    # 36 full people + 14 octahedron markers. This guards the crowd LOD path.
    assert overlay.triangle_vertex_count > 36 * 100 * 3
