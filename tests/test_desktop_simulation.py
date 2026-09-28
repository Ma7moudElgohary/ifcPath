from __future__ import annotations

import os

import pytest
from shapely.geometry import Polygon

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space
from ifcpath.ui.scenario_window import IFCPathScenarioWindow
from ifcpath.validation import validate_model


def _single_space_model() -> InavModel:
    level = Level("level:L1", "Ground", 0.0)
    space = Space("space:A", "Lobby", level.id)
    cells = [
        NavCell(
            id="cell:A:0",
            vertices_m=((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 2.0, 0.0)),
            space_id=space.id,
            level_id=level.id,
            neighbor_ids=["cell:A:1"],
        ),
        NavCell(
            id="cell:A:1",
            vertices_m=((2.0, 0.0, 0.0), (2.0, 2.0, 0.0), (0.0, 2.0, 0.0)),
            space_id=space.id,
            level_id=level.id,
            neighbor_ids=["cell:A:0"],
        ),
    ]
    nodes = [
        NavNode("n0", (0.5, 0.5, 0.0), level_id=level.id, space_id=space.id, cell_id=cells[0].id),
        NavNode("n1", (1.5, 0.5, 0.0), level_id=level.id, space_id=space.id, cell_id=cells[1].id),
    ]
    return InavModel(
        levels=[level],
        spaces=[space],
        cells=cells,
        nodes=nodes,
        edges=[NavEdge("n0", "n1", 1.0)],
    )


def _add_rect_space(model: InavModel, space_id: str, x0: float, x1: float) -> None:
    level_id = "level:L1"
    model.spaces.append(Space(space_id, space_id, level_id))
    cdt = build_floor_cdt_navmesh(Polygon([(x0, 0.0), (x1, 0.0), (x1, 1.0), (x0, 1.0)]), 0.0)
    cell_ids = [f"cell:{space_id}:{index}" for index in range(len(cdt.cells))]
    model.cells.extend(
        NavCell(
            id=cell_ids[index],
            vertices_m=cell.vertices,
            space_id=space_id,
            level_id=level_id,
            neighbor_ids=[cell_ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    )


def _two_space_model() -> InavModel:
    model = InavModel(levels=[Level("level:L1", "Ground", 0.0)])
    _add_rect_space(model, "A", 0.0, 1.0)
    _add_rect_space(model, "B", 1.0, 2.0)
    model.portals = [Portal("door:AB", "door", (1.0, 0.5, 0.0), "A", "B", "level:L1")]
    return model


def _window_with_route() -> tuple[QApplication, IFCPathScenarioWindow]:
    app = QApplication.instance() or QApplication([])
    window = IFCPathScenarioWindow()
    model = _single_space_model()
    window._model = model
    window._populate_model(model, validate_model(model))
    window._start_point = (0.25, 0.5, 0.0)
    window._goal_point = (1.75, 0.5, 0.0)
    window._calculate_route()
    return app, window


def test_live_agent_advances_along_exact_route() -> None:
    app, window = _window_with_route()
    window.walk_speed_spin.setValue(1.0)

    assert window._walker.position == pytest.approx(window._start_point)
    window._walker.play()
    window._advance_simulation(0.5)

    assert window._walker.distance_m == pytest.approx(0.5)
    assert window._walker.position is not None
    assert window._walker.position[0] == pytest.approx(0.75, abs=1e-6)
    assert window.preview.agent_position == pytest.approx(window._walker.position)
    assert "walking" in window.walk_status_label.text()

    window._pause_simulation()
    window.close()
    app.processEvents()


def test_scenario_change_reroutes_from_current_agent_position_and_keeps_walking() -> None:
    app, window = _window_with_route()
    window.walk_speed_spin.setValue(1.0)
    window._walker.play()
    window._advance_simulation(0.4)
    current = window._walker.position
    assert current is not None

    window.hazard_kind_combo.setCurrentIndex(window.hazard_kind_combo.findData("smoke"))
    window.hazard_multiplier_spin.setValue(6.0)
    window._apply_space_hazard()

    assert window._route is not None
    assert window._route.points[0] == pytest.approx(current)
    assert window._walker.position == pytest.approx(current)
    assert window._walker.distance_m == pytest.approx(0.0)
    assert window._walker.running
    assert window._route.weighted_cost > window._route.length_m

    window._pause_simulation()
    window.close()
    app.processEvents()


def test_repeated_scenario_edits_keep_current_reroute_origin() -> None:
    app, window = _window_with_route()
    window.walk_speed_spin.setValue(1.0)
    window._walker.play()
    window._advance_simulation(0.4)
    current = window._walker.position
    assert current is not None

    window.hazard_multiplier_spin.setValue(4.0)
    window._apply_space_hazard()
    assert window._walker.position == pytest.approx(current)
    assert window._walker.distance_m == pytest.approx(0.0)

    window.hazard_multiplier_spin.setValue(8.0)
    window._apply_space_hazard()

    assert window._route is not None
    assert window._route.points[0] == pytest.approx(current)
    assert window._walker.position == pytest.approx(current)
    assert window._walker.running

    window._pause_simulation()
    window.close()
    app.processEvents()


def test_blocking_only_door_stops_agent_and_invalidates_old_route() -> None:
    app = QApplication.instance() or QApplication([])
    window = IFCPathScenarioWindow()
    model = _two_space_model()
    window._model = model
    window._populate_model(model, validate_model(model))
    window._start_point = (0.25, 0.5, 0.0)
    window._goal_point = (1.75, 0.5, 0.0)
    window._calculate_route()
    assert window._route is not None

    window.walk_speed_spin.setValue(1.0)
    window._walker.play()
    window._advance_simulation(0.2)
    current = window._walker.position
    assert current is not None

    assert window.scenario_portal_combo.currentData() == "door:AB"
    window._toggle_portal_block()

    assert window._route is None
    assert not window._walker.points
    assert not window._walker.running
    assert window.preview.agent_position == pytest.approx(current)
    assert not window.walk_play_button.isEnabled()
    assert "no valid route" in window.walk_status_label.text()

    window.close()
    app.processEvents()


def test_restart_returns_agent_to_original_picked_start() -> None:
    app, window = _window_with_route()
    window.walk_speed_spin.setValue(1.0)
    window._walker.play()
    window._advance_simulation(0.6)
    assert window._walker.position != pytest.approx(window._start_point)

    window._restart_simulation()

    assert window._walker.position == pytest.approx(window._start_point)
    assert window._walker.distance_m == pytest.approx(0.0)
    assert not window._walker.running
    assert window.preview.agent_position == pytest.approx(window._start_point)

    window.close()
    app.processEvents()
