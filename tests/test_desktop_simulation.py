from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Space
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

    # Changing a live cost triggers the same reroute path used by smoke/fire/crowd.
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
