from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.model import InavModel, Level, NavCell, Portal, Space
from ifcpath.ui.evacuation_preview import Evacuation3DPreview
from ifcpath.ui.evacuation_window import IFCPathEvacuationWindow
from ifcpath.validation import validate_model


def _evacuation_model() -> InavModel:
    model = InavModel(levels=[Level("L1", "Ground", 0.0)])
    space = Space("space:A", "Hall", "L1")
    model.spaces.append(space)
    cdt = build_floor_cdt_navmesh(Polygon([(0, 0), (10, 0), (10, 4), (0, 4)]), 0.0)
    ids = [f"cell:{i}" for i in range(len(cdt.cells))]
    model.cells.extend(
        NavCell(
            id=ids[i],
            vertices_m=cell.vertices,
            space_id=space.id,
            level_id="L1",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for i, cell in enumerate(cdt.cells)
    )
    model.portals = [
        Portal(
            "exit:left",
            "door",
            (0.5, 2.0, 0.0),
            from_space_id=space.id,
            level_id="L1",
            width_m=1.0,
            is_exit=True,
        ),
        Portal(
            "exit:right",
            "door",
            (9.5, 2.0, 0.0),
            from_space_id=space.id,
            level_id="L1",
            width_m=1.0,
            is_exit=True,
        ),
    ]
    return model


def _window() -> tuple[QApplication, IFCPathEvacuationWindow]:
    app = QApplication.instance() or QApplication([])
    window = IFCPathEvacuationWindow()
    model = _evacuation_model()
    window._model = model
    window._populate_model(model, validate_model(model))
    return app, window


def test_desktop_prepares_population_and_renders_agents() -> None:
    app, window = _window()
    window.evacuation_count_spin.setValue(6)
    window.evacuation_seed_spin.setValue(12)
    window._prepare_evacuation()

    simulator = window._evacuation_simulator
    assert simulator is not None
    assert len(simulator.agents) == 6
    assert isinstance(window.preview, Evacuation3DPreview)
    assert window.preview.evacuation_agent_count == 6
    assert "evacuated 0/6" in window.evacuation_summary_label.text()
    assert "Assigned exits:" in window.evacuation_exit_label.text()
    assert not window.person_check.isChecked()

    window.close()
    app.processEvents()


def test_desktop_evacuation_advances_and_reports_completion_metrics() -> None:
    app, window = _window()
    window.evacuation_count_spin.setValue(4)
    window.evacuation_min_speed_spin.setValue(2.0)
    window.evacuation_max_speed_spin.setValue(2.0)
    window.evacuation_flow_spin.setValue(5.0)
    window._prepare_evacuation()

    for _ in range(200):
        if window._evacuation_simulator is None or window._evacuation_simulator.finished:
            break
        window._advance_evacuation(0.1)

    simulator = window._evacuation_simulator
    assert simulator is not None
    assert simulator.finished
    assert simulator.stats.evacuated_agents == 4
    assert simulator.stats.trapped_agents == 0
    assert simulator.stats.clearance_time_s is not None
    assert "evacuated 4/4" in window.evacuation_summary_label.text()
    assert "Exits:" in window.evacuation_exit_label.text()

    window.close()
    app.processEvents()


def test_live_scenario_replans_multi_agent_population() -> None:
    app, window = _window()
    window.evacuation_count_spin.setValue(4)
    window._prepare_evacuation()
    simulator = window._evacuation_simulator
    assert simulator is not None

    before_positions = {state.spec.id: state.position for state in simulator.agents}
    # Select and block the left exit through the same live scenario control used by users.
    index = window.scenario_portal_combo.findData("exit:left")
    assert index >= 0
    window.scenario_portal_combo.setCurrentIndex(index)
    window._toggle_portal_block()

    simulator = window._evacuation_simulator
    assert simulator is not None
    assert all(
        state.plan is None or state.plan.exit_portal_id != "exit:left"
        for state in simulator.agents
        if state.status != "evacuated"
    )
    assert {state.spec.id: state.position for state in simulator.agents} == before_positions

    window.close()
    app.processEvents()
