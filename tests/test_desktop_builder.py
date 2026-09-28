from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space
from ifcpath.ui.main_window_3d import IFCPathBuilder3DWindow
from ifcpath.ui.person_mesh import build_person_mesh
from ifcpath.ui.preview_3d import Projected3DPreview
from ifcpath.ui.scenario_preview import Scenario3DPreview
from ifcpath.ui.scenario_window import IFCPathScenarioWindow
from ifcpath.validation import validate_model


def _preview_model() -> InavModel:
    level = Level("level:L1", "Ground", 0.0)
    space = Space("space:A", "Lobby", level.id)
    cell = NavCell(
        id="cell:A:0",
        vertices_m=((0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (0.0, 5.0, 0.0)),
        space_id=space.id,
        level_id=level.id,
    )
    nodes = [
        NavNode("n0", (1.0, 1.0, 0.0), level_id=level.id, space_id=space.id, cell_id=cell.id),
        NavNode("n1", (2.0, 1.0, 0.0), level_id=level.id, space_id=space.id, cell_id=cell.id),
    ]
    portal = Portal(
        "door:exit",
        "door",
        (0.2, 0.2, 0.0),
        from_space_id=space.id,
        level_id=level.id,
        is_exit=True,
    )
    return InavModel(
        levels=[level],
        spaces=[space],
        portals=[portal],
        cells=[cell],
        nodes=nodes,
        edges=[NavEdge("n0", "n1", 1.0)],
    )


def test_desktop_window_binds_navigation_model() -> None:
    app = QApplication.instance() or QApplication([])
    window = IFCPathBuilder3DWindow()
    model = _preview_model()
    report = validate_model(model)

    window._populate_model(model, report)
    window._model = model

    assert window.card_levels.value.text() == "1"
    assert window.card_spaces.value.text() == "1"
    assert window.card_cells.value.text() == "1"
    assert window.level_combo.count() == 2
    assert window.preview.scene().items()
    assert window.pick_start_button.isEnabled()

    window.close()
    app.processEvents()


def test_projected_3d_preview_recovers_world_point_from_cdt_cell() -> None:
    app = QApplication.instance() or QApplication([])
    preview = Projected3DPreview()
    model = _preview_model()
    preview.set_model(model)
    preview.set_level(model.levels[0].id)

    expected = (1.0, 1.0, 0.0)
    scene_point, _ = preview.project_world(expected)
    picked = preview.pick_scene_point(scene_point)

    assert picked is not None
    point, space_id, level_id = picked
    assert space_id == "space:A"
    assert level_id == "level:L1"
    assert point[0] == pytest.approx(expected[0], abs=1e-6)
    assert point[1] == pytest.approx(expected[1], abs=1e-6)
    assert point[2] == pytest.approx(expected[2], abs=1e-6)

    preview.close()
    app.processEvents()


def test_builder_calculates_route_between_picked_world_points() -> None:
    app = QApplication.instance() or QApplication([])
    window = IFCPathBuilder3DWindow()
    model = _preview_model()
    report = validate_model(model)
    window._model = model
    window._populate_model(model, report)

    window._start_point = (0.5, 0.5, 0.0)
    window._goal_point = (1.5, 0.5, 0.0)
    window._calculate_route()

    assert window._route is not None
    assert window._route.length_m == pytest.approx(1.0, abs=1e-6)
    assert window.preview.route_points[0] == window._start_point
    assert window.preview.route_points[-1] == window._goal_point
    assert "1.00 m" in window.route_label.text()

    window.close()
    app.processEvents()


def test_person_agent_is_procedural_triangular_3d_mesh() -> None:
    mesh = build_person_mesh((4.0, 5.0, 2.0), (1.0, 0.0), height_m=1.72)

    assert len(mesh) >= 100
    assert all(len(triangle) == 3 for triangle in mesh)
    vertices = [point for triangle in mesh for point in triangle]
    z_values = [point[2] for point in vertices]
    assert min(z_values) == pytest.approx(2.0, abs=1e-6)
    assert 1.70 <= max(z_values) - min(z_values) <= 1.76
    # The requested facing rotation should create geometry on both sides of base X/Y.
    assert min(point[0] for point in vertices) < 4.0 < max(point[0] for point in vertices)
    assert min(point[1] for point in vertices) < 5.0 < max(point[1] for point in vertices)


def test_scenario_preview_draws_person_at_route_start() -> None:
    app = QApplication.instance() or QApplication([])
    preview = Scenario3DPreview()
    model = _preview_model()
    preview.set_model(model)
    preview.set_route((0.5, 0.5, 0.0), (1.5, 0.5, 0.0), [(0.5, 0.5, 0.0), (1.5, 0.5, 0.0)])

    assert preview.person_triangle_count >= 100
    assert preview.route_points[0] == (0.5, 0.5, 0.0)

    preview.set_person_visible(False)
    assert preview.person_triangle_count == 0
    preview.close()
    app.processEvents()


def test_scenario_editor_keeps_runtime_state_out_of_inav() -> None:
    app = QApplication.instance() or QApplication([])
    window = IFCPathScenarioWindow()
    model = _preview_model()
    report = validate_model(model)
    original_portal_count = len(model.portals)
    original_space_count = len(model.spaces)
    window._model = model
    window._populate_model(model, report)

    assert window.scenario_portal_combo.currentData() == "door:exit"
    assert window.scenario_space_combo.currentData() == "space:A"

    window._toggle_portal_block()
    assert window._blocked_portals == {"door:exit"}
    assert window.preview.blocked_portals == {"door:exit"}

    window.hazard_kind_combo.setCurrentIndex(window.hazard_kind_combo.findData("smoke"))
    window.hazard_multiplier_spin.setValue(7.0)
    window._apply_space_hazard()
    assert window._space_cost_multipliers == {"space:A": 7.0}

    window._toggle_space_block()
    assert window._blocked_spaces == {"space:A"}
    assert window.preview.blocked_spaces == {"space:A"}

    # Scenario edits are runtime-only; the portable navigation model is untouched.
    assert len(model.portals) == original_portal_count
    assert len(model.spaces) == original_space_count
    assert model.portals[0].id == "door:exit"

    window._reset_scenario()
    assert not window._blocked_portals
    assert not window._blocked_spaces
    assert not window._space_cost_multipliers

    window.close()
    app.processEvents()
