from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space
from ifcpath.ui.main_window_3d import IFCPathBuilder3DWindow
from ifcpath.ui.preview_3d import Projected3DPreview
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
