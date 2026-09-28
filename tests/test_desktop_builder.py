from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space
from ifcpath.ui.main_window import IFCPathBuilderWindow
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
    window = IFCPathBuilderWindow()
    model = _preview_model()
    report = validate_model(model)

    window._populate_model(model, report)

    assert window.card_levels.value.text() == "1"
    assert window.card_spaces.value.text() == "1"
    assert window.card_cells.value.text() == "1"
    assert window.level_combo.count() == 2
    assert window.preview.scene().items()

    window.close()
    app.processEvents()
