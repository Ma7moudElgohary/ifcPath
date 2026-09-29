from __future__ import annotations

from PySide6.QtWidgets import QApplication

from ifcpath.ui.gpu_profile_window import IFCPathGpuProfileWindow
from ifcpath.ui.gpu_selection import ElementSelectionHit
from ifcpath.ui.preview_geometry import PreviewElement


def _app() -> QApplication:
    return QApplication.instance() or QApplication([])


def test_gpu_profile_window_populates_ifc_element_inspector() -> None:
    _app()
    window = IFCPathGpuProfileWindow()
    element = PreviewElement(
        guid="3ABC",
        express_id=123,
        ifc_class="IfcWall",
        category="wall",
        level_id="level:ground",
        name="Wall A",
        predefined_type="STANDARD",
        attributes={"OverallHeight": 3.2},
        property_sets={
            "Pset_WallCommon": {
                "FireRating": "2HR",
                "IsExternal": False,
            }
        },
    )
    hit = ElementSelectionHit(
        guid=element.guid,
        point_m=(1.0, 2.0, 3.0),
        distance_m=5.0,
        category="wall",
        level_id="level:ground",
        element=element,
    )

    window._show_element_selection(hit)

    assert "Wall A" in window.element_hint_label.text()
    assert window.element_property_tree.topLevelItemCount() >= 3
    texts = []
    for index in range(window.element_property_tree.topLevelItemCount()):
        top = window.element_property_tree.topLevelItem(index)
        texts.append(top.text(0))
        for child_index in range(top.childCount()):
            child = top.child(child_index)
            texts.append(child.text(0))
            texts.append(child.text(1))
    assert "Identity" in texts
    assert "Pset_WallCommon" in texts
    assert "3ABC" in texts
    assert "2HR" in texts
    window.close()
