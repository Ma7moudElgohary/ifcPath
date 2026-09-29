from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.ui.profile_window import IFCPathProfileWindow


def test_builder_exposes_route_profiles() -> None:
    app = QApplication.instance() or QApplication([])
    window = IFCPathProfileWindow()

    ids = [window.route_profile_combo.itemData(i) for i in range(window.route_profile_combo.count())]
    assert ids == [
        "standard",
        "accessible",
        "emergency_responder",
        "security",
        "maintenance",
    ]

    index = window.route_profile_combo.findData("accessible")
    window.route_profile_combo.setCurrentIndex(index)
    app.processEvents()

    assert window._route_profile_id() == "accessible"
    assert "stairs" in window.route_profile_description_label.text().lower()

    window.close()
    app.processEvents()
