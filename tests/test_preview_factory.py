from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from ifcpath.ui.evacuation_preview import Evacuation3DPreview
from ifcpath.ui.preview_factory import create_preview


def test_explicit_gpu_request_falls_back_cleanly_when_gpu_extra_is_absent(monkeypatch) -> None:
    app = QApplication.instance() or QApplication([])
    monkeypatch.setenv("IFCPATH_VIEWPORT", "gpu")

    preview = create_preview("evacuation")

    # desktop-smoke installs [desktop,test], intentionally not [gpu].
    assert isinstance(preview, Evacuation3DPreview)
    assert getattr(preview, "gpu_requested", False) is True
    assert getattr(preview, "gpu_fallback_reason", "")

    preview.close()
    app.processEvents()
