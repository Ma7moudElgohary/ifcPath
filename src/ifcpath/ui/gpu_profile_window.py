from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from .preview_factory import create_preview
from .profile_window import IFCPathProfileWindow


class IFCPathGpuProfileWindow(IFCPathProfileWindow):
    """Final Builder shell with the opt-in WebGPU preview installed last.

    The existing window hierarchy is intentionally left untouched. We let the
    qualified route/scenario/evacuation windows construct normally, then replace
    only their final preview widget with the renderer selected by the factory.
    """

    def _build_workspace(self):
        workspace = super()._build_workspace()
        old_preview = self.preview
        splitter = old_preview.parentWidget()
        index = splitter.indexOf(old_preview)
        preview = create_preview("evacuation")
        old_preview.setParent(None)
        old_preview.deleteLater()
        splitter.insertWidget(index, preview)
        self.preview = preview
        self.preview.pointPicked.connect(self._point_picked)
        return workspace


def run_gpu_profile_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathGpuProfileWindow()
    window.show()
    return app.exec()
