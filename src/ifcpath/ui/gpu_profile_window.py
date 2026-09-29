from __future__ import annotations

import sys

from PySide6.QtCore import QThread
from PySide6.QtWidgets import QApplication

from ..ifc_loader import BuildOptions
from .preview_factory import create_preview
from .profile_window import IFCPathProfileWindow
from .worker_3d import Builder3DWorker


class IFCPathGpuProfileWindow(IFCPathProfileWindow):
    """Final Builder shell with the opt-in WebGPU preview installed last."""

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

    def _start_worker(
        self,
        operation: str,
        path: str,
        options: BuildOptions | None,
    ) -> None:
        if self._thread is not None:
            return
        self._set_busy(True, "Generating navigation + BIM preview…" if operation == "build" else "Loading INAV…")
        self._append_log(f"{operation.upper()}: {path}")

        # Only a successfully initialized GPU preview advertises the larger
        # budget. If WebGPU fell back to Qt, this remains the CPU-safe 18k cap.
        preview_budget = int(getattr(self.preview, "preferred_preview_triangle_budget", 18_000))
        thread = QThread(self)
        worker = Builder3DWorker(
            operation=operation,
            path=path,
            options=options,
            preview_max_triangles=preview_budget,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(self._worker_finished_3d)
        worker.failed.connect(self._worker_failed)
        worker.finished.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._thread_finished)
        self._thread = thread
        self._worker = worker
        thread.start()


def run_gpu_profile_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathGpuProfileWindow()
    window.show()
    return app.exec()
