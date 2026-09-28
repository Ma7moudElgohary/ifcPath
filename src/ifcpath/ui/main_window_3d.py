from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QThread
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

from ..hierarchical_routing import HierarchicalRoute, find_hierarchical_path
from ..ifc_loader import BuildOptions
from ..model import InavModel, Vec3
from ..validation import ValidationReport
from .main_window import IFCPathBuilderWindow
from .preview_3d import Projected3DPreview
from .preview_geometry import PreviewGeometry
from .worker_3d import Builder3DWorker


class IFCPathBuilder3DWindow(IFCPathBuilderWindow):
    """Desktop Builder with BIM context, exact picking and hierarchical route QA."""

    def __init__(self) -> None:
        super().__init__()
        self._preview_geometry: PreviewGeometry | None = None
        self._start_point: Vec3 | None = None
        self._goal_point: Vec3 | None = None
        self._route: HierarchicalRoute | None = None
        self.setWindowTitle("IFCPath Builder")

    # ---------- extend the existing Builder shell ----------

    def _build_sidebar(self):
        scroll = super()._build_sidebar()
        content = scroll.widget()
        layout = content.layout()

        group = QGroupBox("3D route QA")
        group_layout = QVBoxLayout(group)

        self.view_combo = QComboBox()
        self.view_combo.addItem("3D orbit", "3d")
        self.view_combo.addItem("Top view", "top")
        self.view_combo.currentIndexChanged.connect(self._refresh_preview)
        group_layout.addWidget(self.view_combo)

        self.bim_check = QCheckBox("BIM context")
        self.bim_check.setChecked(True)
        self.bim_check.toggled.connect(self._refresh_preview)
        group_layout.addWidget(self.bim_check)

        pick_row = QHBoxLayout()
        self.pick_start_button = QPushButton("Pick Start")
        self.pick_goal_button = QPushButton("Pick Goal")
        self.pick_start_button.setEnabled(False)
        self.pick_goal_button.setEnabled(False)
        self.pick_start_button.clicked.connect(lambda: self._begin_pick("start"))
        self.pick_goal_button.clicked.connect(lambda: self._begin_pick("goal"))
        pick_row.addWidget(self.pick_start_button)
        pick_row.addWidget(self.pick_goal_button)
        group_layout.addLayout(pick_row)

        self.start_label = QLabel("Start: —")
        self.goal_label = QLabel("Goal: —")
        self.start_label.setWordWrap(True)
        self.goal_label.setWordWrap(True)
        group_layout.addWidget(self.start_label)
        group_layout.addWidget(self.goal_label)

        route_row = QHBoxLayout()
        self.route_button = QPushButton("Calculate Route")
        self.clear_route_button = QPushButton("Clear")
        self.route_button.setEnabled(False)
        self.clear_route_button.setEnabled(False)
        self.route_button.clicked.connect(self._calculate_route)
        self.clear_route_button.clicked.connect(self._clear_route)
        route_row.addWidget(self.route_button)
        route_row.addWidget(self.clear_route_button)
        group_layout.addLayout(route_row)

        self.route_label = QLabel("Route: —")
        self.route_label.setWordWrap(True)
        self.route_label.setObjectName("routeSummary")
        group_layout.addWidget(self.route_label)

        insert_at = layout.indexOf(self.build_button)
        if insert_at < 0:
            insert_at = max(0, layout.count() - 1)
        layout.insertWidget(insert_at, group)
        return scroll

    def _build_workspace(self):
        workspace = super()._build_workspace()
        old_preview = self.preview
        splitter = old_preview.parentWidget()
        index = splitter.indexOf(old_preview)
        preview = Projected3DPreview()
        old_preview.setParent(None)
        old_preview.deleteLater()
        splitter.insertWidget(index, preview)
        self.preview = preview
        self.preview.pointPicked.connect(self._point_picked)
        return workspace

    # ---------- worker with BIM preview extraction ----------

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

        thread = QThread(self)
        worker = Builder3DWorker(operation=operation, path=path, options=options)
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

    def _worker_finished_3d(
        self,
        model: InavModel,
        report: ValidationReport,
        preview_geometry: PreviewGeometry | None,
        path: str,
    ) -> None:
        self._preview_geometry = preview_geometry
        super()._worker_finished(model, report, path)
        self.preview.set_bim_geometry(preview_geometry)
        self._clear_route()
        self.pick_start_button.setEnabled(bool(model.cells))
        self.pick_goal_button.setEnabled(bool(model.cells))
        if self.level_combo.count() > 0:
            self.level_combo.setCurrentIndex(0)
        if preview_geometry is not None:
            suffix = " (capped)" if preview_geometry.truncated else ""
            self._append_log(f"BIM preview: {len(preview_geometry.triangles)} triangles{suffix}")
        self._refresh_preview()

    def _populate_model(self, model: InavModel, report: ValidationReport) -> None:
        super()._populate_model(model, report)
        self.pick_start_button.setEnabled(bool(model.cells))
        self.pick_goal_button.setEnabled(bool(model.cells))
        if self.level_combo.count() > 0:
            self.level_combo.setCurrentIndex(0)
        self._refresh_preview()

    def open_ifc(self) -> None:
        super().open_ifc()
        if self._model is None:
            self._preview_geometry = None
            self.preview.set_bim_geometry(None)
            self._clear_route()

    def _set_busy(self, busy: bool, message: str = "") -> None:
        super()._set_busy(busy, message)
        enabled = not busy and self._model is not None and bool(self._model.cells)
        if hasattr(self, "pick_start_button"):
            self.pick_start_button.setEnabled(enabled)
            self.pick_goal_button.setEnabled(enabled)
            self.route_button.setEnabled(enabled and self._start_point is not None and self._goal_point is not None)
            self.clear_route_button.setEnabled(enabled and (self._start_point is not None or self._goal_point is not None))

    # ---------- 3D preview / exact route QA ----------

    def _refresh_preview(self) -> None:
        if self._model is None:
            return
        super()._refresh_preview()
        if isinstance(self.preview, Projected3DPreview):
            self.preview.set_bim_visible(self.bim_check.isChecked())
            self.preview.set_view_mode(self.view_combo.currentData() or "3d")

    def _begin_pick(self, mode: str) -> None:
        if self._model is None or not self._model.cells:
            return
        self.preview.set_pick_mode(mode)
        self.statusBar().showMessage(
            "Click a walkable navmesh cell for the route start" if mode == "start" else "Click a walkable navmesh cell for the route goal"
        )

    def _point_picked(self, mode: str, point, space_id, level_id) -> None:
        world: Vec3 = (float(point[0]), float(point[1]), float(point[2]))
        label = self._point_label(world, space_id, level_id)
        if mode == "start":
            self._start_point = world
            self.start_label.setText(f"Start: {label}")
        else:
            self._goal_point = world
            self.goal_label.setText(f"Goal: {label}")

        self._route = None
        self.preview.set_route(self._start_point, self._goal_point, [])
        both = self._start_point is not None and self._goal_point is not None
        self.route_button.setEnabled(both)
        self.clear_route_button.setEnabled(True)
        self.route_label.setText("Route: ready to calculate" if both else "Route: pick the other endpoint")
        if both:
            self._calculate_route()

    def _calculate_route(self) -> None:
        if self._model is None or self._start_point is None or self._goal_point is None:
            return
        route = find_hierarchical_path(self._model, self._start_point, self._goal_point)
        self._route = route
        if route is None:
            self.route_label.setText("Route: no valid path")
            self.preview.set_route(self._start_point, self._goal_point, [])
            self._append_log("ROUTE: no valid hierarchical path")
            return

        if self.level_combo.count() > 0:
            self.level_combo.setCurrentIndex(0)
        self.preview.set_route(self._start_point, self._goal_point, route.points)
        transitions = len(route.transition_ids)
        self.route_label.setText(
            f"Route: {route.length_m:.2f} m · {len(route.space_ids)} space(s) · "
            f"{transitions} transfer(s) · cost {route.weighted_cost:.2f}"
        )
        self._append_log(
            f"ROUTE: length={route.length_m:.3f}m cost={route.weighted_cost:.3f} "
            f"spaces={' -> '.join(route.space_ids)} transitions={' -> '.join(route.transition_ids) or 'none'}"
        )
        self.statusBar().showMessage("Hierarchical route calculated", 6000)

    def _clear_route(self) -> None:
        self._start_point = None
        self._goal_point = None
        self._route = None
        self.start_label.setText("Start: —")
        self.goal_label.setText("Goal: —")
        self.route_label.setText("Route: —")
        self.route_button.setEnabled(False)
        self.clear_route_button.setEnabled(False)
        if isinstance(self.preview, Projected3DPreview):
            self.preview.set_pick_mode(None)
            self.preview.set_route(None, None, [])

    def _point_label(self, point: Vec3, space_id, level_id) -> str:
        space_name = str(space_id or "unknown space")
        level_name = str(level_id or "unknown level")
        if self._model is not None:
            space = next((item for item in self._model.spaces if item.id == space_id), None)
            level = next((item for item in self._model.levels if item.id == level_id), None)
            if space is not None:
                space_name = space.name
            if level is not None:
                level_name = level.name
        return f"{space_name} / {level_name}  ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})"

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        if self._thread is not None and self._thread.isRunning():
            self.statusBar().showMessage("IFC generation is still running; close after it completes", 6000)
            event.ignore()
            return
        super().closeEvent(event)


def run_app_3d() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathBuilder3DWindow()
    window.show()
    return app.exec()
