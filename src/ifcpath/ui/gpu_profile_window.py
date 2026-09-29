from __future__ import annotations

import sys
from typing import Any

from PySide6.QtCore import QThread, Qt
from PySide6.QtWidgets import (
    QApplication,
    QDockWidget,
    QLabel,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..ifc_loader import BuildOptions
from .gpu_selection import ElementSelectionHit
from .preview_factory import create_preview
from .profile_window import IFCPathProfileWindow
from .worker_3d import Builder3DWorker


class IFCPathGpuProfileWindow(IFCPathProfileWindow):
    """Final Builder shell with the opt-in WebGPU preview installed last."""

    def __init__(self) -> None:
        super().__init__()
        self._build_element_inspector()
        if hasattr(self.preview, "elementSelected"):
            self.preview.elementSelected.connect(self._show_element_selection)
            self.preview.selectionCleared.connect(self._clear_element_selection)
            self.element_inspector_dock.setEnabled(True)
            self.element_hint_label.setText("Click a BIM element to inspect its IFC data.")
        else:
            self.element_inspector_dock.setEnabled(False)
            self.element_hint_label.setText("IFC element picking is available in WebGPU mode.")

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

    def _build_element_inspector(self) -> None:
        dock = QDockWidget("IFC Element", self)
        dock.setObjectName("ifcElementInspector")
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        dock.setMinimumWidth(290)

        content = QWidget(dock)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        self.element_hint_label = QLabel("No BIM element selected")
        self.element_hint_label.setWordWrap(True)
        layout.addWidget(self.element_hint_label)

        tree = QTreeWidget()
        tree.setColumnCount(2)
        tree.setHeaderLabels(["Property", "Value"])
        tree.setAlternatingRowColors(True)
        tree.setUniformRowHeights(True)
        tree.setRootIsDecorated(True)
        tree.setColumnWidth(0, 150)
        layout.addWidget(tree, 1)
        self.element_property_tree = tree

        dock.setWidget(content)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)
        self.element_inspector_dock = dock

    def _show_element_selection(self, hit: ElementSelectionHit) -> None:
        element = hit.element
        self.element_property_tree.clear()
        if element is None:
            self.element_hint_label.setText(f"Selected {hit.guid}")
            self._add_property(self.element_property_tree.invisibleRootItem(), "GlobalId", hit.guid)
            self._add_property(self.element_property_tree.invisibleRootItem(), "Category", hit.category)
            return

        title = element.name or element.ifc_class
        self.element_hint_label.setText(f"{title}\n{element.ifc_class} · {element.guid}")
        root = self.element_property_tree.invisibleRootItem()

        identity = QTreeWidgetItem(["Identity", ""])
        root.addChild(identity)
        self._add_property(identity, "GlobalId", element.guid)
        self._add_property(identity, "Express ID", element.express_id)
        self._add_property(identity, "IFC Class", element.ifc_class)
        self._add_property(identity, "Category", element.category)
        self._add_property(identity, "Level", element.level_id or "—")
        self._add_property(identity, "Name", element.name or "—")
        self._add_property(identity, "Description", element.description or "—")
        self._add_property(identity, "Object Type", element.object_type or "—")
        self._add_property(identity, "Predefined Type", element.predefined_type or "—")
        self._add_property(identity, "Tag", element.tag or "—")
        self._add_property(identity, "Hit XYZ", f"{hit.point_m[0]:.3f}, {hit.point_m[1]:.3f}, {hit.point_m[2]:.3f} m")
        identity.setExpanded(True)

        if element.attributes:
            attributes = QTreeWidgetItem(["IFC Attributes", ""])
            root.addChild(attributes)
            for name in sorted(element.attributes):
                self._add_value(attributes, name, element.attributes[name])

        for pset_name in sorted(element.property_sets):
            pset = QTreeWidgetItem([pset_name, ""])
            root.addChild(pset)
            for name in sorted(element.property_sets[pset_name]):
                self._add_value(pset, name, element.property_sets[pset_name][name])

        self.statusBar().showMessage(
            f"Selected {element.ifc_class} · {element.name or element.guid}",
            4000,
        )

    def _clear_element_selection(self) -> None:
        self.element_property_tree.clear()
        self.element_hint_label.setText("Click a BIM element to inspect its IFC data.")

    def _add_value(self, parent: QTreeWidgetItem, name: str, value: Any, *, depth: int = 0) -> None:
        if isinstance(value, dict) and depth < 3:
            item = QTreeWidgetItem([str(name), ""])
            parent.addChild(item)
            for key in sorted(value):
                self._add_value(item, str(key), value[key], depth=depth + 1)
            return
        if isinstance(value, list) and depth < 3:
            item = QTreeWidgetItem([str(name), f"{len(value)} item(s)"])
            parent.addChild(item)
            for index, child in enumerate(value[:64]):
                self._add_value(item, f"[{index}]", child, depth=depth + 1)
            return
        self._add_property(parent, name, value)

    @staticmethod
    def _add_property(parent: QTreeWidgetItem, name: str, value: Any) -> None:
        parent.addChild(QTreeWidgetItem([str(name), "" if value is None else str(value)]))

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
