from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtCore import QSettings, QThread, Qt
from PySide6.QtGui import QAction, QColor
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from ..exporter import save_inav
from ..ifc_loader import BuildOptions
from ..model import InavModel
from ..validation import ValidationReport
from .preview import NavigationPreview
from .worker import ModelWorker


class MetricCard(QFrame):
    def __init__(self, title: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("metricCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 9, 12, 9)
        layout.setSpacing(1)
        caption = QLabel(title.upper())
        caption.setObjectName("metricCaption")
        self.value = QLabel("—")
        self.value.setObjectName("metricValue")
        layout.addWidget(caption)
        layout.addWidget(self.value)

    def set_value(self, value: str | int) -> None:
        self.value.setText(str(value))


class IFCPathBuilderWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("IFCPath Builder")
        self.resize(1500, 900)
        self.setMinimumSize(1120, 720)
        self.setAcceptDrops(True)

        self._settings = QSettings("IFCPath", "Builder")
        self._model: InavModel | None = None
        self._report: ValidationReport | None = None
        self._source_path: str | None = None
        self._thread: QThread | None = None
        self._worker: ModelWorker | None = None

        self._build_actions()
        self._build_ui()
        self._apply_style()
        self._restore_settings()
        self.statusBar().showMessage("Ready")

    # ---------- construction ----------

    def _build_actions(self) -> None:
        self.open_ifc_action = QAction("Open IFC", self)
        self.open_ifc_action.setShortcut("Ctrl+O")
        self.open_ifc_action.triggered.connect(self.open_ifc)

        self.open_inav_action = QAction("Open INAV", self)
        self.open_inav_action.triggered.connect(self.open_inav)

        self.build_action = QAction("Build & Validate", self)
        self.build_action.setShortcut("Ctrl+B")
        self.build_action.setEnabled(False)
        self.build_action.triggered.connect(self.build_model)

        self.export_action = QAction("Export INAV", self)
        self.export_action.setShortcut("Ctrl+S")
        self.export_action.setEnabled(False)
        self.export_action.triggered.connect(self.export_inav)

        self.fit_action = QAction("Fit Preview", self)
        self.fit_action.setShortcut("F")
        self.fit_action.triggered.connect(lambda: self.preview.fit_content())

        toolbar = QToolBar("Builder", self)
        toolbar.setMovable(False)
        toolbar.setToolButtonStyle(Qt.ToolButtonTextOnly)
        toolbar.addAction(self.open_ifc_action)
        toolbar.addAction(self.open_inav_action)
        toolbar.addSeparator()
        toolbar.addAction(self.build_action)
        toolbar.addAction(self.export_action)
        toolbar.addSeparator()
        toolbar.addAction(self.fit_action)
        self.addToolBar(toolbar)

    def _build_ui(self) -> None:
        root_split = QSplitter(Qt.Horizontal)
        root_split.setChildrenCollapsible(False)
        root_split.addWidget(self._build_sidebar())
        root_split.addWidget(self._build_workspace())
        root_split.setSizes([330, 1170])
        self.setCentralWidget(root_split)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(1)
        self.progress.setMaximumWidth(180)
        self.progress.setVisible(False)
        self.statusBar().addPermanentWidget(self.progress)

    def _build_sidebar(self) -> QWidget:
        content = QWidget()
        content.setObjectName("sidebar")
        layout = QVBoxLayout(content)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        heading = QLabel("IFCPath Builder")
        heading.setObjectName("appTitle")
        subtitle = QLabel("BIM → portable indoor navigation")
        subtitle.setObjectName("appSubtitle")
        layout.addWidget(heading)
        layout.addWidget(subtitle)

        source_group = QGroupBox("Source")
        source_layout = QVBoxLayout(source_group)
        self.source_label = QLabel("No model selected")
        self.source_label.setWordWrap(True)
        self.source_label.setObjectName("sourcePath")
        source_layout.addWidget(self.source_label)
        source_buttons = QHBoxLayout()
        open_ifc_button = QPushButton("Open IFC")
        open_ifc_button.clicked.connect(self.open_ifc)
        open_inav_button = QPushButton("Open INAV")
        open_inav_button.clicked.connect(self.open_inav)
        source_buttons.addWidget(open_ifc_button)
        source_buttons.addWidget(open_inav_button)
        source_layout.addLayout(source_buttons)
        layout.addWidget(source_group)

        settings_group = QGroupBox("Navigation generation")
        form = QFormLayout(settings_group)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.backend_combo = QComboBox()
        self.backend_combo.addItems(["cdt", "sampled"])
        form.addRow("Floor backend", self.backend_combo)

        self.clearance_spin = self._spin(0.0, 5.0, 0.30, 0.05, " m")
        form.addRow("Agent clearance", self.clearance_spin)

        self.height_spin = self._spin(0.5, 5.0, 1.80, 0.05, " m")
        form.addRow("Agent height", self.height_spin)

        self.floor_spacing_spin = self._spin(0.05, 5.0, 0.80, 0.05, " m")
        form.addRow("Floor spacing", self.floor_spacing_spin)

        self.stair_spacing_spin = self._spin(0.05, 2.0, 0.25, 0.05, " m")
        form.addRow("Stair spacing", self.stair_spacing_spin)

        self.connect_distance_spin = self._spin(0.10, 10.0, 1.25, 0.05, " m")
        form.addRow("Connect distance", self.connect_distance_spin)

        self.obstacle_edit = QLineEdit("IfcColumn")
        self.obstacle_edit.setPlaceholderText("IfcColumn, IfcFurnishingElement")
        self.obstacle_edit.setToolTip("Comma-separated IFC classes treated as fixed obstacles")
        form.addRow("Fixed obstacles", self.obstacle_edit)
        layout.addWidget(settings_group)

        display_group = QGroupBox("Preview")
        display_layout = QVBoxLayout(display_group)
        self.level_combo = QComboBox()
        self.level_combo.setEnabled(False)
        self.level_combo.currentIndexChanged.connect(self._refresh_preview)
        display_layout.addWidget(self.level_combo)

        self.navmesh_check = QCheckBox("Navmesh cells")
        self.navmesh_check.setChecked(True)
        self.graph_check = QCheckBox("Metric graph")
        self.graph_check.setChecked(False)
        self.portals_check = QCheckBox("Doors / portals")
        self.portals_check.setChecked(True)
        self.exits_check = QCheckBox("Exits")
        self.exits_check.setChecked(True)
        for checkbox in (
            self.navmesh_check,
            self.graph_check,
            self.portals_check,
            self.exits_check,
        ):
            checkbox.toggled.connect(self._refresh_preview)
            display_layout.addWidget(checkbox)
        layout.addWidget(display_group)

        self.build_button = QPushButton("Build & Validate")
        self.build_button.setObjectName("primaryButton")
        self.build_button.setMinimumHeight(42)
        self.build_button.setEnabled(False)
        self.build_button.clicked.connect(self.build_model)
        layout.addWidget(self.build_button)

        self.export_button = QPushButton("Export .INAV")
        self.export_button.setMinimumHeight(38)
        self.export_button.setEnabled(False)
        self.export_button.clicked.connect(self.export_inav)
        layout.addWidget(self.export_button)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(content)
        scroll.setMinimumWidth(300)
        scroll.setMaximumWidth(390)
        return scroll

    def _build_workspace(self) -> QWidget:
        workspace = QWidget()
        outer = QVBoxLayout(workspace)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(10)

        cards = QGridLayout()
        cards.setHorizontalSpacing(8)
        self.card_levels = MetricCard("Levels")
        self.card_spaces = MetricCard("Spaces")
        self.card_portals = MetricCard("Portals")
        self.card_cells = MetricCard("Nav cells")
        self.card_nodes = MetricCard("Nodes")
        self.card_status = MetricCard("Validation")
        for index, card in enumerate(
            (
                self.card_levels,
                self.card_spaces,
                self.card_portals,
                self.card_cells,
                self.card_nodes,
                self.card_status,
            )
        ):
            cards.addWidget(card, 0, index)
        outer.addLayout(cards)

        vertical = QSplitter(Qt.Vertical)
        vertical.setChildrenCollapsible(False)
        self.preview = NavigationPreview()
        vertical.addWidget(self.preview)

        tabs = QTabWidget()
        tabs.setMinimumHeight(220)
        self.issue_table = QTableWidget(0, 4)
        self.issue_table.setHorizontalHeaderLabels(["Severity", "Code", "Entity", "Message"])
        self.issue_table.horizontalHeader().setStretchLastSection(True)
        self.issue_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.issue_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.issue_table.setAlternatingRowColors(True)
        tabs.addTab(self.issue_table, "Validation")

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        tabs.addTab(self.log, "Log")
        vertical.addWidget(tabs)
        vertical.setSizes([620, 240])
        outer.addWidget(vertical, 1)
        return workspace

    @staticmethod
    def _spin(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(2)
        widget.setSingleStep(step)
        widget.setValue(value)
        widget.setSuffix(suffix)
        return widget

    # ---------- file workflow ----------

    def open_ifc(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open IFC model",
            self._last_directory(),
            "IFC files (*.ifc);;All files (*.*)",
        )
        if not path:
            return
        self._remember_directory(path)
        self._source_path = path
        self._model = None
        self._report = None
        self.source_label.setText(path)
        self.setWindowTitle(f"IFCPath Builder — {Path(path).name}")
        self.build_action.setEnabled(True)
        self.build_button.setEnabled(True)
        self.export_action.setEnabled(False)
        self.export_button.setEnabled(False)
        self.preview.set_model(None)
        self._clear_summary()
        self.issue_table.setRowCount(0)
        self._append_log(f"Selected IFC: {path}")
        self.statusBar().showMessage("IFC selected — configure generation and build")

    def open_inav(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Open INAV model",
            self._last_directory(),
            "IFCPath navigation (*.inav *.json);;All files (*.*)",
        )
        if not path:
            return
        self._remember_directory(path)
        self._source_path = path
        self.source_label.setText(path)
        self.setWindowTitle(f"IFCPath Builder — {Path(path).name}")
        self._start_worker("load", path, None)

    def build_model(self) -> None:
        if not self._source_path or Path(self._source_path).suffix.lower() != ".ifc":
            QMessageBox.information(self, "IFCPath Builder", "Open an IFC model first.")
            return

        obstacle_classes = tuple(
            item.strip()
            for item in self.obstacle_edit.text().split(",")
            if item.strip()
        ) or ("IfcColumn",)
        options = BuildOptions(
            floor_backend=self.backend_combo.currentText(),
            agent_clearance_m=max(0.0, self.clearance_spin.value()),
            agent_height_m=max(0.0, self.height_spin.value()),
            fixed_obstacle_classes=obstacle_classes,
            floor_spacing_m=self.floor_spacing_spin.value(),
            stair_spacing_m=self.stair_spacing_spin.value(),
            connect_distance_m=self.connect_distance_spin.value(),
        )
        self._save_settings()
        self._start_worker("build", self._source_path, options)

    def export_inav(self) -> None:
        if self._model is None:
            return
        stem = Path(self._source_path or "Building").stem
        suggested = str(Path(self._last_directory()) / f"{stem}.inav")
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export portable navigation",
            suggested,
            "IFCPath navigation (*.inav)",
        )
        if not path:
            return
        if not path.lower().endswith(".inav"):
            path += ".inav"
        try:
            output = save_inav(self._model, path)
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            self._append_log(f"EXPORT ERROR: {exc}")
            return
        self._remember_directory(path)
        self._append_log(f"Exported INAV: {output}")
        self.statusBar().showMessage(f"Exported {output}", 6000)

    # ---------- worker ----------

    def _start_worker(
        self,
        operation: str,
        path: str,
        options: BuildOptions | None,
    ) -> None:
        if self._thread is not None:
            return
        self._set_busy(True, "Generating navigation…" if operation == "build" else "Loading INAV…")
        self._append_log(f"{operation.upper()}: {path}")

        thread = QThread(self)
        worker = ModelWorker(operation=operation, path=path, options=options)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.finished.connect(self._worker_finished)
        worker.failed.connect(self._worker_failed)
        worker.finished.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._thread_finished)
        self._thread = thread
        self._worker = worker
        thread.start()

    def _worker_finished(self, model, report, path: str) -> None:
        self._model = model
        self._report = report
        self._source_path = path
        self.export_action.setEnabled(True)
        self.export_button.setEnabled(True)
        self._populate_model(model, report)
        self._append_log(
            "Generated model: "
            f"levels={len(model.levels)} spaces={len(model.spaces)} "
            f"portals={len(model.portals)} cells={len(model.cells)} "
            f"nodes={len(model.nodes)} edges={len(model.edges)}"
        )
        self._append_log(
            f"Validation: valid={report.valid} errors={report.stats.get('errors', 0)} "
            f"warnings={report.stats.get('warnings', 0)}"
        )
        self.statusBar().showMessage("Build complete" if report.valid else "Build completed with validation errors", 8000)

    def _worker_failed(self, message: str, details: str) -> None:
        self._append_log(details)
        QMessageBox.critical(self, "IFCPath Builder", message)
        self.statusBar().showMessage("Operation failed", 8000)

    def _thread_finished(self) -> None:
        self._thread = None
        self._worker = None
        self._set_busy(False)

    def _set_busy(self, busy: bool, message: str = "") -> None:
        self.progress.setVisible(busy)
        if busy:
            self.progress.setRange(0, 0)
            self.statusBar().showMessage(message)
        else:
            self.progress.setRange(0, 1)
            self.progress.setValue(1)
        self.open_ifc_action.setEnabled(not busy)
        self.open_inav_action.setEnabled(not busy)
        self.build_action.setEnabled(not busy and bool(self._source_path and self._source_path.lower().endswith(".ifc")))
        self.build_button.setEnabled(self.build_action.isEnabled())
        self.export_action.setEnabled(not busy and self._model is not None)
        self.export_button.setEnabled(self.export_action.isEnabled())

    # ---------- presentation ----------

    def _populate_model(self, model: InavModel, report: ValidationReport) -> None:
        self.card_levels.set_value(len(model.levels))
        self.card_spaces.set_value(len(model.spaces))
        self.card_portals.set_value(len(model.portals))
        self.card_cells.set_value(len(model.cells))
        self.card_nodes.set_value(len(model.nodes))
        errors = int(report.stats.get("errors", 0))
        warnings = int(report.stats.get("warnings", 0))
        self.card_status.set_value("Ready" if report.valid and errors == 0 else f"{errors} error(s)")
        self.card_status.setToolTip(f"{errors} errors, {warnings} warnings")

        self.level_combo.blockSignals(True)
        self.level_combo.clear()
        self.level_combo.addItem("All levels", None)
        for level in sorted(model.levels, key=lambda item: item.elevation_m):
            self.level_combo.addItem(
                f"{level.name}  ·  {level.elevation_m:.2f} m",
                level.id,
            )
        self.level_combo.setEnabled(bool(model.levels))
        if model.levels:
            self.level_combo.setCurrentIndex(1)
        self.level_combo.blockSignals(False)

        self.preview.set_model(model)
        if model.levels:
            self.preview.set_level(self.level_combo.currentData())
        self._refresh_preview()
        self._populate_issues(report)

    def _populate_issues(self, report: ValidationReport) -> None:
        self.issue_table.setRowCount(len(report.issues))
        for row, issue in enumerate(report.issues):
            values = (issue.severity.upper(), issue.code, issue.entity_id or "", issue.message)
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 0:
                    if issue.severity == "error":
                        item.setForeground(QColor("#ff6b6b"))
                    elif issue.severity == "warning":
                        item.setForeground(QColor("#ffc857"))
                    else:
                        item.setForeground(QColor("#64d8a6"))
                self.issue_table.setItem(row, column, item)
        self.issue_table.resizeColumnsToContents()
        self.issue_table.horizontalHeader().setStretchLastSection(True)

    def _refresh_preview(self) -> None:
        if self._model is None:
            return
        self.preview.set_level(self.level_combo.currentData() if self.level_combo.isEnabled() else None)
        self.preview.set_layers(
            navmesh=self.navmesh_check.isChecked(),
            graph=self.graph_check.isChecked(),
            portals=self.portals_check.isChecked(),
            exits=self.exits_check.isChecked(),
        )

    def _clear_summary(self) -> None:
        for card in (
            self.card_levels,
            self.card_spaces,
            self.card_portals,
            self.card_cells,
            self.card_nodes,
            self.card_status,
        ):
            card.set_value("—")
        self.level_combo.clear()
        self.level_combo.setEnabled(False)

    def _append_log(self, text: str) -> None:
        self.log.appendPlainText(text.rstrip())

    # ---------- settings / drag-drop ----------

    def _last_directory(self) -> str:
        value = self._settings.value("lastDirectory", str(Path.home()))
        return str(value or Path.home())

    def _remember_directory(self, path: str) -> None:
        self._settings.setValue("lastDirectory", str(Path(path).resolve().parent))

    def _restore_settings(self) -> None:
        self.backend_combo.setCurrentText(str(self._settings.value("backend", "cdt")))
        self.clearance_spin.setValue(float(self._settings.value("clearance", 0.30)))
        self.height_spin.setValue(float(self._settings.value("height", 1.80)))
        self.floor_spacing_spin.setValue(float(self._settings.value("floorSpacing", 0.80)))
        self.stair_spacing_spin.setValue(float(self._settings.value("stairSpacing", 0.25)))
        self.connect_distance_spin.setValue(float(self._settings.value("connectDistance", 1.25)))
        self.obstacle_edit.setText(str(self._settings.value("obstacles", "IfcColumn")))

    def _save_settings(self) -> None:
        self._settings.setValue("backend", self.backend_combo.currentText())
        self._settings.setValue("clearance", self.clearance_spin.value())
        self._settings.setValue("height", self.height_spin.value())
        self._settings.setValue("floorSpacing", self.floor_spacing_spin.value())
        self._settings.setValue("stairSpacing", self.stair_spacing_spin.value())
        self._settings.setValue("connectDistance", self.connect_distance_spin.value())
        self._settings.setValue("obstacles", self.obstacle_edit.text())

    def dragEnterEvent(self, event) -> None:  # noqa: N802 - Qt override
        urls = event.mimeData().urls()
        if any(Path(url.toLocalFile()).suffix.lower() in {".ifc", ".inav", ".json"} for url in urls):
            event.acceptProposedAction()

    def dropEvent(self, event) -> None:  # noqa: N802 - Qt override
        urls = event.mimeData().urls()
        if not urls:
            return
        path = urls[0].toLocalFile()
        suffix = Path(path).suffix.lower()
        if suffix == ".ifc":
            self._source_path = path
            self.source_label.setText(path)
            self.setWindowTitle(f"IFCPath Builder — {Path(path).name}")
            self.build_action.setEnabled(True)
            self.build_button.setEnabled(True)
            self._append_log(f"Dropped IFC: {path}")
        elif suffix in {".inav", ".json"}:
            self._source_path = path
            self.source_label.setText(path)
            self._start_worker("load", path, None)
        event.acceptProposedAction()

    # ---------- appearance ----------

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #171d25; color: #e7edf5; }
            QToolBar { background: #10151c; border: none; spacing: 6px; padding: 6px; }
            QToolButton { background: #222b37; border: 1px solid #303b49; border-radius: 6px; padding: 7px 11px; }
            QToolButton:hover { background: #2b3644; }
            #sidebar { background: #121820; }
            #appTitle { font-size: 23px; font-weight: 700; color: #f4f7fb; }
            #appSubtitle { color: #8998aa; padding-bottom: 6px; }
            #sourcePath { color: #aab7c6; }
            QGroupBox { border: 1px solid #2a3441; border-radius: 8px; margin-top: 12px; padding: 10px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #b8c5d3; }
            QPushButton { background: #232d3a; border: 1px solid #344152; border-radius: 7px; padding: 7px 10px; }
            QPushButton:hover { background: #2c3847; }
            QPushButton:disabled { color: #5f6976; background: #1c232c; border-color: #252e39; }
            #primaryButton { background: #2e79d6; border-color: #3886e7; font-weight: 700; }
            #primaryButton:hover { background: #3686e7; }
            QComboBox, QLineEdit, QDoubleSpinBox { background: #0f151c; border: 1px solid #303c4b; border-radius: 6px; padding: 6px; }
            QComboBox::drop-down { border: none; }
            QCheckBox { spacing: 8px; }
            #metricCard { background: #111820; border: 1px solid #283341; border-radius: 8px; }
            #metricCaption { color: #738297; font-size: 10px; font-weight: 600; }
            #metricValue { color: #f1f5fa; font-size: 20px; font-weight: 700; }
            QTabWidget::pane { border: 1px solid #293442; background: #111820; }
            QTabBar::tab { background: #171f29; color: #92a0b0; padding: 8px 16px; border: 1px solid #293442; }
            QTabBar::tab:selected { color: #ffffff; background: #222c38; }
            QTableWidget, QPlainTextEdit { background: #0f151c; alternate-background-color: #131b24; border: none; gridline-color: #27313d; }
            QHeaderView::section { background: #1b2530; color: #9eacbb; border: none; padding: 6px; }
            QStatusBar { background: #10151c; color: #8e9cac; }
            QProgressBar { background: #202a35; border: 1px solid #303c49; border-radius: 4px; height: 9px; text-align: center; }
            QProgressBar::chunk { background: #3c8be8; border-radius: 3px; }
            QScrollArea { border: none; }
            """
        )


def run_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathBuilderWindow()
    window.show()
    return app.exec()
