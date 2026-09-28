from __future__ import annotations

import sys

from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

from ..hierarchical_routing import (
    HierarchicalRouteOptions,
    find_hierarchical_path,
)
from ..model import InavModel
from ..validation import ValidationReport
from .main_window_3d import IFCPathBuilder3DWindow
from .preview_geometry import PreviewGeometry
from .scenario_preview import Scenario3DPreview


class IFCPathScenarioWindow(IFCPathBuilder3DWindow):
    """3D Builder with runtime Digital Twin scenarios and a person agent."""

    def __init__(self) -> None:
        self._blocked_portals: set[str] = set()
        self._blocked_spaces: set[str] = set()
        self._space_cost_multipliers: dict[str, float] = {}
        self._hazard_kinds: dict[str, str] = {}
        super().__init__()

    def _build_sidebar(self):
        scroll = super()._build_sidebar()
        content = scroll.widget()
        layout = content.layout()

        group = QGroupBox("Live scenario")
        group_layout = QVBoxLayout(group)

        self.person_check = QCheckBox("Show person agent")
        self.person_check.setChecked(True)
        self.person_check.toggled.connect(self._person_visibility_changed)
        group_layout.addWidget(self.person_check)

        group_layout.addWidget(QLabel("Door / portal"))
        self.scenario_portal_combo = QComboBox()
        self.scenario_portal_combo.currentIndexChanged.connect(self._sync_scenario_controls)
        group_layout.addWidget(self.scenario_portal_combo)
        self.portal_block_button = QPushButton("Block selected door")
        self.portal_block_button.clicked.connect(self._toggle_portal_block)
        group_layout.addWidget(self.portal_block_button)

        group_layout.addWidget(QLabel("Space / room"))
        self.scenario_space_combo = QComboBox()
        self.scenario_space_combo.currentIndexChanged.connect(self._sync_scenario_controls)
        group_layout.addWidget(self.scenario_space_combo)

        hazard_row = QHBoxLayout()
        self.hazard_kind_combo = QComboBox()
        self.hazard_kind_combo.addItem("Smoke", "smoke")
        self.hazard_kind_combo.addItem("Fire", "fire")
        self.hazard_kind_combo.addItem("Crowd", "crowd")
        self.hazard_multiplier_spin = QDoubleSpinBox()
        self.hazard_multiplier_spin.setRange(1.0, 100.0)
        self.hazard_multiplier_spin.setDecimals(1)
        self.hazard_multiplier_spin.setSingleStep(1.0)
        self.hazard_multiplier_spin.setValue(5.0)
        self.hazard_multiplier_spin.setPrefix("×")
        hazard_row.addWidget(self.hazard_kind_combo, 1)
        hazard_row.addWidget(self.hazard_multiplier_spin)
        group_layout.addLayout(hazard_row)

        hazard_buttons = QHBoxLayout()
        self.apply_hazard_button = QPushButton("Apply hazard")
        self.clear_hazard_button = QPushButton("Clear hazard")
        self.apply_hazard_button.clicked.connect(self._apply_space_hazard)
        self.clear_hazard_button.clicked.connect(self._clear_space_hazard)
        hazard_buttons.addWidget(self.apply_hazard_button)
        hazard_buttons.addWidget(self.clear_hazard_button)
        group_layout.addLayout(hazard_buttons)

        self.space_block_button = QPushButton("Block selected space")
        self.space_block_button.clicked.connect(self._toggle_space_block)
        group_layout.addWidget(self.space_block_button)

        self.scenario_summary_label = QLabel("Scenario: clear")
        self.scenario_summary_label.setWordWrap(True)
        group_layout.addWidget(self.scenario_summary_label)

        self.reset_scenario_button = QPushButton("Reset Scenario")
        self.reset_scenario_button.clicked.connect(self._reset_scenario)
        group_layout.addWidget(self.reset_scenario_button)

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
        preview = Scenario3DPreview()
        old_preview.setParent(None)
        old_preview.deleteLater()
        splitter.insertWidget(index, preview)
        self.preview = preview
        self.preview.pointPicked.connect(self._point_picked)
        return workspace

    def _worker_finished_3d(
        self,
        model: InavModel,
        report: ValidationReport,
        preview_geometry: PreviewGeometry | None,
        path: str,
    ) -> None:
        self._clear_scenario_state()
        super()._worker_finished_3d(model, report, preview_geometry, path)

    def _populate_model(self, model: InavModel, report: ValidationReport) -> None:
        super()._populate_model(model, report)
        self._populate_scenario_choices(model)
        self._apply_scenario_to_preview()

    def _populate_scenario_choices(self, model: InavModel) -> None:
        self.scenario_portal_combo.blockSignals(True)
        self.scenario_portal_combo.clear()
        for portal in sorted(model.portals, key=lambda item: (item.level_id or "", item.id)):
            kind = "EXIT" if portal.is_exit else portal.kind.upper()
            label = f"{kind} · {portal.id}"
            self.scenario_portal_combo.addItem(label, portal.id)
        self.scenario_portal_combo.blockSignals(False)

        space_by_level = {level.id: level.name for level in model.levels}
        self.scenario_space_combo.blockSignals(True)
        self.scenario_space_combo.clear()
        for space in sorted(model.spaces, key=lambda item: (item.level_id or "", item.name, item.id)):
            level_name = space_by_level.get(space.level_id or "", space.level_id or "")
            label = f"{space.name} · {level_name}" if level_name else space.name
            self.scenario_space_combo.addItem(label, space.id)
        self.scenario_space_combo.blockSignals(False)
        self._sync_scenario_controls()

    def _selected_portal_id(self) -> str | None:
        value = self.scenario_portal_combo.currentData()
        return str(value) if value else None

    def _selected_space_id(self) -> str | None:
        value = self.scenario_space_combo.currentData()
        return str(value) if value else None

    def _toggle_portal_block(self) -> None:
        portal_id = self._selected_portal_id()
        if not portal_id:
            return
        if portal_id in self._blocked_portals:
            self._blocked_portals.remove(portal_id)
            self._append_log(f"SCENARIO: unblocked portal {portal_id}")
        else:
            self._blocked_portals.add(portal_id)
            self._append_log(f"SCENARIO: blocked portal {portal_id}")
        self._scenario_changed()

    def _toggle_space_block(self) -> None:
        space_id = self._selected_space_id()
        if not space_id:
            return
        if space_id in self._blocked_spaces:
            self._blocked_spaces.remove(space_id)
            self._append_log(f"SCENARIO: unblocked space {space_id}")
        else:
            self._blocked_spaces.add(space_id)
            self._append_log(f"SCENARIO: blocked space {space_id}")
        self._scenario_changed()

    def _apply_space_hazard(self) -> None:
        space_id = self._selected_space_id()
        if not space_id:
            return
        multiplier = max(1.0, float(self.hazard_multiplier_spin.value()))
        if multiplier <= 1.0:
            self._space_cost_multipliers.pop(space_id, None)
            self._hazard_kinds.pop(space_id, None)
        else:
            self._space_cost_multipliers[space_id] = multiplier
            self._hazard_kinds[space_id] = str(self.hazard_kind_combo.currentData() or "hazard")
        self._append_log(
            f"SCENARIO: {self._hazard_kinds.get(space_id, 'clear')} {space_id} cost ×{multiplier:.1f}"
        )
        self._scenario_changed()

    def _clear_space_hazard(self) -> None:
        space_id = self._selected_space_id()
        if not space_id:
            return
        self._space_cost_multipliers.pop(space_id, None)
        self._hazard_kinds.pop(space_id, None)
        self._append_log(f"SCENARIO: cleared hazard in {space_id}")
        self._scenario_changed()

    def _reset_scenario(self) -> None:
        self._clear_scenario_state()
        self._append_log("SCENARIO: reset")
        self._scenario_changed()

    def _clear_scenario_state(self) -> None:
        self._blocked_portals.clear()
        self._blocked_spaces.clear()
        self._space_cost_multipliers.clear()
        self._hazard_kinds.clear()
        if hasattr(self, "hazard_multiplier_spin"):
            self.hazard_multiplier_spin.setValue(5.0)

    def _scenario_changed(self) -> None:
        self._sync_scenario_controls()
        self._apply_scenario_to_preview()
        if self._start_point is not None and self._goal_point is not None:
            self._calculate_route()

    def _sync_scenario_controls(self) -> None:
        if not hasattr(self, "scenario_summary_label"):
            return
        portal_id = self._selected_portal_id()
        space_id = self._selected_space_id()
        self.portal_block_button.setText(
            "Unblock selected door" if portal_id in self._blocked_portals else "Block selected door"
        )
        self.space_block_button.setText(
            "Unblock selected space" if space_id in self._blocked_spaces else "Block selected space"
        )
        self.clear_hazard_button.setEnabled(bool(space_id and space_id in self._space_cost_multipliers))
        self.scenario_summary_label.setText(
            "Scenario: "
            f"{len(self._blocked_portals)} blocked door(s) · "
            f"{len(self._blocked_spaces)} blocked space(s) · "
            f"{len(self._space_cost_multipliers)} hazard space(s)"
        )

    def _apply_scenario_to_preview(self) -> None:
        if not isinstance(self.preview, Scenario3DPreview):
            return
        self.preview.set_scenario(
            blocked_portals=self._blocked_portals,
            blocked_spaces=self._blocked_spaces,
            space_cost_multipliers=self._space_cost_multipliers,
            hazard_kinds=self._hazard_kinds,
        )
        self.preview.set_person_visible(self.person_check.isChecked())

    def _person_visibility_changed(self, visible: bool) -> None:
        if isinstance(self.preview, Scenario3DPreview):
            self.preview.set_person_visible(visible)

    def _refresh_preview(self) -> None:
        super()._refresh_preview()
        if hasattr(self, "scenario_summary_label"):
            self._apply_scenario_to_preview()

    def _calculate_route(self) -> None:
        if self._model is None or self._start_point is None or self._goal_point is None:
            return
        options = HierarchicalRouteOptions(
            blocked_portals=set(self._blocked_portals),
            blocked_spaces=set(self._blocked_spaces),
            space_cost_multipliers=dict(self._space_cost_multipliers),
        )
        route = find_hierarchical_path(
            self._model,
            self._start_point,
            self._goal_point,
            options,
        )
        self._route = route
        if route is None:
            self.route_label.setText("Route: no valid path under current scenario")
            self.preview.set_route(self._start_point, self._goal_point, [])
            self._apply_scenario_to_preview()
            self._append_log("ROUTE: no valid hierarchical path under current scenario")
            return

        if self.level_combo.count() > 0:
            self.level_combo.setCurrentIndex(0)
        self.preview.set_route(self._start_point, self._goal_point, route.points)
        self._apply_scenario_to_preview()
        transitions = len(route.transition_ids)
        self.route_label.setText(
            f"Route: {route.length_m:.2f} m · {len(route.space_ids)} space(s) · "
            f"{transitions} transfer(s) · cost {route.weighted_cost:.2f}"
        )
        self._append_log(
            f"ROUTE: length={route.length_m:.3f}m cost={route.weighted_cost:.3f} "
            f"spaces={' -> '.join(route.space_ids)} transitions={' -> '.join(route.transition_ids) or 'none'}"
        )
        self.statusBar().showMessage("Scenario route recalculated", 6000)


def run_scenario_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathScenarioWindow()
    window.show()
    return app.exec()
