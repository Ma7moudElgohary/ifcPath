from __future__ import annotations

import sys

from PySide6.QtCore import QElapsedTimer, QTimer
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

from ..agent_motion import RouteWalker
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
    """3D Builder with live scenarios and a distance-based walking agent."""

    def __init__(self) -> None:
        self._blocked_portals: set[str] = set()
        self._blocked_spaces: set[str] = set()
        self._space_cost_multipliers: dict[str, float] = {}
        self._hazard_kinds: dict[str, str] = {}
        self._walker = RouteWalker()
        super().__init__()

        self._simulation_timer = QTimer(self)
        self._simulation_timer.setInterval(50)
        self._simulation_timer.timeout.connect(self._simulation_tick)
        self._simulation_clock = QElapsedTimer()
        self._update_simulation_controls()

    def _build_sidebar(self):
        scroll = super()._build_sidebar()
        content = scroll.widget()
        layout = content.layout()

        scenario_group = QGroupBox("Live scenario")
        scenario_layout = QVBoxLayout(scenario_group)

        self.person_check = QCheckBox("Show person agent")
        self.person_check.setChecked(True)
        self.person_check.toggled.connect(self._person_visibility_changed)
        scenario_layout.addWidget(self.person_check)

        scenario_layout.addWidget(QLabel("Door / portal"))
        self.scenario_portal_combo = QComboBox()
        self.scenario_portal_combo.currentIndexChanged.connect(self._sync_scenario_controls)
        scenario_layout.addWidget(self.scenario_portal_combo)
        self.portal_block_button = QPushButton("Block selected door")
        self.portal_block_button.clicked.connect(self._toggle_portal_block)
        scenario_layout.addWidget(self.portal_block_button)

        scenario_layout.addWidget(QLabel("Space / room"))
        self.scenario_space_combo = QComboBox()
        self.scenario_space_combo.currentIndexChanged.connect(self._sync_scenario_controls)
        scenario_layout.addWidget(self.scenario_space_combo)

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
        scenario_layout.addLayout(hazard_row)

        hazard_buttons = QHBoxLayout()
        self.apply_hazard_button = QPushButton("Apply hazard")
        self.clear_hazard_button = QPushButton("Clear hazard")
        self.apply_hazard_button.clicked.connect(self._apply_space_hazard)
        self.clear_hazard_button.clicked.connect(self._clear_space_hazard)
        hazard_buttons.addWidget(self.apply_hazard_button)
        hazard_buttons.addWidget(self.clear_hazard_button)
        scenario_layout.addLayout(hazard_buttons)

        self.space_block_button = QPushButton("Block selected space")
        self.space_block_button.clicked.connect(self._toggle_space_block)
        scenario_layout.addWidget(self.space_block_button)

        self.scenario_summary_label = QLabel("Scenario: clear")
        self.scenario_summary_label.setWordWrap(True)
        scenario_layout.addWidget(self.scenario_summary_label)

        self.reset_scenario_button = QPushButton("Reset Scenario")
        self.reset_scenario_button.clicked.connect(self._reset_scenario)
        scenario_layout.addWidget(self.reset_scenario_button)

        simulation_group = QGroupBox("Walking simulation")
        simulation_layout = QVBoxLayout(simulation_group)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Walking speed"))
        self.walk_speed_spin = QDoubleSpinBox()
        self.walk_speed_spin.setRange(0.1, 5.0)
        self.walk_speed_spin.setDecimals(2)
        self.walk_speed_spin.setSingleStep(0.1)
        self.walk_speed_spin.setValue(1.35)
        self.walk_speed_spin.setSuffix(" m/s")
        self.walk_speed_spin.valueChanged.connect(self._walk_speed_changed)
        speed_row.addWidget(self.walk_speed_spin, 1)
        simulation_layout.addLayout(speed_row)

        button_row = QHBoxLayout()
        self.walk_play_button = QPushButton("Walk")
        self.walk_restart_button = QPushButton("Restart")
        self.walk_play_button.clicked.connect(self._toggle_simulation)
        self.walk_restart_button.clicked.connect(self._restart_simulation)
        button_row.addWidget(self.walk_play_button)
        button_row.addWidget(self.walk_restart_button)
        simulation_layout.addLayout(button_row)

        self.walk_status_label = QLabel("Agent: waiting for route")
        self.walk_status_label.setWordWrap(True)
        simulation_layout.addWidget(self.walk_status_label)

        insert_at = layout.indexOf(self.build_button)
        if insert_at < 0:
            insert_at = max(0, layout.count() - 1)
        layout.insertWidget(insert_at, scenario_group)
        insert_at = layout.indexOf(self.build_button)
        layout.insertWidget(insert_at, simulation_group)
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
        self._stop_simulation(clear=True)
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
        was_running = self._walker.running
        current = self._walker.position if self._walker.points else None
        self._sync_scenario_controls()
        self._apply_scenario_to_preview()
        if self._start_point is not None and self._goal_point is not None:
            self._calculate_route(route_start=current, preserve_running=was_running)

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
            self._sync_agent_preview()

    def _point_picked(self, mode: str, point, space_id, level_id) -> None:
        self._stop_simulation(clear=True)
        super()._point_picked(mode, point, space_id, level_id)

    def _calculate_route(
        self,
        *,
        route_start=None,
        preserve_running: bool = False,
    ) -> None:
        if self._model is None or self._start_point is None or self._goal_point is None:
            return
        origin = route_start or self._start_point
        options = HierarchicalRouteOptions(
            blocked_portals=set(self._blocked_portals),
            blocked_spaces=set(self._blocked_spaces),
            space_cost_multipliers=dict(self._space_cost_multipliers),
        )
        route = find_hierarchical_path(
            self._model,
            origin,
            self._goal_point,
            options,
        )
        self._route = route
        if route is None:
            frozen_position = route_start or self._walker.position or self._start_point
            frozen_forward = self._walker.forward_xy
            self._walker.clear()
            self.route_label.setText("Route: no valid path under current scenario")
            self.preview.set_route(self._start_point, self._goal_point, [])
            self.preview.set_agent_pose(frozen_position, frozen_forward)
            if hasattr(self, "_simulation_timer"):
                self._simulation_timer.stop()
            self._apply_scenario_to_preview()
            self._update_simulation_controls()
            self._append_log("ROUTE: no valid hierarchical path under current scenario")
            return

        if self.level_combo.count() > 0:
            self.level_combo.setCurrentIndex(0)
        self.preview.set_route(self._start_point, self._goal_point, route.points)
        self._apply_scenario_to_preview()
        self._walker.speed_mps = float(self.walk_speed_spin.value())
        self._walker.set_route(route.points, running=preserve_running)
        if self._walker.running:
            self._simulation_clock.start()
            self._simulation_timer.start()
        self._sync_agent_preview()
        self._update_simulation_controls()

        transitions = len(route.transition_ids)
        self.route_label.setText(
            f"Route: {route.length_m:.2f} m · {len(route.space_ids)} space(s) · "
            f"{transitions} transfer(s) · cost {route.weighted_cost:.2f}"
        )
        self._append_log(
            f"ROUTE: length={route.length_m:.3f}m cost={route.weighted_cost:.3f} "
            f"spaces={' -> '.join(route.space_ids)} transitions={' -> '.join(route.transition_ids) or 'none'}"
        )
        if route_start is not None:
            self._append_log(
                f"SIMULATION: rerouted from current position "
                f"({route_start[0]:.2f}, {route_start[1]:.2f}, {route_start[2]:.2f})"
            )
        self.statusBar().showMessage("Scenario route recalculated", 6000)

    # ---------- live walking simulation ----------

    def _walk_speed_changed(self, value: float) -> None:
        self._walker.speed_mps = max(0.0, float(value))
        self._update_simulation_controls()

    def _toggle_simulation(self) -> None:
        if self._walker.running:
            self._pause_simulation()
            return
        if not self._walker.points or self._walker.finished:
            if self._route is None and self._start_point is not None and self._goal_point is not None:
                self._calculate_route()
            elif self._route is not None:
                self._walker.set_route(self._route.points)
        if self._route is None or len(self._walker.points) < 2:
            return
        self._walker.speed_mps = float(self.walk_speed_spin.value())
        self._walker.play()
        self._simulation_clock.start()
        self._simulation_timer.start()
        self._sync_agent_preview()
        self._update_simulation_controls()
        self._append_log(f"SIMULATION: walking at {self._walker.speed_mps:.2f} m/s")

    def _pause_simulation(self) -> None:
        self._walker.pause()
        if hasattr(self, "_simulation_timer"):
            self._simulation_timer.stop()
        self._sync_agent_preview()
        self._update_simulation_controls()

    def _restart_simulation(self) -> None:
        self._pause_simulation()
        if self._start_point is None or self._goal_point is None:
            self._walker.clear()
            self._sync_agent_preview()
            self._update_simulation_controls()
            return
        self._walker.clear()
        self._calculate_route()
        self._sync_agent_preview()
        self._update_simulation_controls()
        self._append_log("SIMULATION: restarted at route start")

    def _stop_simulation(self, *, clear: bool) -> None:
        if hasattr(self, "_simulation_timer"):
            self._simulation_timer.stop()
        self._walker.pause()
        if clear:
            self._walker.clear()
        if hasattr(self, "preview") and isinstance(self.preview, Scenario3DPreview):
            self.preview.set_agent_pose(None)
        self._update_simulation_controls()

    def _simulation_tick(self) -> None:
        if not self._simulation_clock.isValid():
            self._simulation_clock.start()
            return
        elapsed_ms = self._simulation_clock.restart()
        self._advance_simulation(max(0.0, elapsed_ms / 1000.0))

    def _advance_simulation(self, delta_seconds: float) -> None:
        self._walker.speed_mps = float(self.walk_speed_spin.value())
        self._walker.advance(delta_seconds)
        self._sync_agent_preview()
        self._update_simulation_controls()
        if self._walker.finished:
            self._simulation_timer.stop()
            self._append_log("SIMULATION: destination reached")
            self.statusBar().showMessage("Agent reached destination", 6000)

    def _sync_agent_preview(self) -> None:
        if not isinstance(self.preview, Scenario3DPreview):
            return
        if self._walker.position is not None:
            self.preview.set_agent_pose(self._walker.position, self._walker.forward_xy)
        elif self._start_point is not None:
            self.preview.set_agent_pose(self._start_point, (0.0, 1.0))
        else:
            self.preview.set_agent_pose(None)

    def _update_simulation_controls(self) -> None:
        if not hasattr(self, "walk_play_button"):
            return
        self.walk_play_button.setText("Pause" if self._walker.running else "Walk")
        self.walk_play_button.setEnabled(
            self._route is not None and len(self._walker.points) >= 2 and not self._walker.finished
        )
        self.walk_restart_button.setEnabled(self._start_point is not None and self._goal_point is not None)

        if not self._walker.points:
            if self._route is None and self._start_point is not None and self._goal_point is not None:
                self.walk_status_label.setText("Agent: stopped · no valid route")
            else:
                self.walk_status_label.setText("Agent: waiting for route")
            return
        total = self._walker.total_length_m
        if self._walker.finished:
            state = "arrived"
        elif self._walker.running:
            state = "walking"
        elif self._walker.distance_m > 1e-9:
            state = "paused"
        else:
            state = "ready"
        self.walk_status_label.setText(
            f"Agent: {state} · {self._walker.distance_m:.2f}/{total:.2f} m · "
            f"{self._walker.progress * 100.0:.0f}%"
        )

    def _clear_route(self) -> None:
        if hasattr(self, "_walker"):
            self._stop_simulation(clear=True)
        super()._clear_route()
        self._update_simulation_controls()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        if hasattr(self, "_simulation_timer"):
            self._simulation_timer.stop()
        super().closeEvent(event)


def run_scenario_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathScenarioWindow()
    window.show()
    return app.exec()
