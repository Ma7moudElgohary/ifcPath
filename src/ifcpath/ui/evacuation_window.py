from __future__ import annotations

import sys

from PySide6.QtCore import QElapsedTimer, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from ..evacuation import EvacuationConfig, EvacuationSimulator, spawn_agents
from ..hierarchical_routing import HierarchicalRouteOptions
from ..model import InavModel
from ..validation import ValidationReport
from .evacuation_preview import Evacuation3DPreview
from .preview_geometry import PreviewGeometry
from .scenario_window import IFCPathScenarioWindow


class IFCPathEvacuationWindow(IFCPathScenarioWindow):
    """Desktop Builder with deterministic multi-person evacuation simulation."""

    def __init__(self) -> None:
        self._evacuation_simulator: EvacuationSimulator | None = None
        self._evacuation_running = False
        super().__init__()

        self._evacuation_timer = QTimer(self)
        self._evacuation_timer.setInterval(80)
        self._evacuation_timer.timeout.connect(self._evacuation_tick)
        self._evacuation_clock = QElapsedTimer()
        self._update_evacuation_controls()

    def _build_sidebar(self):
        scroll = super()._build_sidebar()
        content = scroll.widget()
        layout = content.layout()

        group = QGroupBox("Multi-person evacuation")
        group_layout = QVBoxLayout(group)

        population_row = QHBoxLayout()
        population_row.addWidget(QLabel("Occupants"))
        self.evacuation_count_spin = QSpinBox()
        self.evacuation_count_spin.setRange(1, 200)
        self.evacuation_count_spin.setValue(20)
        population_row.addWidget(self.evacuation_count_spin, 1)
        group_layout.addLayout(population_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Speed range"))
        self.evacuation_min_speed_spin = QDoubleSpinBox()
        self.evacuation_min_speed_spin.setRange(0.2, 3.0)
        self.evacuation_min_speed_spin.setDecimals(2)
        self.evacuation_min_speed_spin.setValue(0.9)
        self.evacuation_min_speed_spin.setSuffix(" m/s")
        self.evacuation_max_speed_spin = QDoubleSpinBox()
        self.evacuation_max_speed_spin.setRange(0.2, 3.0)
        self.evacuation_max_speed_spin.setDecimals(2)
        self.evacuation_max_speed_spin.setValue(1.4)
        self.evacuation_max_speed_spin.setSuffix(" m/s")
        speed_row.addWidget(self.evacuation_min_speed_spin)
        speed_row.addWidget(self.evacuation_max_speed_spin)
        group_layout.addLayout(speed_row)

        flow_row = QHBoxLayout()
        flow_row.addWidget(QLabel("Door/exit flow"))
        self.evacuation_flow_spin = QDoubleSpinBox()
        self.evacuation_flow_spin.setRange(0.1, 5.0)
        self.evacuation_flow_spin.setDecimals(2)
        self.evacuation_flow_spin.setValue(1.30)
        self.evacuation_flow_spin.setSuffix(" p/s/m")
        flow_row.addWidget(self.evacuation_flow_spin, 1)
        group_layout.addLayout(flow_row)

        stair_row = QHBoxLayout()
        stair_row.addWidget(QLabel("Stair capacity"))
        self.evacuation_stair_capacity_spin = QDoubleSpinBox()
        self.evacuation_stair_capacity_spin.setRange(0.1, 10.0)
        self.evacuation_stair_capacity_spin.setDecimals(2)
        self.evacuation_stair_capacity_spin.setValue(1.0)
        self.evacuation_stair_capacity_spin.setSuffix(" p/s")
        stair_row.addWidget(self.evacuation_stair_capacity_spin, 1)
        group_layout.addLayout(stair_row)

        seed_row = QHBoxLayout()
        seed_row.addWidget(QLabel("Population seed"))
        self.evacuation_seed_spin = QSpinBox()
        self.evacuation_seed_spin.setRange(0, 999999)
        self.evacuation_seed_spin.setValue(42)
        seed_row.addWidget(self.evacuation_seed_spin, 1)
        group_layout.addLayout(seed_row)

        prepare_row = QHBoxLayout()
        self.evacuation_prepare_button = QPushButton("Prepare")
        self.evacuation_start_button = QPushButton("Start")
        self.evacuation_reset_button = QPushButton("Reset")
        self.evacuation_prepare_button.clicked.connect(self._prepare_evacuation)
        self.evacuation_start_button.clicked.connect(self._toggle_evacuation)
        self.evacuation_reset_button.clicked.connect(self._reset_evacuation)
        prepare_row.addWidget(self.evacuation_prepare_button)
        prepare_row.addWidget(self.evacuation_start_button)
        prepare_row.addWidget(self.evacuation_reset_button)
        group_layout.addLayout(prepare_row)

        self.evacuation_summary_label = QLabel("Evacuation: not prepared")
        self.evacuation_summary_label.setWordWrap(True)
        group_layout.addWidget(self.evacuation_summary_label)

        self.evacuation_exit_label = QLabel("Exits: —")
        self.evacuation_exit_label.setWordWrap(True)
        group_layout.addWidget(self.evacuation_exit_label)

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
        preview = Evacuation3DPreview()
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
        self._reset_evacuation()
        super()._worker_finished_3d(model, report, preview_geometry, path)

    def _scenario_changed(self) -> None:
        super()._scenario_changed()
        if self._evacuation_simulator is None:
            return
        self._evacuation_simulator.replan(self._evacuation_options())
        self._append_log("EVACUATION: replanned active occupants under scenario change")
        self._sync_evacuation_preview()
        self._update_evacuation_controls()

    def _evacuation_options(self) -> HierarchicalRouteOptions:
        return HierarchicalRouteOptions(
            blocked_portals=set(self._blocked_portals),
            blocked_spaces=set(self._blocked_spaces),
            space_cost_multipliers=dict(self._space_cost_multipliers),
        )

    def _evacuation_config(self) -> EvacuationConfig:
        flow = float(self.evacuation_flow_spin.value())
        return EvacuationConfig(
            door_specific_flow_pps_per_m=flow,
            exit_specific_flow_pps_per_m=flow,
            stair_capacity_pps=float(self.evacuation_stair_capacity_spin.value()),
        )

    def _prepare_evacuation(self) -> None:
        if self._model is None:
            self.statusBar().showMessage("Load or build a navigation model first", 6000)
            return
        if not any(portal.is_exit for portal in self._model.portals):
            self.evacuation_summary_label.setText("Evacuation: no classified exits in this model")
            self.statusBar().showMessage("No classified exits are available", 6000)
            return

        self._pause_simulation()
        self.person_check.setChecked(False)
        self._stop_evacuation_timer()
        config = self._evacuation_config()
        agents = spawn_agents(
            self._model,
            int(self.evacuation_count_spin.value()),
            min_speed_mps=float(self.evacuation_min_speed_spin.value()),
            max_speed_mps=float(self.evacuation_max_speed_spin.value()),
            seed=int(self.evacuation_seed_spin.value()),
            config=config,
        )
        self._evacuation_simulator = EvacuationSimulator(
            self._model,
            agents,
            options=self._evacuation_options(),
            config=config,
        )
        self._evacuation_running = False
        self._sync_evacuation_preview()
        self._update_evacuation_controls()
        stats = self._evacuation_simulator.stats
        self._append_log(
            f"EVACUATION: prepared {stats.total_agents} occupant(s), "
            f"{stats.trapped_agents} initially trapped"
        )
        self.statusBar().showMessage("Multi-person evacuation prepared", 5000)

    def _toggle_evacuation(self) -> None:
        if self._evacuation_simulator is None:
            self._prepare_evacuation()
        simulator = self._evacuation_simulator
        if simulator is None or simulator.finished:
            return
        if self._evacuation_running:
            self._stop_evacuation_timer()
            self._append_log("EVACUATION: paused")
        else:
            self._evacuation_running = True
            self._evacuation_clock.start()
            self._evacuation_timer.start()
            self._append_log("EVACUATION: started")
        self._update_evacuation_controls()

    def _stop_evacuation_timer(self) -> None:
        self._evacuation_running = False
        if hasattr(self, "_evacuation_timer"):
            self._evacuation_timer.stop()

    def _reset_evacuation(self) -> None:
        self._stop_evacuation_timer()
        self._evacuation_simulator = None
        if hasattr(self, "preview") and isinstance(self.preview, Evacuation3DPreview):
            self.preview.set_evacuation_agents({})
        self._update_evacuation_controls()

    def _evacuation_tick(self) -> None:
        if self._evacuation_simulator is None:
            self._stop_evacuation_timer()
            return
        if not self._evacuation_clock.isValid():
            self._evacuation_clock.start()
            return
        elapsed_ms = self._evacuation_clock.restart()
        self._advance_evacuation(max(0.0, elapsed_ms / 1000.0))

    def _advance_evacuation(self, delta_seconds: float) -> None:
        simulator = self._evacuation_simulator
        if simulator is None:
            return
        simulator.advance(delta_seconds)
        self._sync_evacuation_preview()
        self._update_evacuation_controls()
        if simulator.finished:
            self._stop_evacuation_timer()
            stats = simulator.stats
            self._append_log(
                f"EVACUATION: complete at {stats.elapsed_s:.2f}s · "
                f"evacuated={stats.evacuated_agents} trapped={stats.trapped_agents} "
                f"max_queue={stats.max_queue}"
            )
            self.statusBar().showMessage("Evacuation simulation complete", 7000)

    def _sync_evacuation_preview(self) -> None:
        if not isinstance(self.preview, Evacuation3DPreview):
            return
        simulator = self._evacuation_simulator
        if simulator is None:
            self.preview.set_evacuation_agents({})
            return
        poses = {
            state.spec.id: (state.position, state.forward_xy, state.status)
            for state in simulator.agents
        }
        self.preview.set_evacuation_agents(poses)

    def _update_evacuation_controls(self) -> None:
        if not hasattr(self, "evacuation_start_button"):
            return
        simulator = self._evacuation_simulator
        self.evacuation_start_button.setText("Pause" if self._evacuation_running else "Start")
        self.evacuation_start_button.setEnabled(
            simulator is not None and not simulator.finished
        )
        self.evacuation_reset_button.setEnabled(simulator is not None)
        if simulator is None:
            self.evacuation_summary_label.setText("Evacuation: not prepared")
            self.evacuation_exit_label.setText("Exits: —")
            return

        stats = simulator.stats
        average = "—" if stats.average_evacuation_time_s is None else f"{stats.average_evacuation_time_s:.1f}s"
        clearance = "—" if stats.clearance_time_s is None else f"{stats.clearance_time_s:.1f}s"
        self.evacuation_summary_label.setText(
            f"t={stats.elapsed_s:.1f}s · evacuated {stats.evacuated_agents}/{stats.total_agents} · "
            f"moving {stats.active_agents - stats.waiting_agents} · waiting {stats.waiting_agents} · "
            f"trapped {stats.trapped_agents} · max queue {stats.max_queue} · "
            f"avg {average} · clearance {clearance}"
        )
        if stats.exit_usage:
            usage = " · ".join(f"{exit_id}: {count}" for exit_id, count in stats.exit_usage.items())
            self.evacuation_exit_label.setText(f"Exits: {usage}")
        else:
            assigned: dict[str, int] = {}
            for state in simulator.agents:
                if state.plan is None:
                    continue
                exit_id = state.plan.exit_portal_id
                assigned[exit_id] = assigned.get(exit_id, 0) + 1
            usage = " · ".join(f"{exit_id}: {count}" for exit_id, count in sorted(assigned.items()))
            self.evacuation_exit_label.setText(f"Assigned exits: {usage or '—'}")

    def _refresh_preview(self) -> None:
        super()._refresh_preview()
        if hasattr(self, "evacuation_summary_label"):
            self._sync_evacuation_preview()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        self._stop_evacuation_timer()
        super().closeEvent(event)


def run_evacuation_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathEvacuationWindow()
    window.show()
    return app.exec()
