from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication, QComboBox, QGroupBox, QLabel, QVBoxLayout

from ..route_profiles import (
    available_route_profiles,
    model_for_route_profile,
    resolve_route_profile,
)
from .evacuation_window import IFCPathEvacuationWindow


class IFCPathProfileWindow(IFCPathEvacuationWindow):
    """Final Builder window with user/mission routing policy selection.

    The canonical INAV model is never modified. For each route or evacuation
    preparation, a semantic-policy view is handed to the already-qualified
    routing/simulation engine and the source model is immediately restored.
    """

    def _build_sidebar(self):
        scroll = super()._build_sidebar()
        content = scroll.widget()
        layout = content.layout()

        group = QGroupBox("Route profile")
        group_layout = QVBoxLayout(group)
        group_layout.addWidget(QLabel("User / mission policy"))

        self.route_profile_combo = QComboBox()
        for profile in available_route_profiles():
            self.route_profile_combo.addItem(profile.label, profile.id)
        self.route_profile_combo.setToolTip(
            "Profiles filter semantic transitions without changing the INAV model. "
            "Accessible excludes stairs/escalators and ambiguous vertical connectors; "
            "the conservative emergency responder profile excludes elevators and "
            "ambiguous vertical connectors."
        )
        self.route_profile_combo.currentIndexChanged.connect(self._route_profile_changed)
        group_layout.addWidget(self.route_profile_combo)

        self.route_profile_description_label = QLabel()
        self.route_profile_description_label.setWordWrap(True)
        group_layout.addWidget(self.route_profile_description_label)
        self._sync_route_profile_description()

        insert_at = layout.indexOf(self.build_button)
        if insert_at < 0:
            insert_at = max(0, layout.count() - 1)
        layout.insertWidget(insert_at, group)
        return scroll

    def _route_profile_id(self) -> str:
        if not hasattr(self, "route_profile_combo"):
            return "standard"
        return str(self.route_profile_combo.currentData() or "standard")

    def _sync_route_profile_description(self) -> None:
        if not hasattr(self, "route_profile_description_label"):
            return
        profile = resolve_route_profile(self._route_profile_id())
        self.route_profile_description_label.setText(profile.description)

    def _route_profile_changed(self, _index: int = -1) -> None:
        self._sync_route_profile_description()
        profile = resolve_route_profile(self._route_profile_id())

        # An evacuation simulator owns the profiled model it was created with.
        # Reset it on profile changes instead of silently replanning against a
        # stale policy view.
        if getattr(self, "_evacuation_simulator", None) is not None:
            self._reset_evacuation()
            self.statusBar().showMessage(
                "Route profile changed; prepare the evacuation population again",
                6000,
            )

        was_running = self._walker.running
        current = self._walker.position if self._walker.points else None
        if self._start_point is not None and self._goal_point is not None:
            self._calculate_route(route_start=current, preserve_running=was_running)
        self._append_log(f"PROFILE: {profile.label} ({profile.id})")

    def _profiled_model(self):
        return (
            None
            if self._model is None
            else model_for_route_profile(self._model, self._route_profile_id())
        )

    def _calculate_route(self, *, route_start=None, preserve_running: bool = False) -> None:
        original_model = self._model
        if original_model is None:
            return super()._calculate_route(
                route_start=route_start,
                preserve_running=preserve_running,
            )

        self._model = model_for_route_profile(original_model, self._route_profile_id())
        try:
            super()._calculate_route(
                route_start=route_start,
                preserve_running=preserve_running,
            )
        finally:
            self._model = original_model

    def _prepare_evacuation(self) -> None:
        """Prepare population and simulator against one consistent policy view.

        Automatic population generation tests baseline egress reachability. It
        therefore must see the same profile-filtered topology as the simulator;
        otherwise a step-free run could auto-spawn occupants in a stair-only
        region and report policy-created trapping as a building defect.
        """
        original_model = self._model
        if original_model is None:
            return super()._prepare_evacuation()

        self._model = model_for_route_profile(original_model, self._route_profile_id())
        try:
            # While the profiled model is installed, _create_evacuation_simulator
            # below sees it as canonical for this preparation. It will make one
            # additional shallow policy view, which is safe and preserves the
            # source INAV object.
            super()._prepare_evacuation()
        finally:
            self._model = original_model

    def _create_evacuation_simulator(self, agents, config):
        original_model = self._model
        if original_model is None:
            return super()._create_evacuation_simulator(agents, config)

        self._model = model_for_route_profile(original_model, self._route_profile_id())
        try:
            return super()._create_evacuation_simulator(agents, config)
        finally:
            self._model = original_model


def run_profile_app() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("IFCPath Builder")
    app.setOrganizationName("IFCPath")
    app.setStyle("Fusion")
    window = IFCPathProfileWindow()
    window.show()
    return app.exec()
