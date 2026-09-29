from __future__ import annotations

from ..model import Vec3
from .gpu_preview import GpuBimPreview


class GpuScenarioPreview(GpuBimPreview):
    """GPU preview contract used by the real scenario/evacuation Builder shell.

    Static BIM/navmesh and routes are GPU-rendered in this first milestone.
    Scenario and agent state is retained here so later dynamic GPU buffers can be
    added without changing any window/controller API.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._blocked_portals: set[str] = set()
        self._blocked_spaces: set[str] = set()
        self._space_cost_multipliers: dict[str, float] = {}
        self._hazard_kinds: dict[str, str] = {}
        self._show_person = True
        self._agent_position: Vec3 | None = None
        self._agent_forward: tuple[float, float] = (0.0, 1.0)
        self._evacuation_poses: dict[str, tuple[Vec3, tuple[float, float], str]] = {}

    def set_scenario(
        self,
        *,
        blocked_portals: set[str] | None = None,
        blocked_spaces: set[str] | None = None,
        space_cost_multipliers: dict[str, float] | None = None,
        hazard_kinds: dict[str, str] | None = None,
    ) -> None:
        self._blocked_portals = set(blocked_portals or ())
        self._blocked_spaces = set(blocked_spaces or ())
        self._space_cost_multipliers = dict(space_cost_multipliers or {})
        self._hazard_kinds = dict(hazard_kinds or {})
        self.request_draw()

    def set_person_visible(self, visible: bool) -> None:
        self._show_person = bool(visible)
        self.request_draw()

    def set_agent_pose(
        self,
        position: Vec3 | None,
        forward: tuple[float, float] | None = None,
    ) -> None:
        self._agent_position = position
        if forward is not None:
            self._agent_forward = forward
        self.request_draw()

    def set_evacuation_agents(
        self,
        poses: dict[str, tuple[Vec3, tuple[float, float], str]] | None,
    ) -> None:
        self._evacuation_poses = dict(poses or {})
        self.request_draw()

    @property
    def blocked_portals(self) -> set[str]:
        return set(self._blocked_portals)

    @property
    def blocked_spaces(self) -> set[str]:
        return set(self._blocked_spaces)

    @property
    def agent_position(self) -> Vec3 | None:
        return self._agent_position

    @property
    def evacuation_agent_count(self) -> int:
        return len(self._evacuation_poses)

    @property
    def person_triangle_count(self) -> int:
        # Dynamic GPU person meshes are the next parity slice; returning zero is
        # explicit rather than claiming they are rendered in this milestone.
        return 0
