from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, unary_union

from .model import InavModel, Vec3

_EPSILON = 1e-9


class MicroscopicBackendUnavailable(RuntimeError):
    """Raised when an optional microscopic backend is not installed."""


@dataclass(frozen=True, slots=True)
class MicroscopicMotionConfig:
    """Operational pedestrian-model settings shared by local-motion backends."""

    dt_s: float = 0.05
    model: str = "cfsm_v2"
    default_radius_m: float = 0.20
    time_gap_s: float = 1.0
    target_inset_m: float = 0.01


@dataclass(frozen=True, slots=True)
class MicroscopicAgentSpec:
    """Host-independent pedestrian input for one local-motion backend."""

    id: str
    position_m: Vec3
    target_m: Vec3
    desired_speed_mps: float = 1.2
    radius_m: float = 0.20
    time_gap_s: float = 1.0


@dataclass(frozen=True, slots=True)
class MicroscopicAgentSnapshot:
    id: str
    position_m: Vec3
    forward_xy: tuple[float, float]
    target_m: Vec3


@runtime_checkable
class LocalMotionBackend(Protocol):
    """Small contract between IFCPath routing and an operational crowd model.

    IFCPath remains responsible for BIM semantics, hierarchical routing, exits,
    hazards and route selection. A backend receives local targets and is only
    responsible for physically plausible pedestrian motion towards those targets.
    """

    name: str

    def add_agent(self, spec: MicroscopicAgentSpec) -> None: ...

    def remove_agent(self, agent_id: str) -> None: ...

    def set_target(self, agent_id: str, target_m: Vec3) -> None: ...

    def advance(self, delta_seconds: float) -> None: ...

    def snapshot(self, agent_id: str) -> MicroscopicAgentSnapshot: ...

    def snapshots(self) -> list[MicroscopicAgentSnapshot]: ...


@dataclass(slots=True)
class _KinematicState:
    spec: MicroscopicAgentSpec
    position_m: Vec3
    target_m: Vec3
    forward_xy: tuple[float, float] = (0.0, 1.0)


class KinematicLocalMotionBackend:
    """Deterministic no-interaction fallback implementing the backend contract.

    This deliberately does *not* claim to be crowd physics. It exists so hosts,
    tests and packaged builds can use the same local-motion interface when the
    optional JuPedSim dependency is not installed.
    """

    name = "kinematic"

    def __init__(self) -> None:
        self._agents: dict[str, _KinematicState] = {}

    def add_agent(self, spec: MicroscopicAgentSpec) -> None:
        if spec.id in self._agents:
            raise ValueError(f"duplicate microscopic agent id: {spec.id}")
        self._agents[spec.id] = _KinematicState(spec, spec.position_m, spec.target_m)

    def remove_agent(self, agent_id: str) -> None:
        if agent_id not in self._agents:
            raise KeyError(agent_id)
        del self._agents[agent_id]

    def set_target(self, agent_id: str, target_m: Vec3) -> None:
        state = self._agents[agent_id]
        state.target_m = _vec3(target_m)

    def advance(self, delta_seconds: float) -> None:
        dt = max(0.0, float(delta_seconds))
        if dt <= _EPSILON:
            return
        for agent_id in sorted(self._agents):
            state = self._agents[agent_id]
            dx = state.target_m[0] - state.position_m[0]
            dy = state.target_m[1] - state.position_m[1]
            dz = state.target_m[2] - state.position_m[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance <= _EPSILON:
                continue
            horizontal = math.hypot(dx, dy)
            if horizontal > _EPSILON:
                state.forward_xy = (dx / horizontal, dy / horizontal)
            step = max(0.0, state.spec.desired_speed_mps) * dt
            ratio = min(1.0, step / distance)
            state.position_m = (
                state.position_m[0] + dx * ratio,
                state.position_m[1] + dy * ratio,
                state.position_m[2] + dz * ratio,
            )

    def snapshot(self, agent_id: str) -> MicroscopicAgentSnapshot:
        state = self._agents[agent_id]
        return MicroscopicAgentSnapshot(
            id=agent_id,
            position_m=state.position_m,
            forward_xy=state.forward_xy,
            target_m=state.target_m,
        )

    def snapshots(self) -> list[MicroscopicAgentSnapshot]:
        return [self.snapshot(agent_id) for agent_id in sorted(self._agents)]


class JuPedSimLocalMotionBackend:
    """JuPedSim adapter for one connected IFCPath horizontal motion domain.

    The adapter intentionally uses a JuPedSim direct-steering stage. IFCPath owns
    route choice and continuously supplies local targets; JuPedSim owns collision
    avoidance and pedestrian interaction inside the supplied walkable domain.

    ``space_id`` is optional for standalone/legacy level-local use. The hybrid
    building-wide coordinator supplies it deliberately, because JuPedSim requires
    one connected accessible area and semantic spaces separated by walls can be
    disconnected even when they share the same storey. Doors/openings are handled
    as IFCPath-owned transfer gates between those microscopic domains.
    """

    name = "jupedsim"

    def __init__(
        self,
        model: InavModel,
        level_id: str,
        *,
        space_id: str | None = None,
        config: MicroscopicMotionConfig | None = None,
    ) -> None:
        self.model = model
        self.level_id = level_id
        self.space_id = space_id
        self.config = config or MicroscopicMotionConfig()
        if self.config.dt_s <= 0.0:
            raise ValueError("microscopic dt_s must be > 0")

        self.walkable_geometry = build_level_walkable_geometry(
            model,
            level_id,
            space_id=space_id,
        )
        if _geometry_component_count(self.walkable_geometry) > 1:
            domain = f"space {space_id!r}" if space_id else f"level {level_id!r}"
            raise ValueError(
                f"JuPedSim accessible area for {domain} is disconnected; "
                "use a connected semantic motion domain"
            )

        self._jps = _load_jupedsim()
        operational_model = _create_jupedsim_model(self._jps, self.config.model)
        self._simulation = self._jps.Simulation(
            model=operational_model,
            geometry=self.walkable_geometry,
            dt=float(self.config.dt_s),
        )
        self._stage_id = self._simulation.add_direct_steering_stage()
        journey = self._jps.JourneyDescription([self._stage_id])
        self._journey_id = self._simulation.add_journey(journey)
        self._agent_ids: dict[str, int] = {}
        self._targets: dict[str, Vec3] = {}
        self._z_by_agent: dict[str, float] = {}
        self._accumulator_s = 0.0

    @property
    def backend_version(self) -> str:
        build_info = getattr(self._jps, "build_info", None)
        if build_info is not None:
            version = getattr(build_info, "library_version", None)
            if version:
                return str(version)
        version = getattr(self._jps, "__version__", None)
        return str(version) if version else "unknown"

    def add_agent(self, spec: MicroscopicAgentSpec) -> None:
        if spec.id in self._agent_ids:
            raise ValueError(f"duplicate microscopic agent id: {spec.id}")

        radius = max(0.01, float(spec.radius_m or self.config.default_radius_m))
        position_xy = _project_xy_inside(
            self.walkable_geometry,
            spec.position_m[:2],
            max(radius + 1e-4, self.config.target_inset_m),
        )
        target_xy = _project_xy_inside(
            self.walkable_geometry,
            spec.target_m[:2],
            self.config.target_inset_m,
        )
        params_type = _agent_parameters_type(self._jps, self.config.model)
        params = params_type(
            position=position_xy,
            desired_speed=max(0.01, float(spec.desired_speed_mps)),
            radius=radius,
            time_gap=max(0.01, float(spec.time_gap_s or self.config.time_gap_s)),
            journey_id=self._journey_id,
            stage_id=self._stage_id,
        )
        native_id = int(self._simulation.add_agent(params))
        self._simulation.agent(native_id).target = target_xy
        self._agent_ids[spec.id] = native_id
        self._targets[spec.id] = _vec3(spec.target_m)
        self._z_by_agent[spec.id] = float(spec.position_m[2])

    def remove_agent(self, agent_id: str) -> None:
        native_id = self._agent_ids[agent_id]
        marked = bool(self._simulation.mark_agent_for_removal(native_id))
        if not marked:
            raise RuntimeError(f"JuPedSim could not mark agent {agent_id!r} for removal")
        self._agent_ids.pop(agent_id, None)
        self._targets.pop(agent_id, None)
        self._z_by_agent.pop(agent_id, None)

    def set_target(self, agent_id: str, target_m: Vec3) -> None:
        native_id = self._agent_ids[agent_id]
        target = _vec3(target_m)
        target_xy = _project_xy_inside(
            self.walkable_geometry,
            target[:2],
            self.config.target_inset_m,
        )
        self._simulation.agent(native_id).target = target_xy
        self._targets[agent_id] = target

    def advance(self, delta_seconds: float) -> None:
        self._accumulator_s += max(0.0, float(delta_seconds))
        dt = float(self.config.dt_s)
        while self._accumulator_s + _EPSILON >= dt:
            self._simulation.iterate()
            self._accumulator_s -= dt

    def snapshot(self, agent_id: str) -> MicroscopicAgentSnapshot:
        native_id = self._agent_ids[agent_id]
        agent = self._simulation.agent(native_id)
        orientation = tuple(float(value) for value in agent.orientation)
        length = math.hypot(orientation[0], orientation[1])
        forward = (
            (orientation[0] / length, orientation[1] / length)
            if length > _EPSILON
            else (0.0, 1.0)
        )
        position = agent.position
        return MicroscopicAgentSnapshot(
            id=agent_id,
            position_m=(float(position[0]), float(position[1]), self._z_by_agent[agent_id]),
            forward_xy=forward,
            target_m=self._targets[agent_id],
        )

    def snapshots(self) -> list[MicroscopicAgentSnapshot]:
        return [self.snapshot(agent_id) for agent_id in sorted(self._agent_ids)]


def build_level_walkable_geometry(
    model: InavModel,
    level_id: str,
    *,
    space_id: str | None = None,
) -> BaseGeometry:
    """Union IFCPath CDT triangles into a continuous 2D motion domain.

    With ``space_id=None`` this preserves the original whole-level helper used by
    standalone tests/consumers. Hybrid JuPedSim mode supplies a semantic space ID
    so walls and blocked portals remain hard motion-domain boundaries.
    """
    triangles: list[Polygon] = []
    for cell in model.cells:
        if cell.level_id != level_id:
            continue
        if space_id is not None and cell.space_id != space_id:
            continue
        coordinates = [(float(vertex[0]), float(vertex[1])) for vertex in cell.vertices_m]
        polygon = Polygon(coordinates)
        if not polygon.is_empty and polygon.area > _EPSILON:
            triangles.append(polygon)
    if not triangles:
        domain = f"space {space_id!r} on level {level_id!r}" if space_id else f"level {level_id!r}"
        raise ValueError(f"{domain} has no walkable navigation cells")

    geometry = unary_union(triangles)
    if not geometry.is_valid:
        geometry = geometry.buffer(0)
    if geometry.is_empty or geometry.area <= _EPSILON:
        domain = f"space {space_id!r} on level {level_id!r}" if space_id else f"level {level_id!r}"
        raise ValueError(f"{domain} produced empty walkable geometry")
    return geometry


def create_local_motion_backend(
    name: str,
    *,
    model: InavModel | None = None,
    level_id: str | None = None,
    space_id: str | None = None,
    config: MicroscopicMotionConfig | None = None,
) -> LocalMotionBackend:
    """Create a local-motion backend without leaking solver imports to callers."""
    normalized = str(name).strip().lower().replace("-", "_")
    if normalized in {"kinematic", "route", "fallback"}:
        return KinematicLocalMotionBackend()
    if normalized in {"jupedsim", "jps", "microscopic"}:
        if model is None or not level_id:
            raise ValueError("JuPedSim backend requires model and level_id")
        return JuPedSimLocalMotionBackend(
            model,
            level_id,
            space_id=space_id,
            config=config,
        )
    raise ValueError(f"unknown local-motion backend: {name}")


def _create_jupedsim_model(jps, model_name: str):
    normalized = str(model_name).strip().lower().replace("-", "_")
    if normalized in {"cfsm_v2", "collision_free_speed_v2", "v2"}:
        return jps.CollisionFreeSpeedModelV2()
    if normalized in {"cfsm_v3", "collision_free_speed_v3", "v3"}:
        model_type = getattr(jps, "CollisionFreeSpeedModelV3", None)
        if model_type is None:
            raise MicroscopicBackendUnavailable(
                "installed JuPedSim does not provide CollisionFreeSpeedModelV3"
            )
        return model_type()
    raise ValueError(f"unsupported JuPedSim operational model: {model_name}")


def _agent_parameters_type(jps, model_name: str):
    normalized = str(model_name).strip().lower().replace("-", "_")
    if normalized in {"cfsm_v2", "collision_free_speed_v2", "v2"}:
        return jps.CollisionFreeSpeedModelV2AgentParameters
    if normalized in {"cfsm_v3", "collision_free_speed_v3", "v3"}:
        params_type = getattr(jps, "CollisionFreeSpeedModelV3AgentParameters", None)
        if params_type is None:
            raise MicroscopicBackendUnavailable(
                "installed JuPedSim does not provide CollisionFreeSpeedModelV3AgentParameters"
            )
        return params_type
    raise ValueError(f"unsupported JuPedSim operational model: {model_name}")


def _load_jupedsim():
    try:
        import jupedsim as jps
    except ImportError as exc:
        raise MicroscopicBackendUnavailable(
            "JuPedSim backend is optional; install IFCPath with the 'microscopic' extra"
        ) from exc
    return jps


def _geometry_component_count(geometry: BaseGeometry) -> int:
    geoms = getattr(geometry, "geoms", None)
    if geoms is None:
        return 1
    return sum(1 for part in geoms if not part.is_empty and part.area > _EPSILON)


def _project_xy_inside(
    geometry: BaseGeometry,
    xy: tuple[float, float] | list[float],
    inset_m: float,
) -> tuple[float, float]:
    point = Point(float(xy[0]), float(xy[1]))
    inset = max(0.0, float(inset_m))
    safe = geometry.buffer(-inset) if inset > _EPSILON else geometry
    if safe.is_empty:
        safe = geometry
    if safe.covers(point):
        return (float(point.x), float(point.y))
    projected = nearest_points(safe, point)[0]
    return (float(projected.x), float(projected.y))


def _vec3(value: Vec3 | tuple[float, float, float]) -> Vec3:
    return (float(value[0]), float(value[1]), float(value[2]))
