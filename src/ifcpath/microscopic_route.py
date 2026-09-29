from __future__ import annotations

import math
from dataclasses import dataclass

from .microscopic_motion import (
    LocalMotionBackend,
    MicroscopicAgentSnapshot,
    MicroscopicAgentSpec,
)
from .model import Vec3

_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class MicroscopicRouteConfig:
    """Controls how an IFCPath polyline feeds local targets to a crowd solver."""

    update_step_s: float = 0.05
    waypoint_tolerance_m: float = 0.20


@dataclass(slots=True)
class _RouteState:
    points: list[Vec3]
    target_index: int
    finished: bool = False


class MicroscopicRouteController:
    """Drive one local-motion backend from IFCPath route polylines.

    The controller owns tactical waypoint progression but does not calculate
    routes. All agents in the backend are advanced together, preserving the
    interaction semantics of microscopic solvers such as JuPedSim.

    A backend is allowed to project an IFCPath waypoint to its nearest physically
    valid agent-centre target (for example, inward from a wall/door boundary by an
    agent radius). Waypoint completion therefore compares against the backend's
    effective target, not blindly against the unprojected route vertex.
    """

    def __init__(
        self,
        backend: LocalMotionBackend,
        *,
        config: MicroscopicRouteConfig | None = None,
    ) -> None:
        self.backend = backend
        self.config = config or MicroscopicRouteConfig()
        if self.config.update_step_s <= 0.0:
            raise ValueError("route update_step_s must be > 0")
        if self.config.waypoint_tolerance_m <= 0.0:
            raise ValueError("route waypoint_tolerance_m must be > 0")
        self._routes: dict[str, _RouteState] = {}

    def add_agent(
        self,
        agent_id: str,
        position_m: Vec3,
        route_points: list[Vec3] | tuple[Vec3, ...],
        *,
        desired_speed_mps: float = 1.2,
        radius_m: float = 0.20,
        time_gap_s: float = 1.0,
    ) -> None:
        if agent_id in self._routes:
            raise ValueError(f"duplicate microscopic route agent id: {agent_id}")
        points = _dedupe_route(position_m, route_points)
        target_index = _first_target_index(position_m, points, self.config.waypoint_tolerance_m)
        if target_index >= len(points):
            target = _vec3(position_m)
            state = _RouteState(points=points or [target], target_index=len(points), finished=True)
        else:
            target = points[target_index]
            state = _RouteState(points=points, target_index=target_index)

        self.backend.add_agent(
            MicroscopicAgentSpec(
                id=agent_id,
                position_m=_vec3(position_m),
                target_m=target,
                desired_speed_mps=float(desired_speed_mps),
                radius_m=float(radius_m),
                time_gap_s=float(time_gap_s),
            )
        )
        self._routes[agent_id] = state

    def remove_agent(self, agent_id: str) -> None:
        """Remove an agent so a higher-level coordinator can hand it elsewhere."""
        if agent_id not in self._routes:
            raise KeyError(agent_id)
        self.backend.remove_agent(agent_id)
        del self._routes[agent_id]

    def replace_remaining_route(
        self,
        agent_id: str,
        route_points: list[Vec3] | tuple[Vec3, ...],
    ) -> None:
        """Replace an active route from the agent's exact current position."""
        current = self.backend.snapshot(agent_id).position_m
        points = _dedupe_route(current, route_points)
        target_index = _first_target_index(current, points, self.config.waypoint_tolerance_m)
        state = self._routes[agent_id]
        state.points = points
        state.target_index = target_index
        state.finished = target_index >= len(points)
        target = current if state.finished else points[target_index]
        self.backend.set_target(agent_id, target)

    def advance(self, delta_seconds: float) -> None:
        remaining = max(0.0, float(delta_seconds))
        step_limit = float(self.config.update_step_s)
        while remaining > _EPSILON:
            step = min(step_limit, remaining)
            self.backend.advance(step)
            self._update_waypoints()
            remaining -= step

    def snapshot(self, agent_id: str) -> MicroscopicAgentSnapshot:
        return self.backend.snapshot(agent_id)

    def snapshots(self) -> list[MicroscopicAgentSnapshot]:
        return self.backend.snapshots()

    def finished(self, agent_id: str) -> bool:
        return self._routes[agent_id].finished

    @property
    def all_finished(self) -> bool:
        return bool(self._routes) and all(state.finished for state in self._routes.values())

    def target_index(self, agent_id: str) -> int:
        return self._routes[agent_id].target_index

    def _update_waypoints(self) -> None:
        tolerance = float(self.config.waypoint_tolerance_m)
        for agent_id in sorted(self._routes):
            state = self._routes[agent_id]
            if state.finished:
                continue

            snapshot = self.backend.snapshot(agent_id)
            while state.target_index < len(state.points):
                # The backend may project a route vertex inward from an obstacle or
                # domain boundary. Its snapshot exposes that effective target.
                if math.dist(snapshot.position_m, snapshot.target_m) > tolerance:
                    break

                state.target_index += 1
                if state.target_index < len(state.points):
                    self.backend.set_target(agent_id, state.points[state.target_index])
                    snapshot = self.backend.snapshot(agent_id)

            if state.target_index >= len(state.points):
                state.finished = True
                self.backend.set_target(agent_id, snapshot.position_m)


def _first_target_index(position_m: Vec3, points: list[Vec3], tolerance_m: float) -> int:
    index = 0
    while index < len(points) and math.dist(position_m, points[index]) <= tolerance_m:
        index += 1
    return index


def _dedupe_route(
    position_m: Vec3,
    route_points: list[Vec3] | tuple[Vec3, ...],
) -> list[Vec3]:
    result: list[Vec3] = []
    for raw in route_points:
        point = _vec3(raw)
        if not result or math.dist(result[-1], point) > _EPSILON:
            result.append(point)
    if result and math.dist(_vec3(position_m), result[0]) <= _EPSILON:
        return result
    return [_vec3(position_m), *result]


def _vec3(value: Vec3 | tuple[float, float, float]) -> Vec3:
    return (float(value[0]), float(value[1]), float(value[2]))
