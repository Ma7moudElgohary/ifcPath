from __future__ import annotations

import math
from dataclasses import dataclass, field

from .agent_motion import RouteWalker, polyline_length
from .evacuation import (
    EvacuationAgentSpec,
    EvacuationConfig,
    EvacuationPlan,
    EvacuationSimulator,
    EvacuationStats,
    GateState,
    RouteGate,
)
from .hierarchical_routing import HierarchicalRouteOptions, HierarchicalRouteSegment
from .microscopic_motion import MicroscopicMotionConfig, create_local_motion_backend
from .microscopic_route import MicroscopicRouteConfig, MicroscopicRouteController
from .model import InavModel, Vec3

_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class HybridEvacuationConfig:
    """Settings for hybrid building-wide microscopic evacuation.

    Local movement inside a semantic space is delegated to a microscopic backend.
    IFCPath keeps ownership of semantic transitions, bottleneck queues and the
    exact transfer polylines between spaces and levels.
    """

    local_backend: str = "kinematic"
    microscopic: MicroscopicMotionConfig = field(default_factory=MicroscopicMotionConfig)
    route: MicroscopicRouteConfig = field(default_factory=MicroscopicRouteConfig)


@dataclass(frozen=True, slots=True)
class HybridRouteStep:
    """One executable step of a hierarchical evacuation plan."""

    kind: str
    points: tuple[Vec3, ...] = ()
    level_id: str | None = None
    space_id: str | None = None
    gate: RouteGate | None = None
    speed_factor: float = 1.0
    transition_id: str | None = None
    transition_kind: str | None = None


@dataclass(slots=True)
class HybridEvacuationAgentState:
    spec: EvacuationAgentSpec
    plan: EvacuationPlan | None
    steps: list[HybridRouteStep]
    position: Vec3
    forward_xy: tuple[float, float] = (0.0, 1.0)
    step_index: int = 0
    waiting_gate_id: str | None = None
    status: str = "moving"
    waiting_time_s: float = 0.0
    evacuated_at_s: float | None = None
    active_level_id: str | None = None
    active_space_id: str | None = None
    transfer_walker: RouteWalker | None = None


class HybridEvacuationSimulator:
    """Building-wide evacuation with semantic-domain microscopic motion.

    IFCPath remains the strategic/tactical authority. It selects hierarchical
    routes and exits, applies runtime blocked/hazard state and enforces semantic
    transition capacities. Pedestrian interaction inside a connected semantic
    movement domain is handled by a shared local-motion backend.

    Every semantic transition is an explicit handoff:

    local domain -> capacity gate -> IFCPath transfer polyline -> local domain

    Hybrid JuPedSim partitions horizontal motion by semantic space rather than
    blindly unioning a whole storey. This matches JuPedSim's connected-accessible-
    area requirement and, more importantly, makes walls and blocked portals hard
    domain boundaries. A solver therefore cannot bypass IFCPath door policy.

    ``local_backend='kinematic'`` is the dependency-free deterministic fallback.
    ``local_backend='jupedsim'`` uses the optional JuPedSim operational backend.
    """

    def __init__(
        self,
        model: InavModel,
        agents: list[EvacuationAgentSpec],
        *,
        options: HierarchicalRouteOptions | None = None,
        config: EvacuationConfig | None = None,
        hybrid_config: HybridEvacuationConfig | None = None,
    ) -> None:
        self.model = model
        self.options = options or HierarchicalRouteOptions()
        self.config = config or EvacuationConfig()
        self.hybrid_config = hybrid_config or HybridEvacuationConfig()
        self.elapsed_s = 0.0
        self._states: dict[str, HybridEvacuationAgentState] = {}
        self._gates: dict[str, GateState] = {}
        self._controllers: dict[tuple[str, str | None], MicroscopicRouteController] = {}
        self._exit_usage: dict[str, int] = {}
        self._historical_max_queue = 0

        planned = self._strategic_plans(agents)
        for spec in sorted(agents, key=lambda item: item.id):
            plan = planned.get(spec.id)
            if plan is None:
                state = HybridEvacuationAgentState(
                    spec=spec,
                    plan=None,
                    steps=[],
                    position=_vec3(spec.start_m),
                    status="trapped",
                )
            else:
                steps = compile_hybrid_route_steps(self.model, plan, self.config)
                state = HybridEvacuationAgentState(
                    spec=spec,
                    plan=plan,
                    steps=steps,
                    position=_vec3(spec.start_m),
                )
                for step in steps:
                    if step.gate is not None:
                        self._ensure_gate(step.gate)
            self._states[spec.id] = state

        for state in self.agents:
            if state.plan is not None:
                self._start_current_step(state)

    @property
    def agents(self) -> list[HybridEvacuationAgentState]:
        return [self._states[key] for key in sorted(self._states)]

    @property
    def gates(self) -> list[GateState]:
        return [self._gates[key] for key in sorted(self._gates)]

    @property
    def controller_levels(self) -> tuple[str, ...]:
        return tuple(sorted({level_id for level_id, _space_id in self._controllers}))

    @property
    def controller_domains(self) -> tuple[tuple[str, str | None], ...]:
        return tuple(sorted(self._controllers, key=lambda item: (item[0], item[1] or "")))

    @property
    def finished(self) -> bool:
        return all(state.status in {"evacuated", "trapped"} for state in self._states.values())

    @property
    def stats(self) -> EvacuationStats:
        states = list(self._states.values())
        evacuated = [state for state in states if state.status == "evacuated"]
        waiting = [state for state in states if state.status == "waiting"]
        trapped = [state for state in states if state.status == "trapped"]
        active = [
            state
            for state in states
            if state.status in {"moving", "transfer", "waiting"}
        ]
        times = [state.evacuated_at_s for state in evacuated if state.evacuated_at_s is not None]
        clearance = None
        if states and len(evacuated) + len(trapped) == len(states) and not active:
            clearance = max(times) if times else self.elapsed_s
        current_max_queue = max((gate.max_queue for gate in self._gates.values()), default=0)
        return EvacuationStats(
            elapsed_s=self.elapsed_s,
            total_agents=len(states),
            evacuated_agents=len(evacuated),
            active_agents=len(active),
            waiting_agents=len(waiting),
            trapped_agents=len(trapped),
            average_evacuation_time_s=(sum(times) / len(times)) if times else None,
            clearance_time_s=clearance,
            max_queue=max(self._historical_max_queue, current_max_queue),
            exit_usage=dict(sorted(self._exit_usage.items())),
        )

    def advance(self, delta_seconds: float) -> EvacuationStats:
        remaining = max(0.0, float(delta_seconds))
        step_limit = max(0.005, float(self.config.internal_step_s))
        while remaining > _EPSILON and not self.finished:
            step = min(step_limit, remaining)
            self._advance_step(step)
            remaining -= step
        return self.stats

    def replan(self, options: HierarchicalRouteOptions) -> None:
        """Rebuild active plans from exact live positions under new scenario state."""
        self.options = options
        self._historical_max_queue = max(
            self._historical_max_queue,
            max((gate.max_queue for gate in self._gates.values()), default=0),
        )

        active_specs: list[EvacuationAgentSpec] = []
        for state in self.agents:
            if state.status == "evacuated":
                continue
            active_specs.append(
                EvacuationAgentSpec(
                    id=state.spec.id,
                    start_m=state.position,
                    speed_mps=state.spec.speed_mps,
                )
            )

        # Rebuilding the domain simulations avoids stale native solver agents when
        # a live scenario changes while they are mid-route.
        self._controllers.clear()
        self._gates.clear()
        planned = self._strategic_plans(active_specs)

        for spec in sorted(active_specs, key=lambda item: item.id):
            state = self._states[spec.id]
            plan = planned.get(spec.id)
            state.spec = spec
            state.plan = plan
            state.steps = [] if plan is None else compile_hybrid_route_steps(
                self.model, plan, self.config
            )
            state.step_index = 0
            state.waiting_gate_id = None
            state.active_level_id = None
            state.active_space_id = None
            state.transfer_walker = None
            if plan is None:
                state.status = "trapped"
                continue
            state.status = "moving"
            for step in state.steps:
                if step.gate is not None:
                    self._ensure_gate(step.gate)

        for state in self.agents:
            if state.status not in {"evacuated", "trapped"}:
                self._start_current_step(state)

    def _strategic_plans(
        self,
        specs: list[EvacuationAgentSpec],
    ) -> dict[str, EvacuationPlan | None]:
        """Reuse the qualified mesoscopic planner for exit/route assignment only."""
        planner = EvacuationSimulator(
            self.model,
            specs,
            options=self.options,
            config=self.config,
        )
        return {state.spec.id: state.plan for state in planner.agents}

    def _advance_step(self, dt: float) -> None:
        self._accrue_gate_credit(dt)
        self._release_queues()

        # Each connected semantic domain advances once per tick so all occupants
        # inside that domain interact in the same operational simulation.
        for domain in sorted(self._controllers, key=lambda item: (item[0], item[1] or "")):
            self._controllers[domain].advance(dt)

        for state in self.agents:
            if state.status == "waiting":
                state.waiting_time_s += dt
                continue
            if state.status == "moving":
                self._update_local_state(state)
            elif state.status == "transfer":
                self._update_transfer_state(state, dt)

        self.elapsed_s += dt

    def _update_local_state(self, state: HybridEvacuationAgentState) -> None:
        level_id = state.active_level_id
        if not level_id:
            state.status = "trapped"
            return
        domain = (level_id, state.active_space_id)
        controller = self._controllers[domain]
        snapshot = controller.snapshot(state.spec.id)
        state.position = snapshot.position_m
        state.forward_xy = snapshot.forward_xy
        if not controller.finished(state.spec.id):
            return

        controller.remove_agent(state.spec.id)
        state.active_level_id = None
        state.active_space_id = None
        state.step_index += 1
        self._start_current_step(state)

    def _update_transfer_state(
        self,
        state: HybridEvacuationAgentState,
        dt: float,
    ) -> None:
        walker = state.transfer_walker
        if walker is None:
            state.status = "trapped"
            return
        walker.advance(dt)
        if walker.position is not None:
            state.position = walker.position
            state.forward_xy = walker.forward_xy
        if not walker.finished:
            return

        state.transfer_walker = None
        state.step_index += 1
        self._start_current_step(state)

    def _start_current_step(self, state: HybridEvacuationAgentState) -> None:
        while state.step_index < len(state.steps):
            step = state.steps[state.step_index]
            if step.kind == "gate":
                assert step.gate is not None
                self._enqueue(state, step.gate)
                return

            if step.kind == "level":
                if not step.level_id:
                    state.status = "trapped"
                    return
                route_points = _route_from_position(state.position, step.points)
                if polyline_length(route_points) <= _EPSILON:
                    if route_points:
                        state.position = route_points[-1]
                    state.step_index += 1
                    continue
                controller = self._controller_for(step.level_id, step.space_id)
                controller.add_agent(
                    state.spec.id,
                    state.position,
                    route_points,
                    desired_speed_mps=max(0.01, state.spec.speed_mps),
                    radius_m=max(0.01, self.hybrid_config.microscopic.default_radius_m),
                    time_gap_s=max(0.01, self.hybrid_config.microscopic.time_gap_s),
                )
                state.active_level_id = step.level_id
                state.active_space_id = step.space_id
                state.status = "moving"
                return

            if step.kind == "transfer":
                route_points = _route_from_position(state.position, step.points)
                if polyline_length(route_points) <= _EPSILON:
                    if route_points:
                        state.position = route_points[-1]
                    state.step_index += 1
                    continue
                walker = RouteWalker(
                    speed_mps=max(0.01, state.spec.speed_mps * max(0.05, step.speed_factor))
                )
                walker.set_route(route_points, running=True)
                walker.play()
                state.transfer_walker = walker
                state.status = "transfer"
                return

            raise ValueError(f"unknown hybrid evacuation step kind: {step.kind}")

        # A valid plan always ends with an exit gate. Reaching the end without
        # one should not silently count as a successful evacuation.
        state.status = "trapped"

    def _controller_for(
        self,
        level_id: str,
        space_id: str | None,
    ) -> MicroscopicRouteController:
        domain = (level_id, space_id)
        existing = self._controllers.get(domain)
        if existing is not None:
            return existing
        backend = create_local_motion_backend(
            self.hybrid_config.local_backend,
            model=self.model,
            level_id=level_id,
            space_id=space_id,
            config=self.hybrid_config.microscopic,
        )
        controller = MicroscopicRouteController(
            backend,
            config=self.hybrid_config.route,
        )
        self._controllers[domain] = controller
        return controller

    def _accrue_gate_credit(self, dt: float) -> None:
        for gate in self._gates.values():
            gate.credit = min(1.0, gate.credit + max(0.0, gate.capacity_pps) * dt)

    def _release_queues(self) -> None:
        for gate in self.gates:
            if not gate.queue or gate.credit + _EPSILON < 1.0:
                continue
            agent_id = gate.queue.pop(0)
            gate.credit = max(0.0, gate.credit - 1.0)
            gate.total_served += 1
            state = self._states[agent_id]
            crossing = state.steps[state.step_index].gate
            state.waiting_gate_id = None
            state.step_index += 1
            if crossing is not None and crossing.is_exit:
                state.status = "evacuated"
                state.evacuated_at_s = self.elapsed_s
                key = crossing.portal_id or crossing.id
                self._exit_usage[key] = self._exit_usage.get(key, 0) + 1
            else:
                self._start_current_step(state)

    def _enqueue(self, state: HybridEvacuationAgentState, gate: RouteGate) -> None:
        runtime = self._ensure_gate(gate)
        if state.spec.id not in runtime.queue:
            runtime.queue.append(state.spec.id)
            runtime.max_queue = max(runtime.max_queue, len(runtime.queue))
        state.waiting_gate_id = gate.id
        state.status = "waiting"

    def _ensure_gate(self, gate: RouteGate) -> GateState:
        existing = self._gates.get(gate.id)
        if existing is not None:
            return existing
        runtime = GateState(
            id=gate.id,
            kind=gate.kind,
            capacity_pps=max(0.01, gate.capacity_pps),
        )
        self._gates[gate.id] = runtime
        return runtime


def compile_hybrid_route_steps(
    model: InavModel,
    plan: EvacuationPlan,
    config: EvacuationConfig,
) -> list[HybridRouteStep]:
    """Compile route segments into local movement, gates and semantic handoffs.

    Any segment carrying ``transition_id`` is kept out of the local-motion solver.
    That includes same-floor doors/openings as well as true vertical circulation.
    The transition gate is serviced first, then IFCPath advances the exact transfer
    polyline, then the agent is inserted into the destination semantic domain.
    """
    transition_gates: dict[str, RouteGate] = {}
    exit_gate: RouteGate | None = None
    for gate in plan.gates:
        if gate.is_exit:
            exit_gate = gate
        elif gate.id.startswith("transition:"):
            transition_gates[gate.id[len("transition:") :]] = gate

    steps: list[HybridRouteStep] = []
    for segment in plan.route.segments:
        if segment.transition_id:
            gate = transition_gates.get(segment.transition_id)
            if gate is None:
                raise ValueError(
                    f"route transition {segment.transition_id!r} has no evacuation gate"
                )
            steps.append(
                HybridRouteStep(
                    kind="gate",
                    gate=gate,
                    transition_id=segment.transition_id,
                    transition_kind=segment.kind,
                )
            )
            if segment.points:
                steps.append(
                    HybridRouteStep(
                        kind="transfer",
                        points=tuple(_vec3(point) for point in segment.points),
                        speed_factor=_segment_speed_factor(segment.kind, config),
                        transition_id=segment.transition_id,
                        transition_kind=segment.kind,
                    )
                )
            continue

        if not segment.points:
            continue
        level_id = _segment_level_id(model, segment)
        space_id = _segment_space_id(segment)
        steps.append(
            HybridRouteStep(
                kind="level",
                points=tuple(_vec3(point) for point in segment.points),
                level_id=level_id,
                space_id=space_id,
                speed_factor=1.0,
            )
        )

    if exit_gate is None:
        raise ValueError("hybrid evacuation plan has no exit gate")
    steps.append(HybridRouteStep(kind="gate", gate=exit_gate))
    return steps


def _segment_level_id(model: InavModel, segment: HierarchicalRouteSegment) -> str | None:
    levels = {space.id: space.level_id for space in model.spaces}
    for space_id in (segment.to_space_id, segment.from_space_id):
        if space_id and levels.get(space_id):
            return levels[space_id]
    if segment.points:
        z = sum(point[2] for point in segment.points) / len(segment.points)
        if model.levels:
            return min(model.levels, key=lambda level: abs(level.elevation_m - z)).id
    return None


def _segment_space_id(segment: HierarchicalRouteSegment) -> str | None:
    from_space = segment.from_space_id
    to_space = segment.to_space_id
    if from_space and to_space and from_space != to_space:
        raise ValueError(
            "non-transition hierarchical segment crosses semantic spaces: "
            f"{from_space!r} -> {to_space!r}"
        )
    return from_space or to_space


def _segment_speed_factor(kind: str, config: EvacuationConfig) -> float:
    normalized = kind.lower()
    if normalized in {"stair", "vertical", "escalator", "elevator"}:
        return max(0.05, config.stair_speed_factor)
    if normalized == "ramp":
        return max(0.05, config.ramp_speed_factor)
    return 1.0


def _route_from_position(position: Vec3, points: tuple[Vec3, ...]) -> list[Vec3]:
    result = [_vec3(position)]
    for point in points:
        value = _vec3(point)
        if math.dist(result[-1], value) > _EPSILON:
            result.append(value)
    return result


def _vec3(value: Vec3 | tuple[float, float, float]) -> Vec3:
    return (float(value[0]), float(value[1]), float(value[2]))
