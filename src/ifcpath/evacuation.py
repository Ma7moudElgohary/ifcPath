from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .agent_motion import RouteWalker, polyline_length
from .hierarchical_routing import (
    HierarchicalRoute,
    HierarchicalRouteOptions,
    find_hierarchical_path,
)
from .model import InavModel, NavCell, Portal, Vec3

_EPSILON = 1e-9


@dataclass(slots=True)
class EvacuationConfig:
    """Parameters for the deterministic mesoscopic evacuation model.

    Door/exit flow defaults follow the common engineering maximum specific-flow
    relationship of roughly 1.3 persons/s per metre of effective width. They are
    intentionally configurable because real design studies should calibrate them
    to the population, geometry and governing guidance.
    """

    door_specific_flow_pps_per_m: float = 1.3
    exit_specific_flow_pps_per_m: float = 1.3
    default_portal_width_m: float = 0.9
    stair_capacity_pps: float = 1.0
    ramp_capacity_pps: float = 1.2
    other_transition_capacity_pps: float = 1.0
    stair_speed_factor: float = 0.75
    ramp_speed_factor: float = 0.85
    internal_step_s: float = 0.05
    min_spawn_spacing_m: float = 0.45
    max_spawn_attempts_per_agent: int = 80


@dataclass(frozen=True, slots=True)
class EvacuationAgentSpec:
    id: str
    start_m: Vec3
    speed_mps: float = 1.2


@dataclass(frozen=True, slots=True)
class RouteGate:
    id: str
    kind: str
    distance_m: float
    capacity_pps: float
    portal_id: str | None = None
    is_exit: bool = False


@dataclass(frozen=True, slots=True)
class SpeedRegion:
    start_m: float
    end_m: float
    factor: float


@dataclass(slots=True)
class EvacuationPlan:
    route: HierarchicalRoute
    exit_portal_id: str
    gates: list[RouteGate]
    speed_regions: list[SpeedRegion]
    free_travel_time_s: float
    projected_queue_delay_s: float = 0.0


@dataclass(slots=True)
class EvacuationAgentState:
    spec: EvacuationAgentSpec
    plan: EvacuationPlan | None
    walker: RouteWalker
    next_gate_index: int = 0
    waiting_gate_id: str | None = None
    status: str = "moving"
    waiting_time_s: float = 0.0
    evacuated_at_s: float | None = None

    @property
    def position(self) -> Vec3:
        return self.walker.position or self.spec.start_m

    @property
    def forward_xy(self) -> tuple[float, float]:
        return self.walker.forward_xy


@dataclass(slots=True)
class GateState:
    id: str
    kind: str
    capacity_pps: float
    queue: list[str] = field(default_factory=list)
    credit: float = 1.0
    total_served: int = 0
    max_queue: int = 0


@dataclass(frozen=True, slots=True)
class EvacuationStats:
    elapsed_s: float
    total_agents: int
    evacuated_agents: int
    active_agents: int
    waiting_agents: int
    trapped_agents: int
    average_evacuation_time_s: float | None
    clearance_time_s: float | None
    max_queue: int
    exit_usage: dict[str, int]


class EvacuationSimulator:
    """Deterministic multi-agent evacuation over IFCPath hierarchical routes.

    Agents move on exact route polylines while semantic transitions and exits act
    as capacity-constrained gates. This is deliberately *mesoscopic*: it captures
    heterogeneous walking speeds, bottleneck queues, route/exit choice and live
    scenario constraints without pretending to be a full contact/collision crowd
    physics solver.
    """

    def __init__(
        self,
        model: InavModel,
        agents: list[EvacuationAgentSpec],
        *,
        options: HierarchicalRouteOptions | None = None,
        config: EvacuationConfig | None = None,
    ) -> None:
        self.model = model
        self.options = options or HierarchicalRouteOptions()
        self.config = config or EvacuationConfig()
        self.elapsed_s = 0.0
        self._states: dict[str, EvacuationAgentState] = {}
        self._gates: dict[str, GateState] = {}
        self._exit_usage: dict[str, int] = {}
        self._projected_gate_loads: dict[str, int] = {}
        self._historical_max_queue = 0

        for spec in sorted(agents, key=lambda item: item.id):
            plan = self._choose_plan(spec)
            if plan is None:
                state = EvacuationAgentState(
                    spec=spec,
                    plan=None,
                    walker=RouteWalker(speed_mps=max(0.0, spec.speed_mps)),
                    status="trapped",
                )
            else:
                walker = RouteWalker(speed_mps=max(0.0, spec.speed_mps))
                walker.set_route(plan.route.points)
                state = EvacuationAgentState(spec=spec, plan=plan, walker=walker)
                for gate in plan.gates:
                    self._projected_gate_loads[gate.id] = self._projected_gate_loads.get(gate.id, 0) + 1
                    self._ensure_gate(gate)
            self._states[spec.id] = state

    @property
    def agents(self) -> list[EvacuationAgentState]:
        return [self._states[key] for key in sorted(self._states)]

    @property
    def gates(self) -> list[GateState]:
        return [self._gates[key] for key in sorted(self._gates)]

    @property
    def finished(self) -> bool:
        return all(state.status in {"evacuated", "trapped"} for state in self._states.values())

    def advance(self, delta_seconds: float) -> EvacuationStats:
        remaining = max(0.0, float(delta_seconds))
        step_limit = max(0.005, float(self.config.internal_step_s))
        while remaining > _EPSILON and not self.finished:
            step = min(step_limit, remaining)
            self._advance_step(step)
            remaining -= step
        return self.stats

    def replan(self, options: HierarchicalRouteOptions) -> None:
        """Replan all non-evacuated agents from their exact current positions."""
        self.options = options
        self._historical_max_queue = max(
            self._historical_max_queue,
            max((gate.max_queue for gate in self._gates.values()), default=0),
        )
        self._gates.clear()
        self._projected_gate_loads.clear()

        specs: list[EvacuationAgentSpec] = []
        for state in self.agents:
            if state.status == "evacuated":
                continue
            specs.append(
                EvacuationAgentSpec(
                    id=state.spec.id,
                    start_m=state.position,
                    speed_mps=state.spec.speed_mps,
                )
            )

        for spec in sorted(specs, key=lambda item: item.id):
            old = self._states[spec.id]
            plan = self._choose_plan(spec)
            old.spec = spec
            old.plan = plan
            old.waiting_gate_id = None
            old.next_gate_index = 0
            if plan is None:
                old.walker.clear()
                old.status = "trapped"
                continue
            old.walker.speed_mps = max(0.0, spec.speed_mps)
            old.walker.set_route(plan.route.points)
            old.status = "moving"
            for gate in plan.gates:
                self._projected_gate_loads[gate.id] = self._projected_gate_loads.get(gate.id, 0) + 1
                self._ensure_gate(gate)

    @property
    def stats(self) -> EvacuationStats:
        states = list(self._states.values())
        evacuated = [state for state in states if state.status == "evacuated"]
        waiting = [state for state in states if state.status == "waiting"]
        trapped = [state for state in states if state.status == "trapped"]
        active = [state for state in states if state.status in {"moving", "waiting"}]
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

    def _advance_step(self, dt: float) -> None:
        self._accrue_gate_credit(dt)
        self._release_queues()

        for state in self.agents:
            if state.status == "waiting":
                state.waiting_time_s += dt
                continue
            if state.status != "moving" or state.plan is None:
                continue
            self._move_state(state, dt)

        self.elapsed_s += dt

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
            state.waiting_gate_id = None
            crossing = state.plan.gates[state.next_gate_index] if state.plan else None
            state.next_gate_index += 1
            if crossing and crossing.is_exit:
                state.status = "evacuated"
                state.evacuated_at_s = self.elapsed_s
                state.walker.running = False
                self._exit_usage[crossing.portal_id or crossing.id] = (
                    self._exit_usage.get(crossing.portal_id or crossing.id, 0) + 1
                )
            else:
                state.status = "moving"

    def _move_state(self, state: EvacuationAgentState, dt: float) -> None:
        plan = state.plan
        assert plan is not None
        if state.next_gate_index >= len(plan.gates):
            state.status = "evacuated"
            state.evacuated_at_s = self.elapsed_s
            return

        gate = plan.gates[state.next_gate_index]
        current = state.walker.distance_m
        if gate.distance_m <= current + _EPSILON:
            self._enqueue(state, gate)
            return

        factor = _speed_factor_at_distance(plan.speed_regions, current)
        speed = max(0.0, state.spec.speed_mps) * factor
        target = min(state.walker.total_length_m, current + speed * dt)
        if target + _EPSILON >= gate.distance_m:
            state.walker.distance_m = min(gate.distance_m, state.walker.total_length_m)
            self._enqueue(state, gate)
        else:
            state.walker.distance_m = target

    def _enqueue(self, state: EvacuationAgentState, gate: RouteGate) -> None:
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

    def _choose_plan(self, spec: EvacuationAgentSpec) -> EvacuationPlan | None:
        exits = [
            portal
            for portal in self.model.portals
            if portal.is_exit and portal.id not in (self.options.blocked_portals or set())
        ]
        candidates: list[tuple[float, str, EvacuationPlan]] = []
        for exit_portal in exits:
            route = find_hierarchical_path(
                self.model,
                spec.start_m,
                exit_portal.position_m,
                self.options,
            )
            if route is None:
                continue
            plan = _build_plan(self.model, route, exit_portal, spec.speed_mps, self.config)
            projected_delay = sum(
                self._projected_gate_loads.get(gate.id, 0) / max(gate.capacity_pps, 0.01)
                for gate in plan.gates
            )
            plan.projected_queue_delay_s = projected_delay
            score = plan.free_travel_time_s + projected_delay
            candidates.append((score, exit_portal.id, plan))
        if not candidates:
            return None
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]


def spawn_agents(
    model: InavModel,
    count: int,
    *,
    min_speed_mps: float = 0.9,
    max_speed_mps: float = 1.4,
    seed: int = 42,
    config: EvacuationConfig | None = None,
    egress_reachable_only: bool = True,
) -> list[EvacuationAgentSpec]:
    """Create deterministic synthetic occupants on walkable CDT cells.

    Automatic/demo populations default to spaces that have a baseline route to a
    classified exit. IFC navigation models can legitimately contain roofs,
    exterior terraces, service voids or other walkable semantic spaces that are
    not sensible random occupant origins. Explicitly supplied agent specs are not
    filtered, so a genuine unreachable occupied room is still reported as
    ``trapped`` by :class:`EvacuationSimulator`.
    """
    config = config or EvacuationConfig()
    cells = [cell for cell in model.cells if cell.space_id]
    if egress_reachable_only:
        eligible_spaces = baseline_egress_space_ids(model)
        cells = [cell for cell in cells if cell.space_id in eligible_spaces]
    if not cells or count <= 0:
        return []

    rng = random.Random(int(seed))
    lo = min(float(min_speed_mps), float(max_speed_mps))
    hi = max(float(min_speed_mps), float(max_speed_mps))
    points: list[Vec3] = []
    agents: list[EvacuationAgentSpec] = []

    for index in range(int(count)):
        chosen: Vec3 | None = None
        for _attempt in range(max(1, config.max_spawn_attempts_per_agent)):
            cell = cells[rng.randrange(len(cells))]
            candidate = _random_point_in_triangle(cell, rng)
            if all(math.dist(candidate, existing) >= config.min_spawn_spacing_m for existing in points):
                chosen = candidate
                break
        if chosen is None:
            cell = cells[index % len(cells)]
            chosen = _triangle_centroid(cell)
        points.append(chosen)
        speed = lo if abs(hi - lo) <= _EPSILON else rng.uniform(lo, hi)
        agents.append(EvacuationAgentSpec(f"agent:{index + 1:03d}", chosen, speed))
    return agents


def baseline_egress_space_ids(model: InavModel) -> set[str]:
    """Return semantic spaces with a route to at least one baseline exit.

    One representative point per space is sufficient because qualified CDT
    spaces are internally connected (``split_spaces == 0`` is part of the real
    IFC regression). The function deliberately ignores runtime blocked/hazard
    state: it answers whether a space is a sensible *baseline* automatic
    population origin, not whether a later emergency scenario leaves it safe.
    """
    exits = [portal for portal in model.portals if portal.is_exit]
    if not exits:
        return set()

    cells_by_space: dict[str, list[NavCell]] = {}
    for cell in model.cells:
        if cell.space_id:
            cells_by_space.setdefault(cell.space_id, []).append(cell)

    reachable: set[str] = set()
    for space_id in sorted(cells_by_space):
        cells = sorted(cells_by_space[space_id], key=lambda item: item.id)
        representative = _triangle_centroid(cells[0])
        for exit_portal in exits:
            if find_hierarchical_path(model, representative, exit_portal.position_m) is not None:
                reachable.add(space_id)
                break
    return reachable


def _build_plan(
    model: InavModel,
    route: HierarchicalRoute,
    exit_portal: Portal,
    free_speed_mps: float,
    config: EvacuationConfig,
) -> EvacuationPlan:
    distance_cursor = 0.0
    gates: list[RouteGate] = []
    regions: list[SpeedRegion] = []
    travel_time = 0.0

    for segment in route.segments:
        segment_length = polyline_length(segment.points)
        factor = _segment_speed_factor(segment.kind, config)
        regions.append(SpeedRegion(distance_cursor, distance_cursor + segment_length, factor))
        travel_time += segment_length / max(0.05, free_speed_mps * factor)
        if segment.transition_id:
            capacity = _transition_capacity(model, segment.kind, segment.portal_id, config)
            gates.append(
                RouteGate(
                    id=f"transition:{segment.transition_id}",
                    kind=segment.kind,
                    distance_m=distance_cursor,
                    capacity_pps=capacity,
                    portal_id=segment.portal_id,
                )
            )
        distance_cursor += segment_length

    exit_capacity = _portal_capacity(exit_portal, config.exit_specific_flow_pps_per_m, config)
    gates.append(
        RouteGate(
            id=f"exit:{exit_portal.id}",
            kind="exit",
            distance_m=route.length_m,
            capacity_pps=exit_capacity,
            portal_id=exit_portal.id,
            is_exit=True,
        )
    )
    return EvacuationPlan(
        route=route,
        exit_portal_id=exit_portal.id,
        gates=gates,
        speed_regions=regions,
        free_travel_time_s=travel_time,
    )


def _portal_capacity(portal: Portal, specific_flow: float, config: EvacuationConfig) -> float:
    width = portal.width_m if portal.width_m and portal.width_m > 0.0 else config.default_portal_width_m
    return max(0.01, float(specific_flow) * float(width))


def _transition_capacity(
    model: InavModel,
    kind: str,
    portal_id: str | None,
    config: EvacuationConfig,
) -> float:
    if portal_id:
        portal = next((item for item in model.portals if item.id == portal_id), None)
        if portal is not None:
            return _portal_capacity(portal, config.door_specific_flow_pps_per_m, config)
    normalized = kind.lower()
    if normalized in {"stair", "vertical", "escalator"}:
        return max(0.01, config.stair_capacity_pps)
    if normalized == "ramp":
        return max(0.01, config.ramp_capacity_pps)
    return max(0.01, config.other_transition_capacity_pps)


def _segment_speed_factor(kind: str, config: EvacuationConfig) -> float:
    normalized = kind.lower()
    if normalized in {"stair", "vertical", "escalator"}:
        return max(0.05, config.stair_speed_factor)
    if normalized == "ramp":
        return max(0.05, config.ramp_speed_factor)
    return 1.0


def _speed_factor_at_distance(regions: list[SpeedRegion], distance_m: float) -> float:
    for region in regions:
        if region.start_m - _EPSILON <= distance_m < region.end_m - _EPSILON:
            return region.factor
    return regions[-1].factor if regions else 1.0


def _random_point_in_triangle(cell: NavCell, rng: random.Random) -> Vec3:
    a, b, c = cell.vertices_m
    r1 = math.sqrt(rng.random())
    r2 = rng.random()
    wa = 1.0 - r1
    wb = r1 * (1.0 - r2)
    wc = r1 * r2
    return (
        wa * a[0] + wb * b[0] + wc * c[0],
        wa * a[1] + wb * b[1] + wc * c[1],
        wa * a[2] + wb * b[2] + wc * c[2],
    )


def _triangle_centroid(cell: NavCell) -> Vec3:
    a, b, c = cell.vertices_m
    return (
        (a[0] + b[0] + c[0]) / 3.0,
        (a[1] + b[1] + c[1]) / 3.0,
        (a[2] + b[2] + c[2]) / 3.0,
    )
