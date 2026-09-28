from __future__ import annotations

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.evacuation import (
    EvacuationAgentSpec,
    EvacuationConfig,
    EvacuationSimulator,
    baseline_egress_space_ids,
    spawn_agents,
)
from ifcpath.hierarchical_routing import HierarchicalRouteOptions
from ifcpath.model import InavModel, Level, NavCell, Portal, Space


def _add_rect_space(
    model: InavModel,
    space_id: str,
    *,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> None:
    space = Space(space_id, space_id, "L1")
    model.spaces.append(space)
    cdt = build_floor_cdt_navmesh(Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)]), 0.0)
    ids = [f"cell:{space_id}:{i}" for i in range(len(cdt.cells))]
    model.cells.extend(
        NavCell(
            id=ids[i],
            vertices_m=cell.vertices,
            space_id=space.id,
            level_id="L1",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for i, cell in enumerate(cdt.cells)
    )


def _single_space_two_exit_model() -> InavModel:
    model = InavModel(levels=[Level("L1", "Ground", 0.0)])
    _add_rect_space(model, "space:A", x0=0.0, y0=0.0, x1=10.0, y1=4.0)
    model.portals = [
        Portal(
            "exit:left",
            "door",
            (0.5, 2.0, 0.0),
            from_space_id="space:A",
            level_id="L1",
            width_m=1.0,
            is_exit=True,
        ),
        Portal(
            "exit:right",
            "door",
            (9.5, 2.0, 0.0),
            from_space_id="space:A",
            level_id="L1",
            width_m=1.0,
            is_exit=True,
        ),
    ]
    return model


def _run_to_completion(simulator: EvacuationSimulator, *, max_s: float = 60.0) -> None:
    while not simulator.finished and simulator.elapsed_s < max_s:
        simulator.advance(0.1)


def test_capacity_aware_exit_assignment_balances_equal_routes() -> None:
    model = _single_space_two_exit_model()
    agents = [
        EvacuationAgentSpec(f"a{i}", (5.0, 2.0, 0.0), 1.2)
        for i in range(4)
    ]
    simulator = EvacuationSimulator(
        model,
        agents,
        config=EvacuationConfig(exit_specific_flow_pps_per_m=1.0),
    )

    assignments = [state.plan.exit_portal_id for state in simulator.agents if state.plan]

    assert assignments.count("exit:left") == 2
    assert assignments.count("exit:right") == 2


def test_exit_flow_capacity_creates_queue_and_separates_departures() -> None:
    model = _single_space_two_exit_model()
    options = HierarchicalRouteOptions(blocked_portals={"exit:right"})
    agents = [EvacuationAgentSpec(f"a{i}", (2.0, 2.0, 0.0), 4.0) for i in range(3)]
    simulator = EvacuationSimulator(
        model,
        agents,
        options=options,
        config=EvacuationConfig(exit_specific_flow_pps_per_m=1.0, internal_step_s=0.05),
    )

    _run_to_completion(simulator)

    times = sorted(state.evacuated_at_s for state in simulator.agents if state.evacuated_at_s is not None)
    assert len(times) == 3
    assert times[1] - times[0] >= 0.90
    assert times[2] - times[1] >= 0.90
    assert simulator.stats.max_queue >= 2
    assert simulator.stats.exit_usage == {"exit:left": 3}


def test_blocked_exit_is_never_assigned() -> None:
    model = _single_space_two_exit_model()
    agents = [EvacuationAgentSpec("a", (5.0, 2.0, 0.0), 1.2)]
    simulator = EvacuationSimulator(
        model,
        agents,
        options=HierarchicalRouteOptions(blocked_portals={"exit:left"}),
    )

    assert simulator.agents[0].plan is not None
    assert simulator.agents[0].plan.exit_portal_id == "exit:right"


def test_no_available_exit_marks_agent_trapped() -> None:
    model = _single_space_two_exit_model()
    agents = [EvacuationAgentSpec("a", (5.0, 2.0, 0.0), 1.2)]
    simulator = EvacuationSimulator(
        model,
        agents,
        options=HierarchicalRouteOptions(blocked_portals={"exit:left", "exit:right"}),
    )

    assert simulator.agents[0].status == "trapped"
    assert simulator.finished
    assert simulator.stats.trapped_agents == 1


def test_heterogeneous_free_speeds_change_arrival_time_without_bottleneck() -> None:
    model = _single_space_two_exit_model()
    agents = [
        EvacuationAgentSpec("fast", (2.0, 2.0, 0.0), 2.0),
        EvacuationAgentSpec("slow", (8.0, 2.0, 0.0), 0.8),
    ]
    simulator = EvacuationSimulator(
        model,
        agents,
        config=EvacuationConfig(exit_specific_flow_pps_per_m=10.0),
    )

    _run_to_completion(simulator)
    by_id = {state.spec.id: state for state in simulator.agents}

    assert by_id["fast"].evacuated_at_s is not None
    assert by_id["slow"].evacuated_at_s is not None
    assert by_id["fast"].evacuated_at_s < by_id["slow"].evacuated_at_s


def test_spawn_agents_is_deterministic_and_uses_walkable_cells() -> None:
    model = _single_space_two_exit_model()

    first = spawn_agents(model, 8, min_speed_mps=0.9, max_speed_mps=1.3, seed=7)
    second = spawn_agents(model, 8, min_speed_mps=0.9, max_speed_mps=1.3, seed=7)

    assert first == second
    assert len(first) == 8
    assert all(0.0 <= spec.start_m[0] <= 10.0 for spec in first)
    assert all(0.0 <= spec.start_m[1] <= 4.0 for spec in first)
    assert all(0.9 <= spec.speed_mps <= 1.3 for spec in first)


def test_automatic_population_excludes_non_egress_navigation_space() -> None:
    model = _single_space_two_exit_model()
    _add_rect_space(model, "space:roof", x0=20.0, y0=0.0, x1=24.0, y1=4.0)

    eligible = baseline_egress_space_ids(model)
    agents = spawn_agents(model, 30, seed=11)

    assert eligible == {"space:A"}
    assert len(agents) == 30
    assert all(spec.start_m[0] < 10.0 for spec in agents)

    # The filter is only for synthetic/demo population. Explicit occupants in
    # a genuinely unreachable space remain visible as trapped safety findings.
    explicit = EvacuationSimulator(
        model,
        [EvacuationAgentSpec("roof-worker", (22.0, 2.0, 0.0), 1.0)],
    )
    assert explicit.agents[0].status == "trapped"


def test_spawn_filter_can_be_disabled_for_diagnostic_populations() -> None:
    model = _single_space_two_exit_model()
    _add_rect_space(model, "space:roof", x0=20.0, y0=0.0, x1=24.0, y1=4.0)

    agents = spawn_agents(model, 40, seed=3, egress_reachable_only=False)

    assert any(spec.start_m[0] > 20.0 for spec in agents)


def test_replan_preserves_evacuated_agents_and_moves_active_origins() -> None:
    model = _single_space_two_exit_model()
    agents = [EvacuationAgentSpec("a", (5.0, 2.0, 0.0), 1.0)]
    simulator = EvacuationSimulator(model, agents)
    simulator.advance(0.5)
    current = simulator.agents[0].position

    simulator.replan(HierarchicalRouteOptions(blocked_portals={"exit:left"}))

    state = simulator.agents[0]
    assert state.status == "moving"
    assert state.position == current
    assert state.plan is not None
    assert state.plan.exit_portal_id == "exit:right"
