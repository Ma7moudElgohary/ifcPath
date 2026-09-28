from __future__ import annotations

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.evacuation import (
    EvacuationAgentSpec,
    EvacuationConfig,
    EvacuationSimulator,
    spawn_agents,
)
from ifcpath.hierarchical_routing import HierarchicalRouteOptions
from ifcpath.model import InavModel, Level, NavCell, Portal, Space


def _single_space_two_exit_model() -> InavModel:
    model = InavModel(levels=[Level("L1", "Ground", 0.0)])
    space = Space("space:A", "Hall", "L1")
    model.spaces.append(space)
    cdt = build_floor_cdt_navmesh(Polygon([(0, 0), (10, 0), (10, 4), (0, 4)]), 0.0)
    ids = [f"cell:{i}" for i in range(len(cdt.cells))]
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
    model.portals = [
        Portal(
            "exit:left",
            "door",
            (0.5, 2.0, 0.0),
            from_space_id=space.id,
            level_id="L1",
            width_m=1.0,
            is_exit=True,
        ),
        Portal(
            "exit:right",
            "door",
            (9.5, 2.0, 0.0),
            from_space_id=space.id,
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
    # Block the right exit so all occupants must pass the 1 person/s left exit.
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
    # Give each agent the nearest independent exit so capacity does not dominate.
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
