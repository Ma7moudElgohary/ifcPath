from __future__ import annotations

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.elevator_runtime import ElevatorRuntimeConfig
from ifcpath.evacuation import EvacuationAgentSpec, EvacuationConfig
from ifcpath.hybrid_evacuation import (
    HybridEvacuationConfig,
    HybridEvacuationSimulator,
    compile_hybrid_route_steps,
)
from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space


def _add_rect_space(
    model: InavModel,
    space_id: str,
    level_id: str,
    z: float,
) -> None:
    model.spaces.append(Space(space_id, space_id, level_id))
    cdt = build_floor_cdt_navmesh(
        Polygon([(0.0, 0.0), (6.0, 0.0), (6.0, 4.0), (0.0, 4.0)]),
        z,
    )
    ids = [f"cell:{space_id}:{index}" for index in range(len(cdt.cells))]
    model.cells.extend(
        NavCell(
            id=ids[index],
            vertices_m=cell.vertices,
            space_id=space_id,
            level_id=level_id,
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    )


def _three_floor_elevator_model() -> InavModel:
    model = InavModel(
        levels=[
            Level("L1", "Ground", 0.0),
            Level("L2", "First", 3.0),
            Level("L3", "Second", 6.0),
        ]
    )
    _add_rect_space(model, "S1", "L1", 0.0)
    _add_rect_space(model, "S2", "L2", 3.0)
    _add_rect_space(model, "S3", "L3", 6.0)

    model.nodes = [
        NavNode("landing-1", (3.0, 2.0, 0.0), kind="walk", level_id="L1", space_id="S1"),
        NavNode("elevator-1", (3.0, 2.0, 0.1), kind="elevator"),
        NavNode("elevator-2", (3.0, 2.0, 3.0), kind="elevator"),
        NavNode("elevator-3", (3.0, 2.0, 5.9), kind="elevator"),
        NavNode("landing-2", (3.0, 2.0, 3.0), kind="walk", level_id="L2", space_id="S2"),
        NavNode("landing-3", (3.0, 2.0, 6.0), kind="walk", level_id="L3", space_id="S3"),
    ]
    model.edges = [
        NavEdge("landing-1", "elevator-1", 0.1),
        NavEdge("elevator-1", "elevator-2", 2.9),
        NavEdge("elevator-2", "elevator-3", 2.9),
        NavEdge("elevator-2", "landing-2", 0.1),
        NavEdge("elevator-3", "landing-3", 0.1),
    ]
    model.portals = [
        Portal(
            "exit:ground",
            "door",
            (0.5, 2.0, 0.0),
            from_space_id="S1",
            level_id="L1",
            width_m=1.5,
            is_exit=True,
        )
    ]
    return model


def _simulator(model: InavModel, agents: list[EvacuationAgentSpec]) -> HybridEvacuationSimulator:
    return HybridEvacuationSimulator(
        model,
        agents,
        config=EvacuationConfig(exit_specific_flow_pps_per_m=10.0),
        hybrid_config=HybridEvacuationConfig(
            local_backend="kinematic",
            elevator=ElevatorRuntimeConfig(
                speed_mps=3.0,
                door_dwell_s=0.25,
                capacity_persons=2,
            ),
        ),
    )


def test_three_floor_route_compiles_two_edges_to_one_elevator_resource() -> None:
    model = _three_floor_elevator_model()
    simulator = _simulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 6.0), 1.5)],
    )
    state = simulator.agents[0]
    assert state.plan is not None

    steps = compile_hybrid_route_steps(model, state.plan, simulator.config)
    elevator_steps = [step for step in steps if step.kind == "elevator"]

    assert len(elevator_steps) == 2
    assert {step.resource_id for step in elevator_steps} == {elevator_steps[0].resource_id}
    assert elevator_steps[0].resource_id is not None
    assert all(step.gate is None for step in elevator_steps)
    assert [(step.from_level_id, step.to_level_id) for step in elevator_steps] == [
        ("L3", "L2"),
        ("L2", "L1"),
    ]


def test_hybrid_uses_one_stateful_car_for_chained_three_floor_trip() -> None:
    model = _three_floor_elevator_model()
    simulator = _simulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 6.0), 1.5)],
    )

    saw_waiting = False
    saw_car_motion = False
    saw_mid_level = False
    while not simulator.finished and simulator.elapsed_s < 30.0:
        simulator.advance(0.05)
        state = simulator.agents[0]
        saw_waiting = saw_waiting or state.status == "elevator_waiting"
        saw_car_motion = saw_car_motion or (
            state.status == "elevator" and 0.2 < state.position[2] < 5.8
        )
        saw_mid_level = saw_mid_level or (
            state.active_level_id == "L2" or abs(state.position[2] - 3.0) < 0.05
        )

    state = simulator.agents[0]
    assert saw_waiting  # car starts at the lowest served level and must reposition
    assert saw_car_motion
    assert saw_mid_level
    assert state.status == "evacuated"
    assert simulator.stats.exit_usage == {"exit:ground": 1}
    assert len(simulator.elevator_resource_ids) == 1
    resource_id = simulator.elevator_resource_ids[0]
    assert resource_id.startswith("vertical:elevator:")
    assert simulator.elevator_snapshots[resource_id].current_level_id == "L1"


def test_two_occupants_share_the_same_car_capacity_and_queue() -> None:
    model = _three_floor_elevator_model()
    simulator = _simulator(
        model,
        [
            EvacuationAgentSpec("a", (5.0, 1.8, 6.0), 1.5),
            EvacuationAgentSpec("b", (5.0, 2.2, 6.0), 1.5),
            EvacuationAgentSpec("c", (4.7, 2.0, 6.0), 1.5),
        ],
    )

    maximum_elevator_queue = 0
    while not simulator.finished and simulator.elapsed_s < 60.0:
        simulator.advance(0.05)
        if simulator.elevator_snapshots:
            snapshot = next(iter(simulator.elevator_snapshots.values()))
            maximum_elevator_queue = max(maximum_elevator_queue, len(snapshot.queued_agent_ids))

    assert simulator.stats.evacuated_agents == 3
    assert len(simulator.elevator_resource_ids) == 1
    assert maximum_elevator_queue >= 1
    assert max(state.waiting_time_s for state in simulator.agents) > 0.0


def test_replan_fails_closed_while_agent_is_onboard() -> None:
    model = _three_floor_elevator_model()
    simulator = _simulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 6.0), 2.0)],
    )

    for _ in range(300):
        simulator.advance(0.05)
        if simulator.agents[0].status == "elevator":
            break
    else:
        raise AssertionError("agent never boarded elevator")

    from ifcpath.hierarchical_routing import HierarchicalRouteOptions

    try:
        simulator.replan(HierarchicalRouteOptions())
    except RuntimeError as exc:
        assert "onboard an elevator" in str(exc)
    else:
        raise AssertionError("replan should fail closed while elevator is in use")
