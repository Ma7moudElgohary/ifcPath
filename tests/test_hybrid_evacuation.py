from __future__ import annotations

import math

import pytest
from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.evacuation import EvacuationAgentSpec, EvacuationConfig
from ifcpath.hierarchical_routing import HierarchicalRouteOptions
from ifcpath.hybrid_evacuation import (
    HybridEvacuationConfig,
    HybridEvacuationSimulator,
    compile_hybrid_route_steps,
)
from ifcpath.microscopic_motion import MicroscopicMotionConfig
from ifcpath.microscopic_route import MicroscopicRouteConfig
from ifcpath.model import InavModel, Level, NavCell, NavEdge, NavNode, Portal, Space


def _add_rect_space(
    model: InavModel,
    space_id: str,
    *,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    z: float,
    level_id: str,
) -> None:
    model.spaces.append(Space(space_id, space_id, level_id))
    cdt = build_floor_cdt_navmesh(
        Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)]),
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


def _two_floor_stair_model() -> InavModel:
    model = InavModel(
        levels=[
            Level("L1", "Ground", 0.0),
            Level("L2", "Upper", 3.0),
        ]
    )
    _add_rect_space(
        model,
        "lower",
        x0=0.0,
        y0=0.0,
        x1=6.0,
        y1=4.0,
        z=0.0,
        level_id="L1",
    )
    _add_rect_space(
        model,
        "upper",
        x0=0.0,
        y0=0.0,
        x1=6.0,
        y1=4.0,
        z=3.0,
        level_id="L2",
    )
    model.nodes = [
        NavNode(
            "lower-landing",
            (3.0, 2.0, 0.0),
            kind="walk",
            level_id="L1",
            space_id="lower",
        ),
        NavNode("stair-0", (3.0, 2.0, 0.2), kind="stair"),
        NavNode("stair-1", (3.0, 2.0, 2.8), kind="stair"),
        NavNode(
            "upper-landing",
            (3.0, 2.0, 3.0),
            kind="walk",
            level_id="L2",
            space_id="upper",
        ),
    ]
    model.edges = [
        NavEdge("lower-landing", "stair-0", 0.2),
        NavEdge("stair-0", "stair-1", 2.6),
        NavEdge("stair-1", "upper-landing", 0.2),
    ]
    model.portals = [
        Portal(
            "exit:ground",
            "door",
            (0.5, 2.0, 0.0),
            from_space_id="lower",
            level_id="L1",
            width_m=1.5,
            is_exit=True,
        )
    ]
    return model


def _same_floor_door_gap_model() -> InavModel:
    model = InavModel(levels=[Level("L1", "Ground", 0.0)])
    # The 0.4 m gap intentionally represents non-walkable wall thickness. The
    # semantic portal provides the only valid crossing between the two CDT domains.
    _add_rect_space(
        model,
        "A",
        x0=0.0,
        y0=0.0,
        x1=5.0,
        y1=4.0,
        z=0.0,
        level_id="L1",
    )
    _add_rect_space(
        model,
        "B",
        x0=5.4,
        y0=0.0,
        x1=10.4,
        y1=4.0,
        z=0.0,
        level_id="L1",
    )
    model.portals = [
        Portal(
            "door:AB",
            "door",
            (5.2, 2.0, 0.0),
            from_space_id="A",
            to_space_id="B",
            level_id="L1",
            width_m=1.0,
        ),
        Portal(
            "exit:B",
            "door",
            (9.9, 2.0, 0.0),
            from_space_id="B",
            level_id="L1",
            width_m=1.2,
            is_exit=True,
        ),
    ]
    return model


def _run_to_completion(
    simulator: HybridEvacuationSimulator,
    *,
    max_s: float = 40.0,
) -> None:
    while not simulator.finished and simulator.elapsed_s < max_s:
        simulator.advance(0.05)


def test_hybrid_route_compiles_gate_before_true_3d_vertical_transfer() -> None:
    model = _two_floor_stair_model()
    simulator = HybridEvacuationSimulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 3.0), 1.2)],
    )
    state = simulator.agents[0]

    assert state.plan is not None
    steps = compile_hybrid_route_steps(model, state.plan, simulator.config)
    transfer_index = next(
        index
        for index, step in enumerate(steps)
        if step.kind == "transfer" and step.transition_kind == "stair"
    )

    assert transfer_index > 0
    assert steps[transfer_index - 1].kind == "gate"
    assert steps[transfer_index - 1].gate is not None
    assert steps[transfer_index - 1].gate.kind == "stair"
    assert max(point[2] for point in steps[transfer_index].points) - min(
        point[2] for point in steps[transfer_index].points
    ) > 2.5


def test_same_floor_door_is_gate_then_explicit_transfer_not_local_solver_jump() -> None:
    model = _same_floor_door_gap_model()
    simulator = HybridEvacuationSimulator(
        model,
        [EvacuationAgentSpec("a", (1.0, 2.0, 0.0), 1.2)],
    )
    state = simulator.agents[0]

    assert state.plan is not None
    steps = compile_hybrid_route_steps(model, state.plan, simulator.config)
    door_transfer_index = next(
        index
        for index, step in enumerate(steps)
        if step.kind == "transfer" and step.transition_id == "transition:door:AB"
    )

    assert steps[door_transfer_index - 1].kind == "gate"
    assert steps[door_transfer_index - 1].gate is not None
    assert steps[door_transfer_index - 1].gate.portal_id == "door:AB"
    xs = [point[0] for point in steps[door_transfer_index].points]
    assert min(xs) <= 5.0
    assert max(xs) >= 5.4


def test_kinematic_hybrid_hands_agent_from_upper_to_lower_level() -> None:
    model = _two_floor_stair_model()
    simulator = HybridEvacuationSimulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 3.0), 1.4)],
        config=EvacuationConfig(
            stair_capacity_pps=4.0,
            exit_specific_flow_pps_per_m=5.0,
        ),
        hybrid_config=HybridEvacuationConfig(local_backend="kinematic"),
    )

    saw_upper = False
    saw_transfer = False
    saw_intermediate_z = False
    saw_lower = False
    while not simulator.finished and simulator.elapsed_s < 30.0:
        simulator.advance(0.05)
        state = simulator.agents[0]
        saw_upper = saw_upper or state.active_level_id == "L2"
        saw_transfer = saw_transfer or state.status == "transfer"
        saw_intermediate_z = saw_intermediate_z or (0.25 < state.position[2] < 2.75)
        saw_lower = saw_lower or state.active_level_id == "L1"

    state = simulator.agents[0]
    assert saw_upper
    assert saw_transfer
    assert saw_intermediate_z
    assert saw_lower
    assert state.status == "evacuated"
    assert state.evacuated_at_s is not None
    assert simulator.stats.exit_usage == {"exit:ground": 1}
    assert set(simulator.controller_levels) == {"L1", "L2"}


def test_vertical_gate_capacity_creates_waiting_between_floor_handoffs() -> None:
    model = _two_floor_stair_model()
    simulator = HybridEvacuationSimulator(
        model,
        [
            EvacuationAgentSpec("a", (5.0, 1.8, 3.0), 2.5),
            EvacuationAgentSpec("b", (5.0, 2.2, 3.0), 2.5),
        ],
        config=EvacuationConfig(
            stair_capacity_pps=0.5,
            exit_specific_flow_pps_per_m=10.0,
        ),
        hybrid_config=HybridEvacuationConfig(local_backend="kinematic"),
    )

    _run_to_completion(simulator)

    assert simulator.stats.evacuated_agents == 2
    assert simulator.stats.max_queue >= 1
    assert max(state.waiting_time_s for state in simulator.agents) >= 1.5


def test_hybrid_replan_uses_exact_live_position_and_new_exit_state() -> None:
    model = InavModel(levels=[Level("L1", "Ground", 0.0)])
    _add_rect_space(
        model,
        "room",
        x0=0.0,
        y0=0.0,
        x1=10.0,
        y1=4.0,
        z=0.0,
        level_id="L1",
    )
    model.portals = [
        Portal(
            "exit:left",
            "door",
            (0.5, 2.0, 0.0),
            from_space_id="room",
            level_id="L1",
            is_exit=True,
        ),
        Portal(
            "exit:right",
            "door",
            (9.5, 2.0, 0.0),
            from_space_id="room",
            level_id="L1",
            is_exit=True,
        ),
    ]
    simulator = HybridEvacuationSimulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 0.0), 1.0)],
        hybrid_config=HybridEvacuationConfig(local_backend="kinematic"),
    )
    assert simulator.agents[0].plan is not None
    assert simulator.agents[0].plan.exit_portal_id == "exit:left"

    simulator.advance(0.5)
    current = simulator.agents[0].position
    simulator.replan(HierarchicalRouteOptions(blocked_portals={"exit:left"}))
    state = simulator.agents[0]

    assert math.dist(state.position, current) < 1e-9
    assert state.plan is not None
    assert state.plan.exit_portal_id == "exit:right"
    _run_to_completion(simulator)
    assert simulator.stats.exit_usage == {"exit:right": 1}


def test_jupedsim_hybrid_crosses_real_vertical_handoff() -> None:
    pytest.importorskip("jupedsim")
    model = _two_floor_stair_model()
    simulator = HybridEvacuationSimulator(
        model,
        [EvacuationAgentSpec("a", (5.0, 2.0, 3.0), 1.3)],
        config=EvacuationConfig(
            stair_capacity_pps=4.0,
            exit_specific_flow_pps_per_m=5.0,
        ),
        hybrid_config=HybridEvacuationConfig(
            local_backend="jupedsim",
            microscopic=MicroscopicMotionConfig(dt_s=0.05, model="cfsm_v2"),
            route=MicroscopicRouteConfig(
                update_step_s=0.05,
                waypoint_tolerance_m=0.22,
            ),
        ),
    )

    saw_transfer = False
    saw_lower_solver = False
    while not simulator.finished and simulator.elapsed_s < 35.0:
        simulator.advance(0.05)
        state = simulator.agents[0]
        saw_transfer = saw_transfer or state.status == "transfer"
        saw_lower_solver = saw_lower_solver or state.active_level_id == "L1"

    assert saw_transfer
    assert saw_lower_solver
    assert simulator.agents[0].status == "evacuated"
    assert simulator.stats.exit_usage == {"exit:ground": 1}


def test_jupedsim_hybrid_crosses_disconnected_same_floor_spaces_through_door() -> None:
    pytest.importorskip("jupedsim")
    model = _same_floor_door_gap_model()
    simulator = HybridEvacuationSimulator(
        model,
        [EvacuationAgentSpec("a", (1.0, 2.0, 0.0), 1.3)],
        config=EvacuationConfig(
            door_specific_flow_pps_per_m=5.0,
            exit_specific_flow_pps_per_m=5.0,
        ),
        hybrid_config=HybridEvacuationConfig(
            local_backend="jupedsim",
            microscopic=MicroscopicMotionConfig(dt_s=0.05, model="cfsm_v2"),
            route=MicroscopicRouteConfig(
                update_step_s=0.05,
                waypoint_tolerance_m=0.20,
            ),
        ),
    )

    saw_door_transfer = False
    crossed_wall_gap = False
    while not simulator.finished and simulator.elapsed_s < 30.0:
        simulator.advance(0.05)
        state = simulator.agents[0]
        if state.status == "transfer":
            saw_door_transfer = True
            crossed_wall_gap = crossed_wall_gap or (5.0 < state.position[0] < 5.4)

    assert saw_door_transfer
    assert crossed_wall_gap
    assert simulator.agents[0].status == "evacuated"
    assert simulator.stats.exit_usage == {"exit:B": 1}
