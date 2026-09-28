from __future__ import annotations

import math

import pytest
from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.microscopic_motion import (
    JuPedSimLocalMotionBackend,
    KinematicLocalMotionBackend,
    MicroscopicAgentSpec,
    MicroscopicMotionConfig,
    build_level_walkable_geometry,
    create_local_motion_backend,
)
from ifcpath.microscopic_route import MicroscopicRouteConfig, MicroscopicRouteController
from ifcpath.model import InavModel, Level, NavCell, Space


def _single_floor_model() -> InavModel:
    model = InavModel(levels=[Level("L1", "Ground", 0.0)])
    model.spaces.append(Space("space:A", "Room A", "L1"))
    cdt = build_floor_cdt_navmesh(
        Polygon([(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (0.0, 4.0)]),
        0.0,
    )
    ids = [f"cell:{index}" for index in range(len(cdt.cells))]
    model.cells.extend(
        NavCell(
            id=ids[index],
            vertices_m=cell.vertices,
            space_id="space:A",
            level_id="L1",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    )
    return model


def test_walkable_geometry_is_reconstructed_from_cdt_cells() -> None:
    geometry = build_level_walkable_geometry(_single_floor_model(), "L1")

    assert geometry.is_valid
    assert geometry.area == pytest.approx(40.0, abs=1e-7)
    assert geometry.covers(Polygon([(1.0, 1.0), (2.0, 1.0), (2.0, 2.0), (1.0, 2.0)]))


def test_kinematic_backend_implements_common_contract_deterministically() -> None:
    backend = create_local_motion_backend("kinematic")
    assert isinstance(backend, KinematicLocalMotionBackend)
    backend.add_agent(
        MicroscopicAgentSpec(
            id="a",
            position_m=(1.0, 2.0, 0.0),
            target_m=(9.0, 2.0, 0.0),
            desired_speed_mps=1.25,
        )
    )

    backend.advance(2.0)
    snapshot = backend.snapshot("a")

    assert snapshot.position_m == pytest.approx((3.5, 2.0, 0.0))
    assert snapshot.forward_xy == pytest.approx((1.0, 0.0))
    assert snapshot.target_m == (9.0, 2.0, 0.0)


def test_kinematic_target_can_change_without_recreating_agent() -> None:
    backend = KinematicLocalMotionBackend()
    backend.add_agent(
        MicroscopicAgentSpec("a", (1.0, 1.0, 0.0), (3.0, 1.0, 0.0), desired_speed_mps=1.0)
    )
    backend.advance(0.5)
    before = backend.snapshot("a").position_m
    backend.set_target("a", (1.5, 3.0, 0.0))
    backend.advance(0.5)
    after = backend.snapshot("a")

    assert before == pytest.approx((1.5, 1.0, 0.0))
    assert after.position_m[1] > before[1]
    assert after.forward_xy[1] > 0.9


def test_route_controller_turns_through_ifcpath_waypoints() -> None:
    controller = MicroscopicRouteController(
        KinematicLocalMotionBackend(),
        config=MicroscopicRouteConfig(update_step_s=0.05, waypoint_tolerance_m=0.08),
    )
    controller.add_agent(
        "a",
        (1.0, 1.0, 0.0),
        [(1.0, 1.0, 0.0), (3.0, 1.0, 0.0), (3.0, 3.0, 0.0)],
        desired_speed_mps=1.0,
    )

    controller.advance(2.2)
    at_turn = controller.snapshot("a")
    assert at_turn.position_m[0] == pytest.approx(3.0, abs=0.09)
    assert at_turn.position_m[1] > 1.0
    assert controller.target_index("a") == 2

    controller.advance(2.0)
    finished = controller.snapshot("a")
    assert controller.finished("a")
    assert finished.position_m == pytest.approx((3.0, 3.0, 0.0), abs=0.09)


def test_route_controller_can_replace_remaining_route_from_current_position() -> None:
    controller = MicroscopicRouteController(KinematicLocalMotionBackend())
    controller.add_agent(
        "a",
        (1.0, 1.0, 0.0),
        [(5.0, 1.0, 0.0)],
        desired_speed_mps=1.0,
    )
    controller.advance(1.0)
    current = controller.snapshot("a").position_m

    controller.replace_remaining_route("a", [current, (2.0, 3.0, 0.0)])
    controller.advance(0.5)
    replanned = controller.snapshot("a")

    assert replanned.position_m[1] > current[1]
    assert replanned.target_m == (2.0, 3.0, 0.0)


def test_jupedsim_direct_steering_moves_toward_ifcpath_target() -> None:
    pytest.importorskip("jupedsim")
    model = _single_floor_model()
    backend = JuPedSimLocalMotionBackend(
        model,
        "L1",
        config=MicroscopicMotionConfig(dt_s=0.05, model="cfsm_v2"),
    )
    backend.add_agent(
        MicroscopicAgentSpec(
            id="a",
            position_m=(1.0, 2.0, 0.0),
            target_m=(9.0, 2.0, 0.0),
            desired_speed_mps=1.2,
            radius_m=0.2,
            time_gap_s=1.0,
        )
    )

    initial = backend.snapshot("a")
    backend.advance(1.0)
    moved = backend.snapshot("a")

    assert moved.position_m[0] > initial.position_m[0] + 0.25
    assert abs(moved.position_m[1] - 2.0) < 0.35
    assert math.dist(moved.position_m, (9.0, 2.0, 0.0)) < math.dist(
        initial.position_m,
        (9.0, 2.0, 0.0),
    )


def test_jupedsim_route_controller_follows_a_turn() -> None:
    pytest.importorskip("jupedsim")
    backend = JuPedSimLocalMotionBackend(
        _single_floor_model(),
        "L1",
        config=MicroscopicMotionConfig(dt_s=0.05, model="cfsm_v2"),
    )
    controller = MicroscopicRouteController(
        backend,
        config=MicroscopicRouteConfig(update_step_s=0.05, waypoint_tolerance_m=0.25),
    )
    controller.add_agent(
        "a",
        (1.0, 1.0, 0.0),
        [(4.0, 1.0, 0.0), (4.0, 3.0, 0.0)],
        desired_speed_mps=1.2,
    )

    controller.advance(5.0)
    snapshot = controller.snapshot("a")

    assert snapshot.position_m[0] > 3.5
    assert snapshot.position_m[1] > 1.8
    assert math.dist(snapshot.position_m, (4.0, 3.0, 0.0)) < 1.2


def test_jupedsim_two_agents_do_not_overlap_in_head_on_flow() -> None:
    pytest.importorskip("jupedsim")
    model = _single_floor_model()
    backend = JuPedSimLocalMotionBackend(
        model,
        "L1",
        config=MicroscopicMotionConfig(dt_s=0.05, model="cfsm_v2"),
    )
    backend.add_agent(
        MicroscopicAgentSpec("left", (3.0, 2.0, 0.0), (8.5, 2.0, 0.0), 1.2, 0.2, 1.0)
    )
    backend.add_agent(
        MicroscopicAgentSpec("right", (7.0, 2.0, 0.0), (1.5, 2.0, 0.0), 1.2, 0.2, 1.0)
    )

    minimum_distance = float("inf")
    for _ in range(100):
        backend.advance(0.05)
        left = backend.snapshot("left").position_m
        right = backend.snapshot("right").position_m
        minimum_distance = min(minimum_distance, math.dist(left, right))

    assert minimum_distance >= 0.38
    assert backend.snapshot("left").position_m[0] > 3.5
    assert backend.snapshot("right").position_m[0] < 6.5
