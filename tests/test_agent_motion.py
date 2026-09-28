from __future__ import annotations

import math

import pytest

from ifcpath.agent_motion import RouteWalker, polyline_length, sample_polyline


def test_polyline_distance_is_true_3d_length() -> None:
    points = [(0.0, 0.0, 0.0), (3.0, 0.0, 4.0), (3.0, 4.0, 4.0)]
    assert polyline_length(points) == pytest.approx(9.0)


def test_sample_polyline_interpolates_on_stair_segment() -> None:
    points = [(0.0, 0.0, 0.0), (3.0, 0.0, 4.0), (3.0, 4.0, 4.0)]
    position, forward = sample_polyline(points, 2.5)

    assert position == pytest.approx((1.5, 0.0, 2.0))
    assert forward == pytest.approx((1.0, 0.0))


def test_route_walker_advances_by_speed_and_time() -> None:
    walker = RouteWalker(speed_mps=1.5)
    walker.set_route([(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)])
    walker.play()

    walker.advance(2.0)

    assert walker.position == pytest.approx((3.0, 0.0, 0.0))
    assert walker.distance_m == pytest.approx(3.0)
    assert walker.progress == pytest.approx(0.5)
    assert walker.running


def test_route_walker_stops_exactly_at_destination() -> None:
    walker = RouteWalker(speed_mps=2.0)
    walker.set_route([(0.0, 0.0, 0.0), (3.0, 0.0, 0.0)])
    walker.play()

    walker.advance(10.0)

    assert walker.position == pytest.approx((3.0, 0.0, 0.0))
    assert walker.distance_m == pytest.approx(3.0)
    assert walker.finished
    assert not walker.running


def test_remaining_route_begins_at_exact_current_position() -> None:
    walker = RouteWalker(speed_mps=1.0)
    walker.set_route(
        [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0), (4.0, 3.0, 0.0)]
    )
    walker.play()
    walker.advance(2.5)

    remaining = walker.remaining_points

    assert remaining[0] == pytest.approx((2.5, 0.0, 0.0))
    assert remaining[1:] == [(4.0, 0.0, 0.0), (4.0, 3.0, 0.0)]
    assert polyline_length(remaining) == pytest.approx(4.5)


def test_forward_direction_turns_with_route() -> None:
    walker = RouteWalker(speed_mps=1.0)
    walker.set_route([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (2.0, 2.0, 0.0)])
    walker.play()
    walker.advance(2.5)

    assert walker.position == pytest.approx((2.0, 0.5, 0.0))
    assert walker.forward_xy == pytest.approx((0.0, 1.0))


def test_vertical_segment_preserves_last_horizontal_facing() -> None:
    points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, 2.0)]
    _position, forward = sample_polyline(points, 2.0)

    assert forward == pytest.approx((1.0, 0.0))
