from __future__ import annotations

import math

import pytest

from ifcpath.elevator_runtime import ElevatorDispatcher, ElevatorRuntimeConfig


def test_single_trip_has_boarding_travel_and_alighting_phases() -> None:
    dispatcher = ElevatorDispatcher(
        {"L1": 0.0, "L2": 3.0},
        initial_level_id="L1",
        config=ElevatorRuntimeConfig(speed_mps=1.5, door_dwell_s=1.0, capacity_persons=4),
    )
    dispatcher.enqueue("a", "L1", "L2")

    assert dispatcher.snapshot().phase == "boarding"
    dispatcher.advance(1.0)
    assert dispatcher.snapshot().phase == "travel"
    assert dispatcher.snapshot().current_level_id is None

    dispatcher.advance(1.0)
    mid = dispatcher.snapshot()
    assert mid.phase == "travel"
    assert math.isclose(mid.car_z_m, 1.5, abs_tol=1e-9)

    dispatcher.advance(1.0)
    assert dispatcher.snapshot().phase == "alighting"
    assert dispatcher.snapshot().current_level_id == "L2"

    dispatcher.advance(1.0)
    final = dispatcher.snapshot()
    assert final.phase == "idle"
    assert final.completed_agent_ids == ("a",)
    assert math.isclose(final.car_z_m, 3.0, abs_tol=1e-9)


def test_same_origin_destination_requests_batch_up_to_capacity() -> None:
    dispatcher = ElevatorDispatcher(
        {"L1": 0.0, "L2": 3.0},
        initial_level_id="L1",
        config=ElevatorRuntimeConfig(speed_mps=3.0, door_dwell_s=0.5, capacity_persons=2),
    )
    dispatcher.enqueue("a", "L1", "L2")
    dispatcher.enqueue("b", "L1", "L2")
    dispatcher.enqueue("c", "L1", "L2")

    assert dispatcher.onboard_agent_ids == ("a",)
    # The first request starts boarding immediately. A later compatible request
    # waits for the next car cycle rather than changing an already locked batch.
    assert dispatcher.queued_agent_ids == ("b", "c")

    dispatcher.advance(2.0)
    assert dispatcher.completed_agent_ids == ("a",)
    assert dispatcher.snapshot().phase == "reposition"

    dispatcher.advance(1.0)
    assert dispatcher.snapshot().phase == "boarding"
    assert dispatcher.onboard_agent_ids == ("b", "c")

    dispatcher.advance(2.0)
    assert dispatcher.completed_agent_ids == ("a", "b", "c")
    assert dispatcher.idle


def test_empty_car_repositions_before_boarding_remote_request() -> None:
    dispatcher = ElevatorDispatcher(
        {"L1": 0.0, "L2": 3.0, "L3": 6.0},
        initial_level_id="L1",
        config=ElevatorRuntimeConfig(speed_mps=3.0, door_dwell_s=0.5, capacity_persons=4),
    )
    dispatcher.enqueue("a", "L3", "L1")

    assert dispatcher.snapshot().phase == "reposition"
    dispatcher.advance(1.0)
    assert math.isclose(dispatcher.snapshot().car_z_m, 3.0, abs_tol=1e-9)
    dispatcher.advance(1.0)
    assert dispatcher.snapshot().phase == "boarding"
    assert dispatcher.snapshot().current_level_id == "L3"

    dispatcher.advance(0.5)
    assert dispatcher.snapshot().phase == "travel"
    dispatcher.advance(2.0)
    assert dispatcher.snapshot().phase == "alighting"
    assert dispatcher.snapshot().current_level_id == "L1"


def test_cancel_only_removes_waiting_request() -> None:
    dispatcher = ElevatorDispatcher(
        {"L1": 0.0, "L2": 3.0},
        config=ElevatorRuntimeConfig(speed_mps=1.5, door_dwell_s=1.0, capacity_persons=1),
    )
    dispatcher.enqueue("a", "L1", "L2")
    dispatcher.enqueue("b", "L1", "L2")

    assert not dispatcher.cancel("a")
    assert dispatcher.cancel("b")
    assert dispatcher.queued_agent_ids == ()
    assert dispatcher.onboard_agent_ids == ("a",)


def test_completed_events_can_be_drained_without_affecting_car_state() -> None:
    dispatcher = ElevatorDispatcher(
        {"L1": 0.0, "L2": 3.0},
        config=ElevatorRuntimeConfig(speed_mps=3.0, door_dwell_s=0.0, capacity_persons=2),
    )
    dispatcher.enqueue("a", "L1", "L2")
    dispatcher.advance(1.0)

    assert dispatcher.pop_completed() == ("a",)
    assert dispatcher.pop_completed() == ()
    assert dispatcher.idle
    assert dispatcher.snapshot().current_level_id == "L2"


def test_invalid_requests_fail_closed() -> None:
    with pytest.raises(ValueError):
        ElevatorDispatcher({}, config=ElevatorRuntimeConfig())
    with pytest.raises(ValueError):
        ElevatorDispatcher({"L1": 0.0}, config=ElevatorRuntimeConfig(speed_mps=0.0))

    dispatcher = ElevatorDispatcher({"L1": 0.0, "L2": 3.0})
    with pytest.raises(ValueError):
        dispatcher.enqueue("a", "missing", "L2")
    with pytest.raises(ValueError):
        dispatcher.enqueue("a", "L1", "L1")


def test_large_time_step_preserves_deterministic_phase_order() -> None:
    dispatcher = ElevatorDispatcher(
        {"L1": 0.0, "L2": 3.0},
        initial_level_id="L1",
        config=ElevatorRuntimeConfig(speed_mps=3.0, door_dwell_s=0.5, capacity_persons=1),
    )
    dispatcher.enqueue("a", "L1", "L2")
    dispatcher.enqueue("b", "L2", "L1")

    dispatcher.advance(3.0)

    assert dispatcher.completed_agent_ids == ("a", "b")
    assert dispatcher.idle
    assert dispatcher.snapshot().current_level_id == "L1"
