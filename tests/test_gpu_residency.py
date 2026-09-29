from __future__ import annotations

from ifcpath.ui.gpu_residency import GpuResidentSet, ResidentBatchInfo


def _batch(key: str, size: int, x: float) -> ResidentBatchInfo:
    return ResidentBatchInfo(key=key, byte_size=size, center=(x, 0.0, 0.0))


def test_visible_batches_win_memory_budget_by_camera_distance() -> None:
    manager = GpuResidentSet(250, prefetch_batches=0, max_upload_bytes=10_000)
    manager.reset([
        _batch("A", 100, 0.0),
        _batch("B", 100, 2.0),
        _batch("C", 100, 4.0),
    ])

    plan = manager.plan(visible_keys={"A", "B", "C"}, camera_position=(0.0, 0.0, 0.0))

    assert plan.upload == ("A", "B")
    assert plan.resident == ("A", "B")
    assert plan.resident_bytes == 200
    assert plan.visible_resident == 2
    assert plan.visible_requested == 3
    assert plan.visible_dropped == ("C",)


def test_camera_move_evicts_irrelevant_batches_and_loads_new_visible_region() -> None:
    manager = GpuResidentSet(220, prefetch_batches=0, max_upload_bytes=10_000)
    manager.reset([
        _batch("WEST", 100, -50.0),
        _batch("CENTER", 100, 0.0),
        _batch("EAST", 100, 50.0),
    ])
    first = manager.plan(visible_keys={"WEST", "CENTER"}, camera_position=(-50.0, 0.0, 0.0))
    assert set(first.resident) == {"WEST", "CENTER"}

    second = manager.plan(visible_keys={"EAST"}, camera_position=(50.0, 0.0, 0.0))

    assert "WEST" in second.evict
    assert "EAST" in second.upload
    assert "EAST" in second.resident
    assert second.resident_bytes <= 220


def test_upload_work_is_progressively_throttled_across_frames() -> None:
    manager = GpuResidentSet(
        1000,
        prefetch_batches=0,
        max_upload_batches=1,
        max_upload_bytes=150,
    )
    manager.reset([_batch(str(index), 100, float(index)) for index in range(4)])

    first = manager.plan(visible_keys={"0", "1", "2", "3"}, camera_position=(0.0, 0.0, 0.0))
    second = manager.plan(visible_keys={"0", "1", "2", "3"}, camera_position=(0.0, 0.0, 0.0))
    third = manager.plan(visible_keys={"0", "1", "2", "3"}, camera_position=(0.0, 0.0, 0.0))
    fourth = manager.plan(visible_keys={"0", "1", "2", "3"}, camera_position=(0.0, 0.0, 0.0))

    assert first.upload == ("0",)
    assert second.upload == ("1",)
    assert third.upload == ("2",)
    assert fourth.upload == ("3",)
    assert set(fourth.resident) == {"0", "1", "2", "3"}
    assert fourth.target_pending == ()


def test_prefetch_fills_spare_budget_without_displacing_visible_batches() -> None:
    manager = GpuResidentSet(310, prefetch_batches=5, max_upload_bytes=10_000)
    manager.reset([
        _batch("VISIBLE", 100, 0.0),
        _batch("NEAR", 100, 2.0),
        _batch("FAR", 100, 20.0),
        _batch("TOO-MUCH", 100, 30.0),
    ])

    plan = manager.plan(visible_keys={"VISIBLE"}, camera_position=(0.0, 0.0, 0.0))

    assert plan.resident == ("VISIBLE", "NEAR", "FAR")
    assert plan.resident_bytes == 300
    assert "VISIBLE" not in plan.visible_dropped


def test_single_oversized_visible_batch_is_admitted_for_forward_progress() -> None:
    manager = GpuResidentSet(100, prefetch_batches=0, max_upload_bytes=50)
    manager.reset([_batch("BIG", 250, 0.0)])

    plan = manager.plan(visible_keys={"BIG"}, camera_position=(0.0, 0.0, 0.0))

    assert plan.upload == ("BIG",)
    assert plan.resident == ("BIG",)
    assert plan.resident_bytes == 250


def test_reducing_budget_evicts_resident_prefetch_on_next_plan() -> None:
    manager = GpuResidentSet(400, prefetch_batches=3, max_upload_bytes=10_000)
    manager.reset([
        _batch("VISIBLE", 100, 0.0),
        _batch("P1", 100, 2.0),
        _batch("P2", 100, 3.0),
    ])
    first = manager.plan(visible_keys={"VISIBLE"}, camera_position=(0.0, 0.0, 0.0))
    assert set(first.resident) == {"VISIBLE", "P1", "P2"}

    manager.set_budget(120)
    second = manager.plan(visible_keys={"VISIBLE"}, camera_position=(0.0, 0.0, 0.0))

    assert second.resident == ("VISIBLE",)
    assert set(second.evict) == {"P1", "P2"}
    assert second.resident_bytes == 100
