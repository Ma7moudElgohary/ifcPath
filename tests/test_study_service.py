from __future__ import annotations

from shapely.geometry import Polygon

from ifcpath.cdt import build_floor_cdt_navmesh
from ifcpath.model import InavModel, Level, NavCell, Portal, Space
from ifcpath.study_service import StudyRequestError, run_study


def _single_room_model() -> InavModel:
    model = InavModel(
        levels=[Level("L1", "Ground", 0.0)],
        spaces=[Space("room", "Room", "L1")],
        portals=[
            Portal(
                "exit:1",
                "door",
                (9.7, 5.0, 0.0),
                from_space_id="room",
                level_id="L1",
                width_m=1.5,
                is_exit=True,
            )
        ],
    )
    cdt = build_floor_cdt_navmesh(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]), 0.0)
    ids = [f"cell:{index}" for index in range(len(cdt.cells))]
    model.cells = [
        NavCell(
            id=ids[index],
            vertices_m=cell.vertices,
            space_id="room",
            level_id="L1",
            neighbor_ids=[ids[n] for n in cell.neighbor_indices],
        )
        for index, cell in enumerate(cdt.cells)
    ]
    return model


def test_live_study_expands_population_and_returns_playback() -> None:
    model = _single_room_model()
    result = run_study(
        {
            "model": model.to_dict(),
            "backend": "kinematic",
            "groups": [
                {
                    "id": "office",
                    "position_m": [2.0, 5.0, 0.0],
                    "count": 8,
                    "spacing_m": 0.45,
                    "speed_mps": 1.2,
                }
            ],
            "frame_interval_s": 0.1,
            "max_time_s": 30.0,
        }
    )

    assert result["schema"] == "ifcpath.simulation/0.1"
    assert result["backend"] == "kinematic"
    assert result["scenario"]["agent_count"] == 8
    assert len(result["frames"][0]["agents"]) == 8
    starts = {tuple(agent["position_m"]) for agent in result["frames"][0]["agents"]}
    assert len(starts) == 8
    assert result["frames"][-1]["time_s"] > 0


def test_live_study_rejects_unknown_backend() -> None:
    model = _single_room_model()
    try:
        run_study(
            {
                "model": model.to_dict(),
                "backend": "not-a-solver",
                "agents": [{"id": "a", "start_m": [2, 2, 0]}],
            }
        )
    except StudyRequestError as exc:
        assert "unsupported study backend" in str(exc)
    else:
        raise AssertionError("invalid backend should fail")
