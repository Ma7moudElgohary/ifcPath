from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from .hybrid_evacuation import HybridEvacuationSimulator
from .model import Vec3


PLAYBACK_SCHEMA = "ifcpath.simulation/0.1"


@dataclass(frozen=True, slots=True)
class PlaybackAgent:
    id: str
    position_m: Vec3
    forward_xy: tuple[float, float]
    status: str
    level_id: str | None = None
    space_id: str | None = None
    waiting_gate_id: str | None = None
    elevator_resource_id: str | None = None


@dataclass(frozen=True, slots=True)
class PlaybackFrame:
    time_s: float
    agents: tuple[PlaybackAgent, ...]
    stats: dict[str, Any]
    elevators: dict[str, Any]


def capture_playback_frame(simulator: HybridEvacuationSimulator) -> PlaybackFrame:
    """Capture one renderer-independent frame from live hybrid simulation state."""
    agents = tuple(
        PlaybackAgent(
            id=state.spec.id,
            position_m=tuple(float(value) for value in state.position),
            forward_xy=(float(state.forward_xy[0]), float(state.forward_xy[1])),
            status=state.status,
            level_id=state.active_level_id,
            space_id=state.active_space_id,
            waiting_gate_id=state.waiting_gate_id,
            elevator_resource_id=state.active_elevator_resource_id,
        )
        for state in simulator.agents
    )
    elevators = {
        resource_id: asdict(snapshot)
        for resource_id, snapshot in simulator.elevator_snapshots.items()
    }
    return PlaybackFrame(
        time_s=float(simulator.elapsed_s),
        agents=agents,
        stats=asdict(simulator.stats),
        elevators=elevators,
    )


def record_playback(
    simulator: HybridEvacuationSimulator,
    *,
    frame_interval_s: float = 0.20,
    max_time_s: float = 3600.0,
) -> dict[str, Any]:
    """Run a hybrid evacuation and record browser/engine-neutral snapshots.

    When ``HybridEvacuationConfig.local_backend='jupedsim'`` these frames contain
    JuPedSim collision-avoidance motion. The same contract also works with the
    deterministic kinematic backend for tests and deployments without JuPedSim.
    """
    interval = float(frame_interval_s)
    if interval <= 0.0:
        raise ValueError("frame_interval_s must be > 0")
    limit = max(0.0, float(max_time_s))

    frames = [capture_playback_frame(simulator)]
    while not simulator.finished and simulator.elapsed_s < limit:
        simulator.advance(min(interval, limit - simulator.elapsed_s))
        frames.append(capture_playback_frame(simulator))
        if frames[-1].time_s <= frames[-2].time_s:
            break

    return {
        "schema": PLAYBACK_SCHEMA,
        "frame_interval_s": interval,
        "backend": simulator.hybrid_config.local_backend,
        "frames": [asdict(frame) for frame in frames],
        "summary": asdict(simulator.stats),
    }


def save_playback(playback: dict[str, Any], path: str | Path) -> Path:
    target = Path(path)
    target.write_text(json.dumps(playback, indent=2), encoding="utf-8")
    return target
